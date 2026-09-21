// SPDX-License-Identifier: AGPL-3.0-only
//! The segment-range registry — a pure `#[cfg]`-free classifier that answers
//! "is this pointer inside a registered shared-memory range?" by an EXACT
//! half-open address test, never a heuristic.
//!
//! Two callers register ranges:
//!
//! * The take side (closing the residual): a forged libstdc++ vector
//!   triplet points its `{begin, end, cap}` at a held iceoryx2 sample's bytes,
//!   `capacity == size` so any growth reallocates. When a mutable callback
//!   grows that vector, libstdc++ calls `free(old_begin)` — a shared-memory
//!   address. Sent to glibc `free` it poisons the tcache; classified here it is
//!   routed to the registered release callback instead (`free` never sees it).
//! * The publish side: a loan slot the rmw chose to keep alive can be
//!   registered so its later `free` routes to a slot-release cookie, never
//!   glibc.
//!
//! Every range carries an opaque `cookie` (the caller's sample/slot handle as a
//! `usize`) the release callback receives, so one global callback can release
//! any registered range. Ranges never overlap by construction (distinct SHM
//! samples); registering an overlapping range is a caller bug and is REFUSED
//! loudly rather than silently first-matched.

/// Identifies a registered segment for later unregister/classify. Newtype over
/// the range's start address, which is unique across live ranges (they never
/// overlap) and is exactly what a caller has in hand to unregister.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SegmentId(pub usize);

/// One registered shared-memory range `[start, start + len)` (half-open) plus
/// the opaque cookie handed to the release callback.
///
/// `tombstoned` (quarantine entries only, a design decision,
/// tombstone-on-retire): a LIVE quarantine extent (armed-then-disarmed,
/// awaiting `retire_slot`) has `tombstoned == false`; `retire_slot` sets it
/// `true` — DEAD-BUT-REMEMBERED — instead of deleting the entry, so a late
/// `free` of an incidental in-slot pointer (which can outlive the sample) stays
/// a counted no-op forever rather than reaching glibc. A re-arm at the same
/// base REVIVES the entry (`false`), recycling it — boundedness rests on the
/// pool recycling slot addresses. Take-side registrations are never tombstoned.
///
/// `handoff_refs` (release hand-off refcount — ABA-safe close):
/// a release parks the freed range in the quarantine so it stays covered across
/// the unregister→callback gap, then removes it after the callback. If a
/// SAME-BASE segment is registered and released while the first callback is
/// still running, both releases share this one base-keyed entry — so removal is
/// refcounted: each `open_handoff` increments, each `close_handoff` decrements,
/// and the entry is removed only when the count returns to zero (no overlapping
/// release still depends on it). This is the ABA-proof form of the earlier
/// Created/Extended distinction. Arm-window / atfork-fold inserts do NOT touch
/// this count (they go through `insert_or_extend`), so a `close_handoff` can
/// only ever remove an entry a hand-off actually created.
///
/// `persistent` (arm/tombstone ownership — the INVERSE interleave):
/// `handoff_refs` alone covers persistent-coverage-arrives-
/// FIRST (a release extending arm coverage takes no ref), but not
/// persistent-coverage-arrives-SECOND — a slot RE-ARMED while a hand-off is
/// open extended the entry without establishing ownership, so the hand-off's
/// close (refs 1→0) discarded the re-armed coverage and a later SHM free
/// reached libc. `persistent` is set by every arm/fold insert
/// (`insert_or_extend`) and by `tombstone`, never by `open_handoff`'s own
/// create, and removal happens ONLY when `handoff_refs == 0` AND `!persistent`
/// — a segment is removed once neither hand-off nor persistent coverage
/// remains.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Segment {
    start: usize,
    len: usize,
    cookie: usize,
    tombstoned: bool,
    handoff_refs: u32,
    persistent: bool,
}

impl Segment {
    /// The exclusive end of the range. Saturating: a range whose `start + len`
    /// would wrap `usize` is clamped to `usize::MAX`, so `contains` stays sound
    /// (it can never report `true` for an address past the real end).
    #[inline]
    pub fn end(&self) -> usize {
        self.start.saturating_add(self.len)
    }

    /// The registered start address.
    #[inline]
    pub fn start(&self) -> usize {
        self.start
    }

    /// The registered length in bytes.
    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    /// A zero-length segment registers nothing meaningful.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The opaque caller cookie (sample/slot handle) for the release callback.
    #[inline]
    pub fn cookie(&self) -> usize {
        self.cookie
    }

    /// Whether this quarantine extent has been RETIRED (tombstone-on-retire):
    /// still covers its range for classification, but a `free` of an in-slot
    /// pointer counts as a tombstone hit rather than a live quarantine no-op.
    #[inline]
    pub fn tombstoned(&self) -> bool {
        self.tombstoned
    }

    /// Whether this entry carries PERSISTENT (arm/tombstone) ownership — a
    /// hand-off close can never remove it while set.
    #[inline]
    pub fn persistent(&self) -> bool {
        self.persistent
    }

    /// EXACT half-open membership: `start <= addr < end`.
    ///
    /// This is the classification the `free`/`realloc` interposers turn on, so
    /// the boundary is load-bearing on BOTH ends and both are pinned by tests:
    /// the first byte (`addr == start`) is in; the byte one past the end
    /// (`addr == end`) is out; a zero-length range contains nothing.
    #[inline]
    pub fn contains(&self, addr: usize) -> bool {
        addr >= self.start && addr < self.end()
    }
}

/// Why a [`SegmentRegistry::register`] was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegisterError {
    /// A zero-length range carries no bytes to classify.
    ZeroLength,
    /// The range overlaps a live registration — a caller bug (SHM samples do
    /// not overlap). Carries the id of the range it collided with.
    Overlaps(SegmentId),
}

/// The set of live shared-memory ranges. A plain `Vec` scanned linearly: the
/// live count is tiny (outstanding loans/samples), and the scan runs under the
/// interposer's re-entrancy guard so any bookkeeping allocation bypasses to the
/// real allocator. The Linux FFI wraps ONE of these behind a global lock.
#[derive(Debug, Default)]
pub struct SegmentRegistry {
    segments: Vec<Segment>,
}

impl SegmentRegistry {
    /// An empty registry.
    pub const fn new() -> Self {
        Self {
            segments: Vec::new(),
        }
    }

    /// Live segment count.
    pub fn len(&self) -> usize {
        self.segments.len()
    }

    /// Whether nothing is registered.
    pub fn is_empty(&self) -> bool {
        self.segments.is_empty()
    }

    /// The backing `Vec`'s allocated capacity (element slots before the next
    /// reallocation). Used to pin the `atfork`-child capacity invariant.
    pub fn capacity(&self) -> usize {
        self.segments.capacity()
    }

    /// Spare push slots before the next reallocation (`capacity - len`).
    pub fn spare_capacity(&self) -> usize {
        self.segments.capacity() - self.segments.len()
    }

    /// Ensure at least `additional` spare push slots exist, so `additional`
    /// further `insert_or_extend`s of NEW bases cannot REALLOCATE. This is how
    /// the parent pre-sizes the quarantine for the `atfork` CHILD fold, which
    /// MUST be allocation-free (malloc is not async-signal-safe; a child fold
    /// that reallocated could deadlock on an allocator lock a vanished parent
    /// thread held across the fork). A no-op when the spare already suffices.
    ///
    /// Delegates the arithmetic to `Vec::reserve(additional)`, whose contract
    /// is exactly `capacity >= len + additional`, i.e. spare >= `additional`.
    /// Deliberately no `additional - spare` subtraction:
    /// that form guarantees only `len + additional - spare` — an UNDER-reserve
    /// whenever spare > 0 and the amortized doubling cannot rescue it.
    pub fn reserve(&mut self, additional: usize) {
        self.segments.reserve(additional);
    }

    /// Register `[start, start + len)` with `cookie`.
    ///
    /// Refuses a zero-length range and any range that overlaps a live one (SHM
    /// samples never overlap; an overlap is a caller bug, reported not hidden).
    pub fn register(
        &mut self,
        start: usize,
        len: usize,
        cookie: usize,
    ) -> Result<SegmentId, RegisterError> {
        if len == 0 {
            return Err(RegisterError::ZeroLength);
        }
        let new = Segment {
            start,
            len,
            cookie,
            tombstoned: false,
            handoff_refs: 0,
            persistent: false,
        };
        for existing in &self.segments {
            if ranges_overlap(existing, &new) {
                return Err(RegisterError::Overlaps(SegmentId(existing.start)));
            }
        }
        self.segments.push(new);
        Ok(SegmentId(start))
    }

    /// Insert `[start, start + len)` as a QUARANTINE extent, keyed by `start` and
    /// OWNERSHIP-AWARE — it never coalesces distinct slot bases.
    ///
    /// This is deliberately NOT an interval union: two distinct loan slots whose
    /// extents ABUT (slot A's `tail_limit == slot B's base`) must stay SEPARATE
    /// entries, each independently removable, so that `retire_slot(base_A)`
    /// cannot delete slot B's coverage (and vice versa) — a coalescing union
    /// would merge them into one range starting at the lower base, making the
    /// higher base unretirable AND letting a retire of the lower base
    /// un-quarantine the neighbour, whose next in-slot free would then reach
    /// glibc `free` of shared memory (the exact corruption this crate exists to
    /// prevent). Different-base extents that overlap likewise coexist (distinct
    /// loan slots never overlap in practice; if they somehow do, coverage holds
    /// until BOTH are retired).
    ///
    /// A re-insert at the SAME `start` (a slot re-armed before its prior extent
    /// was retired, OR a slot address the pool RECYCLED after `retire_slot`
    /// tombstoned it) EXTENDS the existing entry to the larger length and
    /// REVIVES it (clears the tombstone — tombstone-on-retire): the recycled
    /// slot is live coverage again, and the entry is reused rather than
    /// duplicated, which is what bounds the tombstone set by the pool's distinct
    /// slot addresses. `cookie` is always 0 (free is a no-op; nothing to
    /// release). `len == 0` is a no-op.
    pub fn insert_or_extend(&mut self, start: usize, len: usize) -> InsertOutcome {
        if len == 0 {
            return InsertOutcome::Noop;
        }
        if let Some(seg) = self.segments.iter_mut().find(|s| s.start == start) {
            seg.len = seg.len.max(len);
            seg.tombstoned = false; // revive a recycled/re-armed slot
                                    // The INVERSE-interleave marker: an arm/fold
                                    // insert establishes PERSISTENT ownership even when it lands on an
                                    // entry an open hand-off created — a re-arm during the callback
                                    // gap must survive that hand-off's close.
            seg.persistent = true;
            InsertOutcome::Extended
        } else {
            self.segments.push(Segment {
                start,
                len,
                cookie: 0,
                tombstoned: false,
                handoff_refs: 0,
                persistent: true,
            });
            InsertOutcome::Created
        }
    }

    /// TOMBSTONE the entry starting at `start` (tombstone-on-retire): mark it
    /// DEAD-BUT-REMEMBERED so a late `free` of an in-slot pointer stays a
    /// counted no-op forever, instead of deleting it (a delete would
    /// let a post-retire in-slot free fall through to glibc). Returns
    /// `true` if an entry started there, `false` otherwise (an unknown/
    /// double-retire — reported, never a silent success). Idempotent: retiring
    /// an already-tombstoned entry re-tombstones it and returns `true`.
    pub fn tombstone(&mut self, start: usize) -> bool {
        if let Some(seg) = self.segments.iter_mut().find(|s| s.start == start) {
            seg.tombstoned = true;
            // A tombstone is PERSISTENT coverage (dead-but-remembered
            // forever) — even one stamped onto a hand-off-created entry must
            // survive that hand-off's close.
            seg.persistent = true;
            true
        } else {
            false
        }
    }

    /// Pending release hand-offs on the entry at `start` (0 for a live/tombstoned
    /// arm-window entry that no release is holding open).
    pub fn handoff_refs(&self, start: usize) -> u32 {
        self.segments
            .iter()
            .find(|s| s.start == start)
            .map(|s| s.handoff_refs)
            .unwrap_or(0)
    }

    /// OPEN a release hand-off over `[start, start + len)` (ABA-safe close).
    /// Ensure the coverage exists, then take a pending-release
    /// REF — and return whether a matching [`Self::close_handoff`] is OWED
    /// (`true`) — under exactly the conditions where this release's close must
    /// participate in removal:
    ///
    /// * NO entry existed → this release CREATED the transient coverage (NOT
    ///   persistent — this is the one insert path that does not mark
    ///   [`Segment::persistent`]) → ref taken, close owed;
    /// * an entry a hand-off already holds open (`handoff_refs > 0`) → an
    ///   OVERLAPPING same-base release → ref taken, close owed (only the LAST
    ///   close can remove — the ABA fix);
    /// * an entry with NO hand-off ref but PERSISTENT (arm-window / tombstone)
    ///   coverage → the range is extended so it stays covered, but NO ref is
    ///   taken and NO close is owed — that coverage is not this release's to
    ///   remove.
    ///
    /// Deliberately does NOT revive a tombstone and does NOT mark persistent —
    /// a hand-off is transient by definition; arm/fold ownership is
    /// [`Self::insert_or_extend`]'s to grant. `len == 0` is a no-op (`false`).
    pub fn open_handoff(&mut self, start: usize, len: usize) -> bool {
        if len == 0 {
            return false;
        }
        if let Some(seg) = self.segments.iter_mut().find(|s| s.start == start) {
            seg.len = seg.len.max(len);
            if seg.handoff_refs == 0 && seg.persistent {
                return false; // pre-existing persistent coverage: not ours to close
            }
            seg.handoff_refs = seg.handoff_refs.saturating_add(1);
            true
        } else {
            self.segments.push(Segment {
                start,
                len,
                cookie: 0,
                tombstoned: false,
                handoff_refs: 1,
                persistent: false,
            });
            true
        }
    }

    /// CLOSE a release hand-off at `start`: decrement the pending-release
    /// refcount, and REMOVE the entry only when neither hand-off NOR persistent
    /// coverage remains — `handoff_refs == 0` AND `!persistent` (a slot
    /// RE-ARMED while this hand-off was open marked the shared
    /// entry persistent, and that re-armed coverage must SURVIVE this close).
    /// Returns [`HandoffClose::Removed`] when the entry was removed,
    /// [`HandoffClose::StillReferenced`] when another release or persistent
    /// ownership keeps it, [`HandoffClose::NotFound`] when no entry started at
    /// `start` (a double-close or unknown base — reported, never silent).
    pub fn close_handoff(&mut self, start: usize) -> HandoffClose {
        let Some(seg) = self.segments.iter_mut().find(|s| s.start == start) else {
            return HandoffClose::NotFound;
        };
        seg.handoff_refs = seg.handoff_refs.saturating_sub(1);
        if seg.handoff_refs == 0 && !seg.persistent {
            self.unregister(start);
            HandoffClose::Removed
        } else {
            HandoffClose::StillReferenced
        }
    }

    /// Unregister the range starting at `start`. Returns `true` if a range was
    /// removed, `false` if none started there (a double-unregister or an
    /// unknown address — the caller reports it; never a silent success).
    pub fn unregister(&mut self, start: usize) -> bool {
        if let Some(idx) = self.segments.iter().position(|s| s.start == start) {
            self.segments.remove(idx);
            true
        } else {
            false
        }
    }

    /// The segment containing `addr`, if any (the exact-range classification).
    pub fn classify(&self, addr: usize) -> Option<Segment> {
        self.segments.iter().copied().find(|s| s.contains(addr))
    }

    /// All live segments (the `atfork` child folds these into the quarantine).
    pub fn segments(&self) -> &[Segment] {
        &self.segments
    }

    /// Drop every registration (the `atfork` child handler resets the process).
    pub fn clear(&mut self) {
        self.segments.clear();
    }
}

/// What [`SegmentRegistry::insert_or_extend`] did. Informational for a release
/// hand-off (whether THIS open created the entry); the pending-release refcount,
/// not this outcome, governs removal (see [`SegmentRegistry::close_handoff`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InsertOutcome {
    /// A new entry was pushed (within reserved capacity on the locked paths).
    Created,
    /// An entry at the same base already existed; its length absorbed `len`.
    Extended,
    /// `len == 0` — nothing recorded.
    Noop,
}

/// The result of [`SegmentRegistry::close_handoff`] — an ABA-safe hand-off
/// close removes the entry only when the last overlapping release lets go.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HandoffClose {
    /// The refcount reached zero; the entry was removed.
    Removed,
    /// Another overlapping release still holds the entry open; kept.
    StillReferenced,
    /// No entry started at that base (a double-close or unknown base).
    NotFound,
}

/// Two half-open ranges overlap iff each starts before the other ends. Uses the
/// saturating `end()` so a range near `usize::MAX` cannot wrap into a false
/// negative.
#[inline]
fn ranges_overlap(a: &Segment, b: &Segment) -> bool {
    a.start < b.end() && b.start < a.end()
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Segment::contains — the exact half-open boundary (off-by-one target).

    #[test]
    fn contains_is_half_open_on_both_ends() {
        let s = Segment {
            start: 1000,
            len: 100,
            cookie: 7,
            tombstoned: false,
            handoff_refs: 0,
            persistent: false,
        }; // [1000, 1100)
        assert!(!s.contains(999), "one before start is OUT");
        assert!(s.contains(1000), "start itself is IN (first byte)");
        assert!(s.contains(1099), "last byte is IN");
        assert!(!s.contains(1100), "end (one past last) is OUT");
        assert!(!s.contains(1101), "past the end is OUT");
    }

    #[test]
    fn a_zero_length_segment_contains_nothing() {
        let s = Segment {
            start: 1000,
            len: 0,
            cookie: 0,
            tombstoned: false,
            handoff_refs: 0,
            persistent: false,
        };
        assert!(!s.contains(1000));
        assert!(!s.contains(999));
    }

    #[test]
    fn a_len_one_segment_contains_exactly_its_start() {
        let s = Segment {
            start: 42,
            len: 1,
            cookie: 0,
            tombstoned: false,
            handoff_refs: 0,
            persistent: false,
        };
        assert!(s.contains(42));
        assert!(!s.contains(43));
        assert!(!s.contains(41));
    }

    #[test]
    fn a_segment_near_usize_max_does_not_wrap() {
        let s = Segment {
            start: usize::MAX - 3,
            len: 8, // would wrap; saturates end() to usize::MAX
            cookie: 0,
            tombstoned: false,
            handoff_refs: 0,
            persistent: false,
        };
        assert_eq!(s.end(), usize::MAX);
        assert!(s.contains(usize::MAX - 3));
        assert!(s.contains(usize::MAX - 1));
        // usize::MAX is < saturated end (usize::MAX)? No — half-open excludes it.
        assert!(!s.contains(usize::MAX));
    }

    // ── Registry register/unregister/classify + overlap rejection.

    #[test]
    fn register_then_classify_finds_the_cookie() {
        let mut r = SegmentRegistry::new();
        let id = r.register(2000, 512, 0xABCD).expect("register");
        assert_eq!(id, SegmentId(2000));
        let seg = r.classify(2000 + 511).expect("last byte classifies");
        assert_eq!(seg.cookie(), 0xABCD);
        assert!(
            r.classify(2000 + 512).is_none(),
            "end byte does not classify"
        );
        assert!(r.classify(1999).is_none(), "before start does not classify");
    }

    #[test]
    fn a_zero_length_registration_is_refused() {
        let mut r = SegmentRegistry::new();
        assert_eq!(r.register(2000, 0, 0), Err(RegisterError::ZeroLength));
        assert!(r.is_empty());
    }

    #[test]
    fn an_overlapping_registration_is_refused_naming_the_collision() {
        let mut r = SegmentRegistry::new();
        let first = r.register(1000, 100, 1).expect("first"); // [1000, 1100)
                                                              // Overlaps at the seam (1050 is inside the first).
        assert_eq!(
            r.register(1050, 100, 2),
            Err(RegisterError::Overlaps(first))
        );
        // Abutting exactly at the end is NOT an overlap (half-open).
        assert!(
            r.register(1100, 100, 3).is_ok(),
            "an abutting range [1100,1200) touches [1000,1100) only at the open end"
        );
        assert_eq!(r.len(), 2);
    }

    #[test]
    fn abutting_the_start_is_not_an_overlap() {
        let mut r = SegmentRegistry::new();
        r.register(1000, 100, 1).expect("first"); // [1000, 1100)
        assert!(
            r.register(900, 100, 2).is_ok(),
            "[900,1000) abuts [1000,1100) at the closed end — no shared byte"
        );
    }

    #[test]
    fn containment_either_way_is_an_overlap() {
        let mut r = SegmentRegistry::new();
        let outer = r.register(1000, 200, 1).expect("outer"); // [1000, 1200)
                                                              // A new range fully INSIDE the existing one overlaps.
        assert_eq!(r.register(1050, 50, 2), Err(RegisterError::Overlaps(outer)));
        // A new range fully CONTAINING an existing one overlaps too.
        let mut r2 = SegmentRegistry::new();
        let inner = r2.register(1050, 50, 1).expect("inner"); // [1050, 1100)
        assert_eq!(
            r2.register(1000, 200, 2),
            Err(RegisterError::Overlaps(inner))
        );
    }

    #[test]
    fn unregister_removes_exactly_one_and_reports_misses() {
        let mut r = SegmentRegistry::new();
        r.register(1000, 100, 1).expect("a");
        r.register(2000, 100, 2).expect("b");
        assert!(r.unregister(1000), "known start removes");
        assert_eq!(r.len(), 1);
        assert!(!r.unregister(1000), "a double-unregister reports false");
        assert!(!r.unregister(9999), "an unknown start reports false");
        // The sibling survived.
        assert!(r.classify(2050).is_some());
    }

    #[test]
    fn classify_picks_the_right_segment_among_many() {
        let mut r = SegmentRegistry::new();
        r.register(1000, 100, 11).expect("a");
        r.register(5000, 100, 55).expect("b");
        r.register(9000, 100, 99).expect("c");
        assert_eq!(r.classify(5050).unwrap().cookie(), 55);
        assert_eq!(r.classify(9000).unwrap().cookie(), 99);
        assert!(r.classify(3000).is_none(), "a gap classifies to nothing");
    }

    // ── insert_or_extend — the ownership-aware quarantine fold ──────────────

    #[test]
    fn insert_or_extend_keeps_distinct_bases_separate() {
        let mut r = SegmentRegistry::new();
        r.insert_or_extend(1000, 100); // [1000, 1100)
        r.insert_or_extend(5000, 100); // disjoint
        assert_eq!(r.len(), 2);
        assert!(r.classify(1050).is_some() && r.classify(5050).is_some());
    }

    #[test]
    fn insert_or_extend_does_not_coalesce_abutting_distinct_slots() {
        // Two DISTINCT loan slots whose extents touch must NOT merge — else one
        // retire would delete the other's coverage.
        let mut r = SegmentRegistry::new();
        r.insert_or_extend(1000, 100); // slot A [1000, 1100)
        r.insert_or_extend(1100, 100); // slot B [1100, 1200) — abuts A
        assert_eq!(r.len(), 2, "abutting distinct slots stay separate");
    }

    #[test]
    fn adjacent_disarmed_slots_retire_independently_in_either_order() {
        // THE regression: retiring one slot must not un-quarantine its neighbour.
        for (first, second) in [(1000usize, 1100usize), (1100usize, 1000usize)] {
            let mut r = SegmentRegistry::new();
            r.insert_or_extend(1000, 100); // A [1000, 1100)
            r.insert_or_extend(1100, 100); // B [1100, 1200)
            assert!(r.unregister(first), "retire the first slot");
            // The OTHER slot's coverage survives.
            let survivor_probe = if second == 1000 { 1050 } else { 1150 };
            assert!(
                r.classify(survivor_probe).is_some(),
                "retiring base {first} must not remove base {second}'s coverage"
            );
            assert!(
                r.unregister(second),
                "the survivor is still retirable by its own base"
            );
            assert!(r.is_empty());
        }
    }

    #[test]
    fn insert_or_extend_at_the_same_base_extends_to_the_larger_len() {
        // A slot re-armed (same base) before its prior extent was retired: the
        // entry EXTENDS so the new, longer tail is covered, `start` unchanged so
        // `retire_slot(base)` still finds it.
        let mut r = SegmentRegistry::new();
        // The outcome discrimination is load-bearing:
        // a release hand-off schedules its post-callback close only for a
        // `Created` entry — a swapped verdict would delete pre-existing
        // coverage (Extended treated as Created) or leak transient entries.
        assert_eq!(r.insert_or_extend(1000, 100), InsertOutcome::Created); // [1000, 1100)
        assert_eq!(r.insert_or_extend(1000, 200), InsertOutcome::Extended); // → [1000, 1200)
        assert_eq!(r.len(), 1, "same base does not add a second entry");
        assert!(r.classify(1150).is_some(), "the extended tail is covered");
        // A shorter re-insert never SHRINKS coverage.
        r.insert_or_extend(1000, 50);
        assert!(
            r.classify(1150).is_some(),
            "a shorter re-insert keeps the longer len"
        );
        assert!(r.unregister(1000) && r.is_empty());
    }

    // Tombstone-on-retire (pure): retire MARKS an entry dead-but-remembered
    // (still covers its range), and a re-arm at the same base REVIVES it —
    // recycling the entry rather than duplicating it, which is what bounds the
    // tombstone set by the pool's distinct slot addresses.
    #[test]
    fn tombstone_marks_dead_but_remembered_and_re_arm_revives() {
        let mut r = SegmentRegistry::new();
        assert_eq!(r.insert_or_extend(1000, 100), InsertOutcome::Created);
        assert!(
            !r.classify(1050).unwrap().tombstoned(),
            "LIVE before retire"
        );

        // Retire tombstones — the entry survives and still covers its range.
        assert!(r.tombstone(1000), "tombstone finds the entry");
        assert_eq!(r.len(), 1, "tombstone does not remove the entry");
        assert!(
            r.classify(1050).map(|s| s.tombstoned()).unwrap_or(false),
            "a tombstoned extent still COVERS its range (late free stays a no-op)"
        );

        // Re-arm at the same base REVIVES (Extended, not Created; tombstone cleared).
        assert_eq!(r.insert_or_extend(1000, 100), InsertOutcome::Extended);
        assert_eq!(
            r.len(),
            1,
            "revive reuses the entry — bounded, not duplicated"
        );
        assert!(
            !r.classify(1050).unwrap().tombstoned(),
            "re-arm cleared the tombstone (recycled slot is live coverage)"
        );

        // Retiring an unknown base reports a miss (never a silent success);
        // re-tombstoning is idempotent.
        assert!(!r.tombstone(9999), "unknown base is a reported miss");
        assert!(
            r.tombstone(1000) && r.tombstone(1000),
            "re-tombstone is idempotent"
        );
        assert!(r.unregister(1000) && r.is_empty());
    }

    // ABA-safe hand-off close (pure): overlapping same-base
    // releases refcount the shared entry, so an EARLIER release's close cannot
    // remove coverage a LATER release still needs. The interleaving this
    // pins: A creates coverage and blocks in its callback; B registers +
    // releases the same base (extending A's entry); A returns and unconditionally
    // removes the entry → B uncovered → a later free reaches libc → abort.
    #[test]
    fn overlapping_handoffs_refcount_so_the_last_release_removes() {
        let mut r = SegmentRegistry::new();
        // A opens (creates), then B opens the SAME base (overlap) — both owe a
        // close (return true), the entry now holds two refs.
        assert!(
            r.open_handoff(1000, 100),
            "A creates → owns a ref, close owed"
        );
        assert!(
            r.open_handoff(1000, 100),
            "B overlaps → owns a ref, close owed"
        );
        assert_eq!(r.handoff_refs(1000), 2);
        assert!(r.classify(1050).is_some(), "covered while both are open");

        // A closes FIRST — the entry must STAY (B still needs it). This is the
        // exact ABA point: an unconditional remove would delete B's coverage.
        assert_eq!(r.close_handoff(1000), HandoffClose::StillReferenced);
        assert!(
            r.classify(1050).is_some(),
            "ABA: A's close must NOT remove coverage B still depends on"
        );
        // B closes LAST — refcount hits zero, entry removed.
        assert_eq!(r.close_handoff(1000), HandoffClose::Removed);
        assert!(
            r.classify(1050).is_none(),
            "the last release removed the entry"
        );
        // A double-close is a reported miss, never a silent success.
        assert_eq!(r.close_handoff(1000), HandoffClose::NotFound);
    }

    // The inverse
    // interleave — hand-off A opens FIRST (creating the entry), then the SAME
    // BASE is RE-ARMED while A's callback is out. The re-arm goes through
    // `insert_or_extend`, which marks the shared entry PERSISTENT; A's close
    // (refs 1→0) must therefore KEEP the entry — unregistering there discards
    // the re-armed coverage, and a later SHM free classifies
    // RealHeap → libc. A close that ignores `persistent` reds this.
    #[test]
    fn a_re_arm_during_an_open_handoff_survives_that_handoffs_close() {
        let mut r = SegmentRegistry::new();
        // A: release hand-off CREATES the entry (transient, not persistent).
        assert!(r.open_handoff(3000, 256), "A creates → owes a close");
        assert!(!r.classify(3050).unwrap().persistent());
        // The pool recycles the slot: RE-ARM the same base while A is out.
        assert_eq!(r.insert_or_extend(3000, 256), InsertOutcome::Extended);
        assert!(
            r.classify(3050).unwrap().persistent(),
            "the re-arm establishes persistent ownership on the shared entry"
        );
        // A returns — refs 1→0, but the persistent marker keeps the entry.
        assert_eq!(r.close_handoff(3000), HandoffClose::StillReferenced);
        assert!(
            r.classify(3050).is_some(),
            "INVERSE ABA: the re-armed coverage survives the hand-off's close"
        );
        assert!(r.classify(3050).unwrap().persistent());
        // The re-armed slot lives its normal life: retire tombstones it (still
        // covered), and only an explicit purge removes it.
        assert!(r.tombstone(3000));
        assert!(r.classify(3050).unwrap().tombstoned());
        assert!(r.unregister(3000) && r.is_empty());
    }

    // Compose check: a TOMBSTONED entry re-armed during an open
    // hand-off must survive the close AS REVIVED coverage (tombstone cleared,
    // persistent kept) — the revive and the inverse-ABA marker compose.
    #[test]
    fn a_tombstone_re_armed_during_an_open_handoff_survives_as_revived_coverage() {
        let mut r = SegmentRegistry::new();
        // A retired slot (persistent tombstone)…
        assert_eq!(r.insert_or_extend(4000, 256), InsertOutcome::Created);
        assert!(r.tombstone(4000));
        // …whose base a release hand-off then extends (no ref: persistent).
        assert!(
            !r.open_handoff(4000, 256),
            "persistent coverage: no close owed"
        );
        // The pool re-arms the base (revive) while nothing is owed.
        assert_eq!(r.insert_or_extend(4000, 256), InsertOutcome::Extended);
        let seg = r.classify(4050).expect("covered");
        assert!(!seg.tombstoned(), "re-arm revived the tombstone");
        assert!(seg.persistent(), "and the coverage stays persistent");
        assert!(r.unregister(4000) && r.is_empty());
    }

    // Control: a PURE hand-off entry (created by open_handoff, never
    // touched by an arm/tombstone) is transient — it still removes at refs 0.
    #[test]
    fn a_pure_handoff_entry_still_removes_at_refs_zero() {
        let mut r = SegmentRegistry::new();
        assert!(r.open_handoff(5000, 128), "created by the hand-off alone");
        assert!(!r.classify(5050).unwrap().persistent());
        assert_eq!(r.close_handoff(5000), HandoffClose::Removed);
        assert!(
            r.classify(5050).is_none(),
            "transient coverage removed at zero"
        );
    }

    // A release that EXTENDS pre-existing arm-window / tombstone
    // coverage (refs == 0) takes NO ref and owes NO close — that coverage must
    // outlive the release, so the release's close
    // must never be able to remove it.
    #[test]
    fn a_release_over_pre_existing_coverage_takes_no_ref_and_leaves_it_standing() {
        let mut r = SegmentRegistry::new();
        // Pre-existing arm-window-style coverage (no hand-off ref).
        assert_eq!(r.insert_or_extend(2000, 512), InsertOutcome::Created);
        assert_eq!(r.handoff_refs(2000), 0);
        // A release extends it but takes NO ref (no close owed).
        assert!(
            !r.open_handoff(2000, 256),
            "extending pre-existing non-release coverage owes no close"
        );
        assert_eq!(
            r.handoff_refs(2000),
            0,
            "no ref taken on pre-existing coverage"
        );
        assert!(
            r.classify(2050).is_some(),
            "the pre-existing coverage still stands"
        );
        // Even if a close were (wrongly) driven, the miss-report protects it:
        // the release owed no close, so it never calls close_handoff. Purge.
        assert!(r.unregister(2000) && r.is_empty());
    }

    #[test]
    fn insert_or_extend_of_a_zero_length_range_is_a_noop() {
        let mut r = SegmentRegistry::new();
        assert_eq!(r.insert_or_extend(1000, 0), InsertOutcome::Noop);
        assert!(r.is_empty());
    }

    #[test]
    fn clear_empties_the_registry() {
        let mut r = SegmentRegistry::new();
        r.register(1000, 100, 1).expect("a");
        r.register(2000, 100, 2).expect("b");
        r.clear();
        assert!(r.is_empty());
        assert!(r.classify(1050).is_none());
    }

    // The load-bearing primitive behind the `atfork`-child allocation-free
    // fold: if the quarantine is reserved for `registry.len()`
    // new bases in the PARENT, the child fold's `insert_or_extend`s cannot
    // REALLOCATE — so no malloc runs in the child atfork handler. Pure, so it
    // runs on every platform (the state-level wiring + real fork are box-gated).
    #[test]
    fn reserving_for_n_new_bases_makes_a_fold_of_them_allocation_free() {
        // Model the parent's registry: N distinct registered bases.
        let mut reg = SegmentRegistry::new();
        const N: usize = 64;
        for i in 0..N {
            reg.register(0x1_0000 + i * 0x1000, 0x100, i)
                .expect("register");
        }
        // The quarantine starts holding some armed slots at a DIFFERENT base
        // range (so the fold's bases are all genuinely new — the push path).
        let mut quar = SegmentRegistry::new();
        for i in 0..8 {
            quar.insert_or_extend(0xF00_0000 + i * 0x1000, 0x80);
        }
        // Parent pre-reserve, as `register_segment_reserving`/`arm_window` do.
        quar.reserve(reg.len());
        let cap_before = quar.capacity();
        assert!(
            quar.spare_capacity() >= reg.len(),
            "the reserve established spare >= registry.len()"
        );
        // The child fold: push every registered base into the quarantine.
        for seg in reg.segments() {
            quar.insert_or_extend(seg.start(), seg.len());
        }
        assert_eq!(
            quar.capacity(),
            cap_before,
            "the fold must not reallocate — that would malloc in the atfork child"
        );
        assert_eq!(quar.len(), 8 + N, "every registered base folded in");
        // Anti-tautology: WITHOUT the reserve the same fold DOES reallocate,
        // proving the reserve is what buys the guarantee (a fresh quarantine
        // whose capacity is below the fold count).
        let mut unreserved = SegmentRegistry::new();
        let empty_cap = unreserved.capacity(); // 0 for a fresh Vec
        for seg in reg.segments() {
            unreserved.insert_or_extend(seg.start(), seg.len());
        }
        assert!(
            unreserved.capacity() > empty_cap,
            "an unreserved fold of N bases must grow the Vec (the bug this guards)"
        );
    }

    #[test]
    fn reserve_is_a_noop_when_spare_already_suffices() {
        let mut r = SegmentRegistry::new();
        r.register(1000, 100, 1).expect("a");
        r.reserve(16); // grow once
        let cap = r.capacity();
        r.reserve(4); // already have >= 4 spare
        assert_eq!(r.capacity(), cap, "no reallocation when spare suffices");
    }

    // `reserve(additional)` must guarantee `additional`
    // SPARE slots. The `additional - spare` form guarantees only
    // `len + additional - spare` and UNDER-reserves whenever spare > 0 and the
    // amortized doubling cannot rescue it. The numbers are DERIVED from the
    // observed capacity, not a hardcoded growth sequence: with
    // `additional > 2*capacity - len + spare`, the buggy form's amortized
    // result (`max(2*cap, len + additional - spare)`) falls short of
    // `len + additional` by exactly `spare`, so this arm catches that bug.
    #[test]
    fn reserve_guarantees_additional_spare_even_when_partial_spare_exists() {
        let mut r = SegmentRegistry::new();
        for i in 0..10 {
            r.register(0x1000 + i * 0x100, 0x10, i).expect("register");
        }
        if r.spare_capacity() == 0 {
            r.reserve(1); // ensure a NONZERO existing spare (the buggy form's trigger)
        }
        let (len, cap, spare) = (r.len(), r.capacity(), r.spare_capacity());
        assert!(spare > 0, "precondition: partial spare exists");
        let additional = 2 * cap - len + spare + 1;
        r.reserve(additional);
        assert!(
            r.spare_capacity() >= additional,
            "reserve({additional}) must leave at least that many spare slots \
             (len {len}, cap {cap}, spare {spare} before; subtracting the \
             existing spare under-reserves)"
        );
    }
}
