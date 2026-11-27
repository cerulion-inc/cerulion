// SPDX-License-Identifier: AGPL-3.0-only
//! The PURE decisions the take paths share.
//!
//! Root `AGENTS.md` requires every mutation check to mutate a pure decision
//! function and NEVER a live syscall/transport path. The schema-hash gate is
//! the take paths' first decision and it existed in THREE inline copies —
//! `take_impl`, `take_adopted` and `take_loaned_impl` — each inside a
//! function that owns a live iceoryx2 receive, so the only way to mutate the
//! decision was to edit a live transport path, which the rule forbids. The
//! gate therefore had arms that asserted its behaviour and no legal way to
//! prove they bite.
//!
//! Extracting the comparison here gives all three copies ONE legal target:
//! the oracle is a table of `(frame, expected) -> verdict` pairs with no
//! transport in sight, any check of the decision touches THIS function, and a fourth copy added
//! later inherits both. The call sites keep their own consequences (which
//! latch, whether the frame is consumed, what the caller is told) — only the
//! decision moved.

/// What the frame's schema hash says about the frame.
///
/// The hash is the layout token: two peers that disagree on it disagree on
/// the message DEFINITION, so the bytes cannot be decoded against the
/// subscription's bridge at all. There is no tolerant middle — a near-miss
/// is as wrong as a random value — which is why this is a two-state verdict
/// and not a comparison the caller is trusted to spell correctly three
/// times.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HashVerdict {
    /// The frame was produced against the same message definition: decode it.
    Match,
    /// A version SKEW: the frame is consumed and dropped through the regime
    /// latch, `taken` stays false, and nothing is decoded.
    Mismatch,
}

impl HashVerdict {
    /// True for [`HashVerdict::Mismatch`] — the spelling the call sites read
    /// better than a `matches!`.
    pub fn is_mismatch(self) -> bool {
        matches!(self, HashVerdict::Mismatch)
    }
}

/// The take paths' schema-hash gate: does this frame's header hash agree
/// with the hash the subscription's bridge was built for?
///
/// Total over `u64 x u64`, allocation-free, and dependent on nothing but its
/// two arguments — the properties that make it a legal mutation target.
pub fn schema_hash_verdict(frame_hash: u64, expected_hash: u64) -> HashVerdict {
    if frame_hash == expected_hash {
        HashVerdict::Match
    } else {
        HashVerdict::Mismatch
    }
}

/// The largest FIXED (recursively plain) message, in bytes, whose loaned take
/// is served as a private copy; a larger one keeps the shared-memory loan.
///
/// The rule it pins ([`fixed_loan_shape`]): a fixed message of at most this
/// many bytes is copied into rmw-owned memory and its sample released at
/// once, so a `std::shared_ptr<MsgT>` callback may write it; a larger one is
/// handed out as the struct inside the held sample, read-only, so its take
/// stays zero-copy. The rule for its value: a private copy may add at most
/// 10% to one hop at the smallest measured size, which gives 4 KiB. Every test
/// derives its boundary types from it, so a new value changes nothing else.
pub const RMW_FIXED_LOAN_COPY_MAX_BYTES: usize = 4 * 1024;

/// What a FIXED type's loaned take hands out.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FixedLoanShape {
    /// A private copy of the struct in rmw-owned memory, writable; the sample
    /// is released before the take returns.
    PrivateCopy,
    /// The struct inside the held shared-memory sample, read-only (the
    /// subscriber's mapping is read-only), until the loan is returned.
    SharedMemory,
}

/// The fixed loaned take's shape for a message of `fixed_size` bytes: a
/// private copy up to and including [`RMW_FIXED_LOAN_COPY_MAX_BYTES`], the
/// shared-memory struct above it.
///
/// Total over `usize`, allocation-free, and dependent on nothing but its
/// argument: a legal mutation target.
pub fn fixed_loan_shape(fixed_size: usize) -> FixedLoanShape {
    if fixed_size <= RMW_FIXED_LOAN_COPY_MAX_BYTES {
        FixedLoanShape::PrivateCopy
    } else {
        FixedLoanShape::SharedMemory
    }
}

/// What one of a frame's VARIABLE entries says about
/// whether the frame can be decoded at all.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntryVerdict {
    /// The entry resolves inside the payload and, for a primitive sequence,
    /// holds a whole number of elements.
    Decodable,
    /// The offset-table entry does not resolve inside the payload — the
    /// frame is malformed and no member can be decoded from it.
    EntryOutOfBounds,
    /// The entry resolves but its length is not a whole number of elements:
    /// a truncated or mis-sized sequence.
    PartialElement,
    /// The entry resolves and holds whole elements, but their COUNT violates
    /// the member's declared bound — a fixed `T[N]` that is not exactly `N`,
    /// or a bounded `T[<=N]` that exceeds `N`.
    ///
    /// Decided by the BRIDGE, not by [`var_entry_decodable`]: the bound is a
    /// property of the type's member table rather than of the entry, and only
    /// the C++ decode enforces one (`write_prim_seq_cpp`). The C bridge's
    /// copy arm has no bound check, so there is nothing to pre-empt there and
    /// its walk never returns this.
    BoundViolated,
}

impl EntryVerdict {
    /// True only for [`EntryVerdict::Decodable`].
    pub fn is_decodable(self) -> bool {
        matches!(self, EntryVerdict::Decodable)
    }

    /// The `reason=` token the refusal line carries, so an operator learns
    /// WHICH way the frame is malformed and not merely that it is. The
    /// decodable case has no reason to report.
    pub fn as_str(self) -> &'static str {
        match self {
            EntryVerdict::Decodable => "decodable",
            EntryVerdict::EntryOutOfBounds => "entry_out_of_bounds",
            EntryVerdict::PartialElement => "partial_element",
            EntryVerdict::BoundViolated => "bound_violated",
        }
    }
}

/// The PURE per-entry verdict the adopted take runs over EVERY variable
/// member BEFORE it writes anything into the caller's message
/// (the take path's READ-ONLY pre-write entry gate).
///
/// `entry` is what `read_var_entry` resolved for the member — `None` when
/// the offset-table entry does not lie inside the payload. `elem_size` is
/// `Some` for a primitive sequence (whose wire length must be a whole
/// number of elements) and `None` for a string or a nested member, whose
/// body carries its own framing.
///
/// This is the ONE definition both bridges' pre-write walks apply, so the C
/// and C++ decode paths cannot drift apart on what "malformed" means — and
/// it is a legal mutation target (pure, total, no transport).
///
/// A zero `elem_size` is UNREACHABLE by construction — `plan_variable_op`
/// and its C++ twin are closed matches that emit only strides of 1, 2, 4 or
/// 8 — and it is refused rather than waved through, which is the fail-safe
/// direction: a zero-width element cannot hold a nonzero body, and two of
/// the four decode arms it would otherwise reach spell the same check as a
/// bare `%` that PANICS on zero. (Passing it through does not leave the
/// decision to the decode: the decode divides too, and
/// `ffi_guard` turns such a panic into `RMW_RET_ERROR`, not an abort. If a
/// COMPUTED stride is ever introduced, revisit those decode arms as well.)
pub fn var_entry_decodable(entry: Option<&[u8]>, elem_size: Option<usize>) -> EntryVerdict {
    let Some(bytes) = entry else {
        return EntryVerdict::EntryOutOfBounds;
    };
    match elem_size {
        Some(0) => EntryVerdict::PartialElement,
        Some(elem) if !bytes.len().is_multiple_of(elem) => EntryVerdict::PartialElement,
        _ => EntryVerdict::Decodable,
    }
}

/// Whether a loaned take is served by COPY rather than as a zero-copy loan.
///
/// `held_loans` is the number of zero-copy loans the subscription holds, each
/// pinning one SHM borrow; `ceiling` is how many borrows the subscription may
/// hold at once (`min(RMW_TAKE_LOAN_BORROW_BUDGET, the service's
/// max_borrowed_samples)`). One borrow is kept in reserve: the take that would
/// hold the last one receives into it, copies the message into private memory
/// and releases it at once, so a consumer that retains loans keeps receiving
/// and never reaches the transport's borrow refusal. Zero-copy loans held are
/// therefore at most `ceiling - 1`, and a ceiling of 1 serves every take by
/// copy.
///
/// Total over `usize x usize` (`held_loans + 1` saturates), allocation-free,
/// and dependent on nothing but its two arguments.
pub fn loaned_take_serves_copy(held_loans: usize, ceiling: usize) -> bool {
    held_loans.saturating_add(1) >= ceiling
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One row of the per-entry oracle: the resolved entry, the member's
    /// stride (`None` for a self-framed member), the verdict a reader can
    /// check by eye, and why.
    type EntryOracleRow<'a> = (Option<&'a [u8]>, Option<usize>, EntryVerdict, &'a str);

    /// The oracle is a hand-written TABLE, never a re-implementation of the
    /// comparison: every row states the verdict a reader can check by eye.
    /// It carries the shapes a sloppier gate gets wrong — the zero hash (a
    /// header field that was never stamped), a one-bit difference (the
    /// near-miss a tolerant comparison would wave through), the byte-swapped
    /// twin (an endianness slip), and the halves-equal pair (a gate that
    /// compared only the low or only the high word).
    #[test]
    fn the_hash_gate_matches_its_oracle_table() {
        let oracle: &[(u64, u64, HashVerdict, &str)] = &[
            (0, 0, HashVerdict::Match, "both unstamped: equal is equal"),
            (
                0x0123_4567_89ab_cdef,
                0x0123_4567_89ab_cdef,
                HashVerdict::Match,
                "the ordinary agreeing pair",
            ),
            (
                u64::MAX,
                u64::MAX,
                HashVerdict::Match,
                "saturated, still equal",
            ),
            (
                0,
                0x0123_4567_89ab_cdef,
                HashVerdict::Mismatch,
                "an unstamped frame against a real type",
            ),
            (
                0x0123_4567_89ab_cdef,
                0,
                HashVerdict::Mismatch,
                "a real frame against an unstamped expectation",
            ),
            (
                0x0123_4567_89ab_cdef,
                0x0123_4567_89ab_cdee,
                HashVerdict::Mismatch,
                "one bit apart is a skew, not a near miss",
            ),
            (
                0x0123_4567_89ab_cdef,
                0xefcd_ab89_6745_2301,
                HashVerdict::Mismatch,
                "the byte-swapped twin",
            ),
            (
                0x0000_0001_0000_0000,
                0x0000_0000_0000_0000,
                HashVerdict::Mismatch,
                "differs only in the HIGH word (a low-word-only gate passes this)",
            ),
            (
                0x0000_0000_0000_0001,
                0x0000_0000_0000_0000,
                HashVerdict::Mismatch,
                "differs only in the LOW word (a high-word-only gate passes this)",
            ),
        ];
        for &(frame, expected, want, why) in oracle {
            assert_eq!(
                schema_hash_verdict(frame, expected),
                want,
                "frame={frame:#018x} expected={expected:#018x}: {why}"
            );
            assert_eq!(
                schema_hash_verdict(frame, expected).is_mismatch(),
                want == HashVerdict::Mismatch,
                "is_mismatch must agree with the verdict for frame={frame:#018x} \
                 expected={expected:#018x}"
            );
        }
    }

    /// Anti-tautology: the table above would still pass a gate that always
    /// answered `Mismatch` if it held no `Match` row, and vice versa. Assert
    /// that BOTH verdicts really occur in it.
    #[test]
    fn the_oracle_table_exercises_both_verdicts() {
        assert_eq!(schema_hash_verdict(7, 7), HashVerdict::Match);
        assert_eq!(schema_hash_verdict(7, 8), HashVerdict::Mismatch);
        assert!(!schema_hash_verdict(7, 7).is_mismatch());
        assert!(schema_hash_verdict(7, 8).is_mismatch());
    }

    /// The per-entry verdict against a hand-written table.
    /// Every row is a shape a real offset table can carry.
    #[test]
    fn the_entry_verdict_matches_its_oracle_table() {
        let four = [0u8; 4];
        let six = [0u8; 6];
        let empty: [u8; 0] = [];
        let oracle: &[EntryOracleRow<'_>] = &[
            (
                None,
                None,
                EntryVerdict::EntryOutOfBounds,
                "an unresolvable string entry",
            ),
            (
                None,
                Some(4),
                EntryVerdict::EntryOutOfBounds,
                "an unresolvable sequence entry — out of bounds beats every other check",
            ),
            (
                Some(&empty),
                Some(4),
                EntryVerdict::Decodable,
                "an EMPTY sequence entry is decodable (0 is a whole number of elements)",
            ),
            (
                Some(&empty),
                None,
                EntryVerdict::Decodable,
                "an empty string",
            ),
            (
                Some(&four),
                Some(4),
                EntryVerdict::Decodable,
                "exactly one element",
            ),
            (
                Some(&six),
                Some(2),
                EntryVerdict::Decodable,
                "three elements",
            ),
            (
                Some(&six),
                Some(4),
                EntryVerdict::PartialElement,
                "six bytes cannot be a whole number of 4-byte elements",
            ),
            (
                Some(&four),
                Some(8),
                EntryVerdict::PartialElement,
                "a TRUNCATED element — shorter than one whole element",
            ),
            (
                Some(&six),
                None,
                EntryVerdict::Decodable,
                "a string of any length is decodable — its body carries its own framing",
            ),
            (
                Some(&six),
                Some(1),
                EntryVerdict::Decodable,
                "a byte sequence is always whole",
            ),
            (
                Some(&six),
                Some(0),
                EntryVerdict::PartialElement,
                "a zero element width is REFUSED (unreachable by construction; the decode's \
                 own modulo would panic on it, so waving it through is the unsafe direction)",
            ),
        ];
        for &(entry, elem, want, why) in oracle {
            assert_eq!(
                var_entry_decodable(entry, elem),
                want,
                "entry={:?} elem_size={elem:?}: {why}",
                entry.map(<[u8]>::len)
            );
            assert_eq!(
                var_entry_decodable(entry, elem).is_decodable(),
                want == EntryVerdict::Decodable,
                "is_decodable must agree with the verdict: {why}"
            );
        }
    }

    /// The fixed loan's shape against a table of sizes, each row placed
    /// relative to the limit so the table holds for any value of it: real
    /// messages far below it, the limit and its two neighbours (an off-by-one
    /// in the comparison moves one of them), and sizes far above it.
    #[test]
    fn the_fixed_loan_shape_matches_its_oracle_table() {
        let limit = RMW_FIXED_LOAN_COPY_MAX_BYTES;
        let oracle: &[(usize, FixedLoanShape, &str)] = &[
            (1, FixedLoanShape::PrivateCopy, "std_msgs/Empty's one byte"),
            (48, FixedLoanShape::PrivateCopy, "a geometry_msgs/Twist"),
            (
                limit - 1,
                FixedLoanShape::PrivateCopy,
                "one byte under the limit",
            ),
            (
                limit,
                FixedLoanShape::PrivateCopy,
                "exactly the limit: inclusive",
            ),
            (
                limit + 1,
                FixedLoanShape::SharedMemory,
                "one byte past the limit",
            ),
            (
                16 * 1024 * 1024,
                FixedLoanShape::SharedMemory,
                "a 16 MiB fixed message keeps its zero-copy take",
            ),
            (
                usize::MAX,
                FixedLoanShape::SharedMemory,
                "saturated: never wraps to a copy",
            ),
        ];
        for &(size, want, why) in oracle {
            assert_eq!(fixed_loan_shape(size), want, "fixed_size={size}: {why}");
        }
    }

    /// Anti-tautology for the table above: all three verdicts really occur,
    /// so a gate stuck on any one of them fails it.
    #[test]
    fn the_entry_oracle_exercises_every_verdict() {
        assert_eq!(
            var_entry_decodable(Some(&[0u8; 4]), Some(4)),
            EntryVerdict::Decodable
        );
        assert_eq!(
            var_entry_decodable(None, Some(4)),
            EntryVerdict::EntryOutOfBounds
        );
        assert_eq!(
            var_entry_decodable(Some(&[0u8; 5]), Some(4)),
            EntryVerdict::PartialElement
        );
    }

    /// The reserve rule against a hand-written table: the boundary sits one
    /// below the ceiling, a ceiling of 1 copies every take, and a held
    /// count at or past the ceiling (unreachable through the take, which never
    /// passes the reserve) still copies.
    #[test]
    fn the_loaned_take_copies_from_one_below_the_ceiling() {
        let oracle: &[(usize, usize, bool, &str)] = &[
            (0, 4, false, "nothing held: zero-copy"),
            (1, 4, false, "one held: zero-copy"),
            (2, 4, false, "two held: the third loan is zero-copy"),
            (3, 4, true, "three held: the fourth borrow stays in reserve"),
            (4, 4, true, "at the ceiling: copy"),
            (9, 4, true, "past the ceiling: copy"),
            (
                0,
                2,
                false,
                "an iceoryx2-default service: one zero-copy loan",
            ),
            (1, 2, true, "...and the second take copies"),
            (
                0,
                1,
                true,
                "a ceiling of 1 has no borrow to lend: every take copies",
            ),
            (usize::MAX, 4, true, "the increment saturates"),
            (
                usize::MAX - 1,
                usize::MAX,
                true,
                "one below a saturated ceiling",
            ),
            (
                usize::MAX - 2,
                usize::MAX,
                false,
                "two below a saturated ceiling",
            ),
        ];
        for &(held, ceiling, want, why) in oracle {
            assert_eq!(
                loaned_take_serves_copy(held, ceiling),
                want,
                "held={held} ceiling={ceiling}: {why}"
            );
        }
    }
}
