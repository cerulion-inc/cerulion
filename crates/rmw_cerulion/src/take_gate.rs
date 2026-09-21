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
}
