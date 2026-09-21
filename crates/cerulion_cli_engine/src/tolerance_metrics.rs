// SPDX-License-Identifier: AGPL-3.0-only
//! The tolerance METRIC MATH — the pure, frame-free
//! comparison of two decoded value sequences (or raw byte spans) under a
//! [`crate::tolerance::MetricKind`].
//!
//! Every function here is a PURE function of its inputs (no I/O, no clock, no
//! allocation beyond bounded scratch), so the same golden/candidate pair yields
//! the same verdict on every host — the determinism the replay engine leans on
//! (Principle #7). The engine ([`crate::replay_engine`]) decodes each nominated
//! field to an `f64` sequence (fixed scalars decode to a one-element sequence,
//! `float64[]` fields to their elements) and hands the two sequences here.
//!
//! # The [`MetricFrame`] result + O(1) streaming accumulation
//!
//! Each per-frame evaluation returns a [`MetricFrame`] carrying two numbers:
//!
//! - `value` — the metric's NATURAL, reportable magnitude for this frame (the
//!   max abs diff, the RMSE, the best-achievable minimum matched IoU
//!   (bottleneck), `1.0` for an unordered/set mismatch, …). Structurally-
//!   incomparable frames (a length mismatch, a non-finite input — `NaN` or `±inf` — on either side, including
//!   in a set/ordered input — a ragged bbox array, an unmatched/degenerate box)
//!   report
//!   the SENTINEL `f64::INFINITY` — documented as "worse than any finite
//!   divergence" (serde_json renders a non-finite `f64` as JSON `null`; a
//!   FINITE numeric violation — the common case — serializes exactly).
//! - `exceedance` — how far past the threshold this frame is: `> 0.0` ⇔ a
//!   violation; LARGER ⇔ worse. Oriented so BIGGER is always worse regardless of
//!   the metric's own comparison direction (an upper-bound metric's
//!   `value - threshold`; a lower-bound IoU floor's `min_iou - value`), so the
//!   engine keeps a single running WORST per `(topic, field)` by retaining the
//!   frame with the maximum `exceedance` — never buffering frames (the O(1)
//!   memory contract). `exceedance` is NEVER `NaN` (a structural failure is
//!   `f64::INFINITY`, ordering strictly above every finite value).
//!
//! Exact equality at the threshold PASSES (`exceedance == 0.0`, not `> 0.0`) —
//! e.g. a `max_abs` diff exactly equal to the bound, or an IoU exactly at the
//! floor.

use crate::tolerance::MetricKind;

/// `max_rel`'s denominator floor — the relative error of `x` vs `x_golden` is
/// `|x - x_golden| / max(|x_golden|, MAX_REL_EPS)`, so a golden value of exactly
/// `0.0` does not divide-by-zero (the candidate's absolute error then divides by
/// this epsilon). Documented, fixed, and pinned by an at-boundary test.
pub const MAX_REL_EPS: f64 = 1e-12;

/// Why a [`MetricFrame::value`] is non-finite — distinguishes a
/// structurally-incomparable frame from a finite-input NUMERIC OVERFLOW so the
/// violation report can word each accurately (both still serialize `value` as JSON
/// `null`, but the reason differs). Irrelevant when `value` is finite
/// ([`Self::Finite`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum NonFiniteCause {
    /// `value` is finite — the common case (no special wording).
    #[default]
    Finite,
    /// A structurally-incomparable frame: length mismatch, non-finite input
    /// (`NaN`/`±inf`) on either side,
    /// ragged bbox, unmatched/degenerate box, or a per-frame decode failure.
    Structural,
    /// FINITE inputs whose metric arithmetic overflowed to `±f64::INFINITY`
    /// (e.g. `max_abs` of `f64::MAX` vs `-f64::MAX`). A genuine numeric
    /// divergence, NOT a structural incomparability.
    Overflow,
}

/// One frame's evaluation of a metric. See the module docs for the `value` /
/// `exceedance` contract.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MetricFrame {
    /// The metric's natural, reportable magnitude (or `f64::INFINITY` sentinel
    /// for a structurally-incomparable frame / a finite-input numeric overflow).
    pub value: f64,
    /// `> 0.0` ⇔ violation; larger ⇔ worse. Oriented bigger-is-worse across
    /// every metric. Never `NaN`.
    pub exceedance: f64,
    /// Why `value` is non-finite (only meaningful when `!value.is_finite()`).
    pub cause: NonFiniteCause,
}

impl MetricFrame {
    /// A finite (or intentionally-sentinel) numeric frame that AUTO-CLASSIFIES
    /// its `cause`: if the computed `value` is non-finite it is a finite-input
    /// numeric [`NonFiniteCause::Overflow`] (the numeric metrics guard their
    /// inputs for `NaN`/length before reaching here, so a non-finite result is an
    /// arithmetic overflow); otherwise [`NonFiniteCause::Finite`]. Every numeric /
    /// discrete metric builds its frame through here.
    fn numeric(value: f64, exceedance: f64) -> Self {
        let cause = if value.is_finite() {
            NonFiniteCause::Finite
        } else {
            NonFiniteCause::Overflow
        };
        MetricFrame {
            value,
            exceedance,
            cause,
        }
    }

    /// A structurally-incomparable frame (length mismatch, non-finite input, ragged bbox,
    /// unmatched/degenerate box): a definite violation, worse than any finite
    /// divergence.
    fn structural_fail() -> Self {
        MetricFrame {
            value: f64::INFINITY,
            exceedance: f64::INFINITY,
            cause: NonFiniteCause::Structural,
        }
    }

    /// `true` iff this frame is a violation (`exceedance > 0.0`; exact-at-
    /// threshold is NOT a violation).
    pub fn is_violation(&self) -> bool {
        self.exceedance > 0.0
    }

    /// A structurally-incomparable frame — the public constructor the engine
    /// uses when a field's per-frame DECODE fails (a corrupt frame the
    /// bounds-checked decoder refused). Treated as a definite violation (worse
    /// than any finite divergence) so a decode failure can never false-PASS a
    /// tolerated field.
    pub fn incomparable() -> Self {
        Self::structural_fail()
    }
}

/// Dispatch a decoded golden/candidate `f64` sequence pair through the metric.
/// `bit_exact` never reaches here (the engine folds a `bit_exact` field into the
/// byte-exact remainder), but is handled defensively as "always matches" so a
/// stray call can never fabricate a violation.
pub fn eval_sequence_metric(metric: &MetricKind, golden: &[f64], candidate: &[f64]) -> MetricFrame {
    match metric {
        MetricKind::BitExact {} => MetricFrame::numeric(0.0, f64::NEG_INFINITY),
        MetricKind::MaxAbs { threshold } => max_abs(golden, candidate, *threshold),
        MetricKind::MaxRel { threshold } => max_rel(golden, candidate, *threshold),
        MetricKind::Rmse { threshold } => rmse(golden, candidate, *threshold),
        MetricKind::BboxIou { min_iou } => bbox_iou(golden, candidate, *min_iou),
        MetricKind::SetEqual {} => set_equal(golden, candidate),
        MetricKind::SetSubset {} => set_subset(golden, candidate),
        MetricKind::OrderedListEqual {} => ordered_list_equal(golden, candidate),
    }
}

/// EXACT 64-bit integer domain: dispatch a decoded golden/candidate
/// `i128` sequence pair through a metric on an `I64`/`U64` SCALAR field, computed
/// WITHOUT the `f64` round-trip that collapses two distinct values above `2^53`
/// (nanosecond timestamps!) into one — the false-PASS this path prevents. Only
/// `max_abs` and the set/ordered metrics reach here: `max_rel`/`rmse` are refused
/// at validation (exit 4) as precision-lossy on 64-bit integers, and `bbox_iou`
/// requires a `float64[]` array (never a 64-bit-int scalar). `bit_exact` never
/// reaches here (folded into the byte remainder). The unreachable arms are
/// handled defensively so a stray call can never fabricate a violation.
pub fn eval_int_metric(metric: &MetricKind, golden: &[i128], candidate: &[i128]) -> MetricFrame {
    match metric {
        MetricKind::MaxAbs { threshold } => max_abs_int(golden, candidate, *threshold),
        MetricKind::SetEqual {} => set_equal_int(golden, candidate),
        MetricKind::SetSubset {} => set_subset_int(golden, candidate),
        MetricKind::OrderedListEqual {} => ordered_list_equal_int(golden, candidate),
        MetricKind::BitExact {}
        | MetricKind::MaxRel { .. }
        | MetricKind::Rmse { .. }
        | MetricKind::BboxIou { .. } => MetricFrame::numeric(0.0, f64::NEG_INFINITY),
    }
}

/// `true` if any element on either side is non-finite (`NaN` OR `±inf`) — a
/// non-finite input is never within a numeric tolerance. The `±inf` half is
/// load-bearing against a FALSE PASS: a golden `+inf` vs a finite candidate
/// makes the `max_rel` per-element ratio `inf/inf = NaN`, and `f64::max`
/// silently discards `NaN`, folding `worst` to `0.0` — a byte-different frame
/// would pass. Gating INPUTS on finiteness (not just `NaN`) kills that class
/// for every numeric metric. Users with
/// legitimate `±inf` sentinels use `bit_exact` on that field.
fn has_non_finite(golden: &[f64], candidate: &[f64]) -> bool {
    golden
        .iter()
        .chain(candidate.iter())
        .any(|x| !x.is_finite())
}

/// Per-element absolute-difference bound: `value = max|golden[i] - candidate[i]|`,
/// `exceedance = value - threshold`. A LENGTH mismatch or a non-finite input on either side
/// is a structural failure (documented). An empty pair matches (value `0.0`).
pub fn max_abs(golden: &[f64], candidate: &[f64], threshold: f64) -> MetricFrame {
    if golden.len() != candidate.len() || has_non_finite(golden, candidate) {
        return MetricFrame::structural_fail();
    }
    let worst = golden
        .iter()
        .zip(candidate.iter())
        .map(|(g, c)| (g - c).abs())
        .fold(0.0f64, f64::max);
    // `worst` can OVERFLOW to +inf on finite-but-huge inputs (f64::MAX vs
    // -f64::MAX). `numeric()` classifies that as a numeric overflow (a real
    // divergence), NOT a structural incomparability.
    MetricFrame::numeric(worst, worst - threshold)
}

/// Per-element relative-difference bound: `value = max |g - c| / max(|g|,
/// MAX_REL_EPS)`, `exceedance = value - threshold`. Length mismatch / non-finite input =
/// structural failure.
pub fn max_rel(golden: &[f64], candidate: &[f64], threshold: f64) -> MetricFrame {
    if golden.len() != candidate.len() || has_non_finite(golden, candidate) {
        return MetricFrame::structural_fail();
    }
    let worst = golden
        .iter()
        .zip(candidate.iter())
        .map(|(g, c)| (g - c).abs() / g.abs().max(MAX_REL_EPS))
        .fold(0.0f64, f64::max);
    MetricFrame::numeric(worst, worst - threshold)
}

/// Root-mean-square-error bound over the sequence: `value = sqrt(mean((g-c)^2))`,
/// `exceedance = value - threshold`. Length mismatch / non-finite input = structural
/// failure. Empty pair → `value 0.0` (matches).
pub fn rmse(golden: &[f64], candidate: &[f64], threshold: f64) -> MetricFrame {
    if golden.len() != candidate.len() || has_non_finite(golden, candidate) {
        return MetricFrame::structural_fail();
    }
    if golden.is_empty() {
        return MetricFrame::numeric(0.0, -threshold);
    }
    let sum_sq: f64 = golden
        .iter()
        .zip(candidate.iter())
        .map(|(g, c)| {
            let d = g - c;
            d * d
        })
        .sum();
    let value = (sum_sq / golden.len() as f64).sqrt();
    // `sum_sq` can overflow to +inf on finite-but-huge inputs → `value`
    // is +inf; classified as a numeric overflow, not a structural failure.
    MetricFrame::numeric(value, value - threshold)
}

/// Bounding-box IoU floor over flattened `[x, y, w, h] * N` sequences.
///
/// Coordinates are SIGNED: `x`/`y` may be negative (an off-image crop like
/// `[-10, -10, 20, 20]` is accepted) — only `w`/`h` must be positive
/// (`degenerate()`). IoU is translation-invariant (intersection and union are
/// computed from absolute corner coordinates), so a shared negative offset on
/// both boxes leaves the IoU unchanged; the contract only requires positive
/// AREA, never non-negative position.
///
/// - A sequence whose length is not a multiple of 4, a non-finite value anywhere, a
///   DEGENERATE box (`w <= 0 || h <= 0`), or a differing box COUNT (⇒ an
///   unmatched box on one side) is a structural failure.
/// - Otherwise the verdict is a FLOOR-check FEASIBILITY question ("does a
///   pairing exist under which EVERY matched golden↔candidate box clears the
///   floor?"), NOT a max-total-IoU assignment. Build the bipartite graph of
///   `(golden, candidate)` pairs whose IoU `>= min_iou`; the frame PASSES iff a
///   PERFECT matching exists in that graph. A max-SUM (Hungarian) assignment is
///   the WRONG objective here — it can pick a high-total pairing whose WEAKEST
///   pair is below the floor even when a different, all-above-floor pairing
///   exists (e.g. IoU `[[0.9,0.6],[0.6,0.4]]` at floor `0.5`: max-sum picks the
///   diagonal, min `0.4` < floor, yet the anti-diagonal `{0.6,0.6}` clears it).
/// - `value` is the BOTTLENECK — the best achievable minimum matched IoU (the
///   MAX over all perfect matchings of that matching's MINIMUM pair). It is a
///   pure property of the IoU matrix (independent of index order), so the
///   verdict is permutation-invariant BY CONSTRUCTION — there is no tie-break to
///   make deterministic. `exceedance = min_iou - value`, so a violation
///   (`exceedance > 0`) is EXACTLY infeasibility (bottleneck below the floor),
///   and the reported `value` is actionable in both directions ("best
///   achievable min IoU = X" vs the floor).
pub fn bbox_iou(golden: &[f64], candidate: &[f64], min_iou: f64) -> MetricFrame {
    if !golden.len().is_multiple_of(4)
        || !candidate.len().is_multiple_of(4)
        || has_non_finite(golden, candidate)
    {
        return MetricFrame::structural_fail();
    }
    let gboxes = to_boxes(golden);
    let cboxes = to_boxes(candidate);
    // Differing counts ⇒ an unmatched box on one side ⇒ violation.
    if gboxes.len() != cboxes.len() {
        return MetricFrame::structural_fail();
    }
    // Degenerate box on either side ⇒ structural failure (documented).
    if gboxes.iter().chain(cboxes.iter()).any(|b| b.degenerate()) {
        return MetricFrame::structural_fail();
    }
    if gboxes.is_empty() {
        // No boxes on either side — vacuously matched (an empty detection frame
        // that replayed empty).
        return MetricFrame::numeric(1.0, min_iou - 1.0);
    }
    let n = gboxes.len();
    let mut iou = vec![vec![0.0f64; n]; n];
    for (i, g) in gboxes.iter().enumerate() {
        for (j, c) in cboxes.iter().enumerate() {
            iou[i][j] = g.iou(c);
        }
    }
    let bottleneck = matched_bottleneck(&iou);
    MetricFrame::numeric(bottleneck, min_iou - bottleneck)
}

/// The BOTTLENECK of the best perfect matching over an `n × n` IoU matrix
/// (`n >= 1`): the maximum, over every perfect golden↔candidate matching, of
/// that matching's MINIMUM pair IoU. Equivalently the largest threshold `t` at
/// which the bipartite graph of pairs with `iou >= t` still admits a perfect
/// matching.
///
/// `feasible(t)` (a perfect matching exists using only `>= t` edges) is
/// MONOTONE in `t` — lowering `t` only ADDS edges — and `feasible(min over all
/// iou)` is always true (every pair is then an edge ⇒ the complete bipartite
/// `K_{n,n}` always has a perfect matching). The optimal bottleneck is always
/// one of the matrix's own IoU values (the weakest edge of the optimal
/// matching), so a binary search over the sorted, deduped IoU values for the
/// largest feasible value is EXACT (float equality against the stored value is
/// safe — same bit pattern) and deterministic.
fn matched_bottleneck(iou: &[Vec<f64>]) -> f64 {
    // Sorted, deduped candidate thresholds (total order so a stray -0.0/NaN is
    // ordered, though NaN was already refused upstream).
    let mut uniq: Vec<f64> = iou.iter().flat_map(|r| r.iter().copied()).collect();
    uniq.sort_by(|a, b| a.total_cmp(b));
    uniq.dedup_by(|a, b| a.total_cmp(b).is_eq());
    // Binary search for the largest index whose value is still feasible. The
    // smallest value is always feasible (complete graph), so `best` starts at 0.
    let mut lo = 0usize;
    let mut hi = uniq.len() - 1;
    let mut best = 0usize;
    while lo <= hi {
        let mid = (lo + hi) / 2;
        if has_perfect_matching(iou, uniq[mid]) {
            best = mid;
            lo = mid + 1;
        } else if mid == 0 {
            break;
        } else {
            hi = mid - 1;
        }
    }
    uniq[best]
}

/// `true` iff the bipartite graph over the `n × n` `iou` matrix, keeping only
/// edges with `iou[i][j] >= t`, admits a PERFECT matching (every golden box
/// paired to a distinct candidate). A simple deterministic augmenting-path
/// (Kuhn) search: golden rows ascending, candidate columns ascending. `O(n^3)`;
/// `n` (box count per frame) is small. The EXISTENCE answer is a graph property,
/// so it is permutation-invariant regardless of the iteration order.
fn has_perfect_matching(iou: &[Vec<f64>], t: f64) -> bool {
    let n = iou.len();
    // match_col[j] = the golden row currently matched to candidate column j.
    let mut match_col = vec![usize::MAX; n];
    for i in 0..n {
        let mut seen = vec![false; n];
        if !augment(i, iou, t, &mut match_col, &mut seen) {
            return false;
        }
    }
    true
}

/// One augmenting-path step for [`has_perfect_matching`]: try to place golden
/// row `i` onto some `>= t` candidate column, displacing (recursively) an
/// existing occupant along an alternating path.
fn augment(i: usize, iou: &[Vec<f64>], t: f64, match_col: &mut [usize], seen: &mut [bool]) -> bool {
    for j in 0..iou.len() {
        if iou[i][j] >= t && !seen[j] {
            seen[j] = true;
            if match_col[j] == usize::MAX || augment(match_col[j], iou, t, match_col, seen) {
                match_col[j] = i;
                return true;
            }
        }
    }
    false
}

/// Order-INSENSITIVE set equality over decoded elements (duplicates DEDUPED via
/// total-order `f64::total_cmp`). `value = 1.0` on mismatch (`exceedance 1.0`),
/// `0.0` on equal.
///
/// `-0.0` is normalized to `+0.0` before compare/dedupe (consistent with
/// numeric `==`, under which `-0.0 == 0.0`), and any `NaN` on either side is a
/// STRUCTURAL failure (consistent with the numeric-metric `NaN` rule) — not the
/// legal self-equal element it once was.
pub fn set_equal(golden: &[f64], candidate: &[f64]) -> MetricFrame {
    if has_non_finite(golden, candidate) {
        return MetricFrame::structural_fail();
    }
    let g = dedup_sorted(golden);
    let c = dedup_sorted(candidate);
    // Compare via total_cmp on the -0.0-normalized, sorted-deduped sets.
    let equal = g.len() == c.len() && g.iter().zip(c.iter()).all(|(a, b)| a.total_cmp(b).is_eq());
    mismatch_frame(!equal)
}

/// The recorded (golden) set must be a SUBSET of the replayed (candidate) set
/// (duplicates deduped, total-order compare). `value = 1.0` when golden has an
/// element candidate lacks. `-0.0`→`+0.0` normalized; a `NaN` on either
/// side is a structural failure.
pub fn set_subset(golden: &[f64], candidate: &[f64]) -> MetricFrame {
    if has_non_finite(golden, candidate) {
        return MetricFrame::structural_fail();
    }
    let g = dedup_sorted(golden);
    let c = dedup_sorted(candidate);
    // g ⊆ c: every g element present in c. Both are total-order-sorted-deduped.
    let missing = g
        .iter()
        .any(|x| c.binary_search_by(|y| y.total_cmp(x)).is_err());
    mismatch_frame(missing)
}

/// Order-SENSITIVE, position-fixed sequence equality (element-wise). A length
/// mismatch is a mismatch. `value = 1.0` on any difference. `-0.0`→`+0.0`
/// normalized before compare; a `NaN` on either side is a STRUCTURAL failure.
pub fn ordered_list_equal(golden: &[f64], candidate: &[f64]) -> MetricFrame {
    if has_non_finite(golden, candidate) {
        return MetricFrame::structural_fail();
    }
    let equal = golden.len() == candidate.len()
        && golden
            .iter()
            .zip(candidate.iter())
            .all(|(g, c)| norm_zero(*g).total_cmp(&norm_zero(*c)).is_eq());
    mismatch_frame(!equal)
}

/// Byte-exact comparison of two raw field spans — the per-field `bit_exact`
/// metric (returned to the engine's byte-remainder path). `value = 1.0` on any
/// byte/length difference.
pub fn bytes_bit_exact(golden: &[u8], candidate: &[u8]) -> MetricFrame {
    mismatch_frame(golden != candidate)
}

/// The discrete-metric [`MetricFrame`]: `1.0`/`exceedance 1.0` on mismatch,
/// `0.0`/`exceedance 0.0` on match (threshold is the implicit `0.0`).
fn mismatch_frame(mismatch: bool) -> MetricFrame {
    if mismatch {
        MetricFrame::numeric(1.0, 1.0)
    } else {
        MetricFrame::numeric(0.0, 0.0)
    }
}

/// Total-order sort + dedup of a value slice (the set-metric canonicalization),
/// with `-0.0` normalized to `+0.0` first so the two zero encodings collapse to
/// one set element.
fn dedup_sorted(xs: &[f64]) -> Vec<f64> {
    let mut v: Vec<f64> = xs.iter().map(|&x| norm_zero(x)).collect();
    v.sort_by(|a, b| a.total_cmp(b));
    v.dedup_by(|a, b| a.total_cmp(b).is_eq());
    v
}

/// Normalize `-0.0` to `+0.0` (`-0.0 == 0.0` is `true`, so this maps both zero
/// encodings to the positive one) while leaving every other value — including
/// `NaN` — untouched. Used by the set/ordered metrics so the two zero bit
/// patterns compare EQUAL under `total_cmp` (consistent with numeric `==`).
fn norm_zero(x: f64) -> f64 {
    if x == 0.0 {
        0.0
    } else {
        x
    }
}

// ── Exact 64-bit integer-domain metrics ─────────────────────────────

/// EXACT-integer-domain absolute-difference bound for a 64-bit integer field.
/// The worst `|g - c|` is computed in `u128` (never `f64`), so a one-unit
/// difference at `~1.7e18` (nanosecond timestamps) is CAUGHT rather than
/// collapsing to `0.0` under `f64` widening. A LENGTH mismatch is a structural
/// failure; an empty pair matches. `threshold` is the validated non-negative
/// finite `max_abs` bound; the pass rule is EXACT — `violation ⇔ worst_diff >
/// floor(threshold)` (a fractional tolerance on an integer field can only be met
/// by a zero integer diff, so flooring is the exact integer bound). `value`
/// reports the worst diff as `f64` FOR THE REPORT ONLY — the VERDICT never
/// round-trips through `f64`.
pub fn max_abs_int(golden: &[i128], candidate: &[i128], threshold: f64) -> MetricFrame {
    if golden.len() != candidate.len() {
        return MetricFrame::structural_fail();
    }
    let worst: u128 = golden
        .iter()
        .zip(candidate.iter())
        .map(|(g, c)| (g - c).unsigned_abs())
        .max()
        .unwrap_or(0);
    // Exact non-negative integer bound (validation refused NaN/negative). A
    // threshold >= 2^128 saturates to u128::MAX ("everything passes") — never a
    // real tolerance, but keeps the cast total.
    let bound: u128 = if threshold >= u128::MAX as f64 {
        u128::MAX
    } else {
        threshold.floor() as u128
    };
    let violation = worst > bound;
    // Sign-correct exceedance: the integer gap is computed EXACTLY in u128 then
    // cast, so is_violation (exceedance > 0.0) is exact at the boundary — a small
    // gap casts to f64 losslessly, a large gap keeps its sign, and the exact
    // boundary (worst == bound) yields a non-positive exceedance (PASS).
    let exceedance = if violation {
        ((worst - bound) as f64).max(f64::MIN_POSITIVE)
    } else {
        -((bound - worst) as f64)
    };
    MetricFrame::numeric(worst as f64, exceedance)
}

/// Order-INSENSITIVE set equality in exact integer domain (sorted + deduped,
/// native `i128` compare — no `f64` round-trip, no `NaN`/`-0.0` concerns).
pub fn set_equal_int(golden: &[i128], candidate: &[i128]) -> MetricFrame {
    mismatch_frame(dedup_sorted_int(golden) != dedup_sorted_int(candidate))
}

/// The recorded set must be a SUBSET of the replayed set, exact integer domain.
pub fn set_subset_int(golden: &[i128], candidate: &[i128]) -> MetricFrame {
    let g = dedup_sorted_int(golden);
    let c = dedup_sorted_int(candidate);
    mismatch_frame(g.iter().any(|x| c.binary_search(x).is_err()))
}

/// Order-SENSITIVE, position-fixed integer sequence equality (exact `i128`).
pub fn ordered_list_equal_int(golden: &[i128], candidate: &[i128]) -> MetricFrame {
    mismatch_frame(golden != candidate)
}

/// Sort + dedup an `i128` slice (the integer set-metric canonicalization).
fn dedup_sorted_int(xs: &[i128]) -> Vec<i128> {
    let mut v = xs.to_vec();
    v.sort_unstable();
    v.dedup();
    v
}

/// An axis-aligned box `[x, y, w, h]`.
#[derive(Debug, Clone, Copy)]
struct BBox {
    x: f64,
    y: f64,
    w: f64,
    h: f64,
}

impl BBox {
    fn degenerate(&self) -> bool {
        self.w <= 0.0 || self.h <= 0.0
    }

    /// Standard intersection-over-union with `other` (both non-degenerate).
    fn iou(&self, other: &BBox) -> f64 {
        let ix1 = self.x.max(other.x);
        let iy1 = self.y.max(other.y);
        let ix2 = (self.x + self.w).min(other.x + other.w);
        let iy2 = (self.y + self.h).min(other.y + other.h);
        let iw = (ix2 - ix1).max(0.0);
        let ih = (iy2 - iy1).max(0.0);
        let inter = iw * ih;
        let union = self.w * self.h + other.w * other.h - inter;
        if union <= 0.0 {
            0.0
        } else {
            inter / union
        }
    }
}

/// Reshape a `len % 4 == 0` flat sequence into `[x, y, w, h]` boxes.
fn to_boxes(flat: &[f64]) -> Vec<BBox> {
    flat.as_chunks::<4>()
        .0
        .iter()
        .map(|c| BBox {
            x: c[0],
            y: c[1],
            w: c[2],
            h: c[3],
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── max_abs ────────────────────────────────────────────────────────────

    #[test]
    fn max_abs_within_and_over() {
        // worst diff 0.02 vs threshold 0.03 → pass; vs 0.01 → fail.
        let g = [1.0, 2.0, 3.0];
        let c = [1.0, 2.02, 3.0];
        let pass = max_abs(&g, &c, 0.03);
        assert!(!pass.is_violation());
        assert!((pass.value - 0.02).abs() < 1e-12);
        let fail = max_abs(&g, &c, 0.01);
        assert!(fail.is_violation());
        assert!((fail.value - 0.02).abs() < 1e-12);
    }

    #[test]
    fn max_abs_exact_at_threshold_passes() {
        // Boundary: worst diff EXACTLY equals the threshold → PASS.
        let f = max_abs(&[0.0], &[0.5], 0.5);
        assert_eq!(f.exceedance, 0.0);
        assert!(!f.is_violation());
    }

    #[test]
    fn max_abs_length_mismatch_is_structural() {
        let f = max_abs(&[1.0, 2.0], &[1.0], 100.0);
        assert!(f.is_violation());
        assert_eq!(f.value, f64::INFINITY);
        assert_eq!(f.exceedance, f64::INFINITY);
        // A length mismatch is a STRUCTURAL non-finite, not an overflow.
        assert_eq!(f.cause, NonFiniteCause::Structural);
    }

    #[test]
    fn max_abs_finite_input_overflow_is_classified_as_overflow_not_structural() {
        // `|f64::MAX - (-f64::MAX)|` overflows to +inf from FINITE inputs
        // (equal length, no NaN — never a structural failure). The non-finite
        // value must be tagged `Overflow` so the report words it as a numeric
        // overflow, not "structurally incomparable".
        let f = max_abs(&[f64::MAX], &[-f64::MAX], 1.0);
        assert!(f.is_violation());
        assert_eq!(f.value, f64::INFINITY);
        assert_eq!(f.cause, NonFiniteCause::Overflow);
        // A finite divergence keeps the `Finite` cause (anti-tautology control).
        let ok = max_abs(&[1.0], &[1.01], 0.001);
        assert!(ok.value.is_finite());
        assert_eq!(ok.cause, NonFiniteCause::Finite);
    }

    #[test]
    fn rmse_finite_input_overflow_is_classified_as_overflow() {
        // `sum_sq` of `(f64::MAX - (-f64::MAX))^2` overflows to +inf → `value`
        // is +inf, tagged Overflow (finite inputs, equal length).
        let f = rmse(&[f64::MAX], &[-f64::MAX], 1.0);
        assert!(f.is_violation());
        assert_eq!(f.value, f64::INFINITY);
        assert_eq!(f.cause, NonFiniteCause::Overflow);
    }

    #[test]
    fn max_abs_nan_either_side_is_violation() {
        assert!(max_abs(&[f64::NAN], &[0.0], 1e9).is_violation());
        assert!(max_abs(&[0.0], &[f64::NAN], 1e9).is_violation());
    }

    #[test]
    fn max_abs_empty_matches() {
        let f = max_abs(&[], &[], 0.0);
        assert!(!f.is_violation());
        assert_eq!(f.value, 0.0);
    }

    // ── max_rel ────────────────────────────────────────────────────────────

    #[test]
    fn max_rel_uses_golden_denominator() {
        // |10 - 10.1| / 10 = 0.01 → pass at 0.02, fail at 0.005.
        let g = [10.0];
        let c = [10.1];
        assert!(!max_rel(&g, &c, 0.02).is_violation());
        let f = max_rel(&g, &c, 0.005);
        assert!(f.is_violation());
        assert!((f.value - 0.01).abs() < 1e-9);
    }

    #[test]
    fn max_rel_zero_golden_uses_eps_denominator() {
        // golden 0.0, candidate 1e-13: |diff| / max(0, 1e-12) = 0.1 → within a
        // threshold of 1.0 (the eps floor prevents divide-by-zero blow-up).
        let f = max_rel(&[0.0], &[1e-13], 1.0);
        assert!(!f.is_violation());
        assert!((f.value - 0.1).abs() < 1e-9);
        // A candidate far above the eps-scaled tolerance still fails.
        assert!(max_rel(&[0.0], &[1.0], 1.0).is_violation());
    }

    #[test]
    fn max_rel_nan_is_violation() {
        assert!(max_rel(&[f64::NAN], &[1.0], 1e9).is_violation());
    }

    // ── rmse ───────────────────────────────────────────────────────────────

    #[test]
    fn max_rel_infinite_golden_vs_finite_candidate_is_violation_not_false_pass() {
        // The FALSE-PASS killer: golden +inf vs finite 1.0
        // makes the per-element ratio inf/inf = NaN, f64::max discards it, and
        // worst folds to 0.0 => PASS. Non-finite INPUT gating must make this
        // a structural violation instead.
        assert!(
            max_rel(&[f64::INFINITY], &[1.0], 1e9).is_violation(),
            "infinite golden must never pass max_rel"
        );
        assert!(
            max_rel(&[1.0], &[f64::NEG_INFINITY], 1e9).is_violation(),
            "infinite candidate must never pass max_rel"
        );
    }

    #[test]
    fn max_abs_and_rmse_infinite_inputs_are_structural_violations() {
        // Consistency: the non-finite input rule applies to every numeric
        // metric, both sides, both signs (inf-vs-inf included: |inf-inf| = NaN
        // is numerically undefined; bit_exact is the metric for inf sentinels).
        assert!(max_abs(&[f64::INFINITY], &[f64::INFINITY], f64::MAX).is_violation());
        assert!(max_abs(&[-1.0, f64::NEG_INFINITY], &[-1.0, -2.0], f64::MAX).is_violation());
        assert!(rmse(&[f64::INFINITY], &[f64::INFINITY], f64::MAX).is_violation());
        assert!(rmse(&[-1.0, f64::NEG_INFINITY], &[-1.0, -2.0], f64::MAX).is_violation());
    }

    #[test]
    fn rmse_hand_oracle() {
        // diffs [0, 0.2, 0]; mean sq = 0.04/3; sqrt ≈ 0.11547.
        let g = [1.0, 2.0, 3.0];
        let c = [1.0, 2.2, 3.0];
        let f = rmse(&g, &c, 0.2);
        assert!((f.value - (0.04f64 / 3.0).sqrt()).abs() < 1e-12);
        assert!(!f.is_violation()); // 0.1155 < 0.2
        assert!(rmse(&g, &c, 0.1).is_violation());
    }

    #[test]
    fn rmse_length_mismatch_and_nan_are_structural() {
        assert_eq!(rmse(&[1.0], &[1.0, 2.0], 1e9).value, f64::INFINITY);
        assert!(rmse(&[f64::NAN], &[0.0], 1e9).is_violation());
    }

    #[test]
    fn rmse_empty_vectors_pass_with_no_nan() {
        // Adversarial: an empty pair is a DEFINED pass — `sqrt(0/0)`
        // must never surface as NaN. `value` is exactly 0.0 (finite), so the
        // frame is not a violation and carries the `Finite` cause.
        let f = rmse(&[], &[], 0.5);
        assert!(!f.is_violation());
        assert_eq!(f.value, 0.0);
        assert!(f.value.is_finite(), "empty rmse must be finite, never NaN");
        assert_eq!(f.cause, NonFiniteCause::Finite);
        // A zero threshold still passes on empty input (exceedance is -0.0, not
        // > 0.0) — no divide-by-zero blow-up anywhere in the empty path.
        assert!(!rmse(&[], &[], 0.0).is_violation());
    }

    // ── bbox_iou ───────────────────────────────────────────────────────────

    #[test]
    fn bbox_identical_is_iou_one() {
        // One box, identical → IoU 1.0 ≥ floor 0.5 → pass.
        let b = [0.0, 0.0, 10.0, 10.0];
        let f = bbox_iou(&b, &b, 0.5);
        assert!(!f.is_violation());
        assert!((f.value - 1.0).abs() < 1e-12);
    }

    #[test]
    fn bbox_half_overlap_hand_oracle() {
        // golden [0,0,10,10]; candidate [5,0,10,10]: inter=5*10=50,
        // union=100+100-50=150, IoU=1/3. Floor 0.5 → fail; floor 0.3 → pass.
        let g = [0.0, 0.0, 10.0, 10.0];
        let c = [5.0, 0.0, 10.0, 10.0];
        let f = bbox_iou(&g, &c, 0.5);
        assert!((f.value - (1.0 / 3.0)).abs() < 1e-9);
        assert!(f.is_violation());
        assert!(!bbox_iou(&g, &c, 0.3).is_violation());
    }

    #[test]
    fn bbox_ragged_length_is_structural() {
        // 5 values → not a multiple of 4.
        let f = bbox_iou(&[0.0, 0.0, 1.0, 1.0, 9.0], &[0.0, 0.0, 1.0, 1.0], 0.1);
        assert_eq!(f.value, f64::INFINITY);
        assert!(f.is_violation());
    }

    #[test]
    fn bbox_degenerate_box_is_structural() {
        // Zero-width box.
        let f = bbox_iou(&[0.0, 0.0, 0.0, 5.0], &[0.0, 0.0, 5.0, 5.0], 0.1);
        assert!(f.is_violation());
        assert_eq!(f.value, f64::INFINITY);
    }

    #[test]
    fn bbox_count_mismatch_is_structural() {
        // Two golden boxes, one candidate → an unmatched box.
        let g = [0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 1.0, 1.0];
        let c = [0.0, 0.0, 1.0, 1.0];
        assert!(bbox_iou(&g, &c, 0.1).is_violation());
    }

    #[test]
    fn bbox_feasible_matching_is_permutation_stable() {
        // Two golden boxes at distinct locations; the candidate lists their
        // near-perfect twins in the SAME then SWAPPED order. The feasible
        // matching pairs each golden to its twin, so the bottleneck (best
        // achievable min IoU) — and the pass/fail verdict — is identical under
        // the permutation (existence of a perfect above-floor matching is a
        // graph property, so it is permutation-invariant by construction).
        let a = [0.0, 0.0, 10.0, 10.0];
        let b = [100.0, 100.0, 10.0, 10.0];
        // near-twins (shifted by 1 → high but <1 IoU)
        let a2 = [1.0, 0.0, 10.0, 10.0];
        let b2 = [101.0, 100.0, 10.0, 10.0];
        let golden: Vec<f64> = a.iter().chain(&b).copied().collect();
        let cand_same: Vec<f64> = a2.iter().chain(&b2).copied().collect();
        let cand_swap: Vec<f64> = b2.iter().chain(&a2).copied().collect();
        let f_same = bbox_iou(&golden, &cand_same, 0.5);
        let f_swap = bbox_iou(&golden, &cand_swap, 0.5);
        // Identical result regardless of candidate ordering.
        assert_eq!(f_same.value.to_bits(), f_swap.value.to_bits());
        assert_eq!(f_same.is_violation(), f_swap.is_violation());
        // Sanity: the optimal matching pairs twins (IoU high), NOT a to b
        // (IoU 0 — far apart), so the min IoU is well above 0.5 → pass.
        assert!(!f_same.is_violation());
        assert!(f_same.value > 0.5);
    }

    #[test]
    fn bbox_nan_is_structural() {
        assert!(bbox_iou(&[0.0, 0.0, f64::NAN, 1.0], &[0.0, 0.0, 1.0, 1.0], 0.1).is_violation());
    }

    #[test]
    fn bbox_feasibility_accepts_offdiagonal_matching_counterexample() {
        // Exact counterexample: IoU matrix [[0.9, 0.6], [0.6, 0.4]], floor
        // 0.5. A max-SUM (Hungarian) assignment picks the DIAGONAL (0.9 + 0.4 =
        // 1.3 > 0.6 + 0.6 = 1.2) whose weakest pair is 0.4 < 0.5 → it would
        // FALSE-FAIL. But an all-above-floor perfect matching EXISTS (the
        // anti-diagonal {0.6, 0.6}), so the FLOOR-check verdict is PASS. The
        // bottleneck (best achievable min IoU) is exactly 0.6.
        let iou = vec![vec![0.9, 0.6], vec![0.6, 0.4]];
        let b = matched_bottleneck(&iou);
        assert!((b - 0.6).abs() < 1e-12, "bottleneck {b} != 0.6");
        // The verdict (value - exceedance) the public path would build.
        let f = MetricFrame::numeric(b, 0.5 - b);
        assert!(!f.is_violation(), "an all-above-floor matching must PASS");
        // Raise the floor above the bottleneck → the matching is infeasible → FAIL,
        // and the reported best-achievable-min is still 0.6 (< 0.7).
        let fail = MetricFrame::numeric(b, 0.7 - b);
        assert!(fail.is_violation());
    }

    #[test]
    fn bbox_max_min_over_three_boxes_is_the_bottleneck() {
        // 3x3 IoU matrix. The identity matching's min pair is 0.2, but the cyclic
        // matching g0->c1 (0.8), g1->c2 (0.9), g2->c0 (0.85) has min 0.8 — and no
        // matching does better (raising the floor to 0.85 leaves g0 with no
        // >=0.85 edge). matched_bottleneck must return 0.8.
        let iou = vec![
            vec![0.2, 0.8, 0.75],
            vec![0.7, 0.2, 0.9],
            vec![0.85, 0.7, 0.2],
        ];
        let b = matched_bottleneck(&iou);
        assert!((b - 0.8).abs() < 1e-12, "bottleneck {b} != 0.8");
    }

    #[test]
    fn bbox_iou_feasibility_matching_scales_to_100_boxes() {
        // Adversarial SCALE pin — 100 boxes per side drive a 100×100
        // IoU matrix through the binary-search + augmenting-path bottleneck
        // matcher. Boxes are spaced 1000 apart (>> the 100-wide box), so golden
        // box i overlaps ONLY candidate box i — the unique perfect matching is
        // the identity, and the bottleneck is the uniform per-box IoU. The
        // candidate shifts every box by +5 in x: IoU = (100-5)/(100+5) = 95/105.
        // Deterministic construction; existence (fast completion) is the pin, no
        // wall clock.
        const N: usize = 100;
        let mut golden = Vec::with_capacity(N * 4);
        let mut candidate = Vec::with_capacity(N * 4);
        for i in 0..N {
            let base = (i as f64) * 1000.0;
            golden.extend_from_slice(&[base, 0.0, 100.0, 100.0]);
            candidate.extend_from_slice(&[base + 5.0, 0.0, 100.0, 100.0]);
        }
        let expect = 95.0 / 105.0;
        // Floor below the bottleneck → PASS, reporting the bottleneck value.
        let pass = bbox_iou(&golden, &candidate, 0.8);
        assert!(
            !pass.is_violation(),
            "bottleneck {} >= 0.8 must pass",
            pass.value
        );
        assert!(
            (pass.value - expect).abs() < 1e-12,
            "bottleneck {} != {expect}",
            pass.value
        );
        // Floor above the bottleneck → FAIL, still reporting the same bottleneck.
        let fail = bbox_iou(&golden, &candidate, 0.95);
        assert!(
            fail.is_violation(),
            "bottleneck {} < 0.95 must fail",
            fail.value
        );
        assert!((fail.value - expect).abs() < 1e-12);
    }

    #[test]
    fn bbox_iou_empty_both_sides_passes_with_vacuous_value_one() {
        // Adversarial: no boxes on either side (an empty detection frame
        // that replayed empty) is vacuously matched — `value` 1.0 (>= any floor),
        // NOT a violation, and the `Finite` cause (these are the defined semantics,
        // pinned here).
        let f = bbox_iou(&[], &[], 0.5);
        assert!(!f.is_violation());
        assert_eq!(f.value, 1.0);
        assert_eq!(f.cause, NonFiniteCause::Finite);
        // Even a floor of exactly 1.0 passes (exceedance 0.0 is not a violation).
        assert!(!bbox_iou(&[], &[], 1.0).is_violation());
    }

    #[test]
    fn bbox_iou_accepts_negative_coordinates_with_positive_area() {
        // Adversarial: signed coords — an off-image crop with negative
        // x/y but positive w/h is ACCEPTED (only w<=0||h<=0 is degenerate). Two
        // overlapping negative-coord boxes, hand-computed IoU:
        //   golden    [-10,-10,20,20] spans [-10,10] x [-10,10]
        //   candidate [ -5,-10,20,20] spans [ -5,15] x [-10,10]
        //   inter = 15 (x: -5..10) * 20 (y: -10..10) = 300
        //   union = 20*20 + 20*20 - 300 = 500 ; IoU = 300/500 = 0.6
        let golden = [-10.0, -10.0, 20.0, 20.0];
        let candidate = [-5.0, -10.0, 20.0, 20.0];
        let f = bbox_iou(&golden, &candidate, 0.5);
        assert_eq!(
            f.cause,
            NonFiniteCause::Finite,
            "negative coords are NOT a structural rejection"
        );
        assert!(
            (f.value - 0.6).abs() < 1e-12,
            "hand IoU 0.6, got {}",
            f.value
        );
        assert!(!f.is_violation(), "0.6 >= floor 0.5 must pass");
        // Raise the floor above 0.6 → fail, still reporting the 0.6 bottleneck.
        let fail = bbox_iou(&golden, &candidate, 0.7);
        assert!(fail.is_violation());
        assert!((fail.value - 0.6).abs() < 1e-12);
        // Translation invariance: shift BOTH boxes by a shared (-100,-100) and
        // the IoU is bit-identical (intersection/union use absolute corners).
        let g2 = [-110.0, -110.0, 20.0, 20.0];
        let c2 = [-105.0, -110.0, 20.0, 20.0];
        let f2 = bbox_iou(&g2, &c2, 0.5);
        assert_eq!(
            f2.value.to_bits(),
            f.value.to_bits(),
            "IoU must be translation-invariant under a shared negative offset"
        );
    }

    // ── set_equal / set_subset ─────────────────────────────────────────────

    #[test]
    fn set_equal_dedupes_and_ignores_order() {
        // {1,2,3} == {3,2,1,1,2} after dedupe.
        assert!(!set_equal(&[1.0, 2.0, 3.0], &[3.0, 2.0, 1.0, 1.0, 2.0]).is_violation());
        // {1,2} != {1,2,3}.
        assert!(set_equal(&[1.0, 2.0], &[1.0, 2.0, 3.0]).is_violation());
    }

    #[test]
    fn set_equal_nan_is_a_structural_violation() {
        // A NaN on either side of a SET metric is a STRUCTURAL failure
        // (consistent with the numeric-metric NaN rule) — never a legal
        // self-equal element.
        let f = set_equal(&[f64::NAN, 1.0], &[1.0, f64::NAN]);
        assert!(f.is_violation());
        assert_eq!(f.value, f64::INFINITY);
        assert!(set_subset(&[f64::NAN], &[f64::NAN, 1.0]).is_violation());
    }

    #[test]
    fn set_and_ordered_normalize_negative_zero() {
        // -0.0 normalizes to +0.0, so {-0.0} == {+0.0} and [-0.0] == [+0.0]
        // (consistent with numeric ==). A -0.0 dedupes against a +0.0 too.
        assert!(!set_equal(&[-0.0], &[0.0]).is_violation());
        assert!(!set_equal(&[-0.0, 0.0, 1.0], &[0.0, 1.0]).is_violation());
        assert!(!set_subset(&[-0.0], &[0.0, 1.0]).is_violation());
        assert!(!ordered_list_equal(&[-0.0, 1.0], &[0.0, 1.0]).is_violation());
    }

    #[test]
    fn set_subset_direction_is_recorded_subset_of_replayed() {
        // golden {1,2} ⊆ candidate {1,2,3} → pass.
        assert!(!set_subset(&[1.0, 2.0], &[1.0, 2.0, 3.0]).is_violation());
        // golden {1,2,4} ⊄ candidate {1,2,3} (4 missing) → fail.
        assert!(set_subset(&[1.0, 2.0, 4.0], &[1.0, 2.0, 3.0]).is_violation());
        // Superset is NOT a subset failure the other way: golden {1,2,3},
        // candidate {1,2} → 3 missing → fail (direction matters).
        assert!(set_subset(&[1.0, 2.0, 3.0], &[1.0, 2.0]).is_violation());
    }

    // ── ordered_list_equal ─────────────────────────────────────────────────

    #[test]
    fn ordered_list_equal_is_position_sensitive() {
        assert!(!ordered_list_equal(&[1.0, 2.0, 3.0], &[1.0, 2.0, 3.0]).is_violation());
        // Same set, different order → mismatch.
        assert!(ordered_list_equal(&[1.0, 2.0, 3.0], &[3.0, 2.0, 1.0]).is_violation());
        // Length mismatch → mismatch.
        assert!(ordered_list_equal(&[1.0, 2.0], &[1.0, 2.0, 3.0]).is_violation());
    }

    #[test]
    fn ordered_list_equal_nan_is_a_structural_violation() {
        // Even bit-identical NaNs are a structural violation for the
        // ordered metric (consistent with the numeric NaN rule).
        let f = ordered_list_equal(&[f64::NAN, 1.0], &[f64::NAN, 1.0]);
        assert!(f.is_violation());
        assert_eq!(f.value, f64::INFINITY);
    }

    // ── Exact 64-bit integer-domain metrics ─────────────────────────

    #[test]
    fn max_abs_int_catches_one_unit_diff_at_1e18_that_f64_would_lose() {
        // THE false-PASS killer: two u64 nanosecond timestamps differing by 1 at
        // ~1.7e18. Both collapse to the SAME f64 (2^53 precision ceiling), so an
        // f64 max_abs would see diff 0.0 and PASS at threshold 0. The integer
        // path computes |a-b| = 1 in u128 → violation at threshold 0.
        let a: i128 = 1_700_000_000_000_000_001;
        let b: i128 = 1_700_000_000_000_000_000;
        // Proof the f64 domain is genuinely blind here.
        assert_eq!(
            a as f64, b as f64,
            "f64 must collapse the pair (precondition)"
        );
        let f = max_abs_int(&[a], &[b], 0.0);
        assert!(f.is_violation(), "1-unit integer diff must be a violation");
        assert_eq!(f.value, 1.0, "reported worst diff is exactly 1");
        // A threshold of 1 (>= the diff) PASSES exactly at the boundary.
        assert!(!max_abs_int(&[a], &[b], 1.0).is_violation());
    }

    #[test]
    fn max_abs_int_boundary_and_length_and_empty() {
        // Exact-at-floor passes; over by one fails.
        assert!(!max_abs_int(&[10], &[7], 3.0).is_violation());
        assert!(max_abs_int(&[10], &[6], 3.0).is_violation());
        // A fractional threshold floors to the integer bound (2.9 -> 2).
        assert!(max_abs_int(&[10], &[7], 2.9).is_violation());
        assert!(!max_abs_int(&[10], &[8], 2.9).is_violation());
        // Length mismatch is structural; empty matches.
        assert_eq!(max_abs_int(&[1, 2], &[1], 1e9).value, f64::INFINITY);
        assert!(!max_abs_int(&[], &[], 0.0).is_violation());
    }

    #[test]
    fn int_set_and_ordered_exact_above_2e53() {
        // Two distinct u64 values above 2^53 that share an f64 image: the set +
        // ordered integer metrics must still tell them apart.
        let a: i128 = 9_007_199_254_740_993; // 2^53 + 1
        let b: i128 = 9_007_199_254_740_992; // 2^53
        assert_eq!(a as f64, b as f64, "precondition: f64-collapsed pair");
        assert!(set_equal_int(&[a], &[b]).is_violation());
        assert!(ordered_list_equal_int(&[a], &[b]).is_violation());
        // set semantics: order-insensitive, dedupe.
        assert!(!set_equal_int(&[a, b, a], &[b, a]).is_violation());
        assert!(!set_subset_int(&[a], &[a, b]).is_violation());
        assert!(set_subset_int(&[a, 5], &[a]).is_violation());
        // ordered is position-sensitive.
        assert!(ordered_list_equal_int(&[a, b], &[b, a]).is_violation());
    }

    #[test]
    fn eval_int_metric_routes_and_never_fabricates() {
        let g = [10i128];
        let c = [12i128];
        assert!(eval_int_metric(&MetricKind::MaxAbs { threshold: 1.0 }, &g, &c).is_violation());
        assert!(!eval_int_metric(&MetricKind::MaxAbs { threshold: 5.0 }, &g, &c).is_violation());
        // The unreachable arms (validation-refused / byte-folded) never fabricate.
        for m in [
            MetricKind::BitExact {},
            MetricKind::MaxRel { threshold: 0.0 },
            MetricKind::Rmse { threshold: 0.0 },
            MetricKind::BboxIou { min_iou: 0.5 },
        ] {
            assert!(!eval_int_metric(&m, &g, &c).is_violation());
        }
    }

    // ── bit_exact byte span + dispatch ─────────────────────────────────────

    #[test]
    fn bytes_bit_exact_detects_difference() {
        assert!(!bytes_bit_exact(&[1, 2, 3], &[1, 2, 3]).is_violation());
        assert!(bytes_bit_exact(&[1, 2, 3], &[1, 2, 4]).is_violation());
        assert!(bytes_bit_exact(&[1, 2], &[1, 2, 3]).is_violation());
    }

    #[test]
    fn dispatch_routes_each_kind() {
        let g = [1.0, 2.0];
        let c = [1.0, 2.5];
        assert!(
            eval_sequence_metric(&MetricKind::MaxAbs { threshold: 0.1 }, &g, &c).is_violation()
        );
        assert!(
            !eval_sequence_metric(&MetricKind::MaxAbs { threshold: 1.0 }, &g, &c).is_violation()
        );
        // bit_exact defensively never fabricates a violation here.
        assert!(!eval_sequence_metric(&MetricKind::BitExact {}, &g, &c).is_violation());
    }
}
