// SPDX-License-Identifier: AGPL-3.0-only
//! Sliding-window payload-size estimator for adaptive `loan_proxy` sizing
//! (the overflow-redirect path lives in the variable-field setters).
//!
//! # What this does
//!
//! Each `CerulionPublisher` holds an
//! `AdaptiveSizer`. On every successful `OutputProxy::Drop`, the proxy
//! reports the actual payload size to the sizer via `AdaptiveSizer::record`.
//! On the next `loan_proxy` call, `AdaptiveSizer::next_loan_size` returns
//! `min(recent_max × 1.5, max_slice_len).max(min_required)` instead of always
//! returning `max_slice_len`.
//!
//! # Where the win actually shows up
//!
//! - **`CerulionPublisher` (iceoryx2)**: the underlying SHM pool's
//!   per-slot reservation is FIXED at `initial_max_slice_len` (set at
//!   publisher creation; default `AllocationStrategy::Static` uses a
//!   `PoolAllocator` with bucket size = max). `loan_slice_uninit(N < max)`
//!   returns a length-N slice view over a full-size bucket. The
//!   user-visible SHM reservation is **unchanged** by the adaptive
//!   sizing on the iceoryx2 path. The smaller loan still benefits from
//!   correct provisional `total_size` in the wire header (used by
//!   subscribers to slice on actual data) and the sliding-window data
//!   feeds the overflow-redirect path. **Measured, not assumed:**
//!   making iceoryx2's reservation track the loan size
//!   (`AllocationStrategy::PowerOfTwo` / `BestFit`) is
//!   moot-for-RAM on Linux+macOS: `Static` pools are already
//!   lazy/demand-paged, so the full-size bucket reservation is
//!   virtual/sparse (resident tracks the working set, not the
//!   reservation), while `PowerOfTwo` measured slightly WORSE resident
//!   and carries a 256-realloc lifetime wall. The tier budgets are
//!   instead sized generously (free, since lazy) — see
//!   `variable_schema_max_slice_len`. `PowerOfTwo` grow-on-demand is
//!   retained only for a future Windows target (eager commit
//!   charge).
//!
//! # Convergence policy
//!
//! Until the ring buffer is fully populated (the first [`WINDOW_SIZE`] ticks),
//! the sizer returns `max_slice_len` because we don't have enough data to
//! safely shrink. Once warm, the loan size tracks `max(recent_N) × 1.5`.
//!
//! **Convergence on shrinkage works**: payload that grows tick-over-tick
//! by ≤ 1.5× fits in the prior tick's loan; each successful publish
//! records its size, the next loan grows, and the sequence completes.
//!
//! **Convergence on payload SPIKES is rescued by the overflow redirect.**
//! Without it, a spike from W (warmed-in) to S > W × 1.5 on a tick
//! where the current loan is sized at W × 1.5 would overflow the loan and
//! surface `ProxyBufferTooSmall` to the user; the Drop path would NOT
//! record the failed size, so subsequent ticks at size S would fail too.
//! The overflow redirect closes this: variable-field setters spill writes to a
//! lazily-allocated heap `Vec<u64>` (8-byte aligned for typed loans)
//! when `cursor + bytes_needed > self.len`. `OutputProxy::Drop`
//! detects the spill via `T::has_overflow`, re-loans a fresh sample
//! sized exactly to fit, memcpies the header + heap bytes into it, and
//! sends. The actual published size is recorded via `record_payload_size`
//! on overflow ticks too, so the next loan grows to absorb the spike.
//! `PayloadTooLarge` (when the spike exceeds `max_capacity`) and
//! `AllocationFailed` (when `Vec::try_reserve_exact` fails) are the
//! only remaining hard-fail paths.
//!
//! # Determinism
//!
//! The sliding window state is INTERNAL to the publisher — it never feeds
//! back into the wire frame. `OutputProxy::Drop` overwrites the
//! provisional `WireHeader::total_size` (set to `loan_size` at loan
//! time) with the actual `payload_wire_size` BEFORE send, so two
//! publishers with different sliding-window states (cold vs warm)
//! emit byte-equal frames for the same input data. Pinned by
//! determinism tests in `adaptive_sizing_test.rs`.
//!
//! # Why not f64 / floats
//!
//! All computation is in `u64`/`u32` with the headroom factor expressed as a
//! `* 3 / 2` (1.5×) integer rule. Floats would force the codegen to import
//! a math intrinsic into the cdylib path AND introduce determinism risk
//! across architectures with different rounding modes.

/// Number of ticks the sliding window remembers.
///
/// Sized to cover ~16 ticks worth of payload-size variance — enough to
/// smooth out short bursts without a warmup window so long that
/// short-lived publishers never benefit.
///
/// `pub` (with `#[doc(hidden)]`) for integration-test introspection
/// (`tests/adaptive_sizing_test.rs` imports it to compute "how many
/// publishes until warm"). Not a stability commitment — external
/// consumers should not depend on the value.
#[doc(hidden)]
pub const WINDOW_SIZE: usize = 16;

/// Compile-time guard: the `head` and `recorded_count` fields are
/// `u8`, so `WINDOW_SIZE` must fit in `u8` to avoid silent truncation
/// in `record()`'s `((self.head as usize + 1) % WINDOW_SIZE) as u8`.
const _: () = assert!(WINDOW_SIZE <= u8::MAX as usize);

/// Headroom multiplier numerator (`* 3 / 2` = 1.5×).
///
/// Applied to the window's max payload size to produce the next loan
/// size. Tighter than 2× (less SHM waste) but generous enough to absorb
/// small per-tick variance without hitting the overflow path on every
/// other tick. `#[doc(hidden)]` for the same reason as `WINDOW_SIZE`:
/// integration test introspection only.
#[doc(hidden)]
pub const HEADROOM_NUM: u64 = 3;
/// Headroom multiplier denominator (`* 3 / 2` = 1.5×).
///
/// Compile-time guard: must be non-zero so the `recent_max ×
/// HEADROOM_NUM / HEADROOM_DEN` arithmetic in `next_loan_size` never
/// hits a division-by-zero path. Caught at construction time rather
/// than panic at runtime.
#[doc(hidden)]
pub const HEADROOM_DEN: u64 = 2;
const _: () = assert!(HEADROOM_DEN > 0);

/// Tracks recent publish sizes and computes the next adaptive loan size.
///
/// Single-threaded — accessed only from the publisher's `&mut self`
/// methods (`loan_proxy` and the `OutputProxy::Drop` callback).
/// `Default` constructs an empty (cold) sizer that returns `max_slice_len`
/// from `next_loan_size` until [`WINDOW_SIZE`] samples have been recorded.
#[derive(Debug, Clone)]
pub(crate) struct AdaptiveSizer {
    /// Ring buffer of the last `WINDOW_SIZE` actual payload sizes (in
    /// bytes — header + fixed + offset table + variable payload).
    /// Slots before `recorded_count` reaches `WINDOW_SIZE` hold zeros
    /// from `Default`; the `next_loan_size` path doesn't read them
    /// until `warm()` is true.
    window: [u32; WINDOW_SIZE],
    /// Index of the next slot to overwrite. Wraps at `WINDOW_SIZE`.
    head: u8,
    /// Total number of `record()` calls. Saturates at `WINDOW_SIZE`
    /// (we only need to know "warm or not"). Reflecting the count
    /// rather than a `bool` makes per-test introspection trivial.
    recorded_count: u8,
}

impl Default for AdaptiveSizer {
    fn default() -> Self {
        Self {
            window: [0u32; WINDOW_SIZE],
            head: 0,
            recorded_count: 0,
        }
    }
}

impl AdaptiveSizer {
    /// Record an actual payload size from a successful publish.
    ///
    /// Called from `OutputProxy::Drop` after iceoryx2 `send()` /
    /// in-process channel fan-out succeeds. `size` is the wire frame's
    /// `total_size` (header + payload). Failed/aborted ticks are NOT
    /// recorded — the adaptive convergence is "learn from
    /// successes only"; the overflow-redirect path adds overflow-tick
    /// recording on top.
    ///
    /// # Precondition
    ///
    /// `size` must be at least `WireHeader::SIZE` — every wire frame
    /// carries the 32-byte header. Empty-payload pollution of the window
    /// is mitigated upstream: variable schemas with un-written fields
    /// trigger an early return in `OutputProxy::Drop` before reaching
    /// here, and fixed schemas always emit `WIRE_FIXED_SIZE + header`
    /// frames. The `debug_assert!` below pins the invariant for future
    /// callers; it never fires in production code paths today.
    pub(crate) fn record(&mut self, size: u32) {
        debug_assert!(
            (size as usize) >= crate::wire::WireHeader::SIZE,
            "AdaptiveSizer::record: size {} below WireHeader::SIZE {} \
             — caller bug; size must be the wire frame total_size",
            size,
            crate::wire::WireHeader::SIZE,
        );
        self.window[self.head as usize] = size;
        self.head = ((self.head as usize + 1) % WINDOW_SIZE) as u8;
        // Saturate at WINDOW_SIZE so the warm() check stays an `==`
        // comparison and `recorded_count` never overflows `u8` for very
        // long-lived publishers.
        if (self.recorded_count as usize) < WINDOW_SIZE {
            self.recorded_count = self.recorded_count.saturating_add(1);
        }
    }

    /// Returns the recommended next loan size, gated against
    /// `min_required` (lower bound) and `max_slice_len` (upper bound).
    ///
    /// Cold publishers (recorded_count < WINDOW_SIZE) fall through to
    /// `max_slice_len` because we don't have enough data to safely
    /// shrink. Warm publishers compute `max(window) × 3 / 2` and clamp
    /// to `[min_required, max_slice_len]`.
    ///
    /// `u32` in/out: matches the wire-format
    /// `WireHeader::total_size: u32` ceiling. Lossless on the call
    /// sites — publishers cast to `usize` only at the iceoryx2 /
    /// `vec![0u8; n]` allocation boundary.
    pub(crate) fn next_loan_size(&self, min_required: u32, max_slice_len: u32) -> u32 {
        if !self.warm() {
            return max_slice_len;
        }
        let recent_max = self.recent_max() as u64;
        // `HEADROOM_DEN` is asserted non-zero at the const site, so
        // direct `/` is sound; `saturating_mul` covers the multiplier
        // overflow case (recent_max ≈ u32::MAX × 3 fits in u64 by a
        // wide margin, but defensive).
        let with_headroom = recent_max.saturating_mul(HEADROOM_NUM) / HEADROOM_DEN;
        // Cap to u32 range before clamping. `max_slice_len` is the
        // hard ceiling; `min_required` is the lower bound from
        // `WireHeader::SIZE + WIRE_FIXED_SIZE + 8 * VARIABLE_FIELD_COUNT`.
        let candidate = u32::try_from(with_headroom).unwrap_or(u32::MAX);
        candidate.clamp(min_required, max_slice_len)
    }

    /// Returns `true` after the ring buffer has filled (the publisher
    /// has seen at least [`WINDOW_SIZE`] successful drops). Until then,
    /// `next_loan_size` returns `max_slice_len` and the sizer is
    /// "cold".
    pub(crate) fn warm(&self) -> bool {
        (self.recorded_count as usize) >= WINDOW_SIZE
    }

    /// Maximum size in the ring buffer. Used by `next_loan_size` to
    /// pick the headroom-multiplied estimate. Returns 0 on a cold
    /// sizer (the slot bytes are still zero from `Default`).
    pub(crate) fn recent_max(&self) -> u32 {
        let mut m = 0u32;
        for &s in &self.window {
            if s > m {
                m = s;
            }
        }
        m
    }

    /// Number of recorded payload sizes (saturated at `WINDOW_SIZE`).
    /// Test introspection only — the publisher uses `warm()` for the
    /// branching decision.
    #[cfg(test)]
    pub(crate) fn recorded_count(&self) -> usize {
        self.recorded_count as usize
    }
}

// ---------------------------------------------------------------------------
// This module's half of the ABI LAYOUT PIN (see `crate::abi_layout`).
//
// `abi_pin_struct!` expands to an exhaustive destructuring pattern with no `..`
// rest pattern, so adding or removing a field of one of these structs is a
// COMPILE ERROR naming the struct and the field; it also measures
// size/align/`offset_of!`, which `crate::abi_layout` compares against the
// snapshot table keyed to `CERULION_ABI_VERSION`. `abi_pin_enum!` does the
// same for a variant set (an enum carries no stable field offsets).
// ---------------------------------------------------------------------------
#[cfg(test)]
pub(crate) fn abi_layout_pins() -> Vec<crate::abi_layout::MeasuredStruct> {
    use crate::abi_layout::abi_pin_struct;
    vec![abi_pin_struct!(AdaptiveSizer {
        window,
        head,
        recorded_count
    })]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cold_sizer_returns_max_slice_len() {
        let sizer = AdaptiveSizer::default();
        assert!(!sizer.warm());
        assert_eq!(sizer.next_loan_size(64, 16 * 1024 * 1024), 16 * 1024 * 1024);
    }

    #[test]
    fn cold_sizer_recorded_count_starts_at_zero() {
        let sizer = AdaptiveSizer::default();
        assert_eq!(sizer.recorded_count(), 0);
    }

    #[test]
    fn warms_after_window_size_records() {
        let mut sizer = AdaptiveSizer::default();
        for _ in 0..WINDOW_SIZE {
            sizer.record(1024);
        }
        assert!(sizer.warm());
        assert_eq!(sizer.recorded_count(), WINDOW_SIZE);
    }

    #[test]
    fn does_not_warm_one_short() {
        let mut sizer = AdaptiveSizer::default();
        for _ in 0..(WINDOW_SIZE - 1) {
            sizer.record(1024);
        }
        assert!(!sizer.warm());
        // Cold path returns max_slice_len even after WINDOW_SIZE-1 records.
        assert_eq!(sizer.next_loan_size(64, 16 * 1024 * 1024), 16 * 1024 * 1024);
    }

    #[test]
    fn warm_constant_payload_returns_payload_times_headroom() {
        let mut sizer = AdaptiveSizer::default();
        for _ in 0..WINDOW_SIZE {
            sizer.record(1024);
        }
        // 1024 × 3 / 2 = 1536; clamp to [64, 16 MiB] = 1536.
        assert_eq!(sizer.next_loan_size(64, 16 * 1024 * 1024), 1536);
    }

    #[test]
    fn warm_loan_capped_by_max_slice_len() {
        let mut sizer = AdaptiveSizer::default();
        // Oversized recent payloads — the headroom multiplier would push
        // past max_slice_len. Cap to max.
        for _ in 0..WINDOW_SIZE {
            sizer.record(20 * 1024 * 1024);
        }
        assert_eq!(sizer.next_loan_size(64, 16 * 1024 * 1024), 16 * 1024 * 1024);
    }

    #[test]
    fn warm_loan_floor_at_min_required() {
        // Small recent payloads — the headroom multiplier would dip
        // below min_required (e.g. 32 bytes × 1.5 = 48, but min_required = 64).
        // `record()` requires size >= WireHeader::SIZE (32),
        // so the test uses the smallest valid value to exercise the
        // floor-clamp path.
        use crate::wire::WireHeader;
        let mut sizer = AdaptiveSizer::default();
        for _ in 0..WINDOW_SIZE {
            sizer.record(WireHeader::SIZE as u32);
        }
        // 32 × 3 / 2 = 48; clamps up to min_required = 64.
        assert_eq!(sizer.next_loan_size(64, 16 * 1024 * 1024), 64);
    }

    #[test]
    fn warm_loan_tracks_max_not_avg() {
        let mut sizer = AdaptiveSizer::default();
        // 15 small + 1 large — the "max" of the window is the large one.
        for _ in 0..(WINDOW_SIZE - 1) {
            sizer.record(100);
        }
        sizer.record(1000);
        // recent_max = 1000; * 1.5 = 1500.
        assert_eq!(sizer.next_loan_size(64, 16 * 1024 * 1024), 1500);
    }

    #[test]
    fn ring_buffer_wraps_after_window_size() {
        let mut sizer = AdaptiveSizer::default();
        // First WINDOW_SIZE records: all 1000.
        for _ in 0..WINDOW_SIZE {
            sizer.record(1000);
        }
        // Next WINDOW_SIZE records: all 100. The 1000s are overwritten.
        for _ in 0..WINDOW_SIZE {
            sizer.record(100);
        }
        // recent_max should be 100 (no 1000s left in the window).
        assert_eq!(sizer.recent_max(), 100);
        // Loan should track 100 × 1.5 = 150.
        assert_eq!(sizer.next_loan_size(64, 16 * 1024 * 1024), 150);
    }

    #[test]
    fn window_shrinks_back_after_spike() {
        // Ring buffer wrap-around test: a single spike followed by 16
        // small ticks must NOT keep the spike's size in the window.
        let mut sizer = AdaptiveSizer::default();
        // Warm with 100s.
        for _ in 0..WINDOW_SIZE {
            sizer.record(100);
        }
        // Spike — one record at 10000.
        sizer.record(10000);
        assert_eq!(sizer.recent_max(), 10000);
        // 15 more 100s — the spike still in the window.
        for _ in 0..(WINDOW_SIZE - 1) {
            sizer.record(100);
        }
        assert_eq!(sizer.recent_max(), 10000);
        // One more 100 — the spike overwritten, max drops to 100.
        sizer.record(100);
        assert_eq!(sizer.recent_max(), 100);
        assert_eq!(sizer.next_loan_size(64, 16 * 1024 * 1024), 150);
    }

    #[test]
    fn recorded_count_saturates_at_window_size() {
        let mut sizer = AdaptiveSizer::default();
        // Record 1000 times — saturated count must stay at WINDOW_SIZE
        // (no u8 overflow).
        for _ in 0..1000 {
            sizer.record(100);
        }
        assert_eq!(sizer.recorded_count(), WINDOW_SIZE);
    }

    #[cfg(debug_assertions)]
    #[test]
    #[should_panic(expected = "AdaptiveSizer::record: size 0 below WireHeader::SIZE")]
    fn record_below_wire_header_size_panics_in_debug() {
        // `record()` rejects sizes below WireHeader::SIZE
        // (32 bytes) in debug builds. In production this is unreachable
        // — every wire frame carries the 32-byte header by construction
        // (variable schemas early-return on un-written fields; fixed
        // schemas always emit `WireHeader::SIZE + WIRE_FIXED_SIZE`).
        // The `debug_assert!` pins the invariant for future callers.
        let mut sizer = AdaptiveSizer::default();
        sizer.record(0);
    }

    #[cfg(not(debug_assertions))]
    #[test]
    fn record_below_wire_header_size_silent_fallback_in_release() {
        // The release-mode counterpart: in release builds the
        // `debug_assert!` is compiled out, so `record(0)` silently sets
        // `window[head] = 0`. The window accumulates zeros, `recent_max`
        // returns 0, and `next_loan_size` falls through to `min_required`
        // (the floor clamp). This silent-fallback contract is pinned
        // here so a future change that narrows or removes the upstream
        // gate (`OutputProxy::Drop`'s `all_variables_written` check)
        // doesn't regress release behavior unnoticed. Together with
        // `record_below_wire_header_size_panics_in_debug` above, this
        // pins the below-header-size contract in both build modes (the
        // two tests are cfg-gated counterparts).
        let mut sizer = AdaptiveSizer::default();
        for _ in 0..WINDOW_SIZE {
            sizer.record(0);
        }
        assert_eq!(sizer.recent_max(), 0);
        assert_eq!(sizer.next_loan_size(40, 16 * 1024 * 1024), 40);
    }

    #[test]
    fn record_u32_max_does_not_overflow_u64_arithmetic() {
        // Adversarial: record `u32::MAX` which is ~4 GiB. The
        // headroom multiplication happens in u64, so it must not
        // overflow.
        let mut sizer = AdaptiveSizer::default();
        for _ in 0..WINDOW_SIZE {
            sizer.record(u32::MAX);
        }
        // Capped at max_slice_len.
        assert_eq!(sizer.next_loan_size(40, 16 * 1024 * 1024), 16 * 1024 * 1024);
    }

    #[test]
    fn min_required_equals_max_slice_len_returns_that_value() {
        // Edge case: tier-3 ceiling exactly matches the schema's
        // minimum (e.g., a fixed-only schema with WIRE_FIXED_SIZE +
        // header == max_slice_len).
        let mut sizer = AdaptiveSizer::default();
        for _ in 0..WINDOW_SIZE {
            sizer.record(40);
        }
        assert_eq!(sizer.next_loan_size(40, 40), 40);
    }

    #[test]
    fn next_loan_size_independent_calls_idempotent() {
        // `next_loan_size` is `&self` (not `&mut self`); calling it
        // 10× in a row returns the same value each time, no state
        // mutation.
        let mut sizer = AdaptiveSizer::default();
        for _ in 0..WINDOW_SIZE {
            sizer.record(1024);
        }
        let first = sizer.next_loan_size(64, 16 * 1024 * 1024);
        for _ in 0..10 {
            assert_eq!(sizer.next_loan_size(64, 16 * 1024 * 1024), first);
        }
    }

    // ============================================================
    // Bench: steady-state per-tick loan-size computation.
    //
    // This lives inline (not in a `tests/` bench binary) because the
    // two operations it measures — `AdaptiveSizer::next_loan_size` and
    // `AdaptiveSizer::record` — are `pub(crate)`, reachable only from
    // within the crate. It is the only bench that touches the sizer's
    // internal hot path directly. The two measurements a transport-level
    // bench would add — round-trip p50 at
    // 64 B / 1 MiB / 16 MiB and a receive-path no-alloc check over the
    // public transport API — are not made here: an `#[ignore]`d bench
    // inside a whole-file `cfg(not(debug_assertions))` is one
    // no job ever runs, and both measurements are made by things
    // that DO run (`benches/latency/` for the payload sweep,
    // `zero_alloc_test` for the receive-path allocation count).
    //
    // Pass criterion (per the issue): `< 10 ns` per tick for the
    // `next_loan_size` + `record` pair in steady state. This is a
    // human-run bench, not a CI gate — debug builds are ~10× slower and
    // CI VMs are noisy, so the body is `#[ignore]`d by default and the
    // `< 10 ns` assertion only fires in release (`--release`). A plain
    // `cargo test` neither runs it (ignored) nor compiles a misleading
    // debug-latency assertion. Run it with:
    //
    //   cargo test -p cerulion_core --lib --release \
    //     adaptive_sizer::tests::bench_adaptive_steady_state_loan_overhead \
    //     -- --ignored --nocapture
    // ============================================================

    /// Bench: steady-state loan-size computation overhead. Warms the
    /// sizer, then times `record` + `next_loan_size` over many
    /// iterations and reports ns/op. Asserts `< 10 ns/op` in release.
    #[test]
    #[ignore = "bench: run explicitly with --release --ignored --nocapture"]
    fn bench_adaptive_steady_state_loan_overhead() {
        use std::time::Instant;

        const WARMUP: usize = 1_000;
        const ITERS: usize = 5_000_000;
        const MIN_REQUIRED: u32 = 64;
        const MAX_SLICE_LEN: u32 = 16 * 1024 * 1024;
        // A representative steady-state payload size (~1 KiB frame).
        const STEADY_SIZE: u32 = 1024;

        let mut sizer = AdaptiveSizer::default();
        for _ in 0..WARMUP.max(WINDOW_SIZE) {
            sizer.record(STEADY_SIZE);
        }
        assert!(sizer.warm(), "sizer must be warm before timing");

        // Time the per-tick pair: record the just-published size, then
        // compute the next loan size. `black_box` defeats the optimizer
        // hoisting the loop-invariant computation out.
        let start = Instant::now();
        let mut acc = 0u32;
        for _ in 0..ITERS {
            sizer.record(std::hint::black_box(STEADY_SIZE));
            acc = acc.wrapping_add(sizer.next_loan_size(
                std::hint::black_box(MIN_REQUIRED),
                std::hint::black_box(MAX_SLICE_LEN),
            ));
        }
        std::hint::black_box(acc);
        let elapsed = start.elapsed();
        let ns_per_op = elapsed.as_nanos() as f64 / ITERS as f64;
        eprintln!(
            "bench_adaptive_steady_state_loan_overhead: {ns_per_op:.3} ns/op \
             over {ITERS} iters ({:?} total)",
            elapsed
        );

        // The `< 10 ns` pass criterion is a release-build claim; debug
        // builds are far slower and would false-fail. Gate the assertion
        // on release so a debug `--ignored` run still prints the number
        // without failing.
        #[cfg(not(debug_assertions))]
        assert!(
            ns_per_op < 10.0,
            "steady-state loan-size computation must be < 10 ns/op, got {ns_per_op:.3} ns/op"
        );
    }
}
