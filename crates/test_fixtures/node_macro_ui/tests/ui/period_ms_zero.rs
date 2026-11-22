use cerulion_core::prelude::*;

/// Macro diagnostic (D-family / policy bounds): `period_ms = 0` would
/// compile to `Duration::from_millis(0)` and let the scheduler spin-loop on a
/// zero-duration period — a silent runtime hang. The macro must reject it at
/// expansion time with the "`period_ms` must be > 0" diagnostic. (Companion of
/// the existing `tick_within_ms_zero` / `*_within_ms_zero` zero-bound cases.)
#[cerulion_node(period_ms = 0)]
struct PeriodZeroNode {
    #[output]
    out: u32,
}

fn main() {}
