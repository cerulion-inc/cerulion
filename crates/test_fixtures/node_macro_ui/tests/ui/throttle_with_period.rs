use cerulion_core::prelude::*;

/// `throttle_ms` (producer rate cap) cannot be combined with
/// `period_ms` — period already pins the fire rate, so a cap is redundant
/// or silently conflicting. Must be rejected at macro-expansion (compile)
/// time. Use `throttle_ms` with a data/sync/external trigger, or
/// just lower `period_ms`.
#[cerulion_node(period_ms = 10, throttle_ms = 15)]
struct ThrottleAndPeriod {
    #[output]
    out: u32,
}

fn main() {}
