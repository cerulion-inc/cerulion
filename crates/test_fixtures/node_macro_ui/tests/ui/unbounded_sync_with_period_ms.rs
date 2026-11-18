use cerulion_core::prelude::*;

/// `unbounded_sync` + `period_ms` — mutually exclusive (only one of
/// period_ms, sync_window_ms, unbounded_sync, or external
/// can be specified).
#[cerulion_node(period_ms = 100, unbounded_sync)]
struct PeriodAndUnboundedSync {
    #[input]
    a: u32,
}

fn main() {}
