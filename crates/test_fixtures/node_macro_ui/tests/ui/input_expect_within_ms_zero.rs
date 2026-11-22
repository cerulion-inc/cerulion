use cerulion_core::prelude::*;

/// Zero-value rejection: `#[input(expect_within_ms = 0)]`
/// must be rejected at compile time — a zero-duration window would miss on
/// every step. Mirror of the existing zero-value rejection for
/// `period_ms` / `sync_window_ms`.
#[cerulion_node(period_ms = 100)]
struct ZeroInputDeadline {
    #[input(expect_within_ms = 0)]
    image: u32,
}

fn main() {}
