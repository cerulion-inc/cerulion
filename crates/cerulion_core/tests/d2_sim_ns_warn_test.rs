// SPDX-License-Identifier: AGPL-3.0-only
//! Group 3 (D2): `RealClock::virt_ns` returns `None`
//! and emits a ONE-TIME `tracing::warn!`.
//!
//! `virt_ns()` is the "are we simulated? if so, what's the time?" probe on the
//! `Clock` trait. Under `RealClock` (the production default)
//! there IS no simulated time, so it must return `None`. To catch user code
//! that reads `virt_ns()` expecting a value, `RealClock::virt_ns` warns ONCE per
//! process (gated by a file-static `AtomicBool::swap`), then stays silent.
//!
//! This is a SINGLE `#[test]` in its OWN file so the once-per-process `WARNED`
//! static starts `false` for the test process. `#[traced_test]` (with the
//! crate's `no-env-filter` feature) captures the warn — without `no-env-filter`
//! the default `tracing-test` `EnvFilter` scopes to this test crate's
//! `module_path!()` and would silently drop the warn emitted from
//! `cerulion_core::clock`.

use cerulion_core::clock::Clock; // bring `virt_ns` into scope via the trait
use cerulion_core::RealClock;
use tracing_test::traced_test;

#[test]
#[traced_test]
fn real_clock_sim_ns_is_none_and_warns_exactly_once() {
    let clock = RealClock;

    // Contract: `virt_ns()` is `None` under RealClock.
    assert_eq!(
        clock.virt_ns(),
        None,
        "RealClock::virt_ns() must return None (no simulated time under RealClock)"
    );

    // The D2 once-per-process warn fired on that first call. The substring is
    // hand-pasted from the production message in `cerulion_core::clock`.
    assert!(
        logs_contain("virt_ns() returned None"),
        "the first RealClock::virt_ns() call must emit the once-per-process warn"
    );

    // A second call also returns None and does NOT panic.
    assert_eq!(
        clock.virt_ns(),
        None,
        "a second RealClock::virt_ns() call still returns None (and must not panic)"
    );

    // PIN the ONCE-ness across BOTH calls: the warn substring must appear
    // EXACTLY once. `logs_contain` above is presence-only (a boolean) and would
    // NOT catch a regression that drops the `AtomicBool::swap` guard and warns
    // on EVERY call (per-tick log spam — the exact thing D2 prevents). This
    // count is the load-bearing mutation pin: drop-the-swap → count == 2 → FAIL.
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains("virt_ns() returned None"))
            .count();
        if n == 1 {
            Ok(())
        } else {
            Err(format!(
                "expected the D2 warn EXACTLY once across two virt_ns() calls, got {n}"
            ))
        }
    });
}
