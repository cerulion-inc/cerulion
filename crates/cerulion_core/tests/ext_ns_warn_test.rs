// SPDX-License-Identifier: AGPL-3.0-only
//! Group 1: `RealClock::ext_ns` returns `None`
//! and emits a ONE-TIME `tracing::warn!`.
//!
//! `ext_ns()` is the "is an external time master connected? if so, what's the
//! time?" probe on the `Clock` trait. Under `RealClock` (the production
//! default) there is NO external master, so it must return `None`. To catch
//! user code that reads `ext_ns()` expecting a value, `RealClock::ext_ns`
//! warns ONCE per process (gated by a file-static `AtomicBool::swap`), then
//! stays silent.
//!
//! This is the EXTERNAL-source twin of `d2_sim_ns_warn_test.rs` (which pins
//! the `virt_ns()` warn). It lives in its OWN file so the once-per-process
//! `WARNED` static for `ext_ns` starts `false` for the test process — any
//! other test that called `RealClock::ext_ns()` first would consume the warn.
//! `#[traced_test]` (with the crate's `no-env-filter` feature) captures the
//! warn — without `no-env-filter` the default `tracing-test` `EnvFilter`
//! scopes to this test crate's `module_path!()` and would silently drop the
//! warn emitted from `cerulion_core::clock`.

use cerulion_core::clock::Clock; // bring `ext_ns` into scope via the trait
use cerulion_core::RealClock;
use tracing_test::traced_test;

#[test]
#[traced_test]
fn real_clock_ext_ns_is_none_and_warns_exactly_once() {
    let clock = RealClock;

    // Contract: `ext_ns()` is `None` under RealClock (no external master).
    assert_eq!(
        clock.ext_ns(),
        None,
        "RealClock::ext_ns() must return None (no external time master under RealClock)"
    );

    // The once-per-process warn fired on that first call. The substring is
    // hand-pasted from the production message in `cerulion_core::clock`.
    assert!(
        logs_contain("ext_ns() returned None"),
        "the first RealClock::ext_ns() call must emit the once-per-process warn"
    );

    // A second call also returns None and does NOT panic.
    assert_eq!(
        clock.ext_ns(),
        None,
        "a second RealClock::ext_ns() call still returns None (and must not panic)"
    );

    // PIN the ONCE-ness across BOTH calls: the warn substring must appear
    // EXACTLY once. `logs_contain` above is presence-only (a boolean) and
    // would NOT catch a regression that drops the `AtomicBool::swap` guard and
    // warns on EVERY call (per-tick log spam). This count is the load-bearing
    // mutation pin: drop-the-swap → count == 2 → FAIL.
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains("ext_ns() returned None"))
            .count();
        if n == 1 {
            Ok(())
        } else {
            Err(format!(
                "expected the ext_ns warn EXACTLY once across two ext_ns() calls, got {n}"
            ))
        }
    });
}
