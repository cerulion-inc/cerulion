// SPDX-License-Identifier: AGPL-3.0-only
//! Group 1+4: clock-source read matrix at the PUBLIC API
//! surface, and the thread-CPU-vs-wall-under-preemption comparison.
//!
//! The clock.rs in-module tests already pin the matrix via `super::*`; these
//! tests exercise the SAME contract through the public `cerulion_core`
//! re-exports (`Clock`, `RealClock`, `VirtualClock`, `ExternalClock`) — the
//! surface a node author / embedder actually consumes. They are deliberately
//! NOT duplicates of the in-module tests: a `pub use` regression that dropped
//! a re-export, or a trait-method-visibility change, would fail here while the
//! in-module `super::*` tests stayed green.
//!
//! The thread-CPU test is the strict counterpart of the loose smoke test in
//! `clock.rs::tests::test_thread_cpu_ns_positive_and_monotonic` (whose comment
//! explicitly leaves "the thread-CPU-vs-wall-under-preemption comparison"
//! uncovered). `#[serial]` because it sleeps + measures wall time and we want it
//! off the contended parallel CPU pool to keep the (already loose) bounds
//! stable on CI.

use cerulion_core::clock::{real_ns, thread_cpu_ns, Clock};
use cerulion_core::{ExternalClock, RealClock, VirtualClock};
use serial_test::serial;

/// Test 2 — RealClock: the active read is wall-ish (> 0, monotonic) and BOTH
/// source-specific reads are `None` (it is neither virtual nor external).
///
/// NOTE: the `virt_ns()` / `ext_ns()` calls here each consume the
/// once-per-process warn for `RealClock` in THIS test binary. That is
/// intentional and harmless: no test in this file asserts the warn count
/// (those live in `d2_sim_ns_warn_test.rs` / `ext_ns_warn_test.rs`, each its
/// own binary with a fresh `WARNED` static).
#[test]
fn real_clock_source_reads_are_none() {
    let clock = RealClock;
    let t = clock.now_ns();
    assert!(t > 0, "RealClock::now_ns() must be > 0 after boot");
    assert_eq!(
        clock.virt_ns(),
        None,
        "RealClock is not a virtual clock — virt_ns() must be None"
    );
    assert_eq!(
        clock.ext_ns(),
        None,
        "RealClock is not external — ext_ns() must be None"
    );
}

/// Test 2 — VirtualClock: `virt_ns() == Some(now_ns())`, `ext_ns() == None`,
/// and the controlled counter tracks `advance()`.
#[test]
fn virtual_clock_source_reads() {
    let clock = VirtualClock::new();
    assert_eq!(clock.now_ns(), 0, "VirtualClock starts at 0");
    assert_eq!(
        clock.virt_ns(),
        Some(0),
        "VirtualClock::virt_ns() must be Some(now) at start"
    );
    assert_eq!(
        clock.ext_ns(),
        None,
        "virtual time is not external time — ext_ns() must be None"
    );

    clock.advance(5_000);
    assert_eq!(clock.now_ns(), 5_000);
    assert_eq!(
        clock.virt_ns(),
        Some(5_000),
        "virt_ns() must track advance() and equal now_ns()"
    );
    assert_eq!(clock.ext_ns(), None, "ext_ns() stays None after advance");
}

/// Test 2 — ExternalClock after `set_external(t)`: `ext_ns() == Some(t)`,
/// `virt_ns() == None`, and `now_ns() == t` (the active read IS the latched
/// external master time).
#[test]
fn external_clock_source_reads() {
    let clock = ExternalClock::new();
    clock.set_external(123_456);
    assert_eq!(
        clock.now_ns(),
        123_456,
        "ExternalClock::now_ns() is the latched external time"
    );
    assert_eq!(
        clock.ext_ns(),
        Some(123_456),
        "ExternalClock IS the external source — ext_ns() == Some(latched)"
    );
    assert_eq!(
        clock.virt_ns(),
        None,
        "external time is not virtual time — virt_ns() must be None"
    );
}

/// Test 4 — thread-CPU time is MUCH less than wall time across a real sleep.
///
/// We sleep ~30 ms on THIS thread while bracketing the sleep with both
/// `thread_cpu_ns()` (on-CPU only) and `real_ns()` (wall, includes off-CPU
/// time). Because a sleeping thread is descheduled, its on-CPU delta must be a
/// tiny fraction of the wall delta. Bounds are deliberately loose to avoid CI
/// flake: we assert wall advanced by at least ~20 ms (the sleep happened) and
/// the CPU delta is well under half the wall delta. A correct implementation
/// burns essentially zero CPU while asleep, so this clears with huge margin; a
/// regression that made `thread_cpu_ns()` actually measure wall time (e.g.
/// reading CLOCK_MONOTONIC by mistake) would push the CPU delta up to ~wall
/// and FAIL.
#[test]
#[serial]
fn thread_cpu_ns_is_far_below_wall_under_sleep() {
    let cpu_before = thread_cpu_ns();
    let wall_before = real_ns();

    std::thread::sleep(std::time::Duration::from_millis(30));

    let cpu_after = thread_cpu_ns();
    let wall_after = real_ns();

    let cpu_delta = cpu_after.saturating_sub(cpu_before);
    let wall_delta = wall_after.saturating_sub(wall_before);

    // The sleep actually elapsed in wall time (loose lower bound: ~20ms).
    assert!(
        wall_delta >= 20_000_000,
        "wall delta across a 30ms sleep must be >= ~20ms, got {wall_delta} ns"
    );

    // The thread was off-CPU for the sleep, so its on-CPU delta is far below
    // wall. Loose bound: cpu_delta < wall_delta / 2.
    assert!(
        cpu_delta < wall_delta / 2,
        "thread-CPU delta ({cpu_delta} ns) must be much less than wall delta \
         ({wall_delta} ns) across a sleep — the thread was descheduled"
    );
}
