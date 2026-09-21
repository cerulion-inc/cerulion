// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! The OPERATOR half of the counter sentinel: `log_hook_counters`' shutdown line must
//! RENDER the counter-sentinel degrade — a kind the hook does not serve
//! (`u64::MAX`, the documented unknown-index sentinel an OLDER v2 hook
//! answers for kinds it predates) reads `unavailable`, NEVER
//! `18446744073709551615` dressed up as a real tombstone/atfork count.
//!
//! The data half (`hook_counters` reporting `None` per sentinel kind) is
//! pinned by `rmw_borrow_publish_test.rs`'s stale-v2 arm; that binary
//! installs no tracing subscriber, so the log line itself is not asserted there —
//! the rendering could regress to raw MAX with the array arm still green
//! (the vacuity this file closes). Its OWN binary because
//! `#[traced_test]` installs a GLOBAL tracing subscriber: any sibling test
//! that brings the rmw runtime up first takes that slot and the capture
//! then panics (`SetGlobalDefaultError`) — the exact shape and reason of
//! `rmw_schema_mismatch_test.rs` / `rmw_publish_reject_test.rs`.
//!
//! ⚠️ The fake hook install is process-global — run serial:
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_heaphook_counter_log_test -- --test-threads=1
//! ```

use std::os::raw::c_void;

use rmw_cerulion::heaphook::{
    hook_counters, log_hook_counters, HookApi, TestHookGuard, RC_ERR_NOT_ARMED, RC_OK,
};
use serial_test::serial;
// no-env-filter so the capture reaches the `rmw_cerulion::heaphook` target,
// not just this test crate.
use tracing_test::traced_test;

// =====================================================================
// A minimal fake hook: only `counter` is ever consulted by the counter
// surfaces under test; the window entries are inert stubs.
// =====================================================================

unsafe extern "C" fn f_disarm() -> i32 {
    RC_ERR_NOT_ARMED
}
unsafe extern "C" fn f_escape() -> i32 {
    RC_ERR_NOT_ARMED
}
unsafe extern "C" fn f_range(_p: *const c_void, _l: usize) -> i32 {
    RC_ERR_NOT_ARMED
}

/// A CURRENT hook's counter surface: every kind 0..=5 served.
unsafe extern "C" fn counter_full(kind: u32) -> u64 {
    match kind {
        0..=5 => 100 + kind as u64,
        _ => u64::MAX,
    }
}

/// A STALE v2 hook's counter surface: built before kinds 4/5 existed, so it
/// answers the documented unknown-index sentinel for them.
unsafe extern "C" fn counter_stale_v2(kind: u32) -> u64 {
    match kind {
        0..=3 => 100 + kind as u64,
        _ => u64::MAX,
    }
}

/// The direct oracle for `HookApi::inert()` — the base six fake-hook
/// literals spread from, which nothing else drives. Each entry is pinned to
/// the value the impl's docs promise: a window ARMS, nothing escapes it,
/// no address is adopted, and every status entry reports success. Without
/// this, changing (say) `window_range_test` to answer `1` would silently
/// shift six suites with no test naming the change.
#[test]
fn the_inert_hook_answers_successfully_and_adopts_nothing() {
    let api = HookApi::inert();
    // SAFETY: every entry is a Rust `extern "C"` item defined in
    // `HookApi::inert`; none dereferences its arguments, so null/zero
    // arguments are sound.
    unsafe {
        assert_eq!(
            (api.arm_window)(std::ptr::null_mut(), std::ptr::null_mut()),
            RC_OK,
            "a window arms"
        );
        assert_eq!((api.disarm_window)(), 0, "no latched escape");
        assert_eq!((api.window_escape)(), 0, "no latched escape");
        assert_eq!(
            (api.window_range_test)(std::ptr::null(), 0),
            0,
            "NOT adopted — a fake must never claim a range nothing registered"
        );
        assert_eq!((api.retire_slot)(std::ptr::null_mut()), RC_OK);
        assert_eq!((api.counter)(0), 0);
        assert_eq!((api.counter)(u32::MAX), 0);
        assert_eq!((api.register_segment)(std::ptr::null_mut(), 0, 0), RC_OK);
        assert_eq!((api.unregister_segment)(std::ptr::null_mut()), RC_OK);
        assert_eq!((api.set_release_callback)(None), RC_OK);
    }
}

/// The fake hook, parameterised on the ONE entry these arms vary. Every
/// entry this suite never reads spreads from `HookApi::inert()`'s
/// inert `RC_OK` stubs.
fn fake_hook(counter: unsafe extern "C" fn(u32) -> u64) -> HookApi {
    HookApi {
        disarm_window: f_disarm,
        window_escape: f_escape,
        window_range_test: f_range,
        counter,
        ..HookApi::inert()
    }
}

// =====================================================================
// Level-token + field-token helpers (the R1-hardened discipline from
// rmw_publish_reject_test.rs: tracing-test renders the SPAN NAME — the
// test fn's own name — into every line, so bare substring matches can
// invert; levels and key=value pairs match as whole whitespace tokens).
// =====================================================================

/// The shutdown line's message marker.
const MARKER: &str = "heap hook diagnostic counters at shutdown";

/// Whether `line` carries `level` as a whole whitespace token.
fn line_at_level(line: &str, level: &str) -> bool {
    line.split_whitespace().any(|tok| tok == level)
}

/// Whether `line` carries `key=value` as a whole whitespace token (a prose
/// or prefixed occurrence is not a token).
fn has_field(line: &str, field: &str) -> bool {
    line.split_whitespace().any(|tok| tok == field)
}

// The operator-half pin: a stale v2 hook's shutdown line renders the
// kinds it serves as numbers and the sentinel kinds as `unavailable` — and
// the raw sentinel value appears NOWHERE (the regression the array arm in
// rmw_borrow_publish_test.rs cannot see: `hook_counters` can keep reporting
// `None` while the log formats the raw u64). Reverting the
// `CounterField` Display to the raw u64 fails this test's `unavailable`
// fields + the no-raw-MAX sweep while the borrow test's array arm stays
// green — which is exactly why this pin exists.
#[test]
#[serial]
#[traced_test]
fn a_stale_hooks_shutdown_line_renders_unserved_kinds_as_unavailable() {
    let _hook = TestHookGuard::install(fake_hook(counter_stale_v2));
    // Precondition (the data half, pinned in full by the borrow test): the
    // sentinel kinds report None — so a raw MAX in the LOG below can only
    // come from the rendering layer, never the data.
    let got = hook_counters().expect("active fake hook");
    assert_eq!(got[4], ("tombstone_hits", None));
    assert_eq!(got[5], ("atfork_interval_leaks", None));

    log_hook_counters();

    logs_assert(|lines: &[&str]| {
        let shutdown: Vec<&&str> = lines.iter().filter(|l| l.contains(MARKER)).collect();
        if shutdown.len() != 1 {
            return Err(format!(
                "expected exactly 1 shutdown counter line, got {}",
                shutdown.len()
            ));
        }
        let line = shutdown[0];
        if !line_at_level(line, "INFO") {
            return Err(format!("the shutdown line must be INFO: {line}"));
        }
        for field in [
            "release_without_callback=100",
            "bootstrap_exhausted=101",
            "pre_resolution_leak=102",
            "quarantine_noop_frees=103",
            "tombstone_hits=unavailable",
            "atfork_interval_leaks=unavailable",
        ] {
            if !has_field(line, field) {
                return Err(format!("missing field token `{field}` in: {line}"));
            }
        }
        // The raw sentinel must appear NOWHERE — not on the shutdown line,
        // not anywhere else in the capture.
        if lines.iter().any(|l| l.contains("18446744073709551615")) {
            return Err("the raw u64::MAX sentinel leaked into the log".into());
        }
        Ok(())
    });
}

// Anti-tautology control: a CURRENT hook renders every kind as its number —
// no `unavailable` anywhere — so the stale arm's `unavailable` fields are
// genuinely the sentinel rendering, not this line's constant shape.
#[test]
#[serial]
#[traced_test]
fn a_current_hooks_shutdown_line_renders_every_kind_as_a_number() {
    let _hook = TestHookGuard::install(fake_hook(counter_full));
    log_hook_counters();

    logs_assert(|lines: &[&str]| {
        let shutdown: Vec<&&str> = lines.iter().filter(|l| l.contains(MARKER)).collect();
        if shutdown.len() != 1 {
            return Err(format!(
                "expected exactly 1 shutdown counter line, got {}",
                shutdown.len()
            ));
        }
        let line = shutdown[0];
        if !line_at_level(line, "INFO") {
            return Err(format!("the shutdown line must be INFO: {line}"));
        }
        for field in [
            "release_without_callback=100",
            "bootstrap_exhausted=101",
            "pre_resolution_leak=102",
            "quarantine_noop_frees=103",
            "tombstone_hits=104",
            "atfork_interval_leaks=105",
        ] {
            if !has_field(line, field) {
                return Err(format!("missing field token `{field}` in: {line}"));
            }
        }
        if lines.iter().any(|l| l.contains("unavailable")) {
            return Err("a fully-served hook must render no `unavailable`".into());
        }
        Ok(())
    });
}
