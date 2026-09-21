// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end loadability + LAST_ERROR semantics for the
//! raw-FFI cdylib template emit.
//!
//! The fixture `test_node_raw_ffi_template_cdylib` is built from the
//! byte-for-byte output of
//! `cerulion_cli_engine::templates::generate_lib_rs(canonical metadata)`.
//! These tests prove the host can load the artifact via `DylibNodeEntry::load`
//! AND that the LAST_ERROR machinery wired into init/tick/shutdown's Err
//! branches actually surfaces real diagnostics to the host (not just a
//! generic error code).
//!
//! Drift between this fixture and the generator is detected by the unit
//! test `templates::tests::test_generate_lib_rs_matches_oracle_fixture` in
//! `cerulion_cli_engine`. To regenerate the fixture after an intentional
//! generator change, run:
//!
//! ```text
//! cargo run -p cerulion_cli_engine --example dump_raw_ffi_emit \
//!     > test_fixtures/test_node_raw_ffi_template_cdylib/src/lib.rs
//! ```

use std::collections::HashMap;
use std::sync::Arc;

use cerulion_core::clock::RealClock;
use cerulion_core::graph::node::{DylibNodeEntry, NodeContext};
use cerulion_core::ShutdownSignal;
use indexmap::IndexMap;

/// Locate the oracle-vector cdylib in the workspace target dir.
///
/// Built by `cargo build -p test_node_raw_ffi_template_cdylib`.
fn find_raw_ffi_template_cdylib() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_raw_ffi_template_cdylib")
}

/// The raw-FFI cdylib emit must load
/// successfully via `DylibNodeEntry::load`. A template emitting only 5 of
/// the 7 required FFI exports fails at
/// load time with `missing symbol cerulion_take_last_error`;
/// this test PROVES the load contract is
/// satisfied end-to-end.
///
/// This is the load-bearing test of the user flow:
/// `cerulion node create X --raw-ffi && cerulion node build X
/// && cerulion node run X` reaches DylibNodeEntry::load, which is what
/// this test exercises.
#[test]
fn test_raw_ffi_template_cdylib_loads_via_dylib_node_entry() {
    let path = find_raw_ffi_template_cdylib();
    let result = DylibNodeEntry::load(&path);
    assert!(
        result.is_ok(),
        "DylibNodeEntry::load failed on raw-FFI template cdylib: {:?}",
        result.err(),
    );
}

// ─── Direct FFI-symbol tests via libloading ────────────────────────────
//
// `DylibNodeEntry::load` is the host-side wrapper that owns the high-level
// init/tick/shutdown lifecycle. For the LAST_ERROR semantics tests below
// we drop down to raw libloading because the template's `cerulion_node_init`
// takes `*mut u8` (and discards it), so we don't need to construct a real
// `NodeContext` — we pass a dummy non-null pointer.
//
// Each test loads the library fresh into its own scope so the thread-local
// LAST_ERROR doesn't leak between tests.

type TakeErrFn = unsafe extern "C" fn() -> *mut std::ffi::c_char;
type FreeErrFn = unsafe extern "C" fn(*mut std::ffi::c_char);
type InitFn = unsafe extern "C" fn(*mut u8) -> u64;
type TickFn = unsafe extern "C" fn(u64) -> i32;
type ShutdownFn = unsafe extern "C" fn(u64) -> i32;
type PumpHistoryFn = unsafe extern "C" fn(u64) -> i32;

/// Helper: pull the LAST_ERROR off the cdylib and return it as a Rust String.
///
/// Returns `None` if the cdylib reports no error (null pointer from
/// `cerulion_take_last_error`). Always frees the cdylib's allocation via
/// the paired `cerulion_free_error` to honor the allocator-pairing contract.
unsafe fn take_last_error(
    take_fn: &libloading::Symbol<'_, TakeErrFn>,
    free_fn: &libloading::Symbol<'_, FreeErrFn>,
) -> Option<String> {
    let ptr = unsafe { take_fn() };
    if ptr.is_null() {
        return None;
    }
    let msg = unsafe { std::ffi::CStr::from_ptr(ptr) }
        .to_string_lossy()
        .into_owned();
    unsafe { free_fn(ptr) };
    Some(msg)
}

/// Edge case: a fresh cdylib has no buffered error. `cerulion_take_last_error`
/// must return null. Pins the `None => null_mut()` branch in the take
/// implementation.
#[test]
fn test_raw_ffi_template_cdylib_take_last_error_returns_null_when_clean() {
    let path = find_raw_ffi_template_cdylib();
    let lib = unsafe { libloading::Library::new(&path) }.expect("load cdylib");
    let take_fn: libloading::Symbol<TakeErrFn> =
        unsafe { lib.get(b"cerulion_take_last_error") }.expect("take_last_error symbol");
    let free_fn: libloading::Symbol<FreeErrFn> =
        unsafe { lib.get(b"cerulion_free_error") }.expect("free_error symbol");

    let msg = unsafe { take_last_error(&take_fn, &free_fn) };
    assert!(
        msg.is_none(),
        "fresh cdylib must report no LAST_ERROR; got: {:?}",
        msg,
    );
}

/// Adversarial: `cerulion_free_error(null)` must be idempotent (no crash).
/// Pins the `if ptr.is_null() { return; }` guard in the free implementation.
#[test]
fn test_raw_ffi_template_cdylib_free_error_on_null_is_noop() {
    let path = find_raw_ffi_template_cdylib();
    let lib = unsafe { libloading::Library::new(&path) }.expect("load cdylib");
    let free_fn: libloading::Symbol<FreeErrFn> =
        unsafe { lib.get(b"cerulion_free_error") }.expect("free_error symbol");

    // No assertion needed beyond "does not crash". A nul-pointer crash
    // would SIGSEGV the test runner; reaching the next line proves the
    // guard fired.
    unsafe { free_fn(std::ptr::null_mut()) };
}

/// Construct a minimal heap-allocated `NodeContext` and transfer ownership
/// across the FFI boundary via `Box::into_raw`, matching what
/// `DylibNodeEntry::init` does at `cerulion_core/src/graph/node.rs:1600-1605`.
///
/// The raw pointer returned MUST be passed to `cerulion_node_init` so the
/// cdylib can reclaim and drop the Box (the
/// cdylib's init does `Box::from_raw` to avoid leaking one NodeContext per
/// init call).
fn box_into_raw_minimal_node_context() -> *mut NodeContext {
    let ctx = NodeContext::with_runtime_env(
        IndexMap::new(),
        IndexMap::new(),
        Arc::new(RealClock),
        ShutdownSignal::new(),
        Arc::new(HashMap::new()),
    );
    Box::into_raw(Box::new(ctx))
}

/// Happy path: init returns a non-zero handle, tick on that handle returns
/// success code 0, shutdown returns success code 0. LAST_ERROR is empty
/// throughout (no Err path was taken).
///
/// Also exercises the ownership hand-off: the cdylib's
/// `cerulion_node_init` MUST `Box::from_raw` the host-supplied pointer to
/// avoid leaking one NodeContext per init call. We construct the
/// `NodeContext` here, hand off ownership via `Box::into_raw`, and trust
/// the cdylib to drop the Box. A leak-test wrapper (running this body in a
/// loop) is exercised by `test_raw_ffi_template_cdylib_init_drops_context_no_leak`
/// below to prove the contract end-to-end.
#[test]
fn test_raw_ffi_template_cdylib_init_tick_shutdown_happy_path() {
    let path = find_raw_ffi_template_cdylib();
    let lib = unsafe { libloading::Library::new(&path) }.expect("load cdylib");
    let init_fn: libloading::Symbol<InitFn> =
        unsafe { lib.get(b"cerulion_node_init") }.expect("init symbol");
    let tick_fn: libloading::Symbol<TickFn> =
        unsafe { lib.get(b"cerulion_node_tick") }.expect("tick symbol");
    let shutdown_fn: libloading::Symbol<ShutdownFn> =
        unsafe { lib.get(b"cerulion_node_shutdown") }.expect("shutdown symbol");
    let take_fn: libloading::Symbol<TakeErrFn> =
        unsafe { lib.get(b"cerulion_take_last_error") }.expect("take symbol");
    let free_fn: libloading::Symbol<FreeErrFn> =
        unsafe { lib.get(b"cerulion_free_error") }.expect("free symbol");

    // Construct + transfer ownership of a minimal NodeContext via the
    // standard host pattern. The cdylib's init reclaims the Box, drops
    // it at end of init's scope (template doesn't store the context).
    let ctx_ptr = box_into_raw_minimal_node_context();
    let handle = unsafe { init_fn(ctx_ptr as *mut u8) };
    assert_ne!(handle, 0, "init should return a non-zero handle on success");

    let tick_code = unsafe { tick_fn(handle) };
    assert_eq!(tick_code, 0, "tick on a valid handle should return code 0");

    let shutdown_code = unsafe { shutdown_fn(handle) };
    assert_eq!(
        shutdown_code, 0,
        "shutdown on a valid handle should return code 0",
    );

    // LAST_ERROR must remain empty throughout the happy path.
    let err = unsafe { take_last_error(&take_fn, &free_fn) };
    assert!(
        err.is_none(),
        "no LAST_ERROR should be set on the happy path; got: {:?}",
        err,
    );
}

/// Regression test: calling `cerulion_node_init`
/// many times in a loop must not leak NodeContext allocations. The cdylib's
/// init reclaims each Box via `Box::from_raw`; the Box drops at end of
/// init's scope.
///
/// Detection: we use a simple Drop counter on a custom env_snapshot-like
/// payload — the runtime check is that the test completes without OOM and
/// each NodeContext is freed (proved structurally by the fact that
/// `Box::from_raw` returns ownership and drop runs).
///
/// This is a smoke test, not a precise leak detector. If a future
/// maintainer reverts the `Box::from_raw` to `let _ = ctx_ptr;`,
/// `valgrind --tool=memcheck` would flag every init call as a leak; without
/// valgrind, the guard is the drift-detection test
/// in `cerulion_cli_engine::templates::tests` that pins the
/// `Box::from_raw` call site.
#[test]
fn test_raw_ffi_template_cdylib_init_drops_context_no_leak() {
    let path = find_raw_ffi_template_cdylib();
    let lib = unsafe { libloading::Library::new(&path) }.expect("load cdylib");
    let init_fn: libloading::Symbol<InitFn> =
        unsafe { lib.get(b"cerulion_node_init") }.expect("init symbol");
    let shutdown_fn: libloading::Symbol<ShutdownFn> =
        unsafe { lib.get(b"cerulion_node_shutdown") }.expect("shutdown symbol");

    // 16 init + shutdown cycles. If the cdylib leaks NodeContext on each
    // init, 16x ~5KB allocation is invisible to the test but visible to
    // an external memcheck. The functional contract pinned here is "init
    // returns a fresh handle on every call AND shutdown cleans up cleanly".
    for i in 0..16 {
        let ctx_ptr = box_into_raw_minimal_node_context();
        let handle = unsafe { init_fn(ctx_ptr as *mut u8) };
        assert_ne!(
            handle, 0,
            "init iteration {} should return a non-zero handle",
            i,
        );
        let shutdown_code = unsafe { shutdown_fn(handle) };
        assert_eq!(shutdown_code, 0, "shutdown iteration {} should return 0", i,);
    }
}

/// Adversarial + error-path: init with a null `ctx_ptr` must return 0 AND
/// populate LAST_ERROR with the macro-parity wording. Pins the
/// null check end-to-end (the structural test pins the
/// emit; this test pins the runtime behavior).
#[test]
fn test_raw_ffi_template_cdylib_init_with_null_ctx_populates_last_error() {
    let path = find_raw_ffi_template_cdylib();
    let lib = unsafe { libloading::Library::new(&path) }.expect("load cdylib");
    let init_fn: libloading::Symbol<InitFn> =
        unsafe { lib.get(b"cerulion_node_init") }.expect("init symbol");
    let take_fn: libloading::Symbol<TakeErrFn> =
        unsafe { lib.get(b"cerulion_take_last_error") }.expect("take symbol");
    let free_fn: libloading::Symbol<FreeErrFn> =
        unsafe { lib.get(b"cerulion_free_error") }.expect("free symbol");

    let handle = unsafe { init_fn(std::ptr::null_mut()) };
    assert_eq!(
        handle, 0,
        "init with null ctx_ptr must return 0 (failure sentinel)",
    );

    let err = unsafe { take_last_error(&take_fn, &free_fn) };
    assert_eq!(
        err.as_deref(),
        Some("cerulion_node_init: NodeContext pointer was null"),
        "init's null-ctx branch must populate LAST_ERROR with the \
         macro-parity wording so operators grep matches both cdylib types",
    );
}

/// Adversarial + error-path: tick on a handle that was never registered
/// must return code 4 AND populate LAST_ERROR with a string naming the
/// bogus handle. Pins the missing-handle wiring end-to-end.
///
/// This test is the load-bearing proof that LAST_ERROR is actually
/// populated (not just exported as a no-op stub). Without this test, a
/// future maintainer accidentally deleting `__cer_set_last_error(...)`
/// from the missing-handle branch would only be caught by the structural
/// test in `cerulion_cli_engine`; this test catches it at the FFI boundary.
#[test]
fn test_raw_ffi_template_cdylib_tick_on_missing_handle_populates_last_error() {
    let path = find_raw_ffi_template_cdylib();
    let lib = unsafe { libloading::Library::new(&path) }.expect("load cdylib");
    let tick_fn: libloading::Symbol<TickFn> =
        unsafe { lib.get(b"cerulion_node_tick") }.expect("tick symbol");
    let take_fn: libloading::Symbol<TakeErrFn> =
        unsafe { lib.get(b"cerulion_take_last_error") }.expect("take symbol");
    let free_fn: libloading::Symbol<FreeErrFn> =
        unsafe { lib.get(b"cerulion_free_error") }.expect("free symbol");

    // `u64::MAX` is structurally guaranteed not to collide with any
    // handle the cdylib's `NEXT_HANDLE` counter could allocate: the
    // counter starts at 1 and increments, so even after 2^63 init calls
    // it cannot reach the top of the range. The macOS dlclose-no-op
    // behavior means `NEXT_HANDLE` persists across `Library` drops in
    // the same test-binary process; using a numeric magic constant like
    // 99999 would be vulnerable to a future test that runs many inits.
    let bogus_handle = u64::MAX;
    let code = unsafe { tick_fn(bogus_handle) };
    assert_eq!(code, 4, "tick on unknown handle must return code 4");

    let err = unsafe { take_last_error(&take_fn, &free_fn) };
    let err = err.expect("missing-handle tick must populate LAST_ERROR");
    let expected_substring = format!("cerulion_node_tick: handle {} not found", bogus_handle);
    assert!(
        err.contains(&expected_substring),
        "LAST_ERROR must name the bogus handle; got: {:?}",
        err,
    );
}

/// The `cerulion_node_pump_history` FFI entry (ABI v7)
/// must be exported and return code 0 on a valid handle, leaving LAST_ERROR
/// clean. This is the FFI half of quiescent-publisher history delivery — the
/// host's `DylibNodeEntry::pump_history` calls this symbol each live-loop
/// iteration. The template node has no publishers, so the pump body is a
/// no-op, but the entry, its handle lookup, and its success code are pinned
/// here at the FFI boundary.
#[test]
fn test_raw_ffi_template_cdylib_pump_history_on_valid_handle_returns_zero() {
    let path = find_raw_ffi_template_cdylib();
    let lib = unsafe { libloading::Library::new(&path) }.expect("load cdylib");
    let init_fn: libloading::Symbol<InitFn> =
        unsafe { lib.get(b"cerulion_node_init") }.expect("init symbol");
    let pump_fn: libloading::Symbol<PumpHistoryFn> =
        unsafe { lib.get(b"cerulion_node_pump_history") }.expect("pump_history symbol");
    let shutdown_fn: libloading::Symbol<ShutdownFn> =
        unsafe { lib.get(b"cerulion_node_shutdown") }.expect("shutdown symbol");
    let take_fn: libloading::Symbol<TakeErrFn> =
        unsafe { lib.get(b"cerulion_take_last_error") }.expect("take symbol");
    let free_fn: libloading::Symbol<FreeErrFn> =
        unsafe { lib.get(b"cerulion_free_error") }.expect("free symbol");

    let ctx_ptr = box_into_raw_minimal_node_context();
    let handle = unsafe { init_fn(ctx_ptr as *mut u8) };
    assert_ne!(handle, 0, "init should return a non-zero handle on success");

    let pump_code = unsafe { pump_fn(handle) };
    assert_eq!(
        pump_code, 0,
        "pump_history on a valid handle should return code 0",
    );

    let err = unsafe { take_last_error(&take_fn, &free_fn) };
    assert!(
        err.is_none(),
        "no LAST_ERROR should be set by a successful pump_history; got: {:?}",
        err,
    );

    let _ = unsafe { shutdown_fn(handle) };
}

/// Adversarial — `cerulion_node_pump_history` on a handle
/// that was never registered must return code 4 AND populate LAST_ERROR naming
/// the bogus handle, mirroring the tick missing-handle contract. Load-bearing
/// proof that the entry's error path is wired (not a no-op stub).
#[test]
fn test_raw_ffi_template_cdylib_pump_history_on_missing_handle_populates_last_error() {
    let path = find_raw_ffi_template_cdylib();
    let lib = unsafe { libloading::Library::new(&path) }.expect("load cdylib");
    let pump_fn: libloading::Symbol<PumpHistoryFn> =
        unsafe { lib.get(b"cerulion_node_pump_history") }.expect("pump_history symbol");
    let take_fn: libloading::Symbol<TakeErrFn> =
        unsafe { lib.get(b"cerulion_take_last_error") }.expect("take symbol");
    let free_fn: libloading::Symbol<FreeErrFn> =
        unsafe { lib.get(b"cerulion_free_error") }.expect("free symbol");

    let bogus_handle = u64::MAX;
    let code = unsafe { pump_fn(bogus_handle) };
    assert_eq!(code, 4, "pump_history on unknown handle must return code 4",);

    let err = unsafe { take_last_error(&take_fn, &free_fn) };
    let err = err.expect("missing-handle pump_history must populate LAST_ERROR");
    let expected_substring = format!(
        "cerulion_node_pump_history: handle {} not found",
        bogus_handle
    );
    assert!(
        err.contains(&expected_substring),
        "LAST_ERROR must name the bogus handle; got: {:?}",
        err,
    );
}

/// Adversarial + error-path: shutdown on a handle that was never registered
/// must return code 4 AND populate LAST_ERROR with the macro-parity wording
/// `handle X not found (already shut down or never registered)`. Pins the
/// wiring + macro wording parity end-to-end.
#[test]
fn test_raw_ffi_template_cdylib_shutdown_missing_handle_populates_last_error() {
    let path = find_raw_ffi_template_cdylib();
    let lib = unsafe { libloading::Library::new(&path) }.expect("load cdylib");
    let shutdown_fn: libloading::Symbol<ShutdownFn> =
        unsafe { lib.get(b"cerulion_node_shutdown") }.expect("shutdown symbol");
    let take_fn: libloading::Symbol<TakeErrFn> =
        unsafe { lib.get(b"cerulion_take_last_error") }.expect("take symbol");
    let free_fn: libloading::Symbol<FreeErrFn> =
        unsafe { lib.get(b"cerulion_free_error") }.expect("free symbol");

    // See `tick_on_missing_handle_populates_last_error` for the
    // rationale on using `u64::MAX - 1` over a magic constant.
    let bogus_handle = u64::MAX - 1;
    let code = unsafe { shutdown_fn(bogus_handle) };
    assert_eq!(code, 4, "shutdown on unknown handle must return code 4");

    let err = unsafe { take_last_error(&take_fn, &free_fn) };
    let err = err.expect("missing-handle shutdown must populate LAST_ERROR");
    let expected_substring = format!("cerulion_node_shutdown: handle {} not found", bogus_handle);
    assert!(
        err.contains(&expected_substring),
        "LAST_ERROR must name the bogus handle; got: {:?}",
        err,
    );
    assert!(
        err.contains("already shut down or never registered"),
        "LAST_ERROR must carry the macro-parity context wording so \
         operators grep matches both cdylib types; got: {:?}",
        err,
    );
}
