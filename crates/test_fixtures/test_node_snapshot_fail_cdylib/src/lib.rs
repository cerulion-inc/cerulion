// SPDX-License-Identifier: AGPL-3.0-only
//! Regression fixture: a raw-FFI cdylib whose `set_snapshot_inputs`
//! FFI call FAILS on demand.
//!
//! # What it pins
//!
//! When a cdylib's one-time `cerulion_node_set_snapshot_inputs` FFI call fails
//! (realistically a sibling cdylib tick poisoned the process-global `NODES`
//! mutex), the host (`DylibNodeEntry::snapshot_inputs`) must transition to the
//! terminal `SnapshotState::Failed` and STOP calling the per-step
//! `cerulion_node_snapshot_inputs` FFI — the node runs UNHELD, loudly, never
//! silently masked. A host that latches before `set` returns (or hardcodes `Active`)
//! would keep calling `snapshot` every step even after `set` failed, freezing
//! nothing yet returning 0, which masks the lost hold.
//! See `cerulion_core/src/graph/node.rs` (`SnapshotState::after_set` /
//! `should_invoke_snapshot`).
//!
//! # How the test observes it
//!
//! This fixture is COUNTER-based — no zero-copy I/O needed.
//! A process-global `SNAPSHOT_CALLS` counter increments on EVERY `cerulion_node_snapshot_inputs`
//! invocation. The fault is injected via `CER_FAIL_MODE == "snapshot_set"`:
//! `set_snapshot_inputs` then refuses (returns -1, sets LAST_ERROR, stores
//! nothing). The regression guard asserts the counter stays 0 in fault mode
//! (the host went `Failed` and never called `snapshot`); the control asserts it
//! goes > 0 when `set` succeeds (the host calls `snapshot` every firing step).
//!
//! The node declares ONE non-trigger `#[input]`-equivalent "inp" + a
//! `period_ms` policy in its info JSON, so the runtime's per-fire snapshot pass
//! calls `snapshot_inputs(["inp"])` on every step. This fixture does NOT freeze
//! a real input (it drops the `NodeContext` in init, like `test_node_cdylib`):
//! the counter, not a real hold, is what the test measures.
//!
//! Raw-FFI surface mirrors `test_node_cdylib` (the loader's symbol-presence +
//! ABI checks) plus the optional snapshot pair + two test accessors.

// P12 (Logging, see AGENTS.md): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Mutex;

static NEXT_HANDLE: AtomicU64 = AtomicU64::new(1);
static NODES: Mutex<Option<HashMap<u64, u64>>> = Mutex::new(None);

/// PROCESS-GLOBAL count of `cerulion_node_snapshot_inputs` FFI invocations.
///
/// Process-global (not per-handle) so the test can read it through its OWN
/// `libloading::Library` handle of the same .so path: a second `dlopen` of the
/// same object shares this static (the data segment is mapped once and
/// reference-counted), so the runtime's `DylibNodeEntry` increments and the
/// test's handle reads THE SAME counter. The two regression tests share this
/// static within one process and both reset it before stepping, so they isolate
/// regardless of run order.
static SNAPSHOT_CALLS: AtomicU64 = AtomicU64::new(0);

/// Null-terminated JSON info in `.rodata` — no allocation, no leak.
///
/// Declares ONE non-trigger input "inp" (bare-string legacy form — a cdylib
/// input's `schema_hash` is inert; `parse_info_json` forces it to 0) and a
/// `period_ms` policy. The `period_ms` policy is load-bearing: a Period node has
/// NO triggering inputs, so `build_snapshot_input_names` classifies "inp" as a
/// latest-value input to freeze, and the node fires every step (clock-driven) so
/// the per-fire snapshot pass runs every step.
static INFO_BYTES: &[u8] = b"{\"inputs\":[\"inp\"],\"outputs\":[],\"policy\":{\"period_ms\":10}}\0";

// Per-cdylib thread-local for the most recent FFI error message (mirrors the
// macro-generated surface + `test_node_cdylib`). Populated by the fault path so
// the host's `cerulion_take_last_error()` surfaces the forced-failure detail.
thread_local! {
    static LAST_ERROR: std::cell::RefCell<Option<std::ffi::CString>> =
        const { std::cell::RefCell::new(None) };
}

fn set_last_error(msg: String) {
    let cstring = std::ffi::CString::new(msg.replace('\0', "\\0"))
        .unwrap_or_else(|_| std::ffi::CString::new("error message contained nul byte").unwrap());
    LAST_ERROR.with(|cell| {
        *cell.borrow_mut() = Some(cstring);
    });
}

/// ABI version — tracks `cerulion_core::CERULION_ABI_VERSION` directly so a host
/// ABI bump needs no edit here (this fixture must stay loadable).
#[no_mangle]
pub extern "C" fn cerulion_abi_version() -> u32 {
    cerulion_core::CERULION_ABI_VERSION
}

/// ABI v22: tracks `cerulion_core::rustc_fingerprint_cstr()` directly, same
/// reasoning as `cerulion_abi_version` above.
#[no_mangle]
pub extern "C" fn cerulion_rustc_fingerprint() -> *const std::ffi::c_char {
    cerulion_core::rustc_fingerprint_cstr()
}

/// Take the most recent error message off the thread-local (null if none). The
/// host MUST pair every non-null return with `cerulion_free_error`.
#[no_mangle]
pub extern "C" fn cerulion_take_last_error() -> *mut std::ffi::c_char {
    LAST_ERROR.with(|cell| match cell.borrow_mut().take() {
        Some(cstr) => cstr.into_raw(),
        None => std::ptr::null_mut(),
    })
}

/// # Safety
///
/// `ptr` must be one previously returned by `cerulion_take_last_error` from this
/// same cdylib (CString allocator pairing); null is also allowed (no-op).
#[no_mangle]
pub unsafe extern "C" fn cerulion_free_error(ptr: *mut std::ffi::c_char) {
    if ptr.is_null() {
        return;
    }
    // SAFETY: per the function-level contract above; the pointer originated from
    // `CString::into_raw` in `cerulion_take_last_error`.
    let _ = std::ffi::CString::from_raw(ptr);
}

#[no_mangle]
pub extern "C" fn cerulion_node_info() -> *const std::ffi::c_char {
    INFO_BYTES.as_ptr() as *const std::ffi::c_char
}

/// Initialize a new node instance.
///
/// Reclaims the host's `Box::into_raw(NodeContext)` via `Box::from_raw` so each
/// init doesn't leak one NodeContext, then drops it: this fixture observes the
/// snapshot-CALL count, not a real input freeze, so it never needs the context.
/// Returns a u64 handle (0 = error).
#[no_mangle]
pub extern "C" fn cerulion_node_init(ctx_ptr: *mut u8) -> u64 {
    // SAFETY: `ctx_ptr` was produced by `Box::into_raw(Box::new(NodeContext))`
    // in the host's `DylibNodeEntry::init`. Reclaim via `Box::from_raw`; the Box
    // drops at end of this scope.
    let _ctx = unsafe { Box::from_raw(ctx_ptr as *mut cerulion_core::graph::node::NodeContext) };

    let handle = NEXT_HANDLE.fetch_add(1, Ordering::Relaxed);
    let mut guard = NODES.lock().unwrap();
    let map = guard.get_or_insert_with(HashMap::new);
    map.insert(handle, 0); // value unused (tick is a no-op); presence = "live handle"
    handle
}

#[no_mangle]
pub extern "C" fn cerulion_node_tick(handle: u64) -> i32 {
    // No-op tick (Period node — fires every step on the clock). Just validate
    // the handle so an unknown one returns the host's not-found code 4.
    let guard = NODES.lock().unwrap();
    match guard.as_ref().and_then(|m| m.get(&handle)) {
        Some(_) => 0,
        None => 4, // handle not found
    }
}

#[no_mangle]
pub extern "C" fn cerulion_node_pump_history(handle: u64) -> i32 {
    // No publishers → nothing to pump. Mirror tick's handle-lookup.
    let guard = NODES.lock().unwrap();
    match guard.as_ref().and_then(|m| m.get(&handle)) {
        Some(_) => 0,
        None => 4, // handle not found
    }
}

#[no_mangle]
pub extern "C" fn cerulion_node_shutdown(handle: u64) -> i32 {
    let mut guard = NODES.lock().unwrap();
    match guard.as_mut().and_then(|m| m.remove(&handle)) {
        Some(_) => 0,
        None => 4, // handle not found
    }
}

// ===========================================================================
// OPTIONAL snapshot FFI (the surface under test). Additive symbols that
// do NOT bump the ABI version. The host resolves both best-effort and requires
// BOTH (a one-symbol export disables the hold + warns).
// ===========================================================================

/// `cerulion_node_set_snapshot_inputs`: the host's ONE-TIME name registration.
///
/// FAULT INJECTION: when `CER_FAIL_MODE == "snapshot_set"`, refuse — set
/// LAST_ERROR and return -1 WITHOUT storing anything (modeling a sibling cdylib
/// tick having poisoned the process-global mutex). The host must then go
/// terminal `Failed` and stop calling `snapshot`. Otherwise validate the handle
/// and accept (return 0; -2 on an unknown handle, mirroring the macro ABI). The
/// names pointer is intentionally NOT dereferenced — this fixture counts calls,
/// it does not freeze a real input.
#[no_mangle]
pub extern "C" fn cerulion_node_set_snapshot_inputs(
    handle: u64,
    _names_ptr: *const u8,
    _names_len: usize,
) -> i32 {
    if std::env::var("CER_FAIL_MODE").as_deref() == Ok("snapshot_set") {
        set_last_error(String::from(
            "forced set failure (CER_FAIL_MODE=snapshot_set)",
        ));
        return -1;
    }
    let guard = NODES.lock().unwrap();
    match guard.as_ref().and_then(|m| m.get(&handle)) {
        Some(_) => 0,
        None => {
            set_last_error(format!(
                "cerulion_node_set_snapshot_inputs: handle {handle} not found"
            ));
            -2
        }
    }
}

/// `cerulion_node_snapshot_inputs`: the per-step freeze the host calls on every
/// firing step AFTER a successful `set`. This fixture just COUNTS it — the
/// regression guard asserts this count is 0 after a failed `set` (host went
/// `Failed`, never called this) and > 0 after a successful `set`.
#[no_mangle]
pub extern "C" fn cerulion_node_snapshot_inputs(handle: u64) -> i32 {
    let guard = NODES.lock().unwrap();
    if guard.as_ref().and_then(|m| m.get(&handle)).is_some() {
        SNAPSHOT_CALLS.fetch_add(1, Ordering::Relaxed);
        0
    } else {
        set_last_error(format!(
            "cerulion_node_snapshot_inputs: handle {handle} not found"
        ));
        -2
    }
}

// ===========================================================================
// Test accessors (read/reset the process-global snapshot-call counter).
// ===========================================================================

/// Read the process-global snapshot-call counter. `handle` is accepted for ABI
/// shape but ignored — the counter is process-global, shared across every
/// `dlopen` of this .so, so the test reads it through its own `libloading`
/// handle of the same path.
#[no_mangle]
pub extern "C" fn cerulion_test_get_snapshot_calls(_handle: u64) -> u64 {
    SNAPSHOT_CALLS.load(Ordering::Relaxed)
}

/// Reset the process-global snapshot-call counter to 0 (so the two regression
/// tests, which share this static within one process, isolate regardless of run
/// order).
#[no_mangle]
pub extern "C" fn cerulion_test_reset_snapshot_calls() {
    SNAPSHOT_CALLS.store(0, Ordering::Relaxed);
}
