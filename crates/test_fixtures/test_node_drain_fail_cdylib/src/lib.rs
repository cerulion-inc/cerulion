// SPDX-License-Identifier: AGPL-3.0-only
//! Drain-failure regression fixture: a raw-FFI cdylib whose
//! `cerulion_node_drain_trigger_input` FFI FAILS on demand.
//!
//! # What it pins
//!
//! The host (`DylibNodeEntry::drain_trigger_input`) must map EVERY nonzero
//! drain-FFI return to the safe `(0, None)` — a failed drain must NEVER
//! fabricate a fire signal (a phantom `popped > 0` would fire the node
//! against a queue that was not drained) — while logging loudly ONCE per
//! failure regime (the flood latch) and leaving the runtime alive (steps
//! keep executing; the producer keeps publishing).
//!
//! # How the test observes it
//!
//! COUNTER-based, the `test_node_snapshot_fail_cdylib` pattern: a
//! process-global `DRAIN_CALLS` counter increments on EVERY
//! `cerulion_node_drain_trigger_input` invocation (fault or not), readable
//! through the test's OWN `libloading` handle of the same object (a second
//! `dlopen` shares the data segment). The fault is injected via
//! `CER_FAIL_MODE == "drain_fail"`: the drain then refuses (returns -1, sets
//! LAST_ERROR, writes NO out-params). Without the fault it returns 0 with
//! `popped = 0` / `has_ts = 0` — this fixture never drains a real queue (it
//! drops its `NodeContext` at init); the COUNTER, not real data flow, is
//! what the tests measure, and `popped = 0` means the node never fires in
//! the control arm either (the anti-tautology: the counter proves the FFI
//! was invoked in BOTH arms).
//!
//! The info JSON declares ONE trigger input "inp" + a `data_trigger` policy,
//! so the runtime synthesizes a `DataTriggerBinding`; the drain symbol's
//! PRESENCE makes the host report `unifies_trigger_drain() == true` → the
//! binding is wired `DrainSource::Unified` and `drain_level` calls the drain
//! FFI once per step at the node's level.
//!
//! Raw-FFI surface mirrors `test_node_snapshot_fail_cdylib` (the loader's
//! symbol-presence + ABI checks) plus the drain export + two test
//! accessors.

// P12 (the repo's logging policy): library code never prints — it logs through
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

/// PROCESS-GLOBAL count of `cerulion_node_drain_trigger_input` FFI
/// invocations (fault and non-fault alike).
///
/// Process-global (not per-handle) so the test reads it through its own
/// `libloading::Library` handle of the same .so path: the data segment is
/// mapped once and reference-counted, so the runtime's `DylibNodeEntry`
/// increments and the test's handle reads THE SAME counter. Tests reset it
/// before stepping, so they isolate regardless of run order.
static DRAIN_CALLS: AtomicU64 = AtomicU64::new(0);

/// Null-terminated JSON info in `.rodata` — no allocation, no leak.
///
/// ONE trigger input "inp" + the `data_trigger` policy: the runtime
/// synthesizes the `DataTriggerBinding` this fixture's drain FFI is wired
/// into. (The host synthesizes cdylib `input_meta` as
/// `DropOldest`, so eligibility reduces to the drain symbol's presence.)
static INFO_BYTES: &[u8] =
    b"{\"inputs\":[\"inp\"],\"outputs\":[],\"policy\":{\"data_trigger\":{\"input_name\":\"inp\"}}}\0";

// Per-cdylib thread-local for the most recent FFI error message (mirrors the
// macro-generated surface). Populated by the fault path so the host's
// `cerulion_take_last_error()` surfaces the forced-failure detail.
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

/// ABI version — tracks `cerulion_core::CERULION_ABI_VERSION` directly so a
/// host ABI bump needs no edit here (this fixture must stay loadable).
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

/// Take the most recent error message off the thread-local (null if none).
/// The host MUST pair every non-null return with `cerulion_free_error`.
#[no_mangle]
pub extern "C" fn cerulion_take_last_error() -> *mut std::ffi::c_char {
    LAST_ERROR.with(|cell| match cell.borrow_mut().take() {
        Some(cstr) => cstr.into_raw(),
        None => std::ptr::null_mut(),
    })
}

/// # Safety
///
/// `ptr` must be one previously returned by `cerulion_take_last_error` from
/// this same cdylib (CString allocator pairing); null is allowed (no-op).
#[no_mangle]
pub unsafe extern "C" fn cerulion_free_error(ptr: *mut std::ffi::c_char) {
    if ptr.is_null() {
        return;
    }
    // SAFETY: per the function-level contract above; the pointer originated
    // from `CString::into_raw` in `cerulion_take_last_error`.
    let _ = std::ffi::CString::from_raw(ptr);
}

#[no_mangle]
pub extern "C" fn cerulion_node_info() -> *const std::ffi::c_char {
    INFO_BYTES.as_ptr() as *const std::ffi::c_char
}

/// Initialize a new node instance.
///
/// Reclaims the host's `Box::into_raw(NodeContext)` via `Box::from_raw` so
/// each init doesn't leak one NodeContext, then drops it: this fixture
/// observes the drain-CALL count, not real data flow, so it never needs the
/// context. Returns a u64 handle (0 = error).
#[no_mangle]
pub extern "C" fn cerulion_node_init(ctx_ptr: *mut u8) -> u64 {
    // SAFETY: `ctx_ptr` was produced by `Box::into_raw(Box::new(NodeContext))`
    // in the host's `DylibNodeEntry::init`. Reclaim via `Box::from_raw`; the
    // Box drops at end of this scope.
    let _ctx = unsafe { Box::from_raw(ctx_ptr as *mut cerulion_core::graph::node::NodeContext) };

    let handle = NEXT_HANDLE.fetch_add(1, Ordering::Relaxed);
    let mut guard = NODES.lock().unwrap();
    let map = guard.get_or_insert_with(HashMap::new);
    map.insert(handle, 0); // value unused (tick is a no-op); presence = "live handle"
    handle
}

#[no_mangle]
pub extern "C" fn cerulion_node_tick(handle: u64) -> i32 {
    // No-op tick. With the drain FFI reporting popped = 0 (or failing), the
    // data-trigger never fires, so this is only reachable if a fire signal
    // was FABRICATED — which the tests assert never happens (fire_count 0).
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
// OPTIONAL drain FFI (the surface under test). Additive symbol that
// does NOT bump the ABI version; its PRESENCE makes the host wire the
// binding `DrainSource::Unified`.
// ===========================================================================

/// `cerulion_node_drain_trigger_input`: the host's per-step unified drain.
///
/// Counts EVERY invocation (fault or not) into the process-global
/// `DRAIN_CALLS`. FAULT INJECTION: when `CER_FAIL_MODE == "drain_fail"`,
/// refuse — set LAST_ERROR and return -1 WITHOUT writing the out-params
/// (modeling a poisoned cdylib `NODES` mutex; also exercises the host's
/// "out-params only on success" contract). Otherwise validate the handle and
/// report a successful EMPTY drain (`popped = 0`, no timestamp) — this
/// fixture drains no real queue.
///
/// # Safety
///
/// The host (`DylibNodeEntry::drain_trigger_input`) passes valid, writable
/// out-pointers (its own stack slots) valid for the duration of the call.
/// Declared `unsafe` because the body writes through them (the macro-generated
/// twin routes its writes through a `catch_unwind` closure instead; a raw-FFI
/// fixture states the contract in the signature, per
/// `clippy::not_unsafe_ptr_arg_deref`).
#[no_mangle]
pub unsafe extern "C" fn cerulion_node_drain_trigger_input(
    handle: u64,
    _name_ptr: *const u8,
    _name_len: usize,
    out_popped: *mut u64,
    out_latest_ts: *mut u64,
    out_has_ts: *mut i32,
) -> i32 {
    DRAIN_CALLS.fetch_add(1, Ordering::Relaxed);
    if std::env::var("CER_FAIL_MODE").as_deref() == Ok("drain_fail") {
        set_last_error(String::from(
            "forced drain failure (CER_FAIL_MODE=drain_fail)",
        ));
        return -1;
    }
    let guard = NODES.lock().unwrap();
    match guard.as_ref().and_then(|m| m.get(&handle)) {
        Some(_) => {
            // SAFETY: the host passes valid, writable out-pointers (its own
            // stack slots) for this call.
            unsafe {
                *out_popped = 0;
                *out_latest_ts = 0;
                *out_has_ts = 0;
            }
            0
        }
        None => {
            set_last_error(format!(
                "cerulion_node_drain_trigger_input: handle {handle} not found"
            ));
            -2
        }
    }
}

// ===========================================================================
// Test accessors (read/reset the process-global drain-call counter).
// ===========================================================================

/// Read the process-global drain-call counter. `handle` is accepted for ABI
/// shape but ignored — the counter is process-global, shared across every
/// `dlopen` of this .so, so the test reads it through its own `libloading`
/// handle of the same path.
#[no_mangle]
pub extern "C" fn cerulion_test_get_drain_calls(_handle: u64) -> u64 {
    DRAIN_CALLS.load(Ordering::Relaxed)
}

/// Reset the process-global drain-call counter to 0 (so the fault + control
/// tests, which share this static within one process, isolate regardless of
/// run order).
#[no_mangle]
pub extern "C" fn cerulion_test_reset_drain_calls() {
    DRAIN_CALLS.store(0, Ordering::Relaxed);
}
