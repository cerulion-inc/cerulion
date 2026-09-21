// SPDX-License-Identifier: AGPL-3.0-only
//! Arm 18 (FFI half): a RAW-FFI per-set Sync node whose
//! `cerulion_node_sync_head_op` FAILS for ONE NAMED INPUT while its drain and
//! refill work normally.
//!
//! # Why one named input rather than a poisoned node
//!
//! The align driver's `Failed` policy is POSITION-AWARE (R-Fail): a failed
//! probe resolves to the DESCENT-DISABLING answer for the site it was asked at
//! — at the argmin `None` (fire greedily), at the gate `Present` (this input
//! refuses the gate) — and a failed MUTATING op terminates the pass. A fixture
//! that fails EVERY op cannot exercise that: the argmin's own probe fails first,
//! so the gate is never scanned and the position distinction is invisible.
//!
//! So the fault is scoped to `FAULT_INPUT` and, deliberately, the FILLS are
//! untouched — they ride the drain symbols, not this one. Both heads therefore
//! fill from real frames and the matcher reaches a complete tuple; what the
//! fault removes is the EVIDENCE the descent would have needed. Fail-closed
//! means the answer must be the conservative one: no descent on evidence that
//! was never gathered.
//!
//! # The fault
//!
//! Armed by `CER_FAIL_MODE == "sync_op_fail"`. When armed, every head op naming
//! `FAULT_INPUT` returns `-6` — the code the macro's own export uses for "the op
//! answered Failed" — WITHOUT touching the context, so no accounting moves and
//! no frame is consumed by the fault itself. Ops naming any other input, and
//! every drain / refill, are forwarded to the real `NodeContext`.
//!
//! # Observability
//!
//! Two process-global counters (head-op calls, faulted calls) and the members of
//! the last fire, read back through `sync_probe`. A test reads them via
//! its OWN `dlopen` of this same path — `dlopen` refcounts, so the host's handle
//! and the test's share one copy of these statics.

// P12: library code reports through `tracing`, never `print*!`. Scoped
// `not(test)` so unit tests keep printing diagnostics, and applied at the crate
// root rather than in `[workspace.lints]` because that table cannot distinguish
// a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Mutex;

use cerulion_core::graph::node::NodeContext;
use native_ros2_messages::geometry_msgs::Vector3;

/// The ONE input whose head ops fail while the fault is armed.
const FAULT_INPUT: &str = "b";

static NEXT_HANDLE: AtomicU64 = AtomicU64::new(1);
#[allow(clippy::type_complexity)]
static NODES: Mutex<Option<HashMap<u64, NodeContext>>> = Mutex::new(None);

/// Every `cerulion_node_sync_head_op` entry, faulted or not.
static HEAD_OP_CALLS: AtomicU64 = AtomicU64::new(0);
/// The subset that took the fault arm.
static HEAD_OP_FAULTS: AtomicU64 = AtomicU64::new(0);
/// Fires whose tick actually READ both members.
static FIRES: AtomicU64 = AtomicU64::new(0);
/// The last fire's members, as raw `f64` bits — the value oracle. A fabricated
/// member would show up here as a stamp the test never published.
static LAST_A_BITS: AtomicU64 = AtomicU64::new(0);
static LAST_B_BITS: AtomicU64 = AtomicU64::new(0);

/// Null-terminated JSON info in `.rodata` — no allocation, no leak.
///
/// TWO trigger inputs plus `sync_window_ms`, which is what makes this a Sync
/// node the runtime drives through the per-set align pass. `schema_hash` is
/// deliberately omitted: it defaults to the raw-FFI "no declared schema"
/// sentinel, which is what every other raw-FFI fixture emits.
static INFO_BYTES: &[u8] = b"{\"inputs\":[{\"name\":\"a\",\"trigger\":true},{\"name\":\"b\",\"trigger\":true}],\"outputs\":[],\"policy\":{\"sync_window_ms\":50}}\0";

thread_local! {
    static LAST_ERROR: std::cell::RefCell<Option<std::ffi::CString>> =
        const { std::cell::RefCell::new(None) };
}

fn set_last_error(msg: &str) {
    let cstring = std::ffi::CString::new(msg.replace('\0', "\\0"))
        .unwrap_or_else(|_| std::ffi::CString::new("error message contained nul byte").unwrap());
    LAST_ERROR.with(|cell| *cell.borrow_mut() = Some(cstring));
}

/// The ABI the host checks before it will load this cdylib at all.
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

#[no_mangle]
pub extern "C" fn cerulion_take_last_error() -> *mut std::ffi::c_char {
    LAST_ERROR.with(|cell| match cell.borrow_mut().take() {
        Some(s) => s.into_raw(),
        None => std::ffi::CString::new("").unwrap().into_raw(),
    })
}

/// # Safety
/// `ptr` must be a pointer previously returned by `cerulion_take_last_error`
/// from this same cdylib (CString allocator pairing); null is allowed (no-op).
#[no_mangle]
pub unsafe extern "C" fn cerulion_free_error(ptr: *mut std::ffi::c_char) {
    if ptr.is_null() {
        return;
    }
    let _ = std::ffi::CString::from_raw(ptr);
}

#[no_mangle]
pub extern "C" fn cerulion_node_info() -> *const std::ffi::c_char {
    INFO_BYTES.as_ptr() as *const std::ffi::c_char
}

/// # Safety
/// `ctx_ptr` must be the `Box::into_raw(NodeContext)` the host's
/// `DylibNodeEntry::init` produced.
#[no_mangle]
pub unsafe extern "C" fn cerulion_node_init(ctx_ptr: *mut u8) -> u64 {
    let ctx = unsafe { Box::from_raw(ctx_ptr as *mut NodeContext) };
    let handle = NEXT_HANDLE.fetch_add(1, Ordering::Relaxed);
    let mut guard = match NODES.lock() {
        Ok(g) => g,
        Err(_) => return 0,
    };
    guard.get_or_insert_with(HashMap::new).insert(handle, *ctx);
    handle
}

/// Reads BOTH members and records them. A fire that cannot read a member
/// records nothing, which is what lets the test tell "fired on real frames"
/// from "fired on something fabricated".
#[no_mangle]
pub extern "C" fn cerulion_node_tick(handle: u64) -> i32 {
    let mut guard = match NODES.lock() {
        Ok(g) => g,
        Err(_) => return 3,
    };
    let ctx = match guard.as_mut().and_then(|m| m.get_mut(&handle)) {
        Some(c) => c,
        None => return 4,
    };
    let a = ctx
        .subscriber_mut("a")
        .and_then(|s| s.try_view::<Vector3, f64>(|v| v.x).ok().flatten());
    let b = ctx
        .subscriber_mut("b")
        .and_then(|s| s.try_view::<Vector3, f64>(|v| v.x).ok().flatten());
    if let (Some(a), Some(b)) = (a, b) {
        LAST_A_BITS.store(a.to_bits(), Ordering::Relaxed);
        LAST_B_BITS.store(b.to_bits(), Ordering::Relaxed);
        FIRES.fetch_add(1, Ordering::Relaxed);
    }
    0
}

#[no_mangle]
pub extern "C" fn cerulion_node_pump_history(handle: u64) -> i32 {
    let guard = match NODES.lock() {
        Ok(g) => g,
        Err(_) => return 3,
    };
    match guard.as_ref().and_then(|m| m.get(&handle)) {
        Some(_) => 0,
        None => 4,
    }
}

#[no_mangle]
pub extern "C" fn cerulion_node_shutdown(handle: u64) -> i32 {
    let mut guard = match NODES.lock() {
        Ok(g) => g,
        Err(_) => return 3,
    };
    match guard.as_mut().and_then(|m| m.remove(&handle)) {
        Some(_) => 0,
        None => 4,
    }
}

/// Borrow the `name_ptr` / `name_len` pair the host passed.
///
/// # Safety
/// The host passes `str::as_ptr()` + its length, valid for the call.
unsafe fn borrow_name<'a>(name_ptr: *const u8, name_len: usize) -> Option<&'a str> {
    if name_ptr.is_null() {
        return None;
    }
    std::str::from_utf8(unsafe { std::slice::from_raw_parts(name_ptr, name_len) }).ok()
}

fn fault_armed() -> bool {
    std::env::var("CER_FAIL_MODE").as_deref() == Ok("sync_op_fail")
}

/// # Safety
/// The host passes `str::as_ptr()` + its length, valid for the call; the
/// out-pointers are the host's own stack slots.
#[no_mangle]
pub unsafe extern "C" fn cerulion_node_drain_trigger_input(
    handle: u64,
    name_ptr: *const u8,
    name_len: usize,
    out_popped: *mut u64,
    out_latest_ts: *mut u64,
    out_has_ts: *mut i32,
) -> i32 {
    forward_drain(
        handle,
        name_ptr,
        name_len,
        out_popped,
        out_latest_ts,
        out_has_ts,
        false,
    )
}

/// # Safety
/// See [`cerulion_node_drain_trigger_input`].
#[no_mangle]
pub unsafe extern "C" fn cerulion_node_refill_trigger_input(
    handle: u64,
    name_ptr: *const u8,
    name_len: usize,
    out_popped: *mut u64,
    out_latest_ts: *mut u64,
    out_has_ts: *mut i32,
) -> i32 {
    forward_drain(
        handle,
        name_ptr,
        name_len,
        out_popped,
        out_latest_ts,
        out_has_ts,
        true,
    )
}

#[allow(clippy::too_many_arguments)]
fn forward_drain(
    handle: u64,
    name_ptr: *const u8,
    name_len: usize,
    out_popped: *mut u64,
    out_latest_ts: *mut u64,
    out_has_ts: *mut i32,
    refill: bool,
) -> i32 {
    let name = match unsafe { borrow_name(name_ptr, name_len) } {
        Some(n) => n,
        None => {
            set_last_error("drain/refill: bad name pointer");
            return -3;
        }
    };
    if out_popped.is_null() || out_latest_ts.is_null() || out_has_ts.is_null() {
        set_last_error("drain/refill: null out-param");
        return -7;
    }
    let mut guard = match NODES.lock() {
        Ok(g) => g,
        Err(_) => {
            set_last_error("drain/refill: NODES mutex poisoned");
            return -1;
        }
    };
    let ctx = match guard.as_mut().and_then(|m| m.get_mut(&handle)) {
        Some(c) => c,
        None => {
            set_last_error("drain/refill: handle not found");
            return -2;
        }
    };
    // The FILLS are deliberately never faulted: they ride these symbols, and a
    // node whose heads never fill cannot reach the align verdicts this fixture
    // exists to exercise.
    let (popped, latest_ts) = if refill {
        ctx.refill_trigger_input(name)
    } else {
        ctx.drain_trigger_input(name)
    };
    unsafe {
        *out_popped = popped;
        match latest_ts {
            Some(ts) => {
                *out_latest_ts = ts;
                *out_has_ts = 1;
            }
            None => {
                *out_latest_ts = 0;
                *out_has_ts = 0;
            }
        }
    }
    0
}

/// # Safety
/// The host passes `str::as_ptr()` + its length, valid for the call; `out_ts`
/// and `out_kind` are the host's own stack slots.
#[no_mangle]
pub unsafe extern "C" fn cerulion_node_sync_head_op(
    handle: u64,
    name_ptr: *const u8,
    name_len: usize,
    op: u32,
    out_ts: *mut u64,
    out_kind: *mut i32,
) -> i32 {
    use cerulion_core::graph::node::{
        SYNC_HEAD_OP_ADVANCE, SYNC_HEAD_OP_PEEK_NEXT, SYNC_HEAD_OP_PROBE_NEXT, SYNC_HEAD_OP_VOID,
        SYNC_OP_ANSWER_HEAD, SYNC_OP_ANSWER_NOTHING, SYNC_OP_ANSWER_PRESENT, SYNC_OP_ANSWER_STAMP,
    };
    use cerulion_core::{SyncHeadOp, SyncOpAnswer};

    HEAD_OP_CALLS.fetch_add(1, Ordering::Relaxed);
    let name = match unsafe { borrow_name(name_ptr, name_len) } {
        Some(n) => n,
        None => {
            set_last_error("cerulion_node_sync_head_op: bad name pointer");
            return -8;
        }
    };
    // THE FAULT. Returned BEFORE the context is touched, so a faulted op
    // consumes nothing and moves no accounting — the failure the host sees is
    // exactly "this op answered Failed", with no side effect behind it.
    if fault_armed() && name == FAULT_INPUT {
        HEAD_OP_FAULTS.fetch_add(1, Ordering::Relaxed);
        set_last_error("cerulion_node_sync_head_op: forced failure (CER_FAIL_MODE=sync_op_fail)");
        return -6;
    }
    if out_ts.is_null() || out_kind.is_null() {
        set_last_error("cerulion_node_sync_head_op: null out-param");
        return -7;
    }
    let head_op = match op {
        SYNC_HEAD_OP_PROBE_NEXT => SyncHeadOp::ProbeNext,
        SYNC_HEAD_OP_PEEK_NEXT => SyncHeadOp::PeekNext,
        SYNC_HEAD_OP_ADVANCE => SyncHeadOp::Advance,
        SYNC_HEAD_OP_VOID => SyncHeadOp::Void,
        _ => {
            set_last_error("cerulion_node_sync_head_op: unknown op code");
            return -5;
        }
    };
    let mut guard = match NODES.lock() {
        Ok(g) => g,
        Err(_) => {
            set_last_error("cerulion_node_sync_head_op: NODES mutex poisoned");
            return -1;
        }
    };
    let ctx = match guard.as_mut().and_then(|m| m.get_mut(&handle)) {
        Some(c) => c,
        None => {
            set_last_error("cerulion_node_sync_head_op: handle not found");
            return -2;
        }
    };
    let (kind, ts) = match ctx.sync_head_op(name, head_op) {
        SyncOpAnswer::Nothing => (SYNC_OP_ANSWER_NOTHING, 0u64),
        SyncOpAnswer::Present => (SYNC_OP_ANSWER_PRESENT, 0u64),
        SyncOpAnswer::Head(ts) => (SYNC_OP_ANSWER_HEAD, ts),
        SyncOpAnswer::Stamp(ts) => (SYNC_OP_ANSWER_STAMP, ts),
        SyncOpAnswer::Failed => {
            set_last_error("cerulion_node_sync_head_op: the op answered Failed");
            return -6;
        }
    };
    unsafe {
        *out_ts = ts;
        *out_kind = kind;
    }
    0
}

/// Read the fixture's process-global observations: head-op calls, faulted
/// calls, fires, and the last fire's two members as raw `f64` bits.
///
/// # Safety
/// All four out-pointers must be non-null and writable.
#[no_mangle]
pub unsafe extern "C" fn sync_probe(
    out_calls: *mut u64,
    out_faults: *mut u64,
    out_fires: *mut u64,
    out_a_bits: *mut u64,
    out_b_bits: *mut u64,
) {
    unsafe {
        *out_calls = HEAD_OP_CALLS.load(Ordering::Relaxed);
        *out_faults = HEAD_OP_FAULTS.load(Ordering::Relaxed);
        *out_fires = FIRES.load(Ordering::Relaxed);
        *out_a_bits = LAST_A_BITS.load(Ordering::Relaxed);
        *out_b_bits = LAST_B_BITS.load(Ordering::Relaxed);
    }
}

/// Zero every counter, so two arms in one process do not read each other's.
#[no_mangle]
pub extern "C" fn sync_probe_reset() {
    HEAD_OP_CALLS.store(0, Ordering::Relaxed);
    HEAD_OP_FAULTS.store(0, Ordering::Relaxed);
    FIRES.store(0, Ordering::Relaxed);
    LAST_A_BITS.store(0, Ordering::Relaxed);
    LAST_B_BITS.store(0, Ordering::Relaxed);
}
