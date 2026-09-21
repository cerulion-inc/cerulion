// SPDX-License-Identifier: AGPL-3.0-only
//! Test cdylib whose `cerulion_node_info()` returns CORRUPTED JSON
//! (unparseable on purpose).
//!
//! Used by `cerulion_core/tests/dylib_corrupt_info_test.rs` to prove
//! that `NodeEntry::info()` propagates the parse failure as
//! `TransportError::NodeInfoParse` and that `GraphRuntime::build` /
//! `build_in_process` refuse to construct a runtime containing this
//! node (load-time failure, not tick-time silence).
//!
//! A dedicated fixture (rather than a `CER_FAIL_MODE` arm on
//! `test_node_failing_cdylib`) is required because that fixture is
//! macro-generated: its info JSON is assembled once by
//! `#[cerulion_node]` codegen and the env var is NOT consulted at
//! info-call time. This crate hand-rolls the ABI v5 surface so the
//! corrupt bytes sit directly in `.rodata`.
//!
//! Everything except `cerulion_node_info()` mirrors `test_node_cdylib`
//! (the canonical raw-FFI fixture) so `DylibNodeEntry::load()` itself
//! succeeds — the failure MUST surface at info-parse time, proving the
//! load-vs-info boundary.

// P12 (the AGENTS.md logging convention): library code never prints — it logs through
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

/// Deliberately malformed JSON: truncated mid-array, unbalanced braces.
/// `serde_json` fails with an EOF-class error; the host's
/// `DylibNodeEntry::info()` must wrap that into
/// `TransportError::NodeInfoParse` carrying this payload's length and
/// first-80-bytes prefix.
static CORRUPT_INFO_BYTES: &[u8] = b"{\"inputs\":[\"velocity_in\",CORRUPTED!!,\"outputs\":[\0";

/// ABI version — must match `cerulion_core::CERULION_ABI_VERSION` so the
/// loader's version gate passes and the corruption is discovered at
/// info-parse time, not at symbol-load time. PR-merge: read the const
/// directly (was a hardcoded `5`) so a future ABI bump never silently
/// re-strands this fixture behind the loader's version gate.
#[no_mangle]
pub extern "C" fn cerulion_abi_version() -> u32 {
    ::cerulion_core::CERULION_ABI_VERSION
}

/// ABI v22: tracks `cerulion_core::rustc_fingerprint_cstr()` directly, same
/// reasoning as `cerulion_abi_version` above.
#[no_mangle]
pub extern "C" fn cerulion_rustc_fingerprint() -> *const std::ffi::c_char {
    ::cerulion_core::rustc_fingerprint_cstr()
}

#[no_mangle]
pub extern "C" fn cerulion_take_last_error() -> *mut std::ffi::c_char {
    std::ptr::null_mut()
}

/// # Safety
///
/// Pointer must be one previously returned by `cerulion_take_last_error`
/// from this same cdylib (allocator pairing requirement); null is also
/// allowed (no-op).
#[no_mangle]
pub unsafe extern "C" fn cerulion_free_error(ptr: *mut std::ffi::c_char) {
    if ptr.is_null() {
        return;
    }
    // SAFETY: per the function-level contract above; this fixture's
    // take never returns non-null, but defining the free for symmetry
    // keeps the contract complete.
    let _ = std::ffi::CString::from_raw(ptr);
}

#[no_mangle]
pub extern "C" fn cerulion_node_info() -> *const std::ffi::c_char {
    CORRUPT_INFO_BYTES.as_ptr() as *const std::ffi::c_char
}

/// Initialize a new node instance.
///
/// Never reached by the tests (`info` fails first on the
/// graph-build path), but mirrors `test_node_cdylib`'s ownership
/// contract: reclaim the host's `Box::into_raw(NodeContext)` via
/// `Box::from_raw` so a direct-init caller doesn't leak a context.
#[no_mangle]
pub extern "C" fn cerulion_node_init(ctx_ptr: *mut u8) -> u64 {
    // SAFETY: `ctx_ptr` was produced by `Box::into_raw(Box::new(NodeContext))`
    // in the host's `DylibNodeEntry::init` (cerulion_core/src/graph/node.rs).
    // Reclaim via `Box::from_raw`; the Box drops at end of this scope.
    let _ctx = unsafe { Box::from_raw(ctx_ptr as *mut cerulion_core::graph::node::NodeContext) };

    let handle = NEXT_HANDLE.fetch_add(1, Ordering::Relaxed);
    let mut guard = NODES.lock().unwrap();
    let map = guard.get_or_insert_with(HashMap::new);
    map.insert(handle, 0);
    handle
}

#[no_mangle]
pub extern "C" fn cerulion_node_tick(handle: u64) -> i32 {
    let mut guard = NODES.lock().unwrap();
    match guard.as_mut().and_then(|m| m.get_mut(&handle)) {
        Some(count) => {
            *count += 1;
            0
        }
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

/// ABI v7: the loader resolves this symbol after the
/// version gate. This fixture never delivers history (its `info()` is corrupt,
/// so it never reaches a live graph), so it's a no-op.
#[no_mangle]
pub extern "C" fn cerulion_node_pump_history(_handle: u64) -> i32 {
    0
}
