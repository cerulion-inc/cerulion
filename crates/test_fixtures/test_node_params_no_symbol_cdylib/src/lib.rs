// SPDX-License-Identifier: AGPL-3.0-only
//! Test cdylib whose `cerulion_node_info()` DECLARES a param but which
//! exports no `cerulion_node_init_with_params` symbol: a raw-FFI node with
//! a hand-written info block, or one copied from a macro node.
//!
//! No value could ever reach such a node, so the host refuses the library
//! when it reads its info (`DylibNodeEntry::info`), naming the symbol and
//! the params. `cerulion_core/tests/cdylib_params_test.rs` proves the load
//! itself succeeds and the refusal happens at that one seam, where
//! `graph validate`, `graph run` and `node info` all read the library.
//!
//! Everything except `cerulion_node_info()` mirrors `test_node_cdylib` (the
//! canonical raw-FFI fixture), so `DylibNodeEntry::load()` passes and the
//! refusal is proven to be the params check, not a missing symbol.
//!
//! With `CER_INFO_FIRST_READ_PLAIN=1` in its environment the FIRST
//! `cerulion_node_info()` call returns a document with no params (one the
//! host accepts) and every later call the declaring one: a library whose
//! metadata varies per call. `cerulion_wsd/tests/inspect_channel_test.rs`
//! uses it to prove the inspector child forwards the document that passed
//! the checks, not a second read.
//!
//! With `CER_INFO_UNKNOWN_KEY=1` every call returns a document with no params
//! and one key the host does not know (`polciy`): the same test file uses it
//! to prove the child does not log the unknown-key warning the parent logs
//! again when it parses the forwarded document. With `CER_INFO_MALFORMED=1`
//! every call returns a document that is not JSON, to prove the child still
//! logs the host's ERROR for it.

// P12 (the AGENTS.md logging convention): library code never prints; it logs
// through `tracing`. Scoped `not(test)` so unit tests keep printing
// diagnostics, and applied at the crate root rather than in
// `[workspace.lints]` because that table cannot distinguish a lib target
// from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Mutex;

static NEXT_HANDLE: AtomicU64 = AtomicU64::new(1);
static NODES: Mutex<Option<HashMap<u64, u64>>> = Mutex::new(None);

/// A well-formed info document with one declared param. The host parses it
/// without complaint; the refusal is the missing symbol, not the JSON.
static INFO_BYTES: &[u8] = b"{\"inputs\":[],\"outputs\":[],\"params\":[{\"name\":\"gain\",\"type\":\"f64\",\"default\":1.0,\"min\":0.0,\"max\":10.0}]}\0";

/// ABI version: tracks `cerulion_core::CERULION_ABI_VERSION` directly so the
/// loader's version gate passes and the refusal is discovered at info time,
/// not at symbol-load time.
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

/// The document the first call returns under `CER_INFO_FIRST_READ_PLAIN=1`.
static PLAIN_INFO_BYTES: &[u8] = b"{\"inputs\":[],\"outputs\":[]}\0";

/// The document every call returns under `CER_INFO_UNKNOWN_KEY=1`.
static UNKNOWN_KEY_INFO_BYTES: &[u8] = b"{\"inputs\":[],\"outputs\":[],\"polciy\":\"x\"}\0";

/// The document every call returns under `CER_INFO_MALFORMED=1`: not JSON.
static MALFORMED_INFO_BYTES: &[u8] = b"{not json\0";

/// How many times `cerulion_node_info()` has been called in this process.
static INFO_CALLS: AtomicU64 = AtomicU64::new(0);

#[no_mangle]
pub extern "C" fn cerulion_node_info() -> *const std::ffi::c_char {
    let first = INFO_CALLS.fetch_add(1, Ordering::Relaxed) == 0;
    let varies = std::env::var_os("CER_INFO_FIRST_READ_PLAIN").is_some_and(|v| v == "1");
    let unknown_key = std::env::var_os("CER_INFO_UNKNOWN_KEY").is_some_and(|v| v == "1");
    let malformed = std::env::var_os("CER_INFO_MALFORMED").is_some_and(|v| v == "1");
    let bytes = if malformed {
        MALFORMED_INFO_BYTES
    } else if unknown_key {
        UNKNOWN_KEY_INFO_BYTES
    } else if first && varies {
        PLAIN_INFO_BYTES
    } else {
        INFO_BYTES
    };
    bytes.as_ptr() as *const std::ffi::c_char
}

/// Initialize a new node instance.
///
/// Never reached by the tests (`info` refuses first on every path that
/// reads the library), but mirrors `test_node_cdylib`'s ownership contract:
/// reclaim the host's `Box::into_raw(NodeContext)` via `Box::from_raw` so a
/// direct-init caller doesn't leak a context.
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

/// ABI v7: the loader resolves this symbol after the version gate. This
/// fixture has no publishers and never reaches a live graph, so it is a
/// no-op.
#[no_mangle]
pub extern "C" fn cerulion_node_pump_history(_handle: u64) -> i32 {
    0
}
