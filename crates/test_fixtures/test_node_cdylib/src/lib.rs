// SPDX-License-Identifier: AGPL-3.0-only
//! Test cdylib node for integration testing of DylibNodeEntry.
//!
//! Implements the Cerulion raw-FFI node entry point convention. The
//! `cerulion_abi_version()` export tracks `cerulion_core::CERULION_ABI_VERSION`
//! directly, so this fixture stays loadable across ABI bumps:
//! - `cerulion_abi_version()` → ABI version (the host's current `CERULION_ABI_VERSION`)
//! - `cerulion_rustc_fingerprint()` → rustc that compiled this cdylib (ABI v22);
//!   honors `CER_RUSTC_FAULT` the same way `cerulion_abi_version` honors `CER_ABI_FAULT`
//! - `cerulion_node_info()` → JSON metadata (static, no allocation)
//!   * v4 outputs shape: array of `{name, schema_hash, max_slice_len_default}`
//!     objects. This fixture has zero outputs so the array is empty;
//!     post-A2 hosts also accept the legacy `["name1"]` strings shape
//!     via `serde(untagged)` for backward compat.
//! - `cerulion_node_init(*mut u8) -> u64` → handle-based init
//! - `cerulion_node_tick(u64) -> i32` → tick by handle
//! - `cerulion_node_pump_history(u64) -> i32` → service quiescent late joiners by handle (ABI v7,; no-op here — fixture has no publishers)
//! - `cerulion_node_shutdown(u64) -> i32` → shutdown by handle
//! - `cerulion_take_last_error -> *mut c_char` → most recent error message
//! - `cerulion_free_error(*mut c_char)` → free a string returned by take_last_error
//!
//! Also exports `cerulion_test_get_tick_count(u64)` for test verification.
//!
//! `cerulion_node_init` also applies `IOX2_LOG_LEVEL` from the node's
//! frozen env snapshot to THIS cdylib's own `iceoryx2-log` static (a
//! hand-written raw-FFI node must do what the macro generates), and — behind
//! the env-gated `CER_CDYLIB_IOX2_PROBE` switch, inert for every other test that
//! loads this fixture — prints that level as `RAWFFI_IOX2_LEVEL=<n>` for
//! `cdylib_iox2_log_level_test.rs` to read off stderr.

// Principle #12 (logging): library code never prints — it logs through
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

/// Null-terminated JSON info string in .rodata — no allocation, no leak.
///
/// The info JSON carries no `node_type`; the loader resolves type
/// from the cdylib's folder name. The
/// `outputs` shape is per-output objects; an empty array is valid in
/// that shape, so this zero-port fixture's JSON needs no entries.
static INFO_BYTES: &[u8] = b"{\"inputs\":[],\"outputs\":[]}\0";

/// ABI version — tracks `cerulion_core::CERULION_ABI_VERSION` directly so a
/// host ABI bump needs no edit here (this fixture must stay loadable).
/// A hardcoded literal
/// silently breaks `node_test` on every ABI bump; referencing the const
/// makes the host the single source of truth.
#[no_mangle]
pub extern "C" fn cerulion_abi_version() -> u32 {
    // Test-only fault toggle (env-gated): when `CER_ABI_FAULT` is set, report a
    // deliberately-wrong ABI so the host's version-mismatch rejection path
    // (`DylibNodeEntry::load`) is exercisable end-to-end. WITHOUT the env var,
    // tracks `cerulion_core::CERULION_ABI_VERSION` so this fixture stays loadable
    // across ABI bumps. (This fixture is hand-written, not a drift-checked
    // mirror, so the env branch is safe here.)
    if std::env::var("CER_ABI_FAULT").is_ok() {
        9999 // test-only: a deliberately-wrong ABI to exercise the host's version-mismatch rejection
    } else {
        cerulion_core::CERULION_ABI_VERSION
    }
}

/// ABI v22 rustc fingerprint: tracks `cerulion_core::rustc_fingerprint_cstr()`
/// directly, same reasoning as `cerulion_abi_version` above, EXCEPT for two
/// test-only fault toggles read from the same `CER_RUSTC_FAULT` env var:
/// - unset: report the real fingerprint (the control path).
/// - any other value (conventionally `"1"`): report a deliberately-FAKE
///   fingerprint, so the host's rustc-MISMATCH rejection path
///   (`DylibNodeEntry::load`) is exercisable end-to-end without needing a
///   second rustc toolchain installed
///   (`cerulion_core/tests/rustc_fingerprint_mismatch_test.rs`). The fake
///   value is a static C string, not a fresh allocation per call, which
///   matches `rustc_fingerprint_cstr()`'s own "no allocation after the first
///   call" contract.
/// - exactly `"null"`: return a NULL pointer, so the host's null-pointer arm
///   (a defensive check for a symbol that resolves but returns nothing
///   readable, distinct from a missing symbol, which `libloading::Symbol`
///   itself already refuses) is exercisable the same way, rather than only by
///   code inspection.
#[no_mangle]
pub extern "C" fn cerulion_rustc_fingerprint() -> *const std::ffi::c_char {
    match std::env::var("CER_RUSTC_FAULT") {
        Ok(v) if v == "null" => std::ptr::null(),
        Ok(_) => {
            // test-only: a deliberately-fake fingerprint, kept in lockstep with
            // `rustc_fingerprint_mismatch_test.rs::FAKE_FINGERPRINT`.
            static FAKE: &[u8] = b"9.9.9 (deadbeefdead)\0";
            FAKE.as_ptr() as *const std::ffi::c_char
        }
        Err(_) => cerulion_core::rustc_fingerprint_cstr(),
    }
}

// This manual fixture mirrors the
// macro-generated FFI surface. It never produces an error so the take
// function always returns null, but it must export the symbols so the
// loader's symbol-presence check passes at v3.
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
    INFO_BYTES.as_ptr() as *const std::ffi::c_char
}

/// Initialize a new node instance.
///
/// Receives `*mut NodeContext` (typed as `*mut u8` at the raw-FFI boundary —
/// the host owns the concrete `NodeContext` type).
/// Reclaim the host's `Box::into_raw(NodeContext)` via `Box::from_raw` so
/// each init call doesn't leak one NodeContext. Ignoring the pointer
/// (`let _ = ctx_ptr;`) would leak it; the leak is negligible for a test
/// that passes an empty context, but the same pattern in the
/// user-facing `generate_lib_rs` template would be a correctness defect,
/// and reclaiming the context here keeps the
/// test fixture faithful to the FFI ownership contract.
///
/// Returns a u64 handle (0 = error).
#[no_mangle]
pub extern "C" fn cerulion_node_init(ctx_ptr: *mut u8) -> u64 {
    // SAFETY: `ctx_ptr` was produced by `Box::into_raw(Box::new(NodeContext))`
    // in the host's `DylibNodeEntry::init` (cerulion_core/src/graph/node.rs).
    // Reclaim via `Box::from_raw`; the Box drops at end of this scope.
    let ctx = unsafe { Box::from_raw(ctx_ptr as *mut cerulion_core::graph::node::NodeContext) };

    // A cdylib statically links its OWN `cerulion_core -> iceoryx2 ->
    // iceoryx2-log`, so it has its OWN `LOG_LEVEL` static that the host's
    // `set_log_level` cannot touch. The macro-generated `cerulion_node_init`
    // applies `IOX2_LOG_LEVEL` from the node's FROZEN env snapshot; a
    // HAND-WRITTEN raw-FFI node (this fixture, and anything grown from the
    // `cerulion node create --raw-ffi` template) must do it itself. Reproduced here so
    // `cdylib_iox2_log_level_test.rs` can pin the raw-FFI half of the contract
    // behaviorally, over a real dlopen.
    {
        let iox2_log = ctx.env_str("IOX2_LOG_LEVEL", "");
        cerulion_core::iceoryx_logger::init_iceoryx_log_level(if iox2_log.is_empty() {
            None
        } else {
            Some(iox2_log.as_str())
        });
    }
    // Env-gated probe (INERT for every other test that loads this fixture):
    // report the level of THIS cdylib's own linked copy. Consumed by
    // `cerulion_core/tests/cdylib_iox2_log_level_test.rs`.
    // P12 exception: this line is a MACHINE-READ MARKER, not a log. The parent
    // test process reads it off this cdylib's stderr; routing it through
    // `tracing` would make it depend on a subscriber this fixture does not
    // install and on RUST_LOG, which is exactly what the test is measuring.
    #[allow(clippy::print_stderr)]
    if std::env::var_os("CER_CDYLIB_IOX2_PROBE").is_some() {
        eprintln!(
            "RAWFFI_IOX2_LEVEL={}",
            cerulion_core::iceoryx_logger::current_iox2_log_level()
        );
    }
    drop(ctx);

    let handle = NEXT_HANDLE.fetch_add(1, Ordering::Relaxed);
    let mut guard = NODES.lock().unwrap();
    let map = guard.get_or_insert_with(HashMap::new);
    map.insert(handle, 0); // tick count starts at 0
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

// ABI v7: required symbol so the loader's
// symbol-presence check passes at v7. This fixture's node has no
// publishers, so servicing late joiners is a no-op — return 0 on a
// known handle, mirroring tick's handle-lookup + code-4 not-found path.
#[no_mangle]
pub extern "C" fn cerulion_node_pump_history(handle: u64) -> i32 {
    let guard = NODES.lock().unwrap();
    match guard.as_ref().and_then(|m| m.get(&handle)) {
        Some(_count) => 0, // nothing to pump (no publishers)
        None => 4,         // handle not found
    }
}

#[no_mangle]
pub extern "C" fn cerulion_node_shutdown(handle: u64) -> i32 {
    // Parity with macro-generated FFI:
    // a missing handle returns code 4, never a silent 0.
    let mut guard = NODES.lock().unwrap();
    match guard.as_mut().and_then(|m| m.remove(&handle)) {
        Some(_) => 0,
        None => 4, // handle not found
    }
}

/// Helper for tests to read tick count for a given handle.
#[no_mangle]
pub extern "C" fn cerulion_test_get_tick_count(handle: u64) -> u64 {
    let guard = NODES.lock().unwrap();
    guard
        .as_ref()
        .and_then(|m| m.get(&handle))
        .copied()
        .unwrap_or(0)
}
