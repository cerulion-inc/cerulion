// A node is a LIBRARY the runtime loads — log with `tracing`,
// never `println!`. (Your own #[cfg(test)] tests may print.)
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Mutex;

struct RawFfiTemplateState {
    tick_count: u64,
}

static NEXT_HANDLE: AtomicU64 = AtomicU64::new(1);
static NODES: Mutex<Option<HashMap<u64, RawFfiTemplateState>>> = Mutex::new(None);

thread_local! {
    static LAST_ERROR: ::std::cell::RefCell<Option<::std::ffi::CString>> = const { ::std::cell::RefCell::new(None) };
}

fn __cer_set_last_error(msg: ::std::string::String) {
    let cstring = ::std::ffi::CString::new(msg.replace('\0', "\\0"))
        .unwrap_or_else(|_| ::std::ffi::CString::new("error message contained nul byte").unwrap());
    LAST_ERROR.with(|cell| {
        *cell.borrow_mut() = Some(cstring);
    });
}

#[no_mangle]
pub extern "C" fn cerulion_abi_version() -> u32 {
    ::cerulion_core::CERULION_ABI_VERSION
}

#[no_mangle]
pub extern "C" fn cerulion_rustc_fingerprint() -> *const ::std::ffi::c_char {
    ::cerulion_core::rustc_fingerprint_cstr()
}

// CERULION:INFO_START
static INFO_BYTES: &[u8] = b"{\"inputs\":[],\"outputs\":[]}\0";

#[no_mangle]
pub extern "C" fn cerulion_node_info() -> *const std::ffi::c_char {
    // SAFETY: INFO_BYTES is a static &[u8] with a trailing null byte.
    // Casting *const u8 to *const c_char is safe: identical layout, valid and 'static.
    INFO_BYTES.as_ptr() as *const std::ffi::c_char
}
// CERULION:INFO_END

// =================================================================
// NOTE TO USERS REPLACING THE PLACEHOLDER BODY
// =================================================================
// The template's tick body is a no-op counter (`tick_count += 1`) and
// CANNOT FAIL. As a result, this raw-FFI template DOES NOT EMIT:
//   - `init failed: {e}` / `tick failed: {e}` Err-propagation wiring
//     (the placeholder body has no `Result<_, _>` return).
//   - `std::panic::catch_unwind(...)` panic-safety wrapper around
//     init/tick/shutdown (the placeholder body cannot panic).
//
// If your real logic CAN return Err OR CAN panic, you MUST either:
//   1. Migrate to `#[cerulion_node]` (drop `--raw-ffi` from
//      `cerulion node create`) — the recommended path; the macro
//      carries full panic-safety + Err propagation with no extra wiring.
//   2. Hand-roll the equivalent wiring in every entry point below:
//      wrap the body in `catch_unwind`, and on the error path call
//      `__cer_set_last_error(format!("init failed: {}", e))` before
//      returning the matching non-success code.
//
// Unwinding across the FFI boundary aborts the process (and was
// undefined behaviour on older toolchains pre-Rust-1.71). Either way,
// a panic must NOT cross the cdylib boundary back into the host.
// Failing to translate Err to LAST_ERROR silently strips the diagnostic
// — the host falls back to a generic "no detail provided" message that
// is nearly impossible to debug from operator logs.

#[no_mangle]
pub extern "C" fn cerulion_node_init(ctx_ptr: *mut u8) -> u64 {
    if ctx_ptr.is_null() {
        __cer_set_last_error(::std::string::String::from(
            "cerulion_node_init: NodeContext pointer was null",
        ));
        return 0;
    }
    // SAFETY: `ctx_ptr` is a `NodeContext` the runtime boxed and handed to
    // this cdylib. Ownership transferred with it — the runtime will not free
    // it — so pairing with `Box::from_raw` is required to avoid leaking one
    // `NodeContext` per init call. This placeholder body doesn't use
    // transport, so dropping the context at end of scope is the correct
    // behavior. A node written with `#[cerulion_node]` instead STORES the
    // boxed context so it can publish / subscribe — and so should you, once
    // this node does real work.
    let ctx = unsafe {
        ::std::boxed::Box::from_raw(ctx_ptr as *mut ::cerulion_core::graph::node::NodeContext)
    };

    // KEEP THIS, and keep it in `init`. A cdylib statically links its OWN
    // copy of `iceoryx2-log`, whose level is a private static: the host
    // process setting `IOX2_LOG_LEVEL` does NOTHING for this copy, so
    // without this call iceoryx2 logs from THIS node at its crate default
    // (`Info`) no matter what you configured. That is not cosmetic — a node
    // logging at `Info` on a busy robot can emit thousands of lines a second
    // and fill the disk. The value comes from the node's FROZEN env
    // snapshot, never live `std::env`, so replay stays deterministic; empty
    // means unset, which leaves Cerulion's default (`error`). Nodes written
    // with `#[cerulion_node]` get this generated for them.
    {
        let iox2_log = ctx.env_str("IOX2_LOG_LEVEL", "");
        ::cerulion_core::iceoryx_logger::init_iceoryx_log_level(if iox2_log.is_empty() {
            ::std::option::Option::None
        } else {
            ::std::option::Option::Some(iox2_log.as_str())
        });
    }

    let handle = NEXT_HANDLE.fetch_add(1, Ordering::Relaxed);
    if let Ok(mut guard) = NODES.lock() {
        let map = guard.get_or_insert_with(HashMap::new);
        map.insert(handle, RawFfiTemplateState { tick_count: 0 });
        handle
    } else {
        // Mutex poisoned: do NOT recover via `into_inner()` — a panic
        // while holding the lock can leave per-node state inconsistent.
        // Surface code + LAST_ERROR; let the host decide to restart.
        __cer_set_last_error(::std::string::String::from(
            "cerulion_node_init: NODES mutex poisoned",
        ));
        0
    }
}

#[no_mangle]
pub extern "C" fn cerulion_node_tick(handle: u64) -> i32 {
    if let Ok(mut guard) = NODES.lock() {
        match guard.as_mut().and_then(|m| m.get_mut(&handle)) {
            Some(state) => {
                state.tick_count += 1;
                0
            }
            None => {
                __cer_set_last_error(format!("cerulion_node_tick: handle {} not found", handle));
                4
            }
        }
    } else {
        // Mutex poisoned: do NOT recover via `into_inner()` — see init.
        __cer_set_last_error(::std::string::String::from(
            "cerulion_node_tick: NODES mutex poisoned",
        ));
        3
    }
}

#[no_mangle]
pub extern "C" fn cerulion_node_pump_history(handle: u64) -> i32 {
    if let Ok(mut guard) = NODES.lock() {
        match guard.as_mut().and_then(|m| m.get_mut(&handle)) {
            Some(_state) => {
                // Placeholder node has no publishers — nothing to re-pump.
                0
            }
            None => {
                __cer_set_last_error(format!(
                    "cerulion_node_pump_history: handle {} not found",
                    handle
                ));
                4
            }
        }
    } else {
        // Mutex poisoned: do NOT recover via `into_inner()` — see init.
        __cer_set_last_error(::std::string::String::from(
            "cerulion_node_pump_history: NODES mutex poisoned",
        ));
        3
    }
}

#[no_mangle]
pub extern "C" fn cerulion_node_shutdown(handle: u64) -> i32 {
    let mut guard = match NODES.lock() {
        Ok(g) => g,
        Err(_) => {
            // Mutex poisoned: do NOT recover via `into_inner()` — see init.
            __cer_set_last_error(::std::string::String::from(
                "cerulion_node_shutdown: NODES mutex poisoned",
            ));
            return 3;
        }
    };
    match guard.as_mut().and_then(|m| m.remove(&handle)) {
        Some(_node) => 0,
        None => {
            __cer_set_last_error(format!(
                "cerulion_node_shutdown: handle {} not found (already shut down or never registered)",
                handle
            ));
            4
        }
    }
}

#[no_mangle]
pub extern "C" fn cerulion_take_last_error() -> *mut std::ffi::c_char {
    LAST_ERROR.with(|cell| match cell.borrow_mut().take() {
        Some(cstr) => cstr.into_raw(),
        None => std::ptr::null_mut(),
    })
}

/// # Safety
///
/// `ptr` must be a pointer previously returned by
/// `cerulion_take_last_error` from this same cdylib (allocator pairing
/// requirement). Null is permitted and is a no-op. Calling with any
/// other pointer is undefined behaviour.
#[no_mangle]
pub unsafe extern "C" fn cerulion_free_error(ptr: *mut std::ffi::c_char) {
    if ptr.is_null() {
        return;
    }
    // SAFETY: pointer originated from `CString::into_raw` in
    // `cerulion_take_last_error`. Reclaiming via `from_raw`
    // returns ownership and the CString drops here.
    let _ = ::std::ffi::CString::from_raw(ptr);
}
