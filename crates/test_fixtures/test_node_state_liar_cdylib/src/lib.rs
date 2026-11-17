// SPDX-License-Identifier: AGPL-3.0-only
//! A HAND-WRITTEN raw-FFI cdylib whose state exporter **lies** —
//! it ignores the host sink callback's non-zero return, keeps streaming, and
//! still reports success.
//!
//! # Why this fixture has to exist
//!
//! The three state symbols are ADDITIVE and resolved by NAME. The host therefore
//! cannot assume the exporter behind them came out of `#[cerulion_node]`: any
//! hand-written raw-FFI node may export them, and `test_node_cdylib` next door
//! proves the repo already ships hand-written cdylibs. The macro-generated
//! exporter maps a refused chunk onto `-6`; nothing in the ABI makes a
//! hand-written one do the same. If the host believes the return code alone, a
//! capture that was cut short by the host's OWN bound is accepted as whole
//! state — and a truncated anchor restored later is silent corruption
//! (Principle #6: no data loss), not a loud failure.
//!
//! Every macro fixture in this tree is, by construction, incapable of
//! reproducing that: the generated exporter is correct. So the misbehaviour has
//! to be written by hand, which is exactly the population it models.
//!
//! # The payload
//!
//! Fixed, hand-checkable, and streamed as THREE chunks so a bounded sink can
//! accept the first and refuse the second:
//!
//! | chunk | bytes | content |
//! |---|---|---|
//! | 0 | 8 | `counter: u64` LE (0 until a restore sets it) |
//! | 1 | 12 | `b"liar_fixture"` |
//! | 2 | 4 | `0xDEAD_BEEF` LE — the TAIL marker |
//!
//! 24 bytes total. The tail marker is what makes truncation observable: a
//! prefix that stops after chunk 0 or chunk 1 does not carry it.
//!
//! # Modes (`STATE_CAPTURE_MODE`, read per call)
//!
//! | value | behaviour |
//! |---|---|
//! | unset / `conforming` | stop at the callback's first non-zero return and report `-6` — what the generated exporter does |
//! | `liar` | ignore every non-zero return, offer ALL three chunks anyway, return `0` |
//! | `hint_refuse` | consult `capacity_hint`; if it cannot hold all 24 bytes, call the sink ZERO times and report `-6` — the hash-like-container shape, where the host's own sink never learns it was refused |
//! | `encoder_error` | write chunk 0 (accepted), then report `-5` — the node's OWN encoder failed, which is a different classification from a refused sink and must be charged differently |
//!
//! Two further switches, independent of the capture mode and of each other,
//! serve the LOAD-TIME arm (an exporter that refuses to answer for its shape):
//!
//! | variable | value | behaviour |
//! |---|---|---|
//! | `STATE_SHAPE_MODE` | `fail` | `cerulion_node_state_shape` leaves [`SHAPE_REFUSAL_DETAIL`] in the error slot and returns `-7` |
//! | `STATE_TICK_MODE` | `fail_silent` | `cerulion_node_tick` returns non-zero and sets NO error — so whatever is still in the slot is served as its cause |
//!
//! Nothing here is a "simulated checkpoint": the bytes are real bytes crossing
//! a real `dlopen`'d C ABI into the host's real `StateSink`. Only the
//! exporter's *conformance* is the variable.

#![deny(unused_imports)]
// P12 (the project logging rule): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Mutex;

/// This fixture's `STATE_SHAPE`. A raw-FFI node has no `CerulionState` impl to
/// derive one from, so it declares a constant — which is all the host reads.
/// The test spells the same literal out by hand.
pub const LIAR_STATE_SHAPE: u64 = 0x1053_D600_5A1E_0001;

/// What `cerulion_node_state_shape` leaves in the error slot under
/// `STATE_SHAPE_MODE=fail`. Spelled out here so the test can match the exact
/// string rather than a substring it also produces itself.
pub const SHAPE_REFUSAL_DETAIL: &str = "liar: state_shape refused — schema registry unavailable";

/// Chunk 1 of the payload. Exactly 12 bytes: the tests size their sinks so
/// that an 8-byte sink accepts chunk 0 and refuses this one.
const LABEL: &[u8] = b"liar_fixture";
/// Chunk 2 of the payload — present only in a capture that was NOT truncated.
const TAIL: u32 = 0xDEAD_BEEF;
/// 8 (counter) + 12 (label) + 4 (tail).
const PAYLOAD_LEN: usize = 8 + 12 + 4;

static NEXT_HANDLE: AtomicU64 = AtomicU64::new(1);
/// handle -> the node's whole "state": one counter.
static NODES: Mutex<Option<HashMap<u64, u64>>> = Mutex::new(None);

// ---------------------------------------------------------------------------
// the ordinary raw-FFI surface (mirrors `test_node_cdylib`)
// ---------------------------------------------------------------------------

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

/// Null-terminated JSON info in `.rodata` — no ports, no allocation.
static INFO_BYTES: &[u8] = b"{\"inputs\":[],\"outputs\":[]}\0";

#[no_mangle]
pub extern "C" fn cerulion_node_info() -> *const std::ffi::c_char {
    INFO_BYTES.as_ptr() as *const std::ffi::c_char
}

/// The FFI error slot. Set only on the `liar` path — the conforming paths report
/// through return codes alone, so the host's `-6` / `-5` diagnostics must stand
/// on their own observation.
static LAST_ERROR: Mutex<Option<String>> = Mutex::new(None);

fn set_last_error(msg: String) {
    *LAST_ERROR.lock().expect("LAST_ERROR") = Some(msg);
}

#[no_mangle]
pub extern "C" fn cerulion_take_last_error() -> *mut std::ffi::c_char {
    let taken = LAST_ERROR.lock().expect("LAST_ERROR").take();
    match taken.and_then(|m| std::ffi::CString::new(m).ok()) {
        Some(c) => c.into_raw(),
        None => std::ptr::null_mut(),
    }
}

/// # Safety
///
/// `ptr` must be one previously returned by `cerulion_take_last_error` from
/// this same cdylib; null is a no-op.
#[no_mangle]
pub unsafe extern "C" fn cerulion_free_error(ptr: *mut std::ffi::c_char) {
    if ptr.is_null() {
        return;
    }
    let _ = std::ffi::CString::from_raw(ptr);
}

fn create_local() -> u64 {
    let handle = NEXT_HANDLE.fetch_add(1, Ordering::Relaxed);
    let mut guard = NODES.lock().expect("NODES");
    guard.get_or_insert_with(HashMap::new).insert(handle, 0);
    handle
}

#[no_mangle]
pub extern "C" fn cerulion_node_tick(handle: u64) -> i32 {
    // `STATE_TICK_MODE=fail_silent`: fail WITHOUT setting the error slot.
    //
    // That combination is the whole point. The host reports a tick failure as
    // `take_last_error().unwrap_or_else(|| "…no detail provided")`, so a tick
    // that leaves no message of its own serves whatever is still sitting in the
    // slot — which is how an UNDRAINED message from an earlier, unrelated call
    // gets reported as this tick's cause. A raw-FFI node returning a bare code
    // is ordinary (`cerulion_node_shutdown` right below does it), so this is
    // not a contrived shape; it is the common one.
    if std::env::var("STATE_TICK_MODE").as_deref() == Ok("fail_silent") {
        return 9;
    }
    match std::env::var("STATE_TICK_MODE").as_deref() {
        Ok("fail_empty") => {
            set_last_error(String::new());
            return 9;
        }
        Ok("fail_long") => {
            set_last_error(format!("full reason:{}:end", "x".repeat(1024 * 1024 + 17)));
            return 9;
        }
        _ => {}
    }
    let mut guard = NODES.lock().expect("NODES");
    match guard.as_mut().and_then(|m| m.get_mut(&handle)) {
        Some(counter) => {
            *counter += 1;
            0
        }
        None => 4,
    }
}

#[no_mangle]
pub extern "C" fn cerulion_node_pump_history(handle: u64) -> i32 {
    let guard = NODES.lock().expect("NODES");
    match guard.as_ref().and_then(|m| m.get(&handle)) {
        Some(_) => 0,
        None => 4,
    }
}

#[no_mangle]
pub extern "C" fn cerulion_node_shutdown(handle: u64) -> i32 {
    let mut guard = NODES.lock().expect("NODES");
    match guard.as_mut().and_then(|m| m.remove(&handle)) {
        Some(_) => 0,
        None => 4,
    }
}

/// Test helper: read a handle's counter back, so the restore arm has an oracle
/// that does not go through the (deliberately unreliable) capture path.
#[no_mangle]
pub extern "C" fn cerulion_test_liar_counter(handle: u64) -> u64 {
    let guard = NODES.lock().expect("NODES");
    guard
        .as_ref()
        .and_then(|m| m.get(&handle))
        .copied()
        .unwrap_or(u64::MAX)
}

// ---------------------------------------------------------------------------
// the state trio
// ---------------------------------------------------------------------------

type SinkFn = extern "C" fn(*mut std::ffi::c_void, *const u8, usize) -> i32;

/// # Safety
///
/// `out_shape` must be null or a writable `u64` slot valid for this call — the
/// host passes its own stack slot. Declared `unsafe` because the body writes
/// through the raw pointer; the macro-generated twin routes its write through a
/// `catch_unwind` closure instead, but a raw-FFI fixture states the contract in
/// the signature (`clippy::not_unsafe_ptr_arg_deref`). The host resolves this
/// symbol as `unsafe extern "C" fn(*mut u64) -> i32`, so the marker costs it
/// nothing.
#[no_mangle]
pub unsafe extern "C" fn cerulion_node_state_shape(out_shape: *mut u64) -> i32 {
    if out_shape.is_null() {
        return -3;
    }
    // `STATE_SHAPE_MODE=fail`: refuse to answer, having left a DETAILED
    // reason in the error slot — the shape an exporter takes when it knows why
    // it cannot serve state (a schema it could not resolve, a version it does
    // not recognise). The host reads this once, at load, and then switches the
    // capability off, so this call is the only chance the message ever gets.
    if std::env::var("STATE_SHAPE_MODE").as_deref() == Ok("fail") {
        set_last_error(SHAPE_REFUSAL_DETAIL.to_string());
        return -7;
    }
    // SAFETY: non-null (checked); the host owns a writable slot for this call.
    *out_shape = LIAR_STATE_SHAPE;
    0
}

/// The mode switch, read per call so one loaded library can serve every arm.
fn mode() -> String {
    std::env::var("STATE_CAPTURE_MODE").unwrap_or_else(|_| "conforming".to_string())
}

#[no_mangle]
pub extern "C" fn cerulion_node_capture_state(
    handle: u64,
    sink: Option<SinkFn>,
    user: *mut std::ffi::c_void,
    capacity_hint: u64,
) -> i32 {
    let Some(sink) = sink else { return -3 };
    let counter = {
        let guard = NODES.lock().expect("NODES");
        match guard.as_ref().and_then(|m| m.get(&handle)) {
            Some(c) => *c,
            None => return -2,
        }
    };

    let mode = mode();

    // The hash-like-container shape: refuse up front on the HINT, having called the sink zero
    // times. The host's own sink therefore never learns it was refused — the
    // return code is the only evidence, which is precisely why the host must
    // latch its sink itself.
    if mode == "hint_refuse" && capacity_hint < PAYLOAD_LEN as u64 {
        return -6;
    }

    let counter_bytes = counter.to_le_bytes();
    let tail_bytes = TAIL.to_le_bytes();
    let chunks: [&[u8]; 3] = [&counter_bytes, LABEL, &tail_bytes];

    // The node's OWN encoder failing, with a prefix already ACCEPTED by a sink
    // that is not full. A different classification from a refused sink: the
    // bytes are still discarded, but the sink is not at fault and must not be
    // charged as if it were.
    if mode == "encoder_error" {
        let rc = sink(user, chunks[0].as_ptr(), chunks[0].len());
        if rc != 0 {
            return -6;
        }
        return -5;
    }

    let mut rejections = 0usize;
    for chunk in chunks {
        let rc = sink(user, chunk.as_ptr(), chunk.len());
        if rc != 0 {
            rejections += 1;
            if mode != "liar" {
                // THE CONFORMING BEHAVIOUR: stop at the first refusal and say so.
                return -6;
            }
            // THE LIE: keep streaming as if nothing happened. Every later chunk
            // is offered, and — below — success is reported.
        }
    }
    if rejections > 0 {
        if mode != "liar" {
            return -6;
        }
        // A message left in the slot ALONGSIDE a success return. It is here so
        // the host is seen to DRAIN this slot on the misreport path: an undrained
        // message surfaces later against an unrelated failure, and one that is
        // drained but thrown away loses the only thing the cdylib had to say
        // about a capture it got wrong.
        set_last_error(format!(
            "liar: ignored {rejections} sink rejection(s) and reported success"
        ));
    }
    // `liar` reports SUCCESS even having been rejected. A host that believes
    // this accepts whatever prefix its sink managed to take as whole state.
    0
}

/// # Safety
///
/// `payload_ptr` must be null (only with `payload_len == 0`) or point at
/// `payload_len` readable bytes valid for this call. Declared `unsafe` for the
/// same reason as `cerulion_node_state_shape` above; the host resolves it as
/// `unsafe extern "C" fn(u64, *const u8, usize) -> i32`.
#[no_mangle]
pub unsafe extern "C" fn cerulion_node_restore_state(
    handle: u64,
    payload_ptr: *const u8,
    payload_len: usize,
) -> i32 {
    if payload_len > 0 && payload_ptr.is_null() {
        return -3;
    }
    // `from_raw_parts` over a null pointer is UB even at length zero, so the
    // empty slice is built by hand (the precedent).
    let payload: &[u8] = if payload_len == 0 {
        &[]
    } else {
        // SAFETY: non-null (checked) and the host passes a live, read-only
        // slice of exactly `payload_len` bytes for this call.
        std::slice::from_raw_parts(payload_ptr, payload_len)
    };
    if payload.len() != PAYLOAD_LEN
        || &payload[8..8 + LABEL.len()] != LABEL
        || u32::from_le_bytes(payload[20..24].try_into().expect("4 bytes")) != TAIL
    {
        // A TRUNCATED or otherwise wrong payload is refused rather than decoded
        // as a prefix — the same rule the generated `cer_restore` follows.
        return -5;
    }
    let counter = u64::from_le_bytes(payload[..8].try_into().expect("8 bytes"));
    let mut guard = NODES.lock().expect("NODES");
    match guard.as_mut().and_then(|m| m.get_mut(&handle)) {
        Some(slot) => {
            *slot = counter;
            0
        }
        None => -2,
    }
}

// The adversarial encoder intentionally bypasses NodeExports' sink backstop:
// otherwise the SDK would repair the lie before the host can observe it. Only
// checked C records and borrowed spans cross; all state and text stay here.
use cerulion_core::node_abi as abi;
struct ErrorRow {
    scope: u64,
    token: u64,
    text: Option<String>,
}
static ERRORS: Mutex<Vec<ErrorRow>> = Mutex::new(Vec::new());
static NEXT_ERROR: AtomicU64 = AtomicU64::new(1);
fn status(code: u32) -> abi::Status {
    abi::Status {
        code,
        ..Default::default()
    }
}
fn translated(scope: u64, code: i32) -> abi::Status {
    let text = LAST_ERROR.lock().expect("error slot").take();
    if code == 0 {
        return status(abi::status_code::OK);
    }
    if code == -6 {
        return status(abi::status_code::BUFFER_TOO_SMALL);
    }
    let mut result = status(abi::status_code::FAILED);
    if let Some(text) = text {
        let token = NEXT_ERROR.fetch_add(1, Ordering::Relaxed);
        let mut rows = ERRORS.lock().expect("error rows");
        if let Some(row) = rows.iter_mut().find(|row| row.scope == scope) {
            row.token = token;
            row.text = Some(text);
        } else {
            rows.push(ErrorRow {
                scope,
                token,
                text: Some(text),
            });
        }
        result.error = token;
    }
    result
}
fn guarded(call: impl FnOnce() -> abi::Status) -> abi::Status {
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(call)) {
        Ok(result) => result,
        Err(_) => status(abi::status_code::PANICKED),
    }
}
fn aligned<T>(ptr: *const T) -> bool {
    !ptr.is_null() && (ptr as usize).is_multiple_of(std::mem::align_of::<T>())
}
fn valid_span(ptr: *const u8, len: u64) -> bool {
    len <= isize::MAX as u64
        && (len == 0 || !ptr.is_null())
        && (ptr as usize).checked_add(len as usize).is_some()
}
unsafe extern "C" fn stable_create(
    host: *const abi::HostApiV1,
    context: abi::ContextHandle,
    params: abi::Bytes,
    output: *mut abi::NodeHandle,
) -> abi::Status {
    guarded(|| {
        if !aligned(output) || !valid_span(params.ptr, params.len) {
            return status(abi::status_code::INVALID_ARGUMENT);
        }
        // SAFETY: aligned exclusive caller output lives for the whole call.
        unsafe {
            output.write(abi::NodeHandle::default());
        }
        // SAFETY: caller retains the checked host table/context for the instance;
        // the maintained client validates identity before invoking a callback.
        if unsafe { cerulion_core::graph::node_plugin_abi::HostClient::connect(host, context, 0) }
            .is_err()
        {
            return status(abi::status_code::INVALID_ARGUMENT);
        }
        let value = create_local();
        // SAFETY: same exclusive output, after local node-owned allocation.
        unsafe {
            output.write(abi::NodeHandle { value });
        }
        status(abi::status_code::OK)
    })
}
unsafe extern "C" fn stable_tick(handle: abi::NodeHandle) -> abi::Status {
    guarded(|| {
        *LAST_ERROR.lock().unwrap() = None;
        translated(handle.value, cerulion_node_tick(handle.value))
    })
}
unsafe extern "C" fn stable_shutdown(handle: abi::NodeHandle) -> abi::Status {
    guarded(|| {
        if NODES
            .lock()
            .unwrap()
            .as_ref()
            .is_some_and(|rows| rows.contains_key(&handle.value))
        {
            status(abi::status_code::OK)
        } else {
            status(abi::status_code::INVALID_HANDLE)
        }
    })
}
unsafe extern "C" fn stable_destroy(handle: abi::NodeHandle) -> abi::Status {
    guarded(|| {
        let code = cerulion_node_shutdown(handle.value);
        ERRORS
            .lock()
            .unwrap()
            .retain(|row| row.scope != handle.value);
        if code == 0 {
            status(abi::status_code::OK)
        } else {
            status(abi::status_code::INVALID_HANDLE)
        }
    })
}
unsafe extern "C" fn stable_shape(output: *mut u64) -> abi::Status {
    guarded(|| {
        if !aligned(output) {
            return status(abi::status_code::INVALID_ARGUMENT);
        }
        *LAST_ERROR.lock().unwrap() = None;
        // SAFETY: caller owns the checked aligned scalar through this query.
        translated(0, unsafe { cerulion_node_state_shape(output) })
    })
}
struct SinkBridge(abi::StateSinkV1);
extern "C" fn forward_sink(context: *mut std::ffi::c_void, bytes: *const u8, len: usize) -> i32 {
    // SAFETY: capture passes a live exclusive node-local bridge; the encoder's
    // initialized chunk and host sink context remain borrowed synchronously.
    let result = unsafe {
        let bridge = &*context.cast::<SinkBridge>();
        (bridge.0.write)(
            bridge.0.context,
            abi::Bytes {
                ptr: bytes,
                len: len as u64,
            },
        )
    };
    if result.code == abi::status_code::OK {
        0
    } else {
        -6
    }
}
unsafe extern "C" fn stable_capture(
    handle: abi::NodeHandle,
    sink: *const abi::StateSinkV1,
    capacity: u64,
) -> abi::Status {
    guarded(|| {
        if !aligned(sink) {
            return status(abi::status_code::INVALID_ARGUMENT);
        }
        // SAFETY: caller retains initialized concrete sink, callback and context
        // through the synchronous capture; the copied record owns no allocation.
        let mut bridge = SinkBridge(unsafe { sink.read() });
        *LAST_ERROR.lock().unwrap() = None;
        translated(
            handle.value,
            cerulion_node_capture_state(
                handle.value,
                Some(forward_sink),
                (&mut bridge as *mut SinkBridge).cast(),
                capacity,
            ),
        )
    })
}
unsafe extern "C" fn stable_restore(handle: abi::NodeHandle, payload: abi::Bytes) -> abi::Status {
    guarded(|| {
        if !valid_span(payload.ptr, payload.len) {
            return status(abi::status_code::INVALID_ARGUMENT);
        }
        *LAST_ERROR.lock().unwrap() = None;
        // SAFETY: caller retains initialized bounded payload bytes throughout
        // restore; the local decoder copies only its counter into node state.
        translated(handle.value, unsafe {
            cerulion_node_restore_state(handle.value, payload.ptr, payload.len as usize)
        })
    })
}
unsafe extern "C" fn stable_inline(handle: abi::NodeHandle, output: *mut u32) -> abi::Status {
    if !aligned(output) {
        return status(abi::status_code::INVALID_ARGUMENT);
    }
    // SAFETY: checked exclusive scalar is live through this query.
    unsafe {
        output.write(1);
    }
    // SAFETY: only a scalar handle crosses; shutdown checks membership without
    // dropping state, and guarded contains every local registry panic.
    unsafe { stable_shutdown(handle) }
}
unsafe extern "C" fn stable_probe(handle: abi::NodeHandle) -> abi::Status {
    // SAFETY: scalar membership query retains every local owner.
    unsafe { stable_shutdown(handle) }
}
unsafe extern "C" fn stable_error(
    token: u64,
    output: abi::MutableBytes,
    required: *mut u64,
) -> abi::Status {
    guarded(|| {
        if !aligned(required) || !valid_span(output.ptr, output.len) {
            return status(abi::status_code::INVALID_ARGUMENT);
        }
        let mut rows = ERRORS.lock().unwrap();
        let Some(row) = rows
            .iter_mut()
            .find(|row| row.token == token && row.text.is_some())
        else {
            return status(abi::status_code::INVALID_HANDLE);
        };
        let text = row.text.as_ref().unwrap();
        // SAFETY: caller owns nonoverlapping output and length scalar, lending
        // bounded writable storage. Text remains node-owned until complete copy.
        unsafe {
            required.write(text.len() as u64);
            if output.len < text.len() as u64 {
                return status(abi::status_code::BUFFER_TOO_SMALL);
            }
            if !text.is_empty() {
                std::ptr::copy_nonoverlapping(text.as_ptr(), output.ptr, text.len());
            }
        }
        row.text = None;
        status(abi::status_code::OK)
    })
}
unsafe extern "C" fn stable_extension(
    _: u64,
    _: u32,
    _: *mut abi::ExtensionDescriptorV1,
) -> abi::Status {
    status(abi::status_code::UNSUPPORTED)
}
static STATE: abi::StateApiV1 = abi::StateApiV1 {
    header: abi::TableHeaderV1::for_type::<abi::StateApiV1>(),
    shape: stable_shape,
    capture: stable_capture,
    restore: stable_restore,
    inline_safe: stable_inline,
    probe: stable_probe,
};
static API: abi::NodeApiV1 = abi::NodeApiV1 {
    header: abi::TableHeaderV1::for_type::<abi::NodeApiV1>(),
    metadata: abi::Bytes {
        ptr: INFO_BYTES.as_ptr(),
        len: INFO_BYTES.len() as u64 - 1,
    },
    required_host_capabilities: 0,
    capabilities: abi::capability::STATE,
    create: stable_create,
    tick: stable_tick,
    shutdown: stable_shutdown,
    destroy: stable_destroy,
    copy_error: stable_error,
    query_extension: stable_extension,
    state: &STATE,
    external: std::ptr::null(),
};
fn stable_api() -> *const abi::NodeApiV1 {
    &API
}
cerulion_core::node_abi::export_node_api!(stable_api);
