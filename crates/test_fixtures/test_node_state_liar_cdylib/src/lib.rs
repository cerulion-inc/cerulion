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

#[no_mangle]
pub extern "C" fn cerulion_node_init(ctx_ptr: *mut u8) -> u64 {
    // SAFETY: `ctx_ptr` came from `Box::into_raw(Box::new(NodeContext))` in the
    // host's `DylibNodeEntry::init`; reclaim it so each init does not leak one
    // context.
    let ctx = unsafe { Box::from_raw(ctx_ptr as *mut cerulion_core::graph::node::NodeContext) };
    drop(ctx);

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
