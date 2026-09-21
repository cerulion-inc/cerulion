// SPDX-License-Identifier: AGPL-3.0-only
//! The ARGUMENT GUARDS of `cerulion_node_sync_head_op`, exercised by
//! RAW libloading — the contract every FFI consumer sees, not just the loader
//! Cerulion ships.
//!
//! # Why raw libloading, and why these codes at all
//!
//! `DylibNodeEntry::sync_head_op` always passes `input_name.as_ptr()` (never
//! null) and always passes `&mut` slots of its own stack, so NONE of the
//! argument-guard codes are reachable through the loader in this repo. That is
//! exactly the `chunk_c_ffi_codes_3_4_test.rs` situation: the cdylib FFI
//! surface is PUBLIC, a future raw consumer (a custom loader, a language
//! binding, a hand-written host) can pass anything, and a guard nothing
//! exercises is a guard nobody knows is missing.
//!
//! The null-name guard is the sharpest of them. `slice::from_raw_parts` requires a NON-NULL
//! pointer EVEN AT LENGTH ZERO, and a body that calls it on `name_ptr`
//! BEFORE any check makes a null name undefined behaviour outright: a debug
//! build ABORTS on the precondition, and a release build compiles that check
//! out and reads from address 0. So the export validates
//! first, then answers with a dedicated code (-8).
//!
//! # The ladder this file pins
//!
//! The guards run in a FIXED order and each has its OWN code, because they are
//! different caller bugs with different fixes:
//!
//! ```text
//!   -8  null name pointer      (checked FIRST — it is the only deref)
//!   -3  name not valid UTF-8
//!   -5  unknown op code
//!   -7  null out-param
//!   -1  NODES mutex poisoned
//!   -2  handle not found
//! ```
//!
//! Every arm below that asserts -8 is paired IN THE SAME BODY with an arm that
//! must reach a LATER rung. Without those, "a null name is refused" is
//! satisfied by a function that refuses everything, and the guard would look
//! identical to a regression that broke the export outright.
//!
//! `#[serial]`: the cdylib's `NODES` static is process-global (`dlopen`
//! refcounts, so every `Library::new` of one path in one process shares it).
//! Nothing here poisons it, but the convention is cheap and the file may grow.

use std::ffi::CStr;

use serial_test::serial;

/// The four op codes this ABI carries, read from the SHARED consts rather than
/// re-spelled: the emitting side reaches them by path too, and a test that
/// hardcoded `0` would keep passing across a renumbering that broke the wire.
use cerulion_core::graph::node::{SYNC_HEAD_OP_PROBE_NEXT, SYNC_OP_ANSWER_NOTHING};

type SyncHeadOpFn = unsafe extern "C" fn(u64, *const u8, usize, u32, *mut u64, *mut i32) -> i32;

/// Read + free `LAST_ERROR` through the cdylib's own free function.
unsafe fn take_last_error(
    take: &libloading::Symbol<'_, unsafe extern "C" fn() -> *mut std::ffi::c_char>,
    free: &libloading::Symbol<'_, unsafe extern "C" fn(*mut std::ffi::c_char)>,
    context: &str,
) -> String {
    let raw = unsafe { take() };
    assert!(
        !raw.is_null(),
        "cerulion_take_last_error must return non-null in {context}"
    );
    let msg = unsafe { CStr::from_ptr(raw) }
        .to_string_lossy()
        .into_owned();
    unsafe { free(raw) };
    msg
}

/// A null `name_ptr` is REFUSED with its own code, and the refusal does not
/// swallow the rungs below it.
///
/// ONE body, because the arms are a LADDER: each later arm's value is that it
/// proves the earlier arm's guard did not simply eat the call. Splitting them
/// would leave every "-8" assertion satisfiable by an export that returned -8
/// unconditionally.
///
/// Reverting the guard to the bare `from_raw_parts` makes phase 1
/// SIGABRT this binary on a debug build (`unsafe precondition(s) violated:
/// slice::from_raw_parts requires the pointer to be aligned and non-null`) —
/// which is also the proof it was real UB rather than a tidied style point.
#[test]
#[serial]
fn a_null_name_pointer_is_refused_with_its_own_code_and_the_ladder_below_still_runs() {
    let path =
        cerulion_core::testing::find_fixture_cdylib("test_node_macro_sync_nontrigger_cdylib");
    let lib = unsafe { libloading::Library::new(&path) }.expect("load the sync cdylib fixture");

    let sync_head_op: libloading::Symbol<SyncHeadOpFn> =
        unsafe { lib.get(b"cerulion_node_sync_head_op") }
            .expect("a macro cdylib exports cerulion_node_sync_head_op (symbol IS the capability)");
    let take_last_error_fn: libloading::Symbol<unsafe extern "C" fn() -> *mut std::ffi::c_char> =
        unsafe { lib.get(b"cerulion_take_last_error") }.expect("cerulion_take_last_error export");
    let free_error_fn: libloading::Symbol<unsafe extern "C" fn(*mut std::ffi::c_char)> =
        unsafe { lib.get(b"cerulion_free_error") }.expect("cerulion_free_error export");

    // A handle nothing registered. Every phase below uses it deliberately: the
    // guards under test all run BEFORE the handle lookup, so a phase that
    // reaches -2 has provably passed every one of them.
    const STALE_HANDLE: u64 = 99_999;

    // ---- PHASE 1: null name, length 0 — the exact UB shape ----------------
    let mut out_ts: u64 = 0;
    let mut out_kind: i32 = -1;
    let code = unsafe {
        sync_head_op(
            STALE_HANDLE,
            std::ptr::null(),
            0,
            SYNC_HEAD_OP_PROBE_NEXT,
            &mut out_ts,
            &mut out_kind,
        )
    };
    assert_eq!(
        code, -8,
        "a null name pointer must be REFUSED with its own code before anything \
         dereferences it — `from_raw_parts` requires non-null even at len 0"
    );
    let msg = unsafe { take_last_error(&take_last_error_fn, &free_error_fn, "phase 1") };
    assert!(
        msg.contains("name_ptr was null"),
        "the refusal must NAME the offending argument so a raw consumer can fix \
         it without reading our source; got {msg:?}"
    );
    assert_eq!(
        out_kind, -1,
        "a refused call writes NO answer kind — the host reads a nonzero return \
         as `Failed` and must never see a fabricated answer beside it"
    );

    // ---- PHASE 2: null name, NON-ZERO length ------------------------------
    // A caller that also lied about the length. The length is not consulted:
    // the pointer is what cannot be dereferenced.
    let code = unsafe {
        sync_head_op(
            STALE_HANDLE,
            std::ptr::null(),
            7,
            SYNC_HEAD_OP_PROBE_NEXT,
            &mut out_ts,
            &mut out_kind,
        )
    };
    assert_eq!(
        code, -8,
        "the refusal is on the POINTER, so a nonzero length cannot buy a deref"
    );
    let _ = unsafe { take_last_error(&take_last_error_fn, &free_error_fn, "phase 2") };

    // ---- PHASE 2b: a real pointer, an IMPOSSIBLE length -------------------
    // The other half of the same `from_raw_parts` precondition: the slice's
    // total size must fit in `isize`. Same code, because it is the same caller
    // mistake — the name argument is not a readable slice.
    let name = "a";
    let code = unsafe {
        sync_head_op(
            STALE_HANDLE,
            name.as_ptr(),
            usize::MAX,
            SYNC_HEAD_OP_PROBE_NEXT,
            &mut out_ts,
            &mut out_kind,
        )
    };
    assert_eq!(
        code, -8,
        "a length past `isize::MAX` is refused before the deref for the same \
         reason a null pointer is — it is the other half of one precondition"
    );
    let msg = unsafe { take_last_error(&take_last_error_fn, &free_error_fn, "phase 2b") };
    assert!(
        msg.contains("exceeds isize::MAX"),
        "and the message says WHICH half of the precondition failed; got {msg:?}"
    );

    // ---- PHASE 2c: a readable slice that is not UTF-8 ---------------------
    // The rung between the slice guards and the op-code match. Its bytes are a
    // valid slice, so the deref is sound; what fails is the DECODE.
    let bad_utf8: [u8; 3] = [0xff, 0xfe, 0xfd];
    let code = unsafe {
        sync_head_op(
            STALE_HANDLE,
            bad_utf8.as_ptr(),
            bad_utf8.len(),
            SYNC_HEAD_OP_PROBE_NEXT,
            &mut out_ts,
            &mut out_kind,
        )
    };
    assert_eq!(
        code, -3,
        "a readable but non-UTF-8 name is refused on its own terms — a port \
         name is a `&str`, and guessing at invalid bytes would look up a port \
         nobody declared"
    );
    let _ = unsafe { take_last_error(&take_last_error_fn, &free_error_fn, "phase 2c") };

    // ---- PHASE 3: a real name, a NULL out-param — a LATER rung ------------
    let mut out_kind_3: i32 = -1;
    let code = unsafe {
        sync_head_op(
            STALE_HANDLE,
            name.as_ptr(),
            name.len(),
            SYNC_HEAD_OP_PROBE_NEXT,
            std::ptr::null_mut(),
            &mut out_kind_3,
        )
    };
    assert_eq!(
        code, -7,
        "a null OUT-param keeps its own code: the two are different caller bugs \
         (the host forgot its own stack slots vs it has no port to ask about), \
         and collapsing them would make the diagnostic useless"
    );
    let msg = unsafe { take_last_error(&take_last_error_fn, &free_error_fn, "phase 3") };
    assert!(
        msg.contains("out_ts or out_kind pointer was null"),
        "the out-param refusal names ITS argument, not the name pointer; got {msg:?}"
    );

    // ---- PHASE 4: a real name, an UNKNOWN op code — a later rung still ----
    let mut out_ts_4: u64 = 0;
    let mut out_kind_4: i32 = -1;
    let code = unsafe {
        sync_head_op(
            STALE_HANDLE,
            name.as_ptr(),
            name.len(),
            99,
            &mut out_ts_4,
            &mut out_kind_4,
        )
    };
    assert_eq!(
        code, -5,
        "an unknown op code is still refused on its own terms — the name guard \
         must not short-circuit the op-code match"
    );

    // ---- PHASE 5: every argument VALID — the call reaches the handle -------
    // The anti-tautology arm for the whole file. Without it, each assertion
    // above is satisfied by an export that refuses unconditionally, and the
    // fix would be indistinguishable from having broken the symbol.
    let mut out_ts_5: u64 = 0;
    let mut out_kind_5: i32 = SYNC_OP_ANSWER_NOTHING;
    let code = unsafe {
        sync_head_op(
            STALE_HANDLE,
            name.as_ptr(),
            name.len(),
            SYNC_HEAD_OP_PROBE_NEXT,
            &mut out_ts_5,
            &mut out_kind_5,
        )
    };
    assert_eq!(
        code, -2,
        "a fully-valid call must pass EVERY argument guard and fail only at the \
         handle lookup — this is what proves the guards above refuse their own \
         condition rather than the call"
    );
    let msg = unsafe { take_last_error(&take_last_error_fn, &free_error_fn, "phase 5") };
    assert!(
        msg.contains("handle") && msg.contains("not found"),
        "the last rung reached must be the handle lookup; got {msg:?}"
    );
}
