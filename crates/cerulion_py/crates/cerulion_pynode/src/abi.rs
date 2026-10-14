//! The optional node ABI exports behind `export_node!`, as plain functions.
//!
//! `export_node!` expands in the node's own crate, where the `NODES` map and
//! the error slot live, so each `#[no_mangle]` wrapper there is a few lines
//! that call one function here with that map and the error setter. The wire
//! contract (signatures, return codes, out-parameter rules) is the one the
//! `#[cerulion_node]` macro emits for a Rust cdylib and the loader resolves
//! (`DylibNodeEntry`): the host cannot tell the two apart.
//!
//! Return codes, shared by every function (negative = failure; the host pulls
//! the error text through `cerulion_take_last_error`):
//!
//! * `0` success
//! * `-1` the `NODES` mutex is poisoned
//! * `-2` the handle is unknown
//! * `-3` a name was not valid UTF-8
//! * `-4` a panic was caught (set by the macro wrapper)
//! * `-5` an unknown Sync op code
//! * `-6` the Sync op answered `Failed`
//! * `-7` a null out-pointer
//! * `-8` a name pointer that cannot be read: null, or a length above
//!   `isize::MAX` (the slice bound `from_raw_parts` requires)

use std::collections::HashMap;
use std::sync::Mutex;

use cerulion_core::graph::node::{
    SYNC_HEAD_OP_ADVANCE, SYNC_HEAD_OP_PEEK_NEXT, SYNC_HEAD_OP_PROBE_NEXT, SYNC_HEAD_OP_VOID,
    SYNC_OP_ANSWER_HEAD, SYNC_OP_ANSWER_NOTHING, SYNC_OP_ANSWER_PRESENT, SYNC_OP_ANSWER_STAMP,
};
use cerulion_core::{SyncHeadOp, SyncOpAnswer};

use crate::Host;

/// The per-cdylib handle map `export_node!` declares.
pub type Nodes = Mutex<Option<HashMap<u64, Host>>>;

/// The per-cdylib error setter `export_node!` declares.
pub type SetError = fn(String);

/// Run `f` on the host behind `handle`, or report why it could not.
fn with_host<R>(
    nodes: &Nodes,
    set_error: SetError,
    export: &str,
    handle: u64,
    f: impl FnOnce(&mut Host) -> R,
) -> Result<R, i32> {
    let mut guard = nodes.lock().map_err(|_| {
        set_error(format!("{export}: NODES mutex poisoned"));
        -1
    })?;
    match guard.as_mut().and_then(|map| map.get_mut(&handle)) {
        Some(host) => Ok(f(host)),
        None => {
            set_error(format!("{export}: handle {handle} not found"));
            Err(-2)
        }
    }
}

/// Borrow the `len` bytes at `ptr` as a `&str`.
///
/// # Safety
///
/// `ptr` must point to `len` readable bytes that stay valid for the call.
unsafe fn name_from_raw<'a>(
    set_error: SetError,
    export: &str,
    ptr: *const u8,
    len: usize,
) -> Result<&'a str, i32> {
    if ptr.is_null() {
        set_error(format!("{export}: name pointer was null"));
        return Err(-8);
    }
    if len > isize::MAX as usize {
        set_error(format!("{export}: name length {len} exceeds isize::MAX"));
        return Err(-8);
    }
    // SAFETY: the caller's contract, restated above, plus the two checks just
    // made, which are the conditions `from_raw_parts` states for its pointer
    // and length.
    let bytes = unsafe { std::slice::from_raw_parts(ptr, len) };
    std::str::from_utf8(bytes).map_err(|_| {
        set_error(format!("{export}: name was not valid UTF-8"));
        -3
    })
}

/// `cerulion_node_set_snapshot_inputs(handle, names_ptr, names_len)`: store
/// the `\n`-joined non-trigger input names once. An empty string is an empty
/// set, not one empty name.
///
/// # Safety
///
/// `names_ptr` must point to `names_len` readable bytes valid for the call,
/// or `names_len` must be zero.
pub unsafe fn set_snapshot_inputs(
    nodes: &Nodes,
    set_error: SetError,
    handle: u64,
    names_ptr: *const u8,
    names_len: usize,
) -> i32 {
    const EXPORT: &str = "cerulion_node_set_snapshot_inputs";
    let names: Vec<String> = if names_len == 0 {
        Vec::new()
    } else {
        // SAFETY: the caller's contract, restated above.
        match unsafe { name_from_raw(set_error, EXPORT, names_ptr, names_len) } {
            Ok(joined) => joined.split('\n').map(String::from).collect(),
            Err(code) => return code,
        }
    };
    match with_host(nodes, set_error, EXPORT, handle, |host| {
        host.set_snapshot_inputs(names)
    }) {
        Ok(()) => 0,
        Err(code) => code,
    }
}

/// `cerulion_node_snapshot_inputs(handle)`: freeze the stored inputs for this
/// step.
pub fn snapshot_inputs(nodes: &Nodes, set_error: SetError, handle: u64) -> i32 {
    match with_host(
        nodes,
        set_error,
        "cerulion_node_snapshot_inputs",
        handle,
        Host::snapshot_inputs,
    ) {
        Ok(()) => 0,
        Err(code) => code,
    }
}

/// Which trigger drain a `(popped, latest_ts)` export runs.
#[derive(Clone, Copy)]
pub enum TriggerDrain {
    /// `cerulion_node_drain_trigger_input`: the level-boundary drain.
    Boundary,
    /// `cerulion_node_refill_trigger_input`: the between-fires refill.
    Refill,
}

/// `cerulion_node_{drain,refill}_trigger_input(handle, name_ptr, name_len,
/// out_popped, out_latest_ts, out_has_ts)`: run one trigger drain and report
/// how many frames it popped and the latest wire stamp, if any. The
/// out-parameters are written only on success.
///
/// # Safety
///
/// `name_ptr` must point to `name_len` readable bytes valid for the call, and
/// the three out-pointers must be writable for the call.
#[expect(
    clippy::too_many_arguments,
    reason = "one parameter per slot of the C ABI signature the loader resolves"
)]
pub unsafe fn trigger_drain(
    nodes: &Nodes,
    set_error: SetError,
    drain: TriggerDrain,
    handle: u64,
    name_ptr: *const u8,
    name_len: usize,
    out_popped: *mut u64,
    out_latest_ts: *mut u64,
    out_has_ts: *mut i32,
) -> i32 {
    let export = match drain {
        TriggerDrain::Boundary => "cerulion_node_drain_trigger_input",
        TriggerDrain::Refill => "cerulion_node_refill_trigger_input",
    };
    if out_popped.is_null() || out_latest_ts.is_null() || out_has_ts.is_null() {
        set_error(format!("{export}: an out-pointer was null"));
        return -7;
    }
    // SAFETY: the caller's contract, restated above.
    let name = match unsafe { name_from_raw(set_error, export, name_ptr, name_len) } {
        Ok(name) => name,
        Err(code) => return code,
    };
    let outcome = with_host(nodes, set_error, export, handle, |host| match drain {
        TriggerDrain::Boundary => host.drain_trigger_input(name),
        TriggerDrain::Refill => host.refill_trigger_input(name),
    });
    match outcome {
        Ok((popped, latest_ts)) => {
            // SAFETY: the out-pointers were null-checked above and are writable
            // for this call by the caller's contract.
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
        Err(code) => code,
    }
}

/// `cerulion_node_sync_head_op(handle, name_ptr, name_len, op, out_ts,
/// out_kind)`: one Sync head operation on an input. The two FILL ops ride the
/// trigger drain exports; this one carries ProbeNext, PeekNext, Advance and
/// Void. The out-parameters are written only on success.
///
/// # Safety
///
/// `name_ptr` must point to `name_len` readable bytes valid for the call, and
/// the two out-pointers must be writable for the call.
#[expect(
    clippy::too_many_arguments,
    reason = "one parameter per slot of the C ABI signature the loader resolves"
)]
pub unsafe fn sync_head_op(
    nodes: &Nodes,
    set_error: SetError,
    handle: u64,
    name_ptr: *const u8,
    name_len: usize,
    op: u32,
    out_ts: *mut u64,
    out_kind: *mut i32,
) -> i32 {
    const EXPORT: &str = "cerulion_node_sync_head_op";
    // The same ladder as the generated `#[cerulion_node]` export
    // (`cerulion_macros` codegen): name (-8 / -3), then op (-5), then the
    // out-pointers (-7), so a caller mixing several mistakes gets the same
    // code from a Python cdylib as from a Rust one.
    // SAFETY: the caller's contract, restated above.
    let name = match unsafe { name_from_raw(set_error, EXPORT, name_ptr, name_len) } {
        Ok(name) => name,
        Err(code) => return code,
    };
    let head_op = match op {
        SYNC_HEAD_OP_PROBE_NEXT => SyncHeadOp::ProbeNext,
        SYNC_HEAD_OP_PEEK_NEXT => SyncHeadOp::PeekNext,
        SYNC_HEAD_OP_ADVANCE => SyncHeadOp::Advance,
        SYNC_HEAD_OP_VOID => SyncHeadOp::Void,
        other => {
            set_error(format!(
                "{EXPORT}: unknown op code {other} (this ABI carries 0=ProbeNext, 1=PeekNext, \
                 2=Advance, 3=Void; the two FILL ops ride cerulion_node_drain_trigger_input / \
                 cerulion_node_refill_trigger_input)"
            ));
            return -5;
        }
    };
    if out_ts.is_null() || out_kind.is_null() {
        set_error(format!("{EXPORT}: out_ts or out_kind pointer was null"));
        return -7;
    }
    let answer = match with_host(nodes, set_error, EXPORT, handle, |host| {
        host.sync_head_op(name, head_op)
    }) {
        Ok(answer) => answer,
        Err(code) => return code,
    };
    let (kind, ts) = match answer {
        SyncOpAnswer::Nothing => (SYNC_OP_ANSWER_NOTHING, 0),
        SyncOpAnswer::Present => (SYNC_OP_ANSWER_PRESENT, 0),
        SyncOpAnswer::Head(ts) => (SYNC_OP_ANSWER_HEAD, ts),
        SyncOpAnswer::Stamp(ts) => (SYNC_OP_ANSWER_STAMP, ts),
        SyncOpAnswer::Failed => {
            set_error(format!("{EXPORT}: the op answered Failed"));
            return -6;
        }
    };
    // SAFETY: the out-pointers were null-checked above and are writable for
    // this call by the caller's contract.
    unsafe {
        *out_ts = ts;
        *out_kind = kind;
    }
    0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn no_error(_: String) {}

    #[test]
    fn unknown_handle_and_poisoned_map_report_their_codes() {
        let nodes: Nodes = Mutex::new(None);
        assert_eq!(snapshot_inputs(&nodes, no_error, 7), -2);
        let mut out = (0_u64, 0_u64, 0_i32);
        let code = unsafe {
            trigger_drain(
                &nodes,
                no_error,
                TriggerDrain::Boundary,
                7,
                b"inp".as_ptr(),
                3,
                &mut out.0,
                &mut out.1,
                &mut out.2,
            )
        };
        assert_eq!(code, -2);
        assert_eq!(out, (0, 0, 0), "out-parameters are written only on success");
    }

    #[test]
    fn malformed_arguments_are_refused_before_the_map_is_touched() {
        let nodes: Nodes = Mutex::new(None);
        let mut ts = 0_u64;
        let mut kind = -1_i32;
        let code = unsafe {
            sync_head_op(
                &nodes,
                no_error,
                1,
                b"inp".as_ptr(),
                3,
                99,
                &mut ts,
                &mut kind,
            )
        };
        assert_eq!(code, -5, "an unknown op code");
        let code = unsafe {
            sync_head_op(
                &nodes,
                no_error,
                1,
                b"inp".as_ptr(),
                3,
                SYNC_HEAD_OP_PROBE_NEXT,
                std::ptr::null_mut(),
                &mut kind,
            )
        };
        assert_eq!(code, -7, "a null out-pointer");
        let code = unsafe {
            sync_head_op(
                &nodes,
                no_error,
                1,
                std::ptr::null(),
                3,
                SYNC_HEAD_OP_PROBE_NEXT,
                &mut ts,
                &mut kind,
            )
        };
        assert_eq!(code, -8, "a null name pointer");
        let code = unsafe {
            sync_head_op(
                &nodes,
                no_error,
                1,
                b"inp".as_ptr(),
                isize::MAX as usize + 1,
                SYNC_HEAD_OP_PROBE_NEXT,
                &mut ts,
                &mut kind,
            )
        };
        assert_eq!(code, -8, "a length no slice can have");
        // Several mistakes at once answer in the generated export's order:
        // the name first, then the op code, then the out-pointers.
        let code = unsafe {
            sync_head_op(
                &nodes,
                no_error,
                1,
                std::ptr::null(),
                3,
                99,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
            )
        };
        assert_eq!(
            code, -8,
            "a null name outranks a bad op and null out-pointers"
        );
        let code = unsafe {
            sync_head_op(
                &nodes,
                no_error,
                1,
                b"inp".as_ptr(),
                3,
                99,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
            )
        };
        assert_eq!(code, -5, "a bad op outranks null out-pointers");
        let bad_utf8 = [0xff_u8, 0xfe];
        let code =
            unsafe { set_snapshot_inputs(&nodes, no_error, 1, bad_utf8.as_ptr(), bad_utf8.len()) };
        assert_eq!(code, -3, "names that are not UTF-8");
    }
}
