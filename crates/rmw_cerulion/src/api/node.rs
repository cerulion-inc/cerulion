// SPDX-License-Identifier: AGPL-3.0-only
//! rmw node lifecycle.

use std::os::raw::c_char;

use crate::ffi::{self, rmw_ret_t, RMW_RET_ERROR, RMW_RET_INVALID_ARGUMENT, RMW_RET_OK};
use crate::runtime::{self, NodeData};

/// # Safety
/// rmw ABI contract: `context` valid + initialized; name/namespace are
/// valid C strings.
#[no_mangle]
pub unsafe extern "C" fn rmw_create_node(
    context: *mut ffi::rmw_context_t,
    name: *const c_char,
    namespace_: *const c_char,
) -> *mut ffi::rmw_node_t {
    ffi::ffi_guard(std::ptr::null_mut(), || unsafe {
        if context.is_null() {
            return std::ptr::null_mut();
        }
        if !ffi::is_our_identifier((*context).implementation_identifier) {
            return std::ptr::null_mut();
        }
        let (Some(name_s), Some(ns_s)) = (ffi::cstr(name), ffi::cstr(namespace_)) else {
            return std::ptr::null_mut();
        };
        let Ok(rt) = runtime::runtime() else {
            return std::ptr::null_mut();
        };

        // Graph guard condition for this node (rcl wires it into executors;
        // we trigger it on local graph changes).
        let graph_guard = super::create_guard_condition_raw();
        if graph_guard.is_null() {
            return std::ptr::null_mut();
        }

        let data = Box::new(NodeData {
            name: name_s.to_string(),
            namespace: ns_s.to_string(),
            graph_guard,
        });

        let node = Box::new(ffi::rmw_node_t {
            implementation_identifier: ffi::implementation_identifier_ptr(),
            data: Box::into_raw(data) as *mut std::os::raw::c_void,
            // rcl reads these as C STRINGS (rcl_node_get_name) — they
            // must be NUL-terminated owned buffers, exactly like topic
            // names (a raw String::as_ptr is NOT terminated).
            name: super::leak_ros_name(name_s),
            namespace_: super::leak_ros_name(ns_s),
            context,
        });
        let node_ptr = Box::into_raw(node);
        let data_ref = &*((*node_ptr).data as *const NodeData);
        tracing::info!(node = %data_ref.name, namespace = %data_ref.namespace, "node created");

        // Registration is the LAST step: register the
        // graph guard so registry mutations wake this node's executor,
        // announce the node, then notify.
        {
            let state = (*graph_guard).data as *const runtime::GuardConditionState;
            let mut guards = rt.graph_guards.lock().unwrap_or_else(|e| e.into_inner());
            // SAFETY: `state` is the guard-condition Box's data; it is
            // unregistered here before being freed in rmw_destroy_node.
            guards.push(runtime::GraphGuardPtr::new(state));
        }
        {
            let mut graph = rt.graph.write().unwrap_or_else(|e| e.into_inner());
            graph
                .nodes
                .insert((ns_s.to_string(), name_s.to_string()), ());
        }
        runtime::notify_graph_change();
        node_ptr
    })
}

/// # Safety
/// `node` must be a valid node created by this implementation.
#[no_mangle]
pub unsafe extern "C" fn rmw_destroy_node(node: *mut ffi::rmw_node_t) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if node.is_null() {
            return RMW_RET_INVALID_ARGUMENT;
        }
        if !ffi::is_our_identifier((*node).implementation_identifier) {
            return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
        }
        // Borrow-first: all fallible/registry work through
        // a borrow; frees are the final infallible block.
        let graph_guard = {
            let data = &*((*node).data as *const NodeData);
            if let Ok(rt) = runtime::runtime() {
                {
                    let mut graph = rt.graph.write().unwrap_or_else(|e| e.into_inner());
                    graph
                        .nodes
                        .remove(&(data.namespace.clone(), data.name.clone()));
                }
                // UNREGISTER the guard before destroying it —
                // notify_graph_change must never touch a freed
                // GuardConditionState.
                {
                    let state = (*data.graph_guard).data as *const runtime::GuardConditionState;
                    let mut guards = rt.graph_guards.lock().unwrap_or_else(|e| e.into_inner());
                    guards.retain(|g| !std::ptr::eq(g.as_ptr(), state));
                }
                runtime::notify_graph_change();
            }
            tracing::info!(node = %data.name, "node destroyed");
            data.graph_guard
        };
        super::destroy_guard_condition_raw(graph_guard);
        super::unleak_ros_name((*node).name);
        super::unleak_ros_name((*node).namespace_);
        drop(Box::from_raw((*node).data as *mut NodeData));
        drop(Box::from_raw(node));
        RMW_RET_OK
    })
}

/// # Safety
/// `node` must be a valid node created by this implementation.
#[no_mangle]
pub unsafe extern "C" fn rmw_node_get_graph_guard_condition(
    node: *const ffi::rmw_node_t,
) -> *const ffi::rmw_guard_condition_t {
    if node.is_null() || !ffi::is_our_identifier((*node).implementation_identifier) {
        return std::ptr::null();
    }
    let data = &*((*node).data as *const NodeData);
    data.graph_guard
}

/// rmw_node_t allocator helpers (impl-internal by contract; provided
/// for ABI completeness).
///
/// # Safety
/// Caller frees with `rmw_node_free`.
#[no_mangle]
pub unsafe extern "C" fn rmw_node_allocate() -> *mut ffi::rmw_node_t {
    Box::into_raw(Box::new(std::mem::zeroed::<ffi::rmw_node_t>()))
}

/// # Safety
/// `node` must come from `rmw_node_allocate`.
#[no_mangle]
pub unsafe extern "C" fn rmw_node_free(node: *mut ffi::rmw_node_t) {
    if !node.is_null() {
        drop(Box::from_raw(node));
    }
}

/// Liveliness assertion: SHM transport has no liveliness protocol; the
/// publisher existing IS liveliness. Accept and return OK (matches
/// rmw_iceoryx-class implementations).
///
/// # Safety
/// `publisher` must be a valid publisher.
#[no_mangle]
pub unsafe extern "C" fn rmw_publisher_assert_liveliness(
    publisher: *const ffi::rmw_publisher_t,
) -> rmw_ret_t {
    if publisher.is_null() {
        return RMW_RET_INVALID_ARGUMENT;
    }
    RMW_RET_OK
}

/// All acked: SHM delivery is synchronous at send; nothing to wait for.
///
/// # Safety
/// `publisher` must be a valid publisher.
#[no_mangle]
pub unsafe extern "C" fn rmw_publisher_wait_for_all_acked(
    publisher: *const ffi::rmw_publisher_t,
    _wait_timeout: ffi::rmw_time_t,
) -> rmw_ret_t {
    if publisher.is_null() {
        return RMW_RET_INVALID_ARGUMENT;
    }
    RMW_RET_OK
}

/// # Safety
/// rmw ABI contract (out-param must be valid).
#[no_mangle]
pub unsafe extern "C" fn rmw_convert_rcutils_ret_to_rmw_ret(
    rcutils_ret: ffi::rcutils_ret_t,
) -> rmw_ret_t {
    match rcutils_ret {
        0 => RMW_RET_OK,
        10 => ffi::RMW_RET_BAD_ALLOC,
        11 => RMW_RET_INVALID_ARGUMENT,
        _ => RMW_RET_ERROR,
    }
}
