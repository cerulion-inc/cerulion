// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, and the two exports exist only where
// the headers declare `rmw_service_endpoint_info_array_t` (Lyrical and
// later), so on any other build this file compiles to nothing.
#![cfg(all(unix, cerulion_has_service_endpoint_info))]
//! `rmw_get_clients_info_by_service` / `rmw_get_servers_info_by_service`
//! through the extern "C" surface: every argument check in
//! `rmw/get_service_endpoint_info.h` returns its documented code, and
//! valid arguments return `RMW_RET_UNSUPPORTED` with an rcutils error
//! string naming the call, the out array left zero-initialized.
//!
//! The library on Jazzy, Humble, Galactic and Foxy defining neither
//! symbol is checked by the `nm` audit in `tools/ci/rmw-distros/gate.sh`
//! (`absent_symbols`), whose rows `rmw_absent_export_table_test` derives.
//!
//! No transport: a zeroed `rmw_node_t` carrying this implementation's
//! identifier is all the two calls read.

use serial_test::serial;
use std::ffi::CString;
use std::os::raw::c_char;

use rmw_cerulion::ffi::{
    self, RMW_RET_INCORRECT_RMW_IMPLEMENTATION, RMW_RET_INVALID_ARGUMENT, RMW_RET_UNSUPPORTED,
};
use rmw_cerulion::{rmw_get_clients_info_by_service, rmw_get_servers_info_by_service};

type Query = unsafe extern "C" fn(
    *const ffi::rmw_node_t,
    *mut ffi::rcutils_allocator_t,
    *const c_char,
    bool,
    *mut ffi::rmw_service_endpoint_info_array_t,
) -> ffi::rmw_ret_t;

const CALLS: [(&str, Query); 2] = [
    (
        "rmw_get_clients_info_by_service",
        rmw_get_clients_info_by_service,
    ),
    (
        "rmw_get_servers_info_by_service",
        rmw_get_servers_info_by_service,
    ),
];

fn node_with_identifier(id: *const c_char) -> ffi::rmw_node_t {
    let mut node: ffi::rmw_node_t = unsafe { std::mem::zeroed() };
    node.implementation_identifier = id;
    node
}

fn zero_array() -> ffi::rmw_service_endpoint_info_array_t {
    unsafe { std::mem::zeroed() }
}

#[test]
#[serial]
fn valid_arguments_answer_unsupported_and_name_the_call() {
    let node = node_with_identifier(ffi::implementation_identifier_ptr());
    let name = CString::new("/add_two_ints").unwrap();
    for (call_name, call) in CALLS {
        for no_mangle in [false, true] {
            let mut allocator = ffi::default_allocator();
            let mut out = zero_array();
            let asks_before = ffi::rcl_error_asks();
            let ret = unsafe { call(&node, &mut allocator, name.as_ptr(), no_mangle, &mut out) };
            assert_eq!(ret, RMW_RET_UNSUPPORTED, "{call_name}");
            assert_eq!(
                ffi::rcl_error_asks() - asks_before,
                1,
                "{call_name} must hand rcl one error string"
            );
            let text = ffi::last_rcl_error_text().expect("an error string was set");
            assert!(
                text.starts_with(call_name),
                "{call_name}: the error string must name the call, got {text:?}"
            );
            assert_eq!(
                out.size, 0,
                "{call_name} must leave the out array unchanged"
            );
            assert!(out.info_array.is_null(), "{call_name}");
        }
    }
}

#[test]
#[serial]
fn each_documented_argument_error_returns_its_code_before_unsupported() {
    let ours = node_with_identifier(ffi::implementation_identifier_ptr());
    let foreign_id = CString::new("rmw_fastrtps_cpp").unwrap();
    let foreign = node_with_identifier(foreign_id.as_ptr());
    let name = CString::new("/add_two_ints").unwrap();
    // A slot to point a non-zero-initialized array at; never read.
    let mut slot: ffi::rmw_service_endpoint_info_t = unsafe { std::mem::zeroed() };

    for (call_name, call) in CALLS {
        let asks_before = ffi::rcl_error_asks();
        let mut valid = ffi::default_allocator();
        let mut invalid: ffi::rcutils_allocator_t = unsafe { std::mem::zeroed() };
        let mut missing_zero_allocate = ffi::default_allocator();
        missing_zero_allocate.zero_allocate = None;
        let mut out = zero_array();
        let mut sized = zero_array();
        sized.size = 1;
        let mut pointed = zero_array();
        pointed.info_array = &mut slot;

        let cases: [(&str, ffi::rmw_ret_t); 9] = unsafe {
            [
                (
                    "null node",
                    call(std::ptr::null(), &mut valid, name.as_ptr(), false, &mut out),
                ),
                (
                    "null allocator",
                    call(&ours, std::ptr::null_mut(), name.as_ptr(), false, &mut out),
                ),
                (
                    "all-null allocator",
                    call(&ours, &mut invalid, name.as_ptr(), false, &mut out),
                ),
                (
                    "allocator without zero_allocate",
                    call(
                        &ours,
                        &mut missing_zero_allocate,
                        name.as_ptr(),
                        false,
                        &mut out,
                    ),
                ),
                (
                    "null service name",
                    call(&ours, &mut valid, std::ptr::null(), false, &mut out),
                ),
                (
                    "null out array",
                    call(
                        &ours,
                        &mut valid,
                        name.as_ptr(),
                        false,
                        std::ptr::null_mut(),
                    ),
                ),
                (
                    "out array with a size",
                    call(&ours, &mut valid, name.as_ptr(), false, &mut sized),
                ),
                (
                    "out array with storage",
                    call(&ours, &mut valid, name.as_ptr(), false, &mut pointed),
                ),
                (
                    "another implementation's node",
                    call(&foreign, &mut valid, name.as_ptr(), false, &mut out),
                ),
            ]
        };
        for (case, ret) in cases {
            let want = if case == "another implementation's node" {
                RMW_RET_INCORRECT_RMW_IMPLEMENTATION
            } else {
                RMW_RET_INVALID_ARGUMENT
            };
            assert_eq!(ret, want, "{call_name}: {case}");
        }
        assert_eq!(
            ffi::rcl_error_asks(),
            asks_before,
            "{call_name}: an argument error must not report the unsupported string"
        );
        assert_eq!(
            sized.size, 1,
            "{call_name}: a refused array is left as it was"
        );
        assert!(std::ptr::eq(pointed.info_array, &slot), "{call_name}");
        assert_eq!(out.size, 0, "{call_name}");
        assert!(out.info_array.is_null(), "{call_name}");
    }
}
