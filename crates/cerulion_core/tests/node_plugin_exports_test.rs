// SPDX-License-Identifier: AGPL-3.0-only
//! Checked bootstrap, metadata and refusal oracles for a real generated library.
//!
//! Prerequisite: `cargo build -p test_node_macro_cdylib`.
//! No transport/session is constructed; this binary is parallel-safe.

use std::mem::align_of;

use cerulion_core::node_abi::stabby::IStable;
use cerulion_core::node_abi::*;
use libloading::Library;

fn library() -> Library {
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_macro_cdylib");
    // SAFETY (U01): trusted compiled fixture; every table/symbol borrow stays
    // inside its owning Library lifetime and no node instance is created.
    unsafe { Library::new(path) }.expect("build the generated macro fixture first")
}

fn api(library: &Library) -> &NodeApiV1 {
    type Report = extern "C" fn() -> &'static stabby::abi::report::TypeReport;
    // SAFETY (U03): trusted fixture exports the pinned report/BootstrapFn
    // signatures through export_node_api; the Library owns their code/data.
    let report =
        unsafe { library.get::<Report>(b"cerulion_node_api_v1_stabbied_v3_report\0") }.unwrap();
    assert!(bootstrap_matches(report()));
    assert_eq!(stabby::abi::report::gen_id(report()), BootstrapFn::ID);
    // SAFETY (U03): complete bootstrap report identity checked first; code is
    // retained by the caller's Library borrow throughout every invocation.
    let bootstrap = unsafe { library.get::<BootstrapFn>(BOOTSTRAP_SYMBOL) }.unwrap();
    let pointer = bootstrap();
    assert!(!pointer.is_null());
    assert!(pointer.addr().is_multiple_of(align_of::<NodeApiV1>()));
    // SAFETY (U04): trusted macro fixture supplies a live immutable fixed
    // header; validate complete size, major and report ID before the full read.
    let header = unsafe { pointer.cast::<TableHeaderV1>().read() };
    validate_header::<NodeApiV1>(&header).unwrap();
    // SAFETY (U04): full checked shape and immutable library-lifetime backing.
    let api = unsafe { &*pointer };
    validate_extensions(api).unwrap();
    api
}

#[test]
fn real_generated_bootstrap_has_complete_identity_and_stable_storage() {
    let library = library();
    let first = api(&library);
    let second = api(&library);
    assert!(std::ptr::eq(first, second));
    assert_eq!(first.required_host_capabilities, 0);
    assert_eq!(first.capabilities, capability::STATE);
    assert!(!first.state.is_null());
    assert!(first.external.is_null());
}

#[test]
fn both_port_directions_emit_checked_wire_facts_and_valid_length_metadata() {
    let library = library();
    let api = api(&library);
    assert!(!api.metadata.ptr.is_null());
    let length = usize::try_from(api.metadata.len).unwrap();
    assert!(length <= isize::MAX as usize);
    // SAFETY (U12): checked metadata span is immutable fixture-owned storage,
    // kept live by Library; no byte is retained after that owner drops.
    let bytes = unsafe { std::slice::from_raw_parts(api.metadata.ptr, length) };
    assert!(!bytes.contains(&0));
    let value: serde_json::Value = serde_json::from_slice(bytes).unwrap();
    for (direction, name) in [("inputs", "velocity_in"), ("outputs", "cmd_out")] {
        let ports = value[direction].as_array().unwrap();
        assert_eq!(ports.len(), 1);
        assert_eq!(ports[0]["name"], name);
        assert_eq!(ports[0]["wire_fixed_size"], 24);
        assert_eq!(ports[0]["variable_count"], 0);
        assert_eq!(ports[0]["alignment"], 8);
        use cerulion_core::message::ShmMessage;
        assert_eq!(
            ports[0]["schema_hash"],
            native_ros2_messages::geometry_msgs::Vector3::SCHEMA_HASH
        );
    }
}

#[test]
fn generated_callbacks_refuse_missing_handles_and_invalid_host_before_creation() {
    let library = library();
    let api = api(&library);
    let invalid = NodeHandle { value: 0 };
    let mut output = NodeHandle { value: 999 };
    // SAFETY (U06): callback table is checked/library-live; invalid handles
    // and null host are deliberately refused, and output is live/aligned.
    unsafe {
        assert_eq!((api.tick)(invalid).code, status_code::INVALID_HANDLE);
        assert_eq!((api.shutdown)(invalid).code, status_code::INVALID_HANDLE);
        assert_eq!((api.destroy)(invalid).code, status_code::INVALID_HANDLE);
        assert_eq!(
            (api.create)(
                std::ptr::null(),
                ContextHandle { value: 1 },
                Bytes {
                    ptr: std::ptr::null(),
                    len: 0
                },
                &mut output
            )
            .code,
            status_code::INVALID_ARGUMENT
        );
        assert_eq!(
            (api.query_extension)(u64::MAX, 1, std::ptr::null_mut()).code,
            status_code::UNSUPPORTED
        );
    }
    assert_eq!(output.value, 0);
}
