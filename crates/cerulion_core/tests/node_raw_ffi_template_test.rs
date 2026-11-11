// SPDX-License-Identifier: AGPL-3.0-only
//! Build test_node_raw_ffi_template_cdylib first. Tests the actual compiled raw
//! template's checked interface, allocator ownership and diagnostic protocol.

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::node::{DylibNodeEntry, NodeContext, NodeEntry};
use cerulion_core::graph::node_host_abi::{self as host, HostContext, HOST_API};
use cerulion_core::node_abi::{self as abi, status_code};
use cerulion_core::ShutdownSignal;
use indexmap::IndexMap;
use std::collections::HashMap;
use std::sync::Arc;

fn library() -> abi::CheckedNodeLibrary {
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_raw_ffi_template_cdylib");
    // SAFETY: explicitly built trusted fixture implements the stable protocol;
    // this owner retains all immutable descriptors and callback code.
    unsafe { abi::CheckedNodeLibrary::load(&path, HOST_API.capabilities) }
        .expect("checked raw fixture")
}

fn admitted<R>(context: u64, call: impl FnOnce() -> R) -> R {
    host::with_context(context, HostContext::begin_node_call).unwrap();
    let result = call();
    host::with_context(context, HostContext::end_node_call).unwrap();
    result
}

struct Instance {
    library: abi::CheckedNodeLibrary,
    context: u64,
    handle: abi::NodeHandle,
}
impl Instance {
    fn new(clock: Arc<VirtualClock>) -> Self {
        let library = library();
        let environment = Arc::new(HashMap::from([("RUST_LOG".into(), "off".into())]));
        let native = NodeContext::with_runtime_env(
            IndexMap::new(),
            IndexMap::new(),
            clock,
            ShutdownSignal::new(),
            Arc::clone(&environment),
        );
        let context = host::register_context(
            HostContext::new(native, vec![], environment, 16).expect("host-owned context"),
        )
        .expect("register host context");
        let mut handle = abi::NodeHandle::default();
        // SAFETY: process-lifetime host table and registered host context remain
        // live; exclusive scalar output and empty parameter borrow last the call.
        let status = admitted(context, || unsafe {
            (library.api().create)(
                &HOST_API,
                abi::ContextHandle { value: context },
                abi::Bytes {
                    ptr: std::ptr::null(),
                    len: 0,
                },
                &mut handle,
            )
        });
        assert_eq!(status.code, status_code::OK);
        assert_ne!(handle.value, 0);
        Self {
            library,
            context,
            handle,
        }
    }
}
impl Drop for Instance {
    fn drop(&mut self) {
        // SAFETY: no concurrent calls or accessors exist; node allocator drops its
        // own instance while host services/library remain live through both calls.
        admitted(self.context, || unsafe {
            assert_eq!(
                (self.library.api().shutdown)(self.handle).code,
                status_code::OK
            );
            assert_eq!(
                (self.library.api().destroy)(self.handle).code,
                status_code::OK
            );
        });
        host::remove_context(self.context).expect("host allocator drops its context");
    }
}

#[test]
fn raw_template_loads_through_the_production_loader_or_names_local_refusal() {
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_raw_ffi_template_cdylib");
    match DylibNodeEntry::load(&path) {
        Ok(mut entry) => {
            assert!(entry
                .info()
                .expect("checked metadata")
                .input_names()
                .is_empty());
            let clock = Arc::new(VirtualClock::new());
            let env = Arc::new(HashMap::from([("RUST_LOG".into(), "off".into())]));
            let native = NodeContext::with_runtime_env(
                IndexMap::new(),
                IndexMap::new(),
                clock,
                ShutdownSignal::new(),
                env,
            );
            entry.init(native).expect("stable init");
            entry.tick().expect("real raw template callback");
            entry.shutdown().expect("stable shutdown");
        }
        Err(error) if cfg!(feature = "local-transport") => {
            assert!(error.to_string().contains("local-transport"));
        }
        Err(error) => panic!("stable raw fixture refused: {error}"),
    }
}

#[test]
fn raw_template_returns_the_host_context_to_its_allocator_after_node_destroy() {
    let clock = Arc::new(VirtualClock::new());
    assert_eq!(Arc::strong_count(&clock), 1);
    {
        let instance = Instance::new(Arc::clone(&clock));
        assert!(Arc::strong_count(&clock) > 1);
        // SAFETY: initialized instance and retained checked library/context;
        // exclusive test ownership prevents simultaneous node calls.
        admitted(instance.context, || unsafe {
            assert_eq!(
                (instance.library.api().tick)(instance.handle).code,
                status_code::OK
            );
        });
        assert!(Arc::strong_count(&clock) > 1);
    }
    assert_eq!(Arc::strong_count(&clock), 1);
}

#[test]
fn raw_template_instances_have_distinct_handles_and_isolated_teardown() {
    let first_clock = Arc::new(VirtualClock::new());
    let second_clock = Arc::new(VirtualClock::new());
    let first = Instance::new(Arc::clone(&first_clock));
    let second = Instance::new(Arc::clone(&second_clock));
    assert_ne!(first.handle, second.handle);
    drop(first);
    assert_eq!(Arc::strong_count(&first_clock), 1);
    assert!(Arc::strong_count(&second_clock) > 1);
    // SAFETY: only the distinct second instance remains, with its own retained
    // host context and code owner; destroying first cannot invalidate it.
    admitted(second.context, || unsafe {
        assert_eq!(
            (second.library.api().tick)(second.handle).code,
            status_code::OK
        );
    });
    drop(second);
    assert_eq!(Arc::strong_count(&second_clock), 1);
}

#[test]
fn invalid_create_arguments_reset_output_and_refuse_before_instance_creation() {
    let library = library();
    for (host_ptr, context, params, expected) in [
        (
            std::ptr::null(),
            1,
            abi::Bytes {
                ptr: std::ptr::null(),
                len: 0,
            },
            status_code::INVALID_ARGUMENT,
        ),
        (
            &HOST_API,
            0,
            abi::Bytes {
                ptr: std::ptr::null(),
                len: 0,
            },
            status_code::INVALID_ARGUMENT,
        ),
        (
            &HOST_API,
            1,
            abi::Bytes {
                ptr: std::ptr::null(),
                len: 1,
            },
            status_code::INVALID_ARGUMENT,
        ),
        (
            &HOST_API,
            1,
            abi::Bytes {
                ptr: std::ptr::null(),
                len: u64::MAX,
            },
            status_code::INVALID_ARGUMENT,
        ),
    ] {
        let mut output = abi::NodeHandle { value: 17 };
        // SAFETY: deliberately malformed pointers/spans are the documented
        // early-rejection cases; scalar output remains exclusive and live.
        let status = unsafe {
            (library.api().create)(
                host_ptr,
                abi::ContextHandle { value: context },
                params,
                &mut output,
            )
        };
        assert_eq!(status.code, expected);
        assert_eq!(output.value, 0);
    }
}

#[test]
fn unknown_and_destroyed_handles_are_never_executed_or_reused() {
    let library = library();
    // SAFETY: checked live callbacks receive invalid scalar handles; registry
    // rejects membership before accessing any instance allocation.
    unsafe {
        for value in [0, u64::MAX] {
            let handle = abi::NodeHandle { value };
            assert_eq!(
                (library.api().tick)(handle).code,
                status_code::INVALID_HANDLE
            );
            assert_eq!(
                (library.api().shutdown)(handle).code,
                status_code::INVALID_HANDLE
            );
            assert_eq!(
                (library.api().destroy)(handle).code,
                status_code::INVALID_HANDLE
            );
        }
    }
    let first = Instance::new(Arc::new(VirtualClock::new()));
    let retired = first.handle;
    drop(first);
    let second = Instance::new(Arc::new(VirtualClock::new()));
    assert_ne!(retired, second.handle);
    // SAFETY: retained library shares the fixture's registry; retired integer
    // must remain invalid even after another instance is allocated.
    unsafe {
        assert_eq!(
            (library.api().tick)(retired).code,
            status_code::INVALID_HANDLE
        );
    }
}

#[test]
fn short_error_buffers_retain_real_diagnostics_until_a_complete_copy() {
    let library = library();
    let mut output = abi::NodeHandle::default();
    // SAFETY: host table/output are valid, but the scalar context was never
    // minted. Actual frozen environment access refuses and becomes a node error.
    let failed = unsafe {
        (library.api().create)(
            &HOST_API,
            abi::ContextHandle { value: u64::MAX },
            abi::Bytes {
                ptr: std::ptr::null(),
                len: 0,
            },
            &mut output,
        )
    };
    assert_eq!(failed.code, status_code::FAILED);
    assert_ne!(failed.error, 0);
    assert_eq!(output.value, 0);
    let mut required = 0;
    // SAFETY: zero-sized query and exclusive size scalar obey copy_error's
    // protocol; no foreign allocator receives or frees a Rust owner.
    unsafe {
        assert_eq!(
            (library.api().copy_error)(
                failed.error,
                abi::MutableBytes {
                    ptr: std::ptr::null_mut(),
                    len: 0
                },
                &mut required
            )
            .code,
            status_code::BUFFER_TOO_SMALL
        );
    }
    assert!(required > 1);
    let mut short = [0u8; 1];
    // SAFETY: exclusive one-byte buffer cannot hold this real diagnostic; the
    // node must retain it rather than partially consuming the token.
    unsafe {
        assert_eq!(
            (library.api().copy_error)(
                failed.error,
                abi::MutableBytes {
                    ptr: short.as_mut_ptr(),
                    len: 1
                },
                &mut required
            )
            .code,
            status_code::BUFFER_TOO_SMALL
        );
    }
    let mut text = vec![0u8; required as usize];
    // SAFETY: exclusive initialized buffer has the returned required size and
    // remains live through synchronous copy; node frees only its owned text.
    unsafe {
        assert_eq!(
            (library.api().copy_error)(
                failed.error,
                abi::MutableBytes {
                    ptr: text.as_mut_ptr(),
                    len: text.len() as u64
                },
                &mut required
            )
            .code,
            status_code::OK
        );
    }
    assert!(std::str::from_utf8(&text)
        .expect("diagnostic UTF-8")
        .contains("host"));
    // SAFETY: the consumed scalar token remains invalid; no source allocation
    // is inspected or freed again when the copy is repeated.
    unsafe {
        assert_eq!(
            (library.api().copy_error)(
                failed.error,
                abi::MutableBytes {
                    ptr: std::ptr::null_mut(),
                    len: 0
                },
                &mut required
            )
            .code,
            status_code::INVALID_HANDLE
        );
    }
}

#[test]
#[serial_test::serial]
fn inline_metadata_is_checked_before_create_and_absence_is_conservative() {
    struct Guard;
    impl Drop for Guard {
        fn drop(&mut self) {
            std::env::remove_var("CER_INLINE_SAFE_METADATA");
        }
    }
    let _guard = Guard;
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_cdylib");
    for (value, expected) in [("absent", false), ("false", false), ("true", true)] {
        std::env::set_var("CER_INLINE_SAFE_METADATA", value);
        let mut entry = DylibNodeEntry::load(&path).expect("valid metadata bool");
        assert_eq!(
            entry.inline_safe(),
            expected,
            "type declaration before create"
        );
        entry
            .init(NodeContext::for_tests(IndexMap::new(), IndexMap::new()))
            .expect("valid context");
        assert!(
            !entry.inline_safe(),
            "after init the fixture's missing state table is unchanged"
        );
        entry.shutdown().expect("teardown");
    }
    std::env::set_var("CER_INLINE_SAFE_METADATA", "invalid");
    let error = match DylibNodeEntry::load(&path) {
        Ok(_) => panic!("string metadata must refuse before create"),
        Err(error) => error,
    };
    assert!(error
        .to_string()
        .contains("inline_safe metadata must be a bool"));
}

#[test]
fn linked_stable_fixtures_export_the_bootstrap_and_no_legacy_maintenance_entries() {
    const LEGACY: &[&str] = &[
        "cerulion_node_pump_history",
        "cerulion_node_pump_history_at",
        "cerulion_node_set_snapshot_inputs",
        "cerulion_node_snapshot_inputs",
        "cerulion_node_drain_trigger_input",
        "cerulion_node_refill_trigger_input",
        "cerulion_node_sync_head_op",
        "cerulion_node_probes_trigger_input",
    ];
    for name in [
        "test_node_raw_ffi_template_cdylib",
        "test_node_snapshot_fail_cdylib",
        "test_node_drain_fail_cdylib",
        "test_node_sync_op_fail_cdylib",
    ] {
        let library = cerulion_core::testing::find_fixture_cdylib(name);
        let mut command = std::process::Command::new("nm");
        if cfg!(target_os = "macos") {
            command.arg("-gU");
        } else {
            command.args(["-D", "--defined-only"]);
        }
        let output = command.arg(&library).output().expect("list linked exports");
        assert!(
            output.status.success(),
            "nm failed: {}",
            String::from_utf8_lossy(&output.stderr)
        );
        let symbols = String::from_utf8(output.stdout).expect("UTF-8 symbol listing");
        let exports: Vec<&str> = symbols
            .lines()
            .filter_map(|line| line.split_whitespace().last())
            .map(|name| {
                if cfg!(target_os = "macos") {
                    name.strip_prefix('_').unwrap_or(name)
                } else {
                    name
                }
            })
            .collect();
        assert!(
            exports.contains(&"cerulion_node_api_v1"),
            "the maintained bootstrap must actually be linked: {name}"
        );
        for forbidden in LEGACY {
            assert!(
                !exports.contains(forbidden),
                "stable fixture {name} still exports obsolete {forbidden}"
            );
        }
    }
}
