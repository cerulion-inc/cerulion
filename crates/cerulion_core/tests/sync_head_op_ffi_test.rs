// SPDX-License-Identifier: AGPL-3.0-only
//! Stable head-operation argument guards and the removed legacy symbol.
//! Each independent run pins real admitted host calls against literal codes.

use cerulion_core::graph::node::NodeContext;
use cerulion_core::graph::node_host_abi::{
    register_context, remove_context, with_context, HostContext, HostPortSpec, HOST_API,
};
use cerulion_core::node_abi as abi;
use cerulion_core::transport::plugin_lease::{LeaseDirection, PluginWireLayout};
use indexmap::IndexMap;
use serial_test::serial;
use std::collections::HashMap;
use std::sync::Arc;

#[test]
#[serial]
fn legacy_sync_head_op_is_absent_and_stable_input_head_refuses_each_bad_argument() {
    let path =
        cerulion_core::testing::find_fixture_cdylib("test_node_macro_sync_nontrigger_cdylib");
    // SAFETY: the trusted fixture stays loaded until its symbol lookup ends.
    let library = unsafe { libloading::Library::new(path) }.expect("load rebuilt sync fixture");
    // SAFETY: absence is tested without calling or dereferencing any symbol.
    assert!(unsafe { library.get::<*const ()>(b"cerulion_node_sync_head_op") }.is_err());
    // SAFETY: the positive control resolves the trusted symbol without dereferencing it.
    assert!(unsafe { library.get::<*const ()>(b"cerulion_node_api_v1") }.is_ok());

    let mut runs = Vec::new();
    for _ in 0..2 {
        let host = HostContext::new(
            NodeContext::for_tests(IndexMap::new(), IndexMap::new()),
            vec![HostPortSpec {
                name: "input".into(),
                direction: LeaseDirection::Read,
                layout: PluginWireLayout {
                    schema_hash: 1,
                    fixed_size: 0,
                    variable_fields: 0,
                    alignment: 8,
                },
                read_mode: abi::read_mode::ONE,
                accumulate_cap: None,
            }],
            Arc::new(HashMap::new()),
            1,
        )
        .expect("valid empty bound input descriptor");
        let context = abi::ContextHandle {
            value: register_context(host).unwrap(),
        };
        with_context(context.value, |host| host.begin_node_call()).unwrap();
        let port = abi::PortHandle {
            value: with_context(context.value, |host| {
                host.bind_port(context.value, 0, 1, LeaseDirection::Read)
            })
            .unwrap(),
        };
        let mut outcome = abi::HeadOutcomeV1 {
            popped: 17,
            timestamp_ns: 19,
            has_timestamp: 1,
            kind: 23,
        };
        let mut codes = Vec::new();
        // SAFETY: the live admitted host and port remain registered throughout.
        // Null and deliberately misaligned outputs are rejected before writes;
        // every other output is an exclusive initialized stack record.
        unsafe {
            codes.push((HOST_API.input_head)(context, port, 2, std::ptr::null_mut()).code);
            codes.push(
                (HOST_API.input_head)(context, port, 2, std::ptr::without_provenance_mut(1)).code,
            );
            codes.push((HOST_API.input_head)(context, port, 99, &mut outcome).code);
            codes.push(
                (HOST_API.input_head)(
                    context,
                    abi::PortHandle { value: u64::MAX },
                    2,
                    &mut outcome,
                )
                .code,
            );
            assert_eq!(
                (
                    outcome.popped,
                    outcome.timestamp_ns,
                    outcome.has_timestamp,
                    outcome.kind
                ),
                (17, 19, 1, 23),
                "refusals never fabricate an answer"
            );
            codes.push((HOST_API.input_head)(context, port, 2, &mut outcome).code);
        }
        assert_eq!(codes, [5, 5, 5, 4, 0]);
        assert_eq!(
            (
                outcome.popped,
                outcome.timestamp_ns,
                outcome.has_timestamp,
                outcome.kind
            ),
            (0, 0, 0, 0),
            "the valid empty port answers NOTHING"
        );
        with_context(context.value, |host| host.end_node_call()).unwrap();
        remove_context(context.value).unwrap();
        runs.push(codes);
    }
    assert_eq!(runs[0], [5, 5, 5, 4, 0]);
    assert_eq!(runs[1], [5, 5, 5, 4, 0]);
    assert_eq!(runs[0], runs[1]);
}
