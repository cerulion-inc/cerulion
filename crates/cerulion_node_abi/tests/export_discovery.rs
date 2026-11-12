// SPDX-License-Identifier: AGPL-3.0-only
use cerulion_node_abi::{stabby::IStable, BootstrapFn, NodeApiV1};

fn panicking_constructor() -> *const NodeApiV1 {
    panic!("constructor panic oracle");
}
cerulion_node_abi::export_node_api!(panicking_constructor);

#[test]
fn discovery_checks_the_complete_signature_before_constructor_execution() {
    assert!(cerulion_node_api_v1_stabbied_v3(BootstrapFn::REPORT).is_some());
    assert!(cerulion_node_api_v1_stabbied_v3(u64::REPORT).is_none());
    assert!(cerulion_node_abi::bootstrap_matches(
        cerulion_node_api_v1_stabbied_v3_report()
    ));
    assert!(cerulion_node_api_v1().is_null());
}
