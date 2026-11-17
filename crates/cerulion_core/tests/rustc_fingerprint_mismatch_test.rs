// SPDX-License-Identifier: AGPL-3.0-only
//! Optional compiler provenance cannot veto a checked stable node. Real
//! different-compiler downstream delivery is in stable_node_abi_cross_compiler_test.
use cerulion_core::graph::node::{DylibNodeEntry, NodeContext, NodeEntry};
use indexmap::IndexMap;
use serial_test::serial;
struct EnvVarGuard;
impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        std::env::remove_var("CER_RUSTC_FAULT");
    }
}
fn load_and_execute(provenance: Option<&str>) {
    let _guard = EnvVarGuard;
    if let Some(value) = provenance {
        std::env::set_var("CER_RUSTC_FAULT", value);
    } else {
        std::env::remove_var("CER_RUSTC_FAULT");
    }
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_cdylib");
    let mut node = DylibNodeEntry::load(&path).expect("stable layout is authoritative");
    node.init(NodeContext::for_tests(IndexMap::new(), IndexMap::new()))
        .unwrap();
    node.tick().unwrap();
    node.shutdown().unwrap();
}
#[test]
#[serial]
fn diagnostic_provenance_mismatch_does_not_refuse_a_checked_node() {
    load_and_execute(Some("1"));
}
#[test]
#[serial]
fn null_optional_provenance_does_not_refuse_a_checked_node() {
    load_and_execute(Some("null"));
}
#[test]
#[serial]
fn ordinary_provenance_loads_the_same_stable_boundary() {
    load_and_execute(None);
}
