// SPDX-License-Identifier: AGPL-3.0-only
//! Dedicated single-test binary proving that `init_resim_transport` refuses
//! with exit 5 when the process's `TransportManager` singleton is already
//! initialized on another iceoryx2 namespace prefix.
//!
//! Lives in its own integration-test binary because `TransportManager` is a
//! process-wide singleton initialized once per process.

#![cfg(unix)]

use cerulion_cli_engine::replay_engine::{
    init_resim_transport_with_slot_for_test, resim_namespace_prefix,
};
use cerulion_core::testing::IsolatedRoot;
use cerulion_core::{TransportConfig, TransportManager};

#[test]
fn init_resim_transport_refuses_already_initialized_non_resim_singleton() {
    let mut root = IsolatedRoot::mint("resim_wrong_pfx");
    root.prefix = "wrong_pfx_".to_string();
    let wrong_cfg = root.config();

    let _existing = TransportManager::init_with_config(
        TransportConfig {
            node_name: format!("wrong_pfx_node_{}", std::process::id()),
            ..Default::default()
        },
        wrong_cfg.clone(),
    )
    .expect("initialize singleton on a non-resim prefix");

    let expected = resim_namespace_prefix(0);
    let err = match init_resim_transport_with_slot_for_test(
        TransportConfig {
            node_name: format!("resim_probe_{}", std::process::id()),
            ..Default::default()
        },
        &wrong_cfg,
        0,
    ) {
        Ok(_) => panic!("resim must refuse a singleton already running on another prefix"),
        Err(err) => err,
    };

    assert_eq!(err.exit_code(), 5);
    let text = err.to_string();
    assert!(
        text.contains("wrong_pfx_") && text.contains(&expected),
        "refusal must name both the running and expected prefixes: {text}"
    );
}
