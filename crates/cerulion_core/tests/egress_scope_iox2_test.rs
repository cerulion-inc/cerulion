// SPDX-License-Identifier: AGPL-3.0-only
//! Native SHM admission and delivery, with a separate manager per process role.
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{TransportConfig, TransportManager};
use native_ros2_messages::geometry_msgs::Vector3;
use std::sync::{Arc, Barrier};

fn managers() -> (Arc<TransportManager>, Arc<TransportManager>) {
    let config = cerulion_core::testing::iceoryx_test_config();
    (
        TransportManager::init_for_test(TransportConfig::default(), config.clone()).unwrap(),
        TransportManager::init_for_test(TransportConfig::default(), config).unwrap(),
    )
}

#[test]
fn excluded_topics_remain_local_and_remote_reads_and_mirrors_are_counted_and_refused() {
    let (producer, remote) = managers();
    let _scope = producer.exclude_remote_topic("/private").unwrap();
    let mut publisher = producer
        .create_publisher_simple("/private", MaxSliceLen::const_new(128))
        .unwrap();
    let mut local = producer.create_data_only_subscriber("/private").unwrap();
    let before = remote.topic_subscriber_count("/private");
    let refusals_before = remote.remote_access_refusals();
    assert!(remote
        .create_remote_data_only_subscriber("/private")
        .is_err());
    assert!(remote
        .create_remote_ingress_injector("/private", 1, MaxSliceLen::const_new(128))
        .is_err());
    assert_eq!(remote.remote_access_refusals() - refusals_before, 2);
    assert_eq!(remote.topic_subscriber_count("/private"), before);
    {
        let mut sample = publisher.loan_proxy::<Vector3>().unwrap();
        sample.x = 1.25;
        sample.y = -2.5;
        sample.z = 3.75;
    }
    let mut received = Vec::new();
    local.drain_owned(1, &mut received).unwrap();
    assert_eq!(received.len(), 1);
    let oracle: Vec<u8> = [1.25_f64, -2.5, 3.75]
        .into_iter()
        .flat_map(f64::to_le_bytes)
        .collect();
    assert_eq!(&received[0].payload()[WireHeader::SIZE..], oracle);
    drop(local);
    drop(publisher);
}

#[test]
fn a_retained_remote_tap_cannot_read_a_successor_local_only_producer() {
    let (producer, remote) = managers();
    let publisher = producer
        .create_publisher_simple("/reuse", MaxSliceLen::const_new(128))
        .unwrap();
    let tap = remote.create_remote_data_only_subscriber("/reuse").unwrap();
    drop(publisher);
    assert!(producer.exclude_remote_topic("/reuse").is_err());
    drop(tap);
    let scope = producer.exclude_remote_topic("/reuse").unwrap();
    assert!(remote.create_remote_data_only_subscriber("/reuse").is_err());
    drop(scope);
    let publisher = producer
        .create_publisher_simple("/reuse", MaxSliceLen::const_new(128))
        .unwrap();
    assert!(remote.create_remote_data_only_subscriber("/reuse").is_ok());
    drop(publisher);
}

#[test]
fn concurrent_opposite_admissions_never_both_succeed() {
    for _ in 0..16 {
        let (producer, remote) = managers();
        let barrier = Arc::new(Barrier::new(2));
        let enter = barrier.clone();
        let local = std::thread::spawn(move || {
            enter.wait();
            let guard = producer.exclude_remote_topic("/race");
            enter.wait();
            guard.is_ok()
        });
        let foreign = std::thread::spawn(move || {
            barrier.wait();
            let guard = remote.acquire_remote_topic_lease("/race");
            barrier.wait();
            guard.is_ok()
        });
        assert!(!(local.join().unwrap() && foreign.join().unwrap()));
    }
}

#[test]
fn exclusion_is_namespace_scoped_and_drop_restores_remote_admission() {
    let (producer, remote) = managers();
    let other = TransportManager::init_for_test(
        TransportConfig::default(),
        cerulion_core::testing::iceoryx_test_config(),
    )
    .unwrap();
    let scope = producer.exclude_remote_topic("/same_name").unwrap();
    assert!(!remote.remote_topic_allowed("/same_name").unwrap());
    assert!(other.remote_topic_allowed("/same_name").unwrap());
    assert!(remote.remote_topic_allowed("/other_topic").unwrap());
    drop(scope);
    assert!(remote.remote_topic_allowed("/same_name").unwrap());
}

#[test]
fn malformed_names_refuse_without_creating_data_services() {
    let (producer, remote) = managers();
    assert!(producer.exclude_remote_topic("").is_err());
    assert!(remote.acquire_remote_topic_lease("").is_err());
    assert!(remote.list_topics().unwrap().is_empty());
}

#[test]
fn exhausted_remote_lease_slots_fail_closed_and_release_on_drop() {
    let (producer, remote) = managers();
    let leases: Vec<_> = (0..64)
        .map(|_| remote.acquire_remote_topic_lease("/quota").unwrap())
        .collect();
    assert!(remote.acquire_remote_topic_lease("/quota").is_err());
    assert!(producer.exclude_remote_topic("/quota").is_err());
    assert!(producer.list_topics().unwrap().is_empty());
    drop(leases);
    assert!(producer.exclude_remote_topic("/quota").is_ok());
}

#[test]
fn a_wrong_topic_attribute_cannot_authorize_remote_access() {
    use iceoryx2::prelude::*;
    use iceoryx2::service::attribute::AttributeVerifier;
    let config = cerulion_core::testing::iceoryx_test_config();
    let node = NodeBuilder::new()
        .config(&config)
        .create::<iceoryx2::service::ipc_threadsafe::Service>()
        .unwrap();
    let name: ServiceName = cerulion_core::transport::egress_scope::marker_service_name("/private")
        .as_str()
        .try_into()
        .unwrap();
    let verifier = AttributeVerifier::new()
        .require(&"topic".try_into().unwrap(), &"/wrong".try_into().unwrap())
        .unwrap();
    let _service = node
        .service_builder(&name)
        .event()
        .open_or_create_with_attributes(&verifier)
        .unwrap();
    let remote = TransportManager::init_for_test(TransportConfig::default(), config).unwrap();
    assert!(remote.remote_topic_allowed("/private").is_err());
    assert!(remote
        .create_remote_data_only_subscriber("/private")
        .is_err());
    assert!(remote.list_topics().unwrap().is_empty());
}
