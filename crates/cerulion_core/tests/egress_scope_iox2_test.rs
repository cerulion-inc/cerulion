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
fn an_export_tap_pins_local_source_identity_until_after_its_data_port_drops() {
    let (producer, remote) = managers();
    let publisher = producer
        .create_publisher_simple("/source_role", MaxSliceLen::const_new(128))
        .unwrap();
    let tap = remote
        .create_remote_data_only_subscriber("/source_role")
        .unwrap();
    drop(publisher);
    let refused =
        producer.create_remote_ingress_injector("/source_role", 11, MaxSliceLen::const_new(128));
    assert!(matches!(
        refused,
        Err(cerulion_core::TransportError::MirrorIdentity { .. })
    ));
    drop(tap);
    let mirror = producer
        .create_remote_ingress_injector("/source_role", 11, MaxSliceLen::const_new(128))
        .unwrap();
    assert!(remote.is_network_mirror("/source_role").unwrap());
    assert!(matches!(
        remote.create_remote_data_only_subscriber("/source_role"),
        Err(cerulion_core::TransportError::LocalObservationLease { .. })
    ));
    drop(mirror);
    let publisher = producer
        .create_publisher_simple("/source_role", MaxSliceLen::const_new(128))
        .unwrap();
    assert!(remote
        .create_remote_data_only_subscriber("/source_role")
        .is_ok());
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
fn declared_run_ingress_cannot_inject_into_an_existing_local_only_run() {
    let config = cerulion_core::testing::iceoryx_test_config();
    let local =
        TransportManager::init_for_test(TransportConfig::default(), config.clone()).unwrap();
    let gateway = TransportManager::init_for_test(
        TransportConfig {
            network: Some(cerulion_core::NetworkConfig::default()),
            ..Default::default()
        },
        config,
    )
    .unwrap();
    let _scope = local.exclude_remote_topic("/private_input").unwrap();
    let before = gateway.remote_access_refusals();
    assert!(gateway
        .register_run_ingress_topic("/private_input", 1, MaxSliceLen::const_new(128))
        .is_err());
    assert_eq!(gateway.remote_access_refusals() - before, 1);
    assert!(gateway.list_topics().unwrap().is_empty());
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

#[test]
fn service_family_exclusion_refuses_requests_and_all_per_client_replies() {
    use cerulion_core::transport::{egress_scope, service};
    let (producer, remote) = managers();
    let parent = egress_scope::service_scope_topic("/private_service");
    let _scope = producer.exclude_remote_topic(&parent).unwrap();
    for topic in [
        service::request_topic("/private_service"),
        service::reply_topic("/private_service", &[0x41; 16]),
        service::reply_topic("/private_service", &[0x42; 16]),
    ] {
        assert!(remote.acquire_remote_topic_lease(&topic).is_err());
        assert!(!remote.remote_topic_allowed(&topic).unwrap());
        let canonical = format!("/{topic}");
        assert!(remote.acquire_remote_topic_lease(&canonical).is_err());
        assert!(!remote.remote_topic_allowed(&canonical).unwrap());
    }
    assert!(producer.list_topics().unwrap().is_empty());
}

#[test]
fn opaque_child_scope_is_deterministic_and_the_environment_kill_switch_wins() {
    use cerulion_core::transport::egress_scope::RunTopicScope;
    let json = r#"{"mode":"declared","topics":["/public"],"ingress":["/input"]}"#;
    for _ in 0..2 {
        let scope = RunTopicScope::from_values(None, Some(json)).unwrap();
        assert!(!scope.excludes("/public"));
        assert!(scope.excludes("/private"));
        assert!(scope.excludes("/input"));
        assert!(!scope.excludes_input("/input"));
        assert!(!scope.excludes_input("/public"));
        assert!(scope.excludes_input("/private"));
        for network in ["off", " off ", "typo"] {
            assert!(RunTopicScope::from_values(Some(network), Some(json))
                .unwrap()
                .excludes("/public"));
        }
        assert!(!RunTopicScope::from_values(None, None)
            .unwrap()
            .excludes("/public"));
        for malformed in [
            "",
            "null",
            r#"{"mode":"unknown"}"#,
            r#"{"mode":"declared","topics":"/public"}"#,
        ] {
            assert!(RunTopicScope::from_values(None, Some(malformed)).is_err());
        }
    }
}
