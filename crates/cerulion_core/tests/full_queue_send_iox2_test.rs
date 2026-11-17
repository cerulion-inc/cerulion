// SPDX-License-Identifier: AGPL-3.0-only
#![cfg(not(feature = "local-transport"))]
//! A publisher built with [`FullQueueSend::Discard`] returns from its send at
//! a connected subscriber whose queue is full, on a data service a raw
//! iceoryx2 node created with safe overflow disabled, and the recipient count
//! it returns leaves that subscriber out. Every constructor without the
//! option builds its port with the iceoryx2 config's backpressure strategy,
//! `RetryUntilDelivered` in this config, under which that send waits.
//!
//! iceoryx2 gives each publisher and subscriber pair its own queue, so a
//! 1-deep subscriber that never takes holds a full queue from a publisher
//! after that publisher's first send: the second send is the one that meets
//! it. A send waits on a full queue only while both ends of it are
//! attached, and a subscriber attaches its end of a later publisher's queue
//! only when it updates its connections, so each test builds the port, has
//! the blocker update its connections with `has_samples`, and then sends.

use std::sync::{mpsc, Arc};
use std::time::Duration;

use cerulion_core::clock::RealClock;
use cerulion_core::transport::service::{derive_client_guid, request_topic};
use cerulion_core::transport::TopicServiceConfig;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{FullQueueSend, ReinjectOutcome, TransportConfig, TransportManager};
use iceoryx2::port::backpressure_strategy::BackpressureStrategy;
use iceoryx2::service::port_factory::PortFactory as _;
use serial_test::serial;

type Raw = iceoryx2::service::ipc::Service;

/// How long a send may take before the test ends the wait and fails.
const BOUND: Duration = Duration::from_secs(20);

const SCHEMA_HASH: u64 = 0x0123_4567_89AB_CDEF;

fn transport(tag: &str) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("{tag}_parent"),
            clock: Arc::new(RealClock),
            subscriber_buffer_size: 4,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("transport")
}

/// A data service on `topic` that a raw iceoryx2 node creates with safe
/// overflow disabled and the request-topic port caps (which the default
/// openers accept too), with a 1-deep subscriber that never takes and a
/// second one with a deep queue.
struct NoOverflowTopic {
    blocker: Option<iceoryx2::port::subscriber::Subscriber<Raw, [u8], ()>>,
    observer: iceoryx2::port::subscriber::Subscriber<Raw, [u8], ()>,
    service: iceoryx2::service::port_factory::publish_subscribe::PortFactory<Raw, [u8], ()>,
    _node: iceoryx2::node::Node<Raw>,
}

impl NoOverflowTopic {
    fn create(transport: &TransportManager, topic: &str) -> Self {
        let caps = TopicServiceConfig::for_service_request(transport.subscriber_buffer_size());
        let node = iceoryx2::node::NodeBuilder::new()
            .config(&transport.iox_config())
            .create::<Raw>()
            .expect("raw node");
        let name: iceoryx2::service::service_name::ServiceName = format!("{topic}/data")
            .as_str()
            .try_into()
            .expect("service name");
        let service = node
            .service_builder(&name)
            .publish_subscribe::<[u8]>()
            .enable_safe_overflow(false)
            .subscriber_max_buffer_size(transport.subscriber_buffer_size())
            .max_publishers(caps.max_publishers.expect("request publisher cap"))
            .max_subscribers(caps.max_subscribers.expect("request subscriber cap"))
            .create_with_attributes(
                &cerulion_core::transport::wire_header_attribute::data_service_specifier()
                    .expect("wire header attribute"),
            )
            .expect("an overflow-disabled data service");
        assert!(!service.static_config().has_safe_overflow());
        let blocker = service
            .subscriber_builder()
            .buffer_size(1)
            .create()
            .expect("blocker");
        let observer = service.subscriber_builder().create().expect("observer");
        Self {
            blocker: Some(blocker),
            observer,
            service,
            _node: node,
        }
    }

    /// Attach the blocker's end of the queue of every publisher on the
    /// service: `has_samples` updates its connections and takes nothing.
    fn attach_blocker(&self) {
        let blocker = self.blocker.as_ref().expect("blocker");
        assert!(!blocker.has_samples().expect("blocker connections"));
    }

    /// What `rx` delivers within [`BOUND`]. Past it the blocker is dropped,
    /// which ends a send waiting on its queue, and the test fails.
    fn bounded<T>(&mut self, rx: &mpsc::Receiver<T>, what: &str) -> T {
        match rx.recv_timeout(BOUND) {
            Ok(value) => value,
            Err(_) => {
                drop(self.blocker.take());
                panic!("{what} did not return within {BOUND:?}: it waited on the full queue");
            }
        }
    }

    /// Samples the observer has queued, taking each one. A publisher's
    /// samples are taken while that publisher is alive.
    fn observed(&self) -> usize {
        let mut n = 0;
        while self.observer.receive().expect("receive").is_some() {
            n += 1;
        }
        n
    }

    /// The subscribers attached to the data service, read off the raw
    /// node's own handle.
    fn subscribers(&self) -> usize {
        self.service.dynamic_config().number_of_subscribers()
    }
}

fn frame(sequence: u32) -> Vec<u8> {
    let payload = [0xA5u8; 8];
    let header = WireHeader {
        schema_hash: SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: 0,
        offset_table_count: 0,
        sequence,
        timestamp_ns: 0,
        step_tag: cerulion_core::wire::StepTag::UNTAGGED,
    };
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(&payload);
    frame
}

/// `f`'s value over a channel, from a new thread.
fn in_thread<T: Send + 'static>(f: impl FnOnce() -> T + Send + 'static) -> mpsc::Receiver<T> {
    let (tx, rx) = mpsc::channel();
    std::thread::spawn(move || {
        let _ = tx.send(f());
    });
    rx
}

#[test]
#[serial]
fn a_discarding_ingress_publisher_returns_at_a_full_queue_and_counts_the_subscribers_it_reached() {
    let transport = transport("fqa");
    let topic = "/fqs/publish";
    let mut full = NoOverflowTopic::create(&transport, topic);
    let mut publisher = transport
        .create_ingress_publisher_with(topic, MaxSliceLen::const_new(256), FullQueueSend::Discard)
        .expect("ingress publisher");
    full.attach_blocker();
    let sends = in_thread(move || {
        let strategy = publisher.backpressure_strategy_for_test();
        let recipients: Vec<usize> = (0..3)
            .map(|i| publisher.publish_raw(&frame(i)).expect("publish_raw"))
            .collect();
        (strategy, recipients, publisher)
    });
    let (strategy, recipients, _publisher) = full.bounded(&sends, "publish_raw");
    assert_eq!(strategy, BackpressureStrategy::DiscardData);
    // The first frame reaches both; the next two skip the full blocker.
    assert_eq!(recipients, vec![2, 1, 1]);
    assert_eq!(full.observed(), 3);
}

#[test]
#[serial]
fn a_discarding_injector_reinjects_past_a_full_queue_and_reads_the_subscriber_count() {
    let transport = transport("fqb");
    let topic = "/fqs/inject";
    let mut full = NoOverflowTopic::create(&transport, topic);
    let injector = transport
        .create_ingress_injector_with(
            topic,
            SCHEMA_HASH,
            MaxSliceLen::const_new(256),
            FullQueueSend::Discard,
        )
        .expect("injector");
    full.attach_blocker();
    let sends = in_thread(move || {
        let count = injector.subscriber_count();
        let outcomes: Vec<ReinjectOutcome> =
            (0..3).map(|i| injector.reinject_raw(&frame(i))).collect();
        (count, outcomes, injector)
    });
    let (count, outcomes, _injector) = full.bounded(&sends, "reinject_raw");
    assert_eq!(count, Some(2));
    assert_eq!(
        outcomes,
        [2, 1, 1].map(|recipients| ReinjectOutcome::Injected { recipients })
    );
    assert_eq!(full.observed(), 3);
}

#[test]
#[serial]
fn a_discarding_service_client_returns_at_a_full_request_queue_and_counts_the_servers_reached() {
    let transport = transport("fqc");
    let service = "/fqs/call";
    let mut full = NoOverflowTopic::create(&transport, &request_topic(service));
    let mut client = transport
        .create_service_client_with(
            service,
            derive_client_guid("fqs", service, 0),
            MaxSliceLen::const_new(256),
            SCHEMA_HASH,
            SCHEMA_HASH ^ 1,
            FullQueueSend::Discard,
        )
        .expect("client");
    full.attach_blocker();
    let sends = in_thread(move || {
        let strategy = client.request_backpressure_strategy_for_test();
        let servers = client.server_count();
        let reached: Vec<usize> = (0..2)
            .map(|_| client.send_request_counted(&[7u8; 4]).expect("send").1)
            .collect();
        (strategy, servers, reached, client)
    });
    let (strategy, servers, reached, _client) = full.bounded(&sends, "send_request_counted");
    assert_eq!(strategy, BackpressureStrategy::DiscardData);
    assert_eq!(full.subscribers(), 2);
    assert_eq!(servers, 2);
    assert_eq!(reached, vec![2, 1]);
    assert_eq!(full.observed(), 2);
}

/// The waiting half is not run: a send under `RetryUntilDelivered` at a full
/// queue does not return. The port's strategy is what this pins.
#[test]
#[serial]
fn every_constructor_without_the_option_keeps_the_config_strategy() {
    let transport = transport("fqd");
    let config_default = transport
        .iox_config()
        .defaults
        .publish_subscribe
        .backpressure_strategy;
    assert_eq!(config_default, BackpressureStrategy::RetryUntilDelivered);
    let msl = MaxSliceLen::const_new(256);
    let strategies = [
        transport
            .create_ingress_publisher("/fqs/default/ingress", msl)
            .expect("ingress publisher")
            .backpressure_strategy_for_test(),
        transport
            .create_ingress_publisher_with(
                "/fqs/default/ingress_with",
                msl,
                FullQueueSend::ConfigDefault,
            )
            .expect("ingress publisher")
            .backpressure_strategy_for_test(),
        transport
            .create_publisher_simple("/fqs/default/publisher", msl)
            .expect("publisher")
            .backpressure_strategy_for_test(),
        transport
            .create_service_client(
                "/fqs/default/call",
                derive_client_guid("fqs", "/fqs/default/call", 0),
                msl,
                SCHEMA_HASH,
                SCHEMA_HASH ^ 1,
            )
            .expect("client")
            .request_backpressure_strategy_for_test(),
    ];
    assert_eq!(strategies, [config_default; 4]);
    let injector = transport
        .create_ingress_injector("/fqs/default/injector", SCHEMA_HASH, msl)
        .expect("injector");
    assert_eq!(injector.subscriber_count(), None);
}
