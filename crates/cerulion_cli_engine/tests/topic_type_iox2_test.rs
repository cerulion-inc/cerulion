// SPDX-License-Identifier: AGPL-3.0-only
//! `topic type` and the typed local listing over real iceoryx2 shared
//! memory: the error arms (a topic that sends nothing, a hash no type names),
//! a first wait that returns with no sample, and the mirror recheck of the
//! typed listing.
//!
//! These tests open REAL iceoryx2 services on the process-global namespace
//! (the engine calls `TransportManager::get_or_init()`), so every test is
//! `#[serial]`; run the binary on its own:
//! ```bash
//! cargo test -p cerulion_cli_engine --test topic_type_iox2_test -- --test-threads=1
//! ```

use std::time::{Duration, SystemTime, UNIX_EPOCH};

use cerulion_cli_engine::topic_cmd::introspect::{
    topic_type_with_scope, type_local_topics, InternalTopics, TopicType,
};
use cerulion_cli_engine::topic_cmd::{
    local_walker_from_workspace, TopicInfo, TopicScope, STD_MSGS_STRING_SCHEMA_HASH,
};
use cerulion_core::{PubSubEvent, WireHeader};
use serial_test::serial;

/// The raw iceoryx2 service policy of the foreign-writer fixture (the
/// on-SHM identity the CLI's open-only subscriber opens).
type RawService = iceoryx2::service::ipc::Service;

/// A pid + nanosecond unique topic, so a stale service cannot collide.
fn unique_topic(tag: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    format!("/{tag}/{}/{}", std::process::id(), nanos)
}

/// A `std_msgs/String` frame carrying `s`.
fn build_string_frame(seq: u32, ts_ns: u64, s: &str) -> Vec<u8> {
    let sbytes = s.as_bytes();
    let payload_len = 8 + sbytes.len();
    let total = WireHeader::SIZE + payload_len;
    let mut frame = vec![0u8; total];

    let mut header = WireHeader::new(STD_MSGS_STRING_SCHEMA_HASH, seq, ts_ns);
    header.total_size = total as u32;
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);

    let payload = &mut frame[WireHeader::SIZE..];
    payload[0..4].copy_from_slice(&8u32.to_le_bytes()); // offset
    payload[4..8].copy_from_slice(&(sbytes.len() as u32).to_le_bytes()); // len
    payload[8..8 + sbytes.len()].copy_from_slice(sbytes);
    frame
}

/// A header-only frame carrying an arbitrary `schema_hash`.
fn build_frame_with_hash(hash: u64, seq: u32, ts_ns: u64) -> Vec<u8> {
    let mut frame = vec![0u8; WireHeader::SIZE];
    let mut header = WireHeader::new(hash, seq, ts_ns);
    header.total_size = WireHeader::SIZE as u32;
    header.write_to_buf(&mut frame);
    frame
}

/// RAII env-var guard for the `#[serial]` kill-switch tests (restores
/// the prior value on drop, panic-safe).
struct EnvGuard {
    key: &'static str,
    prev: Option<String>,
}

impl EnvGuard {
    fn set(key: &'static str, val: &str) -> Self {
        let prev = std::env::var(key).ok();
        std::env::set_var(key, val);
        Self { key, prev }
    }
}

impl Drop for EnvGuard {
    fn drop(&mut self) {
        match &self.prev {
            Some(v) => std::env::set_var(self.key, v),
            None => std::env::remove_var(self.key),
        }
    }
}

/// A foreign raw-iceoryx2 writer on `{topic}/data` (+ optionally
/// `{topic}/event`). Holding the node/services/ports alive keeps the
/// topic discoverable by `topic list` for the whole test.
struct RawTopicWriter {
    // Order-independent liveness holders; the publisher is the port we send on.
    _node: iceoryx2::node::Node<RawService>,
    _data_service:
        iceoryx2::service::port_factory::publish_subscribe::PortFactory<RawService, [u8], ()>,
    publisher: iceoryx2::port::publisher::Publisher<RawService, [u8], ()>,
    /// Present only for the NOTIFYING fixture; `None` = pure data writer
    /// (the adversarial timeout-drain case).
    notifier: Option<iceoryx2::port::notifier::Notifier<RawService>>,
    _event_service: Option<iceoryx2::service::port_factory::event::PortFactory<RawService>>,
}

impl RawTopicWriter {
    /// Create the `{topic}/data` publish-subscribe service (+ a publisher)
    /// and, when `notifying`, the `{topic}/event` service (+ a notifier).
    fn new(topic: &str, notifying: bool) -> Self {
        let node = iceoryx2::node::NodeBuilder::new()
            .create::<RawService>()
            .expect("raw iceoryx2 node");

        let data_name: iceoryx2::service::service_name::ServiceName = format!("{topic}/data")
            .as_str()
            .try_into()
            .expect("data name");
        let data_service = node
            .service_builder(&data_name)
            .publish_subscribe::<[u8]>()
            // Room for several frames between the observer's drains.
            .subscriber_max_buffer_size(16)
            .create()
            .expect("raw data service");
        let publisher = data_service
            .publisher_builder()
            .initial_max_slice_len(4096)
            .create()
            .expect("raw publisher");

        let (event_service, notifier) = if notifying {
            let event_name: iceoryx2::service::service_name::ServiceName = format!("{topic}/event")
                .as_str()
                .try_into()
                .expect("event name");
            let event_service = node
                .service_builder(&event_name)
                .event()
                .create()
                .expect("raw event service");
            let notifier = event_service
                .notifier_builder()
                .create()
                .expect("raw notifier");
            (Some(event_service), Some(notifier))
        } else {
            (None, None)
        };

        Self {
            _node: node,
            _data_service: data_service,
            publisher,
            notifier,
            _event_service: event_service,
        }
    }

    /// Publish one wire frame into `{topic}/data`. Rings the `SentSample`
    /// event ONLY if this writer was built notifying — the adversarial
    /// fixture never notifies, forcing the observer's timeout-drain path.
    fn publish(&self, frame: &[u8]) {
        let mut sample = self
            .publisher
            .loan_slice_uninit(frame.len())
            .expect("raw publisher loan");
        let buf = sample.payload_mut();
        for (i, &b) in frame.iter().enumerate() {
            buf[i].write(b);
        }
        // SAFETY: bytes [0, frame.len()) were all initialized above.
        let sample = unsafe { sample.assume_init() };
        sample.send().expect("raw publisher send");

        if let Some(notifier) = &self.notifier {
            notifier
                .notify_with_custom_event_id(PubSubEvent::SentSample.into())
                .expect("raw notify SentSample");
        }
    }

    /// Ring `{topic}/event` with NO sample behind it: the early-return
    /// wait a reader must retry past.
    fn ring_without_a_sample(&self) {
        self.notifier
            .as_ref()
            .expect("a notifying writer")
            .notify_with_custom_event_id(PubSubEvent::SentSample.into())
            .expect("raw notify SentSample");
    }
}

/// Run `topic type --local` on a thread, then call `act` once its subscriber
/// has had time to attach.
fn type_while(topic: &str, act: impl FnOnce()) -> Result<String, String> {
    let handle = {
        let topic = topic.to_string();
        std::thread::spawn(move || {
            topic_type_with_scope(&topic, None, TopicScope::Local).map_err(|e| e.to_string())
        })
    };
    std::thread::sleep(Duration::from_millis(300));
    act();
    handle.join().expect("type thread")
}

#[test]
#[serial]
fn topic_type_names_a_builtin_from_one_frame() {
    let _net = EnvGuard::set("CERULION_NETWORK", "off");
    let topic = unique_topic("type_known");
    let writer = RawTopicWriter::new(&topic, true);
    let ty = type_while(&topic, || {
        writer.publish(&build_string_frame(0, 1_000, "x"))
    });
    assert_eq!(ty.as_deref(), Ok("std_msgs/String"));
}

#[test]
#[serial]
fn topic_type_of_a_topic_that_sends_nothing_names_the_topic_and_the_wait() {
    let _net = EnvGuard::set("CERULION_NETWORK", "off");
    let topic = unique_topic("type_silent");
    let _writer = RawTopicWriter::new(&topic, true);
    let err = type_while(&topic, || {}).unwrap_err();
    assert_eq!(
        err,
        format!(
            "topic '{topic}' sent no message within 1s, so its type is unknown; retry while \
             its publisher is sending"
        )
    );
}

#[test]
#[serial]
fn topic_type_of_a_hash_no_type_names_shows_the_hash() {
    let _net = EnvGuard::set("CERULION_NETWORK", "off");
    let topic = unique_topic("type_unnamed");
    let writer = RawTopicWriter::new(&topic, true);
    let bogus = 0x0123_4567_89AB_CDEFu64;
    let err = type_while(&topic, || {
        writer.publish(&build_frame_with_hash(bogus, 0, 1))
    })
    .unwrap_err();
    assert_eq!(
        err,
        format!(
            "topic '{topic}' carries schema hash 0x0123456789abcdef, which no built-in or \
             workspace type names; add its `.msg` under `schemas/` to name it"
        )
    );
}

#[test]
#[serial]
fn topic_type_waits_past_an_event_that_carried_no_sample() {
    let _net = EnvGuard::set("CERULION_NETWORK", "off");
    let topic = unique_topic("type_early");
    let writer = RawTopicWriter::new(&topic, true);
    let ty = type_while(&topic, || {
        // The first wait returns early with nothing; the frame comes later.
        writer.ring_without_a_sample();
        std::thread::sleep(Duration::from_millis(200));
        writer.publish(&build_string_frame(0, 1_000, "late"));
    });
    assert_eq!(ty.as_deref(), Ok("std_msgs/String"));
}

/// Type one topic through the typed local listing while `act` publishes.
fn type_listed(topic: &str, local_only: bool, act: impl FnOnce()) -> TopicType {
    let handle = {
        let topic = topic.to_string();
        std::thread::spawn(move || {
            let walker = local_walker_from_workspace(None);
            type_local_topics(
                &[TopicInfo { name: topic }],
                &walker,
                local_only,
                InternalTopics::Read,
            )
            .expect("typed listing")
        })
    };
    std::thread::sleep(Duration::from_millis(300));
    act();
    let mut typed = handle.join().expect("listing thread");
    assert_eq!(typed.len(), 1);
    typed.remove(0).ty
}

#[test]
#[serial]
fn the_typed_listing_types_a_genuine_local_topic() {
    let _net = EnvGuard::set("CERULION_NETWORK", "off");
    for local_only in [false, true] {
        let topic = unique_topic("listed_local");
        let writer = RawTopicWriter::new(&topic, true);
        let ty = type_listed(&topic, local_only, || {
            writer.publish(&build_string_frame(0, 1_000, "x"))
        });
        assert_eq!(
            ty,
            TopicType::Named("std_msgs/String".to_string()),
            "local_only = {local_only}"
        );
    }
}

#[test]
#[serial]
fn the_typed_listing_drops_a_topic_that_is_a_network_mirror() {
    use cerulion_core::wire::MaxSliceLen;
    let _net = EnvGuard::set("CERULION_NETWORK", "off");
    let manager = cerulion_core::TransportManager::get_or_init().unwrap();
    for local_only in [false, true] {
        let topic = unique_topic("listed_mirror");
        // A re-injected remote stream: listed locally, marked as a mirror.
        let injector = manager
            .create_remote_ingress_injector(
                &topic,
                STD_MSGS_STRING_SCHEMA_HASH,
                MaxSliceLen::const_new(256),
            )
            .unwrap();
        let ty = type_listed(&topic, local_only, || {
            injector.reinject_raw(&build_string_frame(0, 1_000, "remote"));
        });
        assert_eq!(ty, TopicType::Unknown, "local_only = {local_only}");
    }
}
