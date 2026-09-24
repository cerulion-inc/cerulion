// SPDX-License-Identifier: AGPL-3.0-only
//! Every Cerulion event service is created with an event-id ceiling sized to
//! the ids the transport actually mints.
//!
//! iceoryx2 sizes a shared-memory counting bitset from the service's
//! `event_id_max_value` and a listener walks that bitset entry by entry on
//! EVERY wait, so the library default of 255 charges each wait 256 atomic
//! read-modify-writes to carry the 7 ids `PubSubEvent` defines.
//!
//! The ceiling is a property of whichever process CREATES the service: an open
//! fails only when the existing ceiling is below the one requested, so a single
//! creation site that forgets the ceiling silently restores the wide walk for
//! every later opener on that service. Both arms below exist because of that
//! asymmetry: the first proves the ceiling reaches a live service, the second
//! proves no creation site was left out.

use std::sync::Arc;

use cerulion_core::clock::VirtualClock;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::MaxSliceLen;
use iceoryx2::service::port_factory::PortFactory as _;
use iceoryx2::service::service_name::ServiceName;

/// The highest id `PubSubEvent` defines (`PublisherDisconnected`). Written out
/// here rather than imported because the constant is crate-private; the enum's
/// own `all_lists_every_variant_and_max_id_covers_it` is what keeps the two in
/// step when a variant is added.
const EXPECTED_CEILING: usize = 6;

/// A topic's event service, read back raw from the shared memory the transport
/// created it in, carries the ceiling and not the library default.
#[test]
fn a_topic_event_service_is_created_with_the_event_id_ceiling() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "event_id_ceiling".into(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 4,
            network: None,
        },
        ix.clone(),
    )
    .expect("transport manager on an isolated config");

    // Any port mints the topic's data + event service pair.
    let _pub = mgr
        .create_publisher_simple("/ceiling_probe", MaxSliceLen::const_new(64))
        .expect("publisher on a fresh topic");

    let raw_node = iceoryx2::node::NodeBuilder::new()
        .config(&ix)
        .create::<iceoryx2::service::ipc::Service>()
        .expect("raw iceoryx2 node on the same isolated config");
    let event_name: ServiceName = "/ceiling_probe/event".try_into().expect("service name");
    let event = raw_node
        .service_builder(&event_name)
        .event()
        .open()
        .expect("the transport must have created the event service");

    assert_eq!(
        event.static_config().event_id_max_value(),
        EXPECTED_CEILING,
        "the topic event service must be created with the transport's event-id \
         ceiling, not iceoryx2's 255 default: a listener walks one bitset entry \
         per id in that space on every wait"
    );
}

/// No event-service creation site in the transport may omit the ceiling.
///
/// A runtime arm can only reach the services a test can build; this one reads
/// the source so a NEW creation site is caught the day it is added rather than
/// the day someone re-measures.
#[test]
fn every_event_service_creation_site_passes_the_ceiling() {
    let src = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src");
    let mut checked = 0usize;
    let mut missing: Vec<String> = Vec::new();
    for path in rust_sources(&src) {
        let text = std::fs::read_to_string(&path).expect("readable source file");
        let lines: Vec<&str> = text.lines().collect();
        for (i, line) in lines.iter().enumerate() {
            // The builder call that opens an event service. `.event()` appears
            // nowhere else in these files as a whole-call token.
            if line.trim() != ".event()" {
                continue;
            }
            checked += 1;
            // The ceiling is the next builder call, modulo the comment lines a
            // reviewer is entitled to put between them.
            let armed = lines[i + 1..]
                .iter()
                .take(6)
                .any(|l| l.contains("event_id_max_value("));
            if !armed {
                missing.push(format!("{}:{}", path.display(), i + 1));
            }
        }
    }
    assert!(
        checked >= 5,
        "expected at least the five known event-service creation sites, walked {checked} \
         (a renamed builder call would make this guard inert)"
    );
    assert!(
        missing.is_empty(),
        "event service created without `event_id_max_value`: {missing:?} — one \
         uncapped creator restores iceoryx2's 255-wide bitset walk for every \
         later opener of that service"
    );
}

/// Every `.rs` file under a directory, recursively.
fn rust_sources(dir: &std::path::Path) -> Vec<std::path::PathBuf> {
    let mut out = Vec::new();
    let mut stack = vec![dir.to_path_buf()];
    while let Some(d) = stack.pop() {
        for entry in std::fs::read_dir(&d).expect("readable source directory") {
            let path = entry.expect("directory entry").path();
            if path.is_dir() {
                stack.push(path);
            } else if path.extension().and_then(|e| e.to_str()) == Some("rs") {
                out.push(path);
            }
        }
    }
    out
}
