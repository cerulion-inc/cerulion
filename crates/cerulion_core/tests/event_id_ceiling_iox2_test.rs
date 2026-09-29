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
//! every later opener on that service. The arms below exist because of that
//! asymmetry: two read the ceiling back off live services, one proves the
//! asymmetry itself by being refused, and the last proves no creation site was
//! left out.

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

/// The identifier every creation site must pass, checked as text by the source
/// walk below. A literal there would be green while restoring the 255-wide walk
/// on the next bump of `PubSubEvent`.
const CEILING_IDENT: &str = "CERULION_MAX_EVENT_ID";

fn raw_node(
    ix: &iceoryx2::config::Config,
) -> iceoryx2::node::Node<iceoryx2::service::ipc::Service> {
    iceoryx2::node::NodeBuilder::new()
        .config(ix)
        .create::<iceoryx2::service::ipc::Service>()
        .expect("raw iceoryx2 node on the same isolated config")
}

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

    let node = raw_node(&ix);
    let event_name: ServiceName = "/ceiling_probe/event".try_into().expect("service name");
    let event = node
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

/// The run registry's doorbell carries the ceiling too.
///
/// Five sites create an event service and only one of them is a topic service,
/// so a single live read-back leaves four covered by the source walk alone.
/// This is the second one read off shared memory: its listener sits in a timed
/// wait that returns on every interval, so a 255-wide bitset here is paid per
/// tick for the life of the run.
#[test]
fn the_run_registry_doorbell_is_created_with_the_event_id_ceiling() {
    use cerulion_core::transport::run_registry::{RunHandle, RunRecord, RunState};

    let ix = cerulion_core::testing::iceoryx_test_config();
    let _run = RunHandle::publish_on_config(
        &ix,
        RunRecord {
            run_id: cerulion_core::transport::run_registry::mint_run_id(),
            supervisor_pid: std::process::id(),
            run_started_at_ns: 0,
            state: RunState::Live,
            graph_name: "ceiling_probe".into(),
            run_dir: "/tmp/ceiling_probe".into(),
        },
    )
    .expect("a run registry writer on an isolated config");

    let node = raw_node(&ix);
    let name: ServiceName =
        cerulion_core::transport::run_registry::RUN_REGISTRY_DOORBELL_SERVICE_NAME
            .try_into()
            .expect("service name");
    let event = node
        .service_builder(&name)
        .event()
        .open()
        .expect("the run registry must have created its doorbell service");

    assert_eq!(
        event.static_config().event_id_max_value(),
        EXPECTED_CEILING,
        "the run registry doorbell must be created with the transport's event-id \
         ceiling, not iceoryx2's 255 default"
    );
}

/// The asymmetry this whole file rests on, asserted rather than assumed.
///
/// The header says an open fails only when the existing ceiling is BELOW the
/// one requested, which is why one forgetful creator is enough to widen the
/// walk for everyone and why a source walk is needed at all. If iceoryx2 ever
/// refused a narrower request too, the argument changes and so does the guard.
#[test]
fn an_open_above_the_ceiling_is_refused_and_an_open_below_it_is_not() {
    use iceoryx2::service::builder::event::EventOpenError;

    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "event_id_ceiling_asymmetry".into(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 4,
            network: None,
        },
        ix.clone(),
    )
    .expect("transport manager on an isolated config");
    let _pub = mgr
        .create_publisher_simple("/ceiling_asymmetry", MaxSliceLen::const_new(64))
        .expect("publisher on a fresh topic");

    let node = raw_node(&ix);
    let event_name: ServiceName = "/ceiling_asymmetry/event".try_into().expect("service name");

    let refused = node
        .service_builder(&event_name)
        .event()
        .event_id_max_value(EXPECTED_CEILING + 1)
        .open();
    assert!(
        matches!(
            refused,
            Err(EventOpenError::DoesNotSupportRequestedMaxEventId)
        ),
        "asking for a HIGHER ceiling than the service was created with must be \
         refused with DoesNotSupportRequestedMaxEventId; got {refused:?}"
    );

    let accepted = node
        .service_builder(&event_name)
        .event()
        .event_id_max_value(EXPECTED_CEILING - 1)
        .open();
    assert!(
        accepted.is_ok(),
        "asking for a LOWER ceiling must be accepted, which is exactly why one \
         creation site that omits the ceiling widens the bitset for every later \
         opener and cannot be caught by an open; got {:?}",
        accepted.err()
    );
}

/// No event-service creation site anywhere in the workspace may omit the
/// ceiling, or pass anything other than the shared constant.
///
/// A runtime arm can only reach the services a test can build; this one reads
/// the source so a NEW creation site is caught the day it is added rather than
/// the day someone re-measures. It is rooted at the workspace rather than at
/// this crate because the property is machine wide: an event service created by
/// the CLI engine, the visualizer or the bag daemon widens the same bitset for
/// every later opener.
#[test]
fn every_event_service_creation_site_passes_the_ceiling() {
    let crates_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("crates/ is the parent of this manifest")
        .to_path_buf();
    let mut roots: Vec<std::path::PathBuf> = Vec::new();
    for entry in std::fs::read_dir(&crates_dir).expect("read crates/") {
        let dir = entry.expect("directory entry").path();
        for sub in ["src", "bin"] {
            let candidate = dir.join(sub);
            if candidate.is_dir() {
                roots.push(candidate);
            }
        }
    }
    roots.sort();
    assert!(
        roots.len() >= 10,
        "the source walk found only {} crate source trees under {}, so it would \
         pass without checking anything",
        roots.len(),
        crates_dir.display()
    );

    let mut checked = 0usize;
    let mut missing: Vec<String> = Vec::new();
    let mut wrong_argument: Vec<String> = Vec::new();
    for root in &roots {
        for path in rust_sources(root) {
            let text = std::fs::read_to_string(&path).expect("readable source file");
            let lines: Vec<&str> = text.lines().collect();
            for (i, line) in lines.iter().enumerate() {
                // The builder call that opens an event service, matched as a
                // TOKEN anywhere on the line: a site written on one line, or
                // with a trailing comment, is the same site.
                if !line.contains(".event()") {
                    continue;
                }
                checked += 1;
                // The ceiling is on the same line or in the next few, modulo the
                // comment lines a reviewer is entitled to put between them.
                let window = lines[i..].iter().take(7);
                let mut armed = false;
                for candidate in window {
                    let Some(at) = candidate.find("event_id_max_value(") else {
                        continue;
                    };
                    let arg = candidate[at + "event_id_max_value(".len()..]
                        .split(')')
                        .next()
                        .unwrap_or("")
                        .trim();
                    armed = true;
                    // `CERULION_MAX_EVENT_ID` or `super::CERULION_MAX_EVENT_ID`,
                    // never a literal: a literal is green today and restores the
                    // 255-wide walk the next time `PubSubEvent` gains a variant.
                    if !arg.ends_with(CEILING_IDENT) {
                        wrong_argument.push(format!("{}:{} passes `{arg}`", path.display(), i + 1));
                    }
                    break;
                }
                if !armed {
                    missing.push(format!("{}:{}", path.display(), i + 1));
                }
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
        "event service created without `event_id_max_value`: {missing:?}, one \
         uncapped creator restores iceoryx2's 255-wide bitset walk for every \
         later opener of that service"
    );
    assert!(
        wrong_argument.is_empty(),
        "event service created with a ceiling that is not `{CEILING_IDENT}`: \
         {wrong_argument:?}, the ceiling must track `PubSubEvent::MAX_ID`, and a \
         hand written number stops tracking it the moment a variant is added"
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
