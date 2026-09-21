// SPDX-License-Identifier: AGPL-3.0-only
//! `graph-max-slice-len`: a real graph built from a LOADED cdylib
//! must PROVISION its iceoryx2 topic's publisher at the schema's tier-2
//! `max_slice_len` default (`OutputMeta::max_slice_len_default`, keyed off
//! `variable_schema_max_slice_len` in
//! `cerulion_core/src/codegen/generator/wire_impl.rs`) — not just carry the
//! value through the FFI info-JSON parse (already pinned in
//! `macro_cdylib_test.rs`).
//!
//! `sensor_msgs/Image` resolves to `TIER_HUGE = 128 * 1024 * 1024` (128 MiB)
//! — VERIFIED by reading `wire_impl.rs` directly (tier constants have
//! changed before; do not trust this comment alone on a future read). That
//! happens to be the SAME numeric value as `DEFAULT_MAX_SLICE_LEN` (the
//! tier-3 universal fallback), so a bare "the live publisher's
//! `max_slice_len` == 128 MiB" assertion is VACUOUS on its own — it cannot
//! distinguish "tier-2 resolved correctly" from "tier-2 was silently
//! skipped and tier-3 fired instead". This file closes that gap two ways:
//!
//! 1. **No-override run**: assert the LIVE cdylib-attached publisher's
//!    `max_slice_len` (read via RAW iceoryx2 `dynamic_config().list_publishers`
//!    — no Cerulion API surfaces this) equals 128 MiB, AND assert the
//!    tier-3 fallback's `tracing::warn!` ("no max_slice_len configured")
//!    did NOT fire — the ABSENCE of that warn is what actually proves
//!    tier-2 (not tier-3) resolved.
//! 2. **Tier-1-override run**: the same graph with an EXPLICIT
//!    `max_slice_len: 4096` YAML override, proving the SAME raw-introspection
//!    mechanism reflects a DIFFERENT, smaller value end-to-end through the
//!    cdylib FFI — the resolver's output genuinely reaches the real
//!    iceoryx2 publisher, not just a log line.
//!
//! # Design note: why there is no "opener requiring more is rejected" arm
//!
//! `topic_buffer_sizing_test.rs` proves buffer-ceiling requirements
//! (`subscriber_max_buffer_size` / `max_subscribers` / `max_publishers`) are
//! real by showing an opener requiring MORE than provisioned is rejected by
//! iceoryx2 itself. `max_slice_len` has NO analogous mechanism: verified by
//! reading iceoryx2 0.9.1 source (`service/builder/publish_subscribe.rs`,
//! `service/port_factory/subscriber.rs`), `initial_max_slice_len` is a
//! PER-PUBLISHER-PORT construction parameter, not a service-level static
//! config a subscriber (or another publisher) can arm as an open-time
//! requirement — there is no `PublishSubscribeOpenError` variant for a
//! slice-len mismatch. The ceiling is real (it sizes that publisher's own
//! SHM data segment) but is only observable via the connected publisher's
//! own `PublisherDetails.max_slice_len`, read through
//! `dynamic_config().list_publishers()` — a raw iceoryx2 introspection API,
//! not a Cerulion one — which is what this file uses instead.
//!
//! # Build requirement + serial
//!
//! Requires `cargo build -p test_node_macro_portwrite_cdylib` (already built
//! for `cdylib_portwrite_e2e_test.rs`). `#[serial]` (cdylib `NODES`
//! singleton + iceoryx2 SHM singleton); each test builds its own isolated
//! iceoryx2 namespace via `cerulion_core::testing::iceoryx_test_config()`
//! and reuses that SAME config for the raw introspection node, so the two
//! tests in this file cannot collide with each other or with other files.

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::{Clock, VirtualClock};
use cerulion_core::graph::config::{
    GraphConfig, InputDef, NodeDef, OutputDef, DEFAULT_MAX_SLICE_LEN,
};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::{TransportConfig, TransportManager};
use iceoryx2::prelude::PortFactory;
use indexmap::IndexMap;
use native_ros2_messages::sensor_msgs::Image;
use native_ros2_messages::std_msgs::String as RosString;
use serial_test::serial;

/// `sensor_msgs/Image`'s tier-2 constant (`TIER_HUGE` in
/// `codegen/generator/wire_impl.rs`) — VERIFIED by reading that file, not
/// assumed. Named separately from `DEFAULT_MAX_SLICE_LEN` even though the
/// two are numerically equal for `Image`, to make the coincidence explicit
/// at every call site below (see the module doc).
const IMAGE_TIER2_MAX_SLICE_LEN: u32 = 128 * 1024 * 1024;

fn find_cdylib(crate_name: &str) -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib(crate_name)
}

fn portwrite_cdylib() -> Box<dyn NodeEntry> {
    Box::new(
        DylibNodeEntry::load(&find_cdylib("test_node_macro_portwrite_cdylib"))
            .expect("load portwrite fixture"),
    )
}

/// Drives the portwrite cdylib's `text_in` trigger input (crib of
/// `cdylib_portwrite_e2e_test.rs::TextProducer`).
#[cerulion_node(period_ms = 10)]
struct TextProducer {
    #[output]
    text: RosString,
}
#[cerulion_node_impl]
impl TextProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.text.data = "drive";
        Ok(())
    }
}

/// Minimal downstream drain — this file only cares about publisher
/// PROVISIONING, not delivered payload content.
#[cerulion_node]
#[derive(Default)]
struct ImageDrain {
    #[input(trigger)]
    image: Image,
}
#[cerulion_node_impl]
impl ImageDrain {
    fn tick(&mut self) -> Result<(), NodeError> {
        Ok(())
    }
}

/// Build `producer(String) -> portwrite(text_in trigger, image out) ->
/// drain(image trigger)` over a manager whose `ix` config + handle the
/// caller keeps ALIVE for the lifetime of the test, so it can later open a
/// RAW iceoryx2 node against the SAME namespace to introspect the live
/// publisher (iceoryx2 reaps dead nodes on open — dropping the manager
/// early could make the live publisher look stale to that raw open).
/// `max_slice_len_override` threads a tier-1 YAML value onto the `image`
/// output when `Some`; `None` leaves it unset so tier-2 (the schema
/// default) resolves. `text`'s own `max_slice_len` is ALWAYS explicit
/// (tier-1) so it never trips the tier-3 warn — keeping the warn-absence
/// assertion in test 1 scoped to the `image` topic only.
fn build_graph(
    prefix: &str,
    max_slice_len_override: Option<usize>,
) -> (
    GraphRuntime,
    Arc<TransportManager>,
    iceoryx2::config::Config,
    String,
) {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("msl_{prefix}"),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "producer".to_string(),
                node_type: "text_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "text".to_string(),
                    schema: "String".to_string(),
                    max_slice_len: Some(4096),
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                ros2: None,
                id: "portwrite".to_string(),
                node_type: "port_write".to_string(),
                inputs: vec![InputDef {
                    name: "text_in".to_string(),
                    source: "producer/text".to_string(),
                }],
                outputs: vec![OutputDef {
                    name: "image".to_string(),
                    schema: "Image".to_string(),
                    max_slice_len: max_slice_len_override,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                ros2: None,
                id: "drain".to_string(),
                node_type: "image_drain".to_string(),
                inputs: vec![InputDef {
                    name: "image".to_string(),
                    source: "portwrite/image".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(TextProducerEntry::new()));
    factories.insert("portwrite".to_string(), portwrite_cdylib());
    factories.insert("drain".to_string(), Box::new(ImageDrainEntry::new()));

    let ix = cerulion_core::testing::iceoryx_test_config();
    let clock = Arc::new(VirtualClock::new());
    let clock_dyn: Arc<dyn Clock> = clock.clone();
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("msl_{prefix}"),
            clock: clock_dyn,
            subscriber_buffer_size: 8,
            network: None,
        },
        ix.clone(),
    )
    .expect("init_for_test");
    let mut runtime =
        GraphRuntime::build(config, factories, &mgr, clock).expect("build max_slice_len graph");
    for _ in 0..8 {
        runtime.step(Duration::from_millis(10));
    }
    let topic = format!("/{prefix}/portwrite/image");
    (runtime, mgr, ix, topic)
}

/// Read every connected publisher's `max_slice_len` on `topic`'s data
/// service via RAW iceoryx2 introspection (no Cerulion API surfaces this —
/// see the module doc). Panics if the service doesn't exist.
fn live_publisher_max_slice_lens(ix: &iceoryx2::config::Config, topic: &str) -> Vec<u32> {
    let raw_node = iceoryx2::node::NodeBuilder::new()
        .config(ix)
        .create::<iceoryx2::service::ipc::Service>()
        .expect("raw iceoryx2 node");
    let data_name: iceoryx2::service::service_name::ServiceName = format!("{topic}/data")
        .as_str()
        .try_into()
        .expect("service name");
    let data_service = raw_node
        .service_builder(&data_name)
        .publish_subscribe::<[u8]>()
        .open()
        .expect("the graph must have already created this topic's data service");
    let mut found = Vec::new();
    data_service.dynamic_config().list_publishers(|details| {
        found.push(details.max_slice_len as u32);
        iceoryx2::prelude::CallbackProgression::Continue
    });
    found
}

// ===========================================================================
// 1. No override: tier-2 (schema default) must resolve, NOT tier-3
// ===========================================================================

#[test]
#[serial]
#[tracing_test::traced_test]
fn tier2_schema_default_reaches_live_cdylib_publisher_with_no_tier3_fallback() {
    let (runtime, _mgr, ix, topic) = build_graph("mslt2", None);
    assert!(
        runtime.node_handle("portwrite").unwrap().fire_count() > 0,
        "the portwrite cdylib must have fired"
    );

    let publishers = live_publisher_max_slice_lens(&ix, &topic);
    assert_eq!(
        publishers.len(),
        1,
        "exactly one live publisher (the graph-owned cdylib producer) on {topic}"
    );
    assert_eq!(
        publishers[0], IMAGE_TIER2_MAX_SLICE_LEN,
        "the live cdylib-attached publisher must be provisioned at the schema's \
         tier-2 max_slice_len ({IMAGE_TIER2_MAX_SLICE_LEN} bytes)"
    );
    assert_eq!(
        IMAGE_TIER2_MAX_SLICE_LEN, DEFAULT_MAX_SLICE_LEN as u32,
        "sanity: Image's tier-2 constant coincides numerically with tier-3's \
         universal default — this is WHY the log-absence assertion below is \
         load-bearing, not the numeric assertion above"
    );
    assert!(
        !logs_contain("no max_slice_len configured"),
        "the tier-3 fallback warn must NOT fire — the resolved value above came \
         from tier-2 (the schema default), not a silent fall-through to tier-3"
    );
}

// ===========================================================================
// 2. Explicit tier-1 YAML override reaches the SAME live introspection,
//    proving the mechanism end-to-end (not degenerate on Image alone)
// ===========================================================================

#[test]
#[serial]
fn tier1_yaml_override_reaches_live_cdylib_publisher_and_differs_from_tier2() {
    let (runtime, _mgr, ix, topic) = build_graph("mslt1", Some(4096));
    assert!(
        runtime.node_handle("portwrite").unwrap().fire_count() > 0,
        "the portwrite cdylib must have fired"
    );

    let publishers = live_publisher_max_slice_lens(&ix, &topic);
    assert_eq!(publishers.len(), 1, "exactly one live publisher on {topic}");
    assert_eq!(
        publishers[0], 4096,
        "an explicit tier-1 YAML max_slice_len must reach the live cdylib-attached \
         publisher, overriding the schema's tier-2 default"
    );
    assert_ne!(
        publishers[0], IMAGE_TIER2_MAX_SLICE_LEN,
        "the override must genuinely differ from the no-override (tier-2) run — \
         proves the resolver's output really varies with YAML, not a hardcoded value"
    );
}
