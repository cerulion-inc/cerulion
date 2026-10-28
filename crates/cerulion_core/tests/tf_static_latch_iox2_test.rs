// SPDX-License-Identifier: AGPL-3.0-only
//! `/tf_static` is latched, over real iceoryx2 (`GraphRuntime::build_for_test`,
//! per-test SHM root).
//!
//! Graph A holds the static publishers: `mounts` and `arm` publish
//! `tf2_msgs/TFMessage` on `/tf_static` with no `history_size` in their
//! output. Graph B starts after A has published and holds one listener with
//! an accumulate input on `/tf_static`. A late listener must still receive
//! every publisher's last message: an output on `/tf_static` with the
//! `TFMessage` schema keeps one message for late listeners by default.
//!
//! The `transforms` bytes are a test encoding, one `child=x` entry per static
//! transform. `mounts` sends its whole static set in every message, merged by
//! child frame: its second message re-publishes `lidar` at a new offset and
//! keeps `cam`, so the message it keeps holds both, with `lidar` replaced.
//!
//! Start order must not matter: a listener graph that starts before its
//! static publisher creates `/tf_static`, and the publisher must still load
//! and keep its message for a listener that starts after it.
//!
//! ```bash
//! cargo test -p cerulion_core --test tf_static_latch_iox2_test -- --test-threads=1
//! ```

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::TransportManager;
use indexmap::IndexMap;
use native_ros2_messages::tf2_msgs::TFMessage;
use serial_test::serial;

const STEP: Duration = Duration::from_millis(1);
const TF_STATIC: &str = "/tf_static";

/// A static publisher: on fire `n` it sends `sets[n]`, or the last set once
/// the list runs out.
#[cerulion_node(external)]
#[derive(Default)]
struct StaticPub {
    #[output]
    tf_static: TFMessage,
    sets: Vec<Vec<u8>>,
    fired: usize,
}

#[cerulion_node_impl]
impl StaticPub {
    fn tick(&mut self) -> Result<(), NodeError> {
        let last = self.sets.len().saturating_sub(1);
        let set = &self.sets[self.fired.min(last)];
        self.fired += 1;
        self.tf_static
            .set_transforms_bytes(set)
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// The listener: records the `transforms` bytes of every frame it receives.
#[cerulion_node(external)]
#[derive(Default)]
struct Listener {
    #[input(accumulate, cap = 4)]
    tf_static: TFMessage,
    #[cerulion(reconstruct)]
    seen: Arc<Mutex<Vec<Vec<u8>>>>,
}

#[cerulion_node_impl]
impl Listener {
    fn tick(&mut self) -> Result<(), NodeError> {
        let mut seen = self.seen.lock().expect("seen lock");
        for frame in self.tf_static.iter() {
            seen.push(frame.transforms_bytes().to_vec());
        }
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// A listener whose cap of 2 cannot hold three static publishers.
#[cerulion_node(external)]
#[derive(Default)]
struct SmallListener {
    #[input(accumulate, cap = 2)]
    tf_static: TFMessage,
}

#[cerulion_node_impl]
impl SmallListener {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.tf_static.len();
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// `mounts` first sends `lidar` and `cam`, then re-publishes `lidar` moved.
const MOUNTS_FIRST: &[u8] = b"lidar=0.25;cam=0.50";
const MOUNTS_MERGED: &[u8] = b"lidar=0.30;cam=0.50";
const ARM: &[u8] = b"gripper=0.10";

fn publisher_def(id: &str, topic: &str) -> NodeDef {
    NodeDef {
        fuse: None,
        params: Default::default(),
        ros2: None,
        id: id.to_string(),
        node_type: "static_pub".to_string(),
        inputs: vec![],
        outputs: vec![OutputDef {
            name: "tf_static".to_string(),
            schema: "tf2_msgs/TFMessage".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: Some(topic.to_string()),
        }],
    }
}

fn listener_def(id: &str, topic: &str) -> NodeDef {
    NodeDef {
        fuse: None,
        params: Default::default(),
        ros2: None,
        id: id.to_string(),
        node_type: "listener".to_string(),
        inputs: vec![InputDef {
            name: "tf_static".to_string(),
            source: topic.to_string(),
        }],
        outputs: vec![],
    }
}

fn graph(prefix: &str, multi: &[&str], nodes: Vec<NodeDef>) -> GraphConfig {
    GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: multi.iter().map(|t| t.to_string()).collect(),
        name: None,
        identity: prefix.to_string(),
        prefix: prefix.to_string(),
        nodes,
    }
}

fn static_pub(sets: Vec<&'static [u8]>) -> Box<dyn NodeEntry> {
    Box::new(StaticPubEntry::with_state(StaticPub {
        sets: sets.into_iter().map(<[u8]>::to_vec).collect(),
        ..Default::default()
    }))
}

/// Graph A: `mounts` and `arm` on `topic`, each fired twice.
fn publishers(prefix: &str, topic: &str) -> GraphRuntime {
    let config = graph(
        prefix,
        &[topic],
        vec![publisher_def("mounts", topic), publisher_def("arm", topic)],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "mounts".to_string(),
        static_pub(vec![MOUNTS_FIRST, MOUNTS_MERGED]),
    );
    factories.insert("arm".to_string(), static_pub(vec![ARM]));
    let mut rt = GraphRuntime::build_for_test(config, factories, Arc::new(VirtualClock::new()), 8)
        .expect("build the publisher graph");
    for _ in 0..2 {
        rt.trigger_external("mounts").expect("trigger mounts");
        rt.trigger_external("arm").expect("trigger arm");
        rt.step(STEP);
    }
    rt
}

/// A listener graph's config, its factory, and the frames its listener
/// records.
type ListenerGraph = (
    GraphConfig,
    IndexMap<String, Box<dyn NodeEntry>>,
    Arc<Mutex<Vec<Vec<u8>>>>,
);

/// The listener graph's config and factory: one `listener` on `topic` that
/// records into the returned `seen`.
fn listener_graph(prefix: &str, topic: &str) -> ListenerGraph {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let config = graph(prefix, &[], vec![listener_def("listener", topic)]);
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "listener".to_string(),
        Box::new(ListenerEntry::with_state(Listener {
            seen: Arc::clone(&seen),
            ..Default::default()
        })),
    );
    (config, factories, seen)
}

/// Graph B, built after graph A has published, on A's transport: one
/// listener on `topic`. Returns every `transforms` payload it received,
/// sorted so the oracle does not depend on which publisher's history
/// lands first.
fn late_listener(a: &mut GraphRuntime, prefix: &str, topic: &str) -> Vec<Vec<u8>> {
    let mgr = Arc::clone(a.test_transport().expect("test transport parked"));
    late_listener_on(&mgr, a, prefix, topic)
}

/// [`late_listener`] on an explicit transport `mgr`, for a publisher graph
/// `a` that was built on another graph's transport.
fn late_listener_on(
    mgr: &Arc<TransportManager>,
    a: &mut GraphRuntime,
    prefix: &str,
    topic: &str,
) -> Vec<Vec<u8>> {
    let (config, factories, seen) = listener_graph(prefix, topic);
    let mut b = GraphRuntime::build(config, factories, mgr, Arc::new(VirtualClock::new()))
        .expect("build the late listener graph");
    // Graph A sends nothing more: the publishers only hand their kept
    // messages to the new subscriber.
    for _ in 0..4 {
        a.pump_history_all_for_test();
        b.trigger_external("listener").expect("trigger listener");
        b.step(STEP);
    }
    let mut got = seen.lock().expect("seen lock").clone();
    got.sort();
    got
}

fn sorted(frames: &[&[u8]]) -> Vec<Vec<u8>> {
    let mut v: Vec<Vec<u8>> = frames.iter().map(|f| f.to_vec()).collect();
    v.sort();
    v
}

#[test]
#[serial]
fn a_listener_that_starts_after_the_static_publishers_receives_every_static_frame() {
    let mut a = publishers("latch_a", TF_STATIC);
    let got = late_listener(&mut a, "latch_b", TF_STATIC);
    // One kept message per publisher, the last each sent: `mounts`'s merged
    // set, with `lidar` replaced and `cam` kept, and `arm`'s set. The first
    // `mounts` message is not kept.
    assert_eq!(got, sorted(&[MOUNTS_MERGED, ARM]));
}

#[test]
#[serial]
fn a_topic_other_than_tf_static_keeps_nothing_for_a_late_listener() {
    // The same publishers on `/tf` with no `history_size`: `/tf` is not
    // latched, so the late listener receives nothing.
    let mut a = publishers("volatile_a", "/tf");
    let got = late_listener(&mut a, "volatile_b", "/tf");
    assert_eq!(got, Vec::<Vec<u8>>::new());
}

#[test]
#[serial]
fn two_late_listeners_in_two_runs_receive_the_same_frames() {
    let run = |a_prefix: &str, b_prefix: &str| {
        let mut a = publishers(a_prefix, TF_STATIC);
        late_listener(&mut a, b_prefix, TF_STATIC)
    };
    let first = run("det_a1", "det_b1");
    let second = run("det_a2", "det_b2");
    let oracle = sorted(&[MOUNTS_MERGED, ARM]);
    assert_eq!(first, oracle);
    assert_eq!(second, oracle);
}

#[test]
#[serial]
fn a_listener_that_starts_before_its_static_publisher_does_not_block_the_latch() {
    // Graph B starts first, with no publisher in it: its listener creates
    // `/tf_static`.
    let (config, factories, seen_b) = listener_graph("first_b", TF_STATIC);
    let mut b = GraphRuntime::build_for_test(config, factories, Arc::new(VirtualClock::new()), 8)
        .expect("build the listener graph first");
    let mgr = Arc::clone(b.test_transport().expect("test transport parked"));
    // Graph A starts after, on the same transport: one static publisher
    // with no `history_size`, which keeps one message by default. The topic
    // B created must admit that.
    let config = graph("first_a", &[], vec![publisher_def("mounts", TF_STATIC)]);
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "mounts".to_string(),
        static_pub(vec![MOUNTS_FIRST, MOUNTS_MERGED]),
    );
    let mut a = GraphRuntime::build(config, factories, &mgr, Arc::new(VirtualClock::new()))
        .expect("a static publisher loads after a listener created /tf_static");
    for _ in 0..2 {
        a.trigger_external("mounts").expect("trigger mounts");
        a.step(STEP);
        b.trigger_external("listener").expect("trigger listener");
        b.step(STEP);
    }
    // B listened from the start, so it received both messages live.
    let mut live = seen_b.lock().expect("seen lock").clone();
    live.sort();
    assert_eq!(live, sorted(&[MOUNTS_FIRST, MOUNTS_MERGED]));
    // Graph C starts after A has published: the topic B created keeps the
    // publisher's last message for it.
    let got = late_listener_on(&mgr, &mut a, "first_c", TF_STATIC);
    assert_eq!(got, sorted(&[MOUNTS_MERGED]));
}

#[test]
#[serial]
fn a_raw_subscriber_that_created_tf_static_without_history_refuses_the_publisher_and_names_the_remedy(
) {
    // A transport to share: a listener graph on `/tf`, which does not touch
    // `/tf_static`.
    let (config, factories, _seen) = listener_graph("raw_t", "/tf");
    let t = GraphRuntime::build_for_test(config, factories, Arc::new(VirtualClock::new()), 8)
        .expect("build the transport holder");
    let mgr = Arc::clone(t.test_transport().expect("test transport parked"));
    // A raw opener outside any graph creates `/tf_static` with no history.
    // It grants the 3 borrowed samples an accumulate listener requires on
    // open, so the history axis is the only one the graphs below meet.
    let mut raw_config = mgr.default_topic_config();
    raw_config.subscriber_max_borrowed_samples = Some(3);
    let _raw = mgr
        .create_subscriber_with_buffers(TF_STATIC, raw_config, 8)
        .expect("a raw subscriber creates /tf_static");
    // The static publisher requires the latch history, which an existing
    // topic cannot gain: the refusal says so and names what to do.
    let config = graph("raw_a", &[], vec![publisher_def("mounts", TF_STATIC)]);
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("mounts".to_string(), static_pub(vec![ARM]));
    match GraphRuntime::build(config, factories, &mgr, Arc::new(VirtualClock::new())) {
        Err(TransportError::PublisherCreation { topic, reason }) => {
            assert_eq!(topic, TF_STATIC);
            assert!(
                reason.contains("DoesNotSupportRequestedMinHistorySize"),
                "{reason}"
            );
            assert!(
                reason.contains(
                    "the service already exists with less history than this publisher \
                     keeps for late listeners (a latched /tf_static output keeps 1, a \
                     declared history_size keeps that many), and the history of an \
                     existing service cannot be raised: stop the earlier creator and \
                     restart this graph, or start the publishing graph first"
                ),
                "{reason}"
            );
        }
        Err(other) => panic!("expected PublisherCreation, got {other}"),
        Ok(_) => panic!("a static publisher must not open a /tf_static that keeps no history"),
    }
    // A listener graph requires no history: it attaches to the topic as it is.
    let (config, factories, _seen) = listener_graph("raw_b", TF_STATIC);
    if let Err(e) = GraphRuntime::build(config, factories, &mgr, Arc::new(VirtualClock::new())) {
        panic!("a listener attaches to a /tf_static that keeps no history: {e}");
    }
}

#[test]
#[serial]
fn a_listener_cap_below_the_static_publisher_count_refuses_the_graph() {
    let config = graph(
        "cap_refused",
        &[TF_STATIC],
        vec![
            publisher_def("mounts", TF_STATIC),
            publisher_def("arm", TF_STATIC),
            publisher_def("head", TF_STATIC),
            listener_def("small", TF_STATIC),
        ],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for id in ["mounts", "arm", "head"] {
        factories.insert(id.to_string(), static_pub(vec![ARM]));
    }
    factories.insert(
        "small".to_string(),
        Box::new(SmallListenerEntry::with_state(SmallListener::default())),
    );
    let result = GraphRuntime::build_for_test(config, factories, Arc::new(VirtualClock::new()), 8);
    match result {
        Err(TransportError::TfStaticCapTooSmall {
            node_id,
            input,
            cap,
            required,
            limit,
            depth,
            publishers,
        }) => {
            assert_eq!((node_id.as_str(), input.as_str()), ("small", "tf_static"));
            assert_eq!((cap, required, limit, depth), (2, 3, 16, 1));
            assert_eq!(
                publishers,
                vec!["mounts.tf_static", "arm.tf_static", "head.tf_static"]
            );
        }
        Err(other) => panic!("expected TfStaticCapTooSmall, got {other}"),
        Ok(_) => panic!("a cap of 2 under three static publishers must refuse the graph"),
    }
}

#[test]
#[serial]
fn a_listener_cap_that_holds_every_static_publisher_builds() {
    let config = graph(
        "cap_fits",
        &[TF_STATIC],
        vec![
            publisher_def("mounts", TF_STATIC),
            publisher_def("arm", TF_STATIC),
            listener_def("small", TF_STATIC),
        ],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for id in ["mounts", "arm"] {
        factories.insert(id.to_string(), static_pub(vec![ARM]));
    }
    factories.insert(
        "small".to_string(),
        Box::new(SmallListenerEntry::with_state(SmallListener::default())),
    );
    assert!(
        GraphRuntime::build_for_test(config, factories, Arc::new(VirtualClock::new()), 8).is_ok(),
        "a cap of 2 holds two static publishers"
    );
}
