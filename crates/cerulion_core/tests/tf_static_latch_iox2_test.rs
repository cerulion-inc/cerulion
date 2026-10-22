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

/// Graph B, built after graph A has published, on A's transport: one
/// listener on `topic`. Returns every `transforms` payload it received,
/// sorted so the oracle does not depend on which publisher's history
/// lands first.
fn late_listener(a: &mut GraphRuntime, prefix: &str, topic: &str) -> Vec<Vec<u8>> {
    let mgr = a.test_transport().expect("test transport parked");
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
