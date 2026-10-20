// SPDX-License-Identifier: AGPL-3.0-only
//! `#[input(accumulate)]` over real iceoryx2 (`GraphRuntime::build_for_test`,
//! per-test SHM root): two writer nodes share one `multi_publisher_topics`
//! topic and one macro reader keeps every frame.
//!
//! The reader records the `x` of every frame each fire receives. Writer A
//! publishes `1, 2, 3, ...`, writer B `101, 102, 103, ...`, both in the same
//! steps, so each step's two frames share a publish stamp and a wire
//! sequence and the documented order breaks the tie on the frame bytes:
//! `x = n` sorts before `x = 100 + n` (the little-endian `f64` of 1.0 is
//! `00 00 00 00 00 00 F0 3F`, of 101.0 `00 00 00 00 00 40 59 40`; the first
//! differing byte is the sixth, `00 < 40`).
//!
//! ```bash
//! cargo test -p cerulion_core --test accumulate_input_iox2_test -- --test-threads=1
//! ```

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

const STEP: Duration = Duration::from_millis(1);
const TOPIC: &str = "/tf";

#[cerulion_node(external)]
#[derive(Default)]
struct Writer {
    #[output]
    out: Vector3,
    base: f64,
    n: u64,
}

#[cerulion_node_impl]
impl Writer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.base + self.n as f64;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// What the reader saw: one entry per fire whose body ran.
#[derive(Debug, Default, Clone, PartialEq)]
struct Seen {
    fires: Vec<Vec<u64>>,
    dropped_per_fire: Vec<u64>,
    /// `(dropped, count_total, buffer_capacity)` of each overflow event.
    events: Vec<(u64, u64, usize)>,
}

#[cerulion_node(external)]
#[derive(Default)]
struct Reader {
    #[input(accumulate)]
    tf: Vector3,
    #[cerulion(reconstruct)]
    seen: Arc<Mutex<Seen>>,
}

#[cerulion_node_impl]
impl Reader {
    fn tick(&mut self) -> Result<(), NodeError> {
        let xs: Vec<u64> = self.tf.iter().map(|f| f.x as u64).collect();
        let dropped = self.tf.dropped();
        let mut seen = self.seen.lock().expect("seen lock");
        seen.fires.push(xs);
        seen.dropped_per_fire.push(dropped);
        Ok(())
    }

    #[on_event(input = "tf")]
    fn on_tf_overflow(&mut self, event: BackpressureEvent) {
        let mut seen = self.seen.lock().expect("seen lock");
        seen.events
            .push((event.dropped, event.count_total, event.buffer_capacity));
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// The same reader with a cap of 4.
#[cerulion_node(external)]
#[derive(Default)]
struct SmallReader {
    #[input(accumulate, cap = 4)]
    tf: Vector3,
    #[cerulion(reconstruct)]
    seen: Arc<Mutex<Seen>>,
}

#[cerulion_node_impl]
impl SmallReader {
    fn tick(&mut self) -> Result<(), NodeError> {
        let xs: Vec<u64> = self.tf.iter().map(|f| f.x as u64).collect();
        let dropped = self.tf.dropped();
        let mut seen = self.seen.lock().expect("seen lock");
        seen.fires.push(xs);
        seen.dropped_per_fire.push(dropped);
        Ok(())
    }

    #[on_event(input = "tf")]
    fn on_tf_overflow(&mut self, event: BackpressureEvent) {
        let mut seen = self.seen.lock().expect("seen lock");
        seen.events
            .push((event.dropped, event.count_total, event.buffer_capacity));
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

fn writer_def(id: &str) -> NodeDef {
    NodeDef {
        fuse: None,
        ros2: None,
        id: id.to_string(),
        node_type: "writer".to_string(),
        inputs: vec![],
        outputs: vec![OutputDef {
            name: "out".to_string(),
            schema: "Vector3".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: Some(TOPIC.to_string()),
        }],
    }
}

fn build(prefix: &str, reader: Box<dyn NodeEntry>) -> GraphRuntime {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: vec![TOPIC.to_string()],
        name: None,
        identity: "accumulate_input".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            writer_def("writer_a"),
            writer_def("writer_b"),
            NodeDef {
                fuse: None,
                ros2: None,
                id: "reader".to_string(),
                node_type: "reader".to_string(),
                inputs: vec![InputDef {
                    name: "tf".to_string(),
                    source: TOPIC.to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for (id, base) in [("writer_a", 0.0), ("writer_b", 100.0)] {
        let w = Writer {
            base,
            ..Default::default()
        };
        factories.insert(id.to_string(), Box::new(WriterEntry::with_state(w)));
    }
    factories.insert("reader".to_string(), reader);
    GraphRuntime::build_for_test(config, factories, Arc::new(VirtualClock::new()), 8)
        .expect("build accumulate graph")
}

fn reader(seen: &Arc<Mutex<Seen>>) -> Box<dyn NodeEntry> {
    Box::new(ReaderEntry::with_state(Reader {
        seen: Arc::clone(seen),
        ..Default::default()
    }))
}

fn small_reader(seen: &Arc<Mutex<Seen>>) -> Box<dyn NodeEntry> {
    Box::new(SmallReaderEntry::with_state(SmallReader {
        seen: Arc::clone(seen),
        ..Default::default()
    }))
}

/// Both writers publish once per step, `steps` times; the reader stays idle.
fn publish(rt: &mut GraphRuntime, steps: usize) {
    for _ in 0..steps {
        rt.trigger_external("writer_a").expect("trigger writer_a");
        rt.trigger_external("writer_b").expect("trigger writer_b");
        rt.step(STEP);
    }
}

fn fire_reader(rt: &mut GraphRuntime) {
    rt.trigger_external("reader").expect("trigger reader");
    rt.step(STEP);
}

fn seen(s: &Arc<Mutex<Seen>>) -> Seen {
    s.lock().expect("seen lock").clone()
}

#[test]
#[serial]
fn every_frame_from_two_writers_reaches_one_fire_in_the_documented_order() {
    let s = Arc::new(Mutex::new(Seen::default()));
    let mut rt = build("acc_order", reader(&s));
    publish(&mut rt, 3);
    fire_reader(&mut rt);
    // Nothing arrived since: the next fire runs and receives no frame.
    fire_reader(&mut rt);
    let got = seen(&s);
    assert_eq!(
        got.fires,
        vec![vec![1, 101, 2, 102, 3, 103], vec![]],
        "every frame of both writers, oldest first, then an empty fire"
    );
    assert_eq!(got.dropped_per_fire, vec![0, 0]);
    assert!(
        got.events.is_empty(),
        "no overflow at 6 frames under cap 10"
    );
}

#[test]
#[serial]
fn two_runs_deliver_identical_frames() {
    let run = |prefix: &str| {
        let s = Arc::new(Mutex::new(Seen::default()));
        let mut rt = build(prefix, reader(&s));
        publish(&mut rt, 2);
        fire_reader(&mut rt);
        publish(&mut rt, 1);
        fire_reader(&mut rt);
        seen(&s).fires
    };
    let a = run("acc_det_a");
    let b = run("acc_det_b");
    let oracle = vec![vec![1, 101, 2, 102], vec![3, 103]];
    assert_eq!(a, oracle);
    assert_eq!(b, oracle);
}

#[test]
#[serial]
fn a_flood_past_the_cap_keeps_the_newest_frames_counts_the_drops_and_raises_one_event() {
    let s = Arc::new(Mutex::new(Seen::default()));
    let mut rt = build("acc_flood", small_reader(&s));
    // 3 steps x 2 writers = 6 frames into a cap of 4: the two oldest
    // (x = 1 and x = 101) are dropped.
    publish(&mut rt, 3);
    fire_reader(&mut rt);
    // A second overflow in the same regime is counted but raises no event.
    publish(&mut rt, 3);
    fire_reader(&mut rt);
    // A collect that drops nothing re-arms the event.
    publish(&mut rt, 1);
    fire_reader(&mut rt);
    publish(&mut rt, 3);
    fire_reader(&mut rt);
    let got = seen(&s);
    assert_eq!(
        got.fires,
        vec![
            vec![2, 102, 3, 103],
            vec![5, 105, 6, 106],
            vec![7, 107],
            vec![9, 109, 10, 110],
        ]
    );
    assert_eq!(got.dropped_per_fire, vec![2, 2, 0, 2]);
    assert_eq!(
        got.events,
        vec![(2, 2, 4), (2, 6, 4)],
        "one event per overflow regime: (dropped, count_total, cap)"
    );
}

#[test]
#[serial]
fn a_writer_past_the_cap_between_fires_loses_its_oldest_frames_in_its_own_queue() {
    let s = Arc::new(Mutex::new(Seen::default()));
    let mut rt = build("acc_queue", small_reader(&s));
    // 6 frames per writer into per-writer queues of depth 4: each queue
    // keeps its newest 4 (x = 3..=6 and 103..=106), the collect takes those
    // 8, and the cap of 4 keeps the newest 4 of them.
    publish(&mut rt, 6);
    fire_reader(&mut rt);
    let got = seen(&s);
    assert_eq!(got.fires, vec![vec![5, 105, 6, 106]]);
    assert_eq!(got.dropped_per_fire, vec![4], "8 collected, 4 held");
    assert_eq!(
        got.events.len(),
        1,
        "one overflow regime, one event: {:?}",
        got.events
    );
}
