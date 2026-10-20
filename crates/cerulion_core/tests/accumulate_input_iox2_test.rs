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
    /// The `x` of every frame, kept as the `f64` it is on the wire.
    fires: Vec<Vec<f64>>,
    dropped_per_fire: Vec<u64>,
    /// `dropped_total()` at each fire (recorded by the readers that track it).
    dropped_totals: Vec<u64>,
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
        let xs: Vec<f64> = self.tf.iter().map(|f| f.x).collect();
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
        let xs: Vec<f64> = self.tf.iter().map(|f| f.x).collect();
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

/// The same reader with a cap of 3.
#[cerulion_node(external)]
#[derive(Default)]
struct TinyReader {
    #[input(accumulate, cap = 3)]
    tf: Vector3,
    #[cerulion(reconstruct)]
    seen: Arc<Mutex<Seen>>,
}

#[cerulion_node_impl]
impl TinyReader {
    fn tick(&mut self) -> Result<(), NodeError> {
        let xs: Vec<f64> = self.tf.iter().map(|f| f.x).collect();
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

/// An accumulate input beside a latest-value input declared first: until
/// `gate` delivers, the tick body cannot run.
#[cerulion_node(external)]
#[derive(Default)]
struct GatedReader {
    #[input]
    gate: Vector3,
    #[input(accumulate)]
    tf: Vector3,
    #[cerulion(reconstruct)]
    seen: Arc<Mutex<Seen>>,
}

#[cerulion_node_impl]
impl GatedReader {
    fn tick(&mut self) -> Result<(), NodeError> {
        let xs: Vec<f64> = self.tf.iter().map(|f| f.x).collect();
        let _gate = self.gate.x;
        self.seen.lock().expect("seen lock").fires.push(xs);
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// A data-trigger node with an accumulate side input: one fire per `tick`
/// frame. Records `(tick x, tf frames)` per fire.
#[cerulion_node]
#[derive(Default)]
struct TriggeredReader {
    #[input(trigger)]
    tick: Vector3,
    #[input(accumulate)]
    tf: Vector3,
    #[cerulion(reconstruct)]
    seen: Arc<Mutex<Seen>>,
}

#[cerulion_node_impl]
impl TriggeredReader {
    fn tick(&mut self) -> Result<(), NodeError> {
        let tick = self.tick.x;
        let mut xs: Vec<f64> = Vec::new();
        xs.push(tick);
        xs.extend(self.tf.iter().map(|f| f.x));
        self.seen.lock().expect("seen lock").fires.push(xs);
        Ok(())
    }
}

/// Two accumulate ports, one named like the other's generated local
/// (`tf` and `tf_frames`), each on its own topic.
#[cerulion_node(external)]
#[derive(Default)]
struct TwoPortReader {
    #[input(accumulate)]
    tf_frames: Vector3,
    #[input(accumulate)]
    tf: Vector3,
    #[cerulion(reconstruct)]
    seen: Arc<Mutex<Seen>>,
}

#[cerulion_node_impl]
impl TwoPortReader {
    fn tick(&mut self) -> Result<(), NodeError> {
        let a: Vec<f64> = self.tf_frames.iter().map(|f| f.x).collect();
        let b: Vec<f64> = self.tf.iter().map(|f| f.x).collect();
        let mut seen = self.seen.lock().expect("seen lock");
        seen.fires.push(a);
        seen.fires.push(b);
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// A cap above the shared multi-publisher buffer ceiling of 16.
#[cerulion_node(external)]
#[derive(Default)]
struct DeepReader {
    #[input(accumulate, cap = 17)]
    tf: Vector3,
}

#[cerulion_node_impl]
impl DeepReader {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.tf.len();
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// [`GatedReader`] with a cap of 3, recording each fire's drop counts.
#[cerulion_node(external)]
#[derive(Default)]
struct GatedTinyReader {
    #[input]
    gate: Vector3,
    #[input(accumulate, cap = 3)]
    tf: Vector3,
    #[cerulion(reconstruct)]
    seen: Arc<Mutex<Seen>>,
}

#[cerulion_node_impl]
impl GatedTinyReader {
    fn tick(&mut self) -> Result<(), NodeError> {
        let xs: Vec<f64> = self.tf.iter().map(|f| f.x).collect();
        let _gate = self.gate.x;
        let (dropped, total) = (self.tf.dropped(), self.tf.dropped_total());
        let mut seen = self.seen.lock().expect("seen lock");
        seen.fires.push(xs);
        seen.dropped_per_fire.push(dropped);
        seen.dropped_totals.push(total);
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

fn writer_def(id: &str) -> NodeDef {
    writer_on(id, TOPIC)
}

fn writer_on(id: &str, topic: &str) -> NodeDef {
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
            topic: Some(topic.to_string()),
        }],
    }
}

fn input(name: &str, source: &str) -> InputDef {
    InputDef {
        name: name.to_string(),
        source: source.to_string(),
    }
}

fn reader_def(node_type: &str, inputs: Vec<InputDef>) -> NodeDef {
    NodeDef {
        fuse: None,
        ros2: None,
        id: "reader".to_string(),
        node_type: node_type.to_string(),
        inputs,
        outputs: vec![],
    }
}

fn writer_entry(base: f64) -> Box<dyn NodeEntry> {
    Box::new(WriterEntry::with_state(Writer {
        base,
        ..Default::default()
    }))
}

/// A graph of `writers` (id, topic, base) and one reader node.
fn build_graph(
    prefix: &str,
    multi: &[&str],
    writers: &[(&str, &str, f64)],
    reader: NodeDef,
    entry: Box<dyn NodeEntry>,
) -> Result<GraphRuntime, TransportError> {
    let mut nodes: Vec<NodeDef> = writers.iter().map(|(id, t, _)| writer_on(id, t)).collect();
    nodes.push(reader);
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: multi.iter().map(|t| t.to_string()).collect(),
        name: None,
        identity: "accumulate_input".to_string(),
        prefix: prefix.to_string(),
        nodes,
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for (id, _, base) in writers {
        factories.insert(id.to_string(), writer_entry(*base));
    }
    factories.insert("reader".to_string(), entry);
    GraphRuntime::build_for_test(config, factories, Arc::new(VirtualClock::new()), 8)
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

/// An oracle of whole-number `x` values as the `f64` frames carry them
/// (`f64::from(u32)` is exact), so a fractional `x` fails the comparison
/// instead of truncating into a match.
fn xs(fires: Vec<Vec<u32>>) -> Vec<Vec<f64>> {
    fires
        .into_iter()
        .map(|f| f.into_iter().map(f64::from).collect())
        .collect()
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
        xs(vec![vec![1, 101, 2, 102, 3, 103], vec![]]),
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
    let oracle = xs(vec![vec![1, 101, 2, 102], vec![3, 103]]);
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
        xs(vec![
            vec![2, 102, 3, 103],
            vec![5, 105, 6, 106],
            vec![7, 107],
            vec![9, 109, 10, 110],
        ])
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
fn before_a_writers_first_delivered_frame_its_queue_evictions_cannot_be_counted() {
    let s = Arc::new(Mutex::new(Seen::default()));
    let mut rt = build("acc_queue", small_reader(&s));
    // 6 frames per writer into per-writer queues of depth 4: each queue
    // keeps its newest 4 (x = 3..=6 and 103..=106), the collect takes those
    // 8, and the cap of 4 keeps the newest 4 of them. The 2 + 2 queue
    // evictions happened before the input saw any frame from either writer,
    // so no sequence gap exists to count them (the documented limit).
    publish(&mut rt, 6);
    fire_reader(&mut rt);
    let got = seen(&s);
    assert_eq!(got.fires, xs(vec![vec![5, 105, 6, 106]]));
    assert_eq!(got.dropped_per_fire, vec![4], "8 collected, 4 held");
    assert_eq!(got.events, vec![(4, 4, 4)]);
}

#[test]
#[serial]
fn queue_evictions_and_cap_drops_are_one_count_and_one_event() {
    let s = Arc::new(Mutex::new(Seen::default()));
    let entry = Box::new(TinyReaderEntry::with_state(TinyReader {
        seen: Arc::clone(&s),
        ..Default::default()
    }));
    let mut rt = build_graph(
        "acc_steady",
        &[TOPIC],
        &[("writer_a", TOPIC, 0.0), ("writer_b", TOPIC, 100.0)],
        reader_def("tiny_reader", vec![input("tf", TOPIC)]),
        entry,
    )
    .expect("build");
    // Fire 1 sees one frame of each writer: the baseline every later gap is
    // measured from.
    publish(&mut rt, 1);
    fire_reader(&mut rt);
    // 6 frames per writer (x = 2..=7, 102..=107) into queues of depth 3:
    // each queue evicts 3 (x = 2..=4, 102..=104), the collect takes 6 and
    // the cap of 3 keeps the newest 3 (106, 7, 107) and drops 3. Lost:
    // 3 + 3 + 3 = 9; published 12 = served 3 + dropped 9.
    publish(&mut rt, 6);
    fire_reader(&mut rt);
    let got = seen(&s);
    assert_eq!(got.fires, xs(vec![vec![1, 101], vec![106, 7, 107]]));
    assert_eq!(got.dropped_per_fire, vec![0, 9]);
    assert_eq!(
        got.events,
        vec![(9, 9, 3)],
        "one event carrying both drop sources: (dropped, count_total, cap)"
    );
    let total: u64 = got.fires.iter().map(|f| f.len() as u64).sum::<u64>()
        + got.dropped_per_fire.iter().sum::<u64>();
    assert_eq!(total, 14, "every published frame is served or counted");
}

#[test]
#[serial]
fn frames_carry_over_while_a_latest_value_input_has_not_delivered() {
    let s = Arc::new(Mutex::new(Seen::default()));
    let entry = Box::new(GatedReaderEntry::with_state(GatedReader {
        seen: Arc::clone(&s),
        ..Default::default()
    }));
    let mut rt = build_graph(
        "acc_gated",
        &[TOPIC],
        &[
            ("writer_a", TOPIC, 0.0),
            ("writer_b", TOPIC, 100.0),
            ("gate_writer", "/gate", 500.0),
        ],
        reader_def(
            "gated_reader",
            vec![input("gate", "/gate"), input("tf", TOPIC)],
        ),
        entry,
    )
    .expect("build");
    publish(&mut rt, 2);
    // `gate` has never delivered: the fire collects tf but the body is
    // skipped, so nothing is served.
    fire_reader(&mut rt);
    publish(&mut rt, 1);
    rt.trigger_external("gate_writer").expect("trigger gate");
    rt.step(STEP);
    fire_reader(&mut rt);
    fire_reader(&mut rt);
    assert_eq!(
        seen(&s).fires,
        xs(vec![vec![1, 101, 2, 102, 3, 103], vec![]]),
        "the skipped fire's frames reach the first fire whose body runs, then none again"
    );
}

#[test]
#[serial]
fn a_node_fired_once_per_trigger_frame_receives_each_accumulate_frame_once() {
    let s = Arc::new(Mutex::new(Seen::default()));
    let entry = Box::new(TriggeredReaderEntry::with_state(TriggeredReader {
        seen: Arc::clone(&s),
        ..Default::default()
    }));
    let mut rt = build_graph(
        "acc_trig",
        &[TOPIC, "/tick"],
        &[
            ("writer_a", TOPIC, 0.0),
            ("writer_b", TOPIC, 100.0),
            ("tick_a", "/tick", 1000.0),
            ("tick_b", "/tick", 2000.0),
        ],
        reader_def(
            "triggered_reader",
            vec![input("tick", "/tick"), input("tf", TOPIC)],
        ),
        entry,
    )
    .expect("build");
    publish(&mut rt, 2);
    // Two trigger frames in one step: two fires.
    rt.trigger_external("tick_a").expect("trigger tick_a");
    rt.trigger_external("tick_b").expect("trigger tick_b");
    for _ in 0..4 {
        rt.step(STEP);
    }
    // `(tick x, tf frames...)` per fire: the first fire receives every tf
    // frame, the second fire none.
    assert_eq!(
        seen(&s).fires,
        xs(vec![vec![1001, 1, 101, 2, 102], vec![2001]]),
        "one fire per trigger frame; each tf frame reaches exactly one fire"
    );
}

#[test]
#[serial]
fn two_accumulate_ports_named_tf_and_tf_frames_each_read_their_own_topic() {
    let s = Arc::new(Mutex::new(Seen::default()));
    let entry = Box::new(TwoPortReaderEntry::with_state(TwoPortReader {
        seen: Arc::clone(&s),
        ..Default::default()
    }));
    let mut rt = build_graph(
        "acc_two",
        &[],
        &[("writer_a", "/frames", 0.0), ("writer_b", TOPIC, 100.0)],
        reader_def(
            "two_port_reader",
            vec![input("tf_frames", "/frames"), input("tf", TOPIC)],
        ),
        entry,
    )
    .expect("build");
    publish(&mut rt, 2);
    fire_reader(&mut rt);
    assert_eq!(seen(&s).fires, xs(vec![vec![1, 2], vec![101, 102]]));
}

#[test]
#[serial]
fn a_cap_above_the_multi_publisher_ceiling_is_refused_at_build_naming_the_cap() {
    let entry = Box::new(DeepReaderEntry::new());
    let err = match build_graph(
        "acc_deep",
        &[TOPIC],
        &[("writer_a", TOPIC, 0.0), ("writer_b", TOPIC, 100.0)],
        reader_def("deep_reader", vec![input("tf", TOPIC)]),
        entry,
    ) {
        Ok(_) => panic!("a cap of 17 on a listed topic must not build"),
        Err(e) => e.to_string(),
    };
    assert!(
        err.contains(
            "a consumer declares depth 17 but the shared multi-publisher buffer ceiling is 16"
        ),
        "{err}"
    );
    assert!(
        err.contains("`#[input(accumulate, cap = N)]` input's cap is its depth"),
        "the refusal names the accumulate cap: {err}"
    );
}

/// A gated tiny reader graph: two `/tf` writers and a `/gate` writer.
fn gated_tiny(prefix: &str, s: &Arc<Mutex<Seen>>) -> GraphRuntime {
    let entry = Box::new(GatedTinyReaderEntry::with_state(GatedTinyReader {
        seen: Arc::clone(s),
        ..Default::default()
    }));
    build_graph(
        prefix,
        &[TOPIC],
        &[
            ("writer_a", TOPIC, 0.0),
            ("writer_b", TOPIC, 100.0),
            ("gate_writer", "/gate", 500.0),
        ],
        reader_def(
            "gated_tiny_reader",
            vec![input("gate", "/gate"), input("tf", TOPIC)],
        ),
        entry,
    )
    .expect("build")
}

/// Deliver the gate, then fire the reader so its body runs.
fn open_gate_and_fire(rt: &mut GraphRuntime) {
    rt.trigger_external("gate_writer").expect("trigger gate");
    rt.step(STEP);
    fire_reader(rt);
}

#[test]
#[serial]
fn a_node_resumed_from_captured_state_serves_the_unserved_frames_and_counts() {
    use cerulion_core::state::VecSink;
    use cerulion_core::state_restore::AnchorBlob;
    use std::collections::BTreeMap;

    // The recorded run: 4 frames into a cap of 3 drops x = 1. The gate has
    // not delivered, so the fire collects and skips its body: 101, 2, 102
    // are held and owed, with one drop.
    let live = Arc::new(Mutex::new(Seen::default()));
    let mut recorded = gated_tiny("acc_anchor_live", &live);
    publish(&mut recorded, 2);
    fire_reader(&mut recorded);
    assert!(seen(&live).fires.is_empty(), "the body has not run yet");
    let mut blob = VecSink::new();
    {
        let entry = recorded.node_entry_arc_for_test("reader").expect("reader");
        let guard = entry.lock().expect("reader lock");
        let shape = guard.state_shape().expect("a macro node declares state");
        AnchorBlob::write_header(shape, &mut blob).expect("header");
        guard.capture_state(&mut blob).expect("capture");
    }
    // The uninterrupted run goes on from here.
    open_gate_and_fire(&mut recorded);

    // The resumed run: a fresh graph restored from the captured state, then
    // the same continuation. No `/tf` frame is published in it.
    let resumed = Arc::new(Mutex::new(Seen::default()));
    let mut rt = gated_tiny("acc_anchor_resume", &resumed);
    let anchors = BTreeMap::from([("reader".to_string(), blob.into_inner())]);
    let report = rt.restore_node_states(&anchors).expect("restore");
    assert_eq!(report.restored, vec!["reader".to_string()]);
    open_gate_and_fire(&mut rt);

    let want = Seen {
        fires: xs(vec![vec![101, 2, 102]]),
        dropped_per_fire: vec![1],
        dropped_totals: vec![1],
        events: vec![],
    };
    assert_eq!(seen(&live), want, "the uninterrupted run");
    assert_eq!(seen(&resumed), want, "the resumed run serves the same");
}

#[test]
#[serial]
fn a_restore_holding_more_frames_than_the_cap_is_refused_naming_the_input() {
    use cerulion_core::state::VecSink;
    use cerulion_core::state_restore::AnchorBlob;
    use std::collections::BTreeMap;

    // A capture of a cap-3 input holding 3 frames, with its frame count
    // edited to 4: more than the running input can hold.
    let live = Arc::new(Mutex::new(Seen::default()));
    let mut recorded = gated_tiny("acc_anchor_cap_live", &live);
    publish(&mut recorded, 2);
    fire_reader(&mut recorded);
    let mut blob = VecSink::new();
    {
        let entry = recorded.node_entry_arc_for_test("reader").expect("reader");
        let guard = entry.lock().expect("reader lock");
        let shape = guard.state_shape().expect("state");
        AnchorBlob::write_header(shape, &mut blob).expect("header");
        guard.capture_state(&mut blob).expect("capture");
    }
    let mut bytes = blob.into_inner();
    // Layout after the 16-byte anchor header and the user's fields (none
    // captured here): u32 inputs, u32 name length, "tf", u64, u64, then the
    // u32 frame count. Raise it from 3 to 4.
    let at = 16 + 4 + 4 + 2 + 8 + 8;
    assert_eq!(&bytes[at..at + 4], &3u32.to_le_bytes(), "the frame count");
    bytes[at..at + 4].copy_from_slice(&4u32.to_le_bytes());
    let s = Arc::new(Mutex::new(Seen::default()));
    let rt = gated_tiny("acc_anchor_cap_resume", &s);
    let anchors = BTreeMap::from([("reader".to_string(), bytes)]);
    let err = match rt.restore_node_states(&anchors) {
        Ok(_) => panic!("4 frames do not fit a cap of 3"),
        Err(e) => e.to_string(),
    };
    assert!(
        err.contains(
            "node 'gated_tiny_reader' state restore failed: accumulate input `tf` cannot be \
             restored: the recording holds 4 frame(s), above the cap of 3"
        ),
        "{err}"
    );
}
