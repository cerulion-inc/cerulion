// SPDX-License-Identifier: AGPL-3.0-only
//! `#[input(accumulate)]` parity: a cdylib node loaded through the real
//! `DylibNodeEntry` FFI receives the same frames, in the same order, as an
//! in-process node built from the same source.
//!
//! The reader is `test_node_macro_accumulate_cdylib` (`period_ms = 30`,
//! `#[input(accumulate, cap = 8)] tf`). Its twin [`AccumulateNode`] below is
//! the same struct and tick, compiled into this test. Each fire publishes what
//! it received (`x` = frame count, `y` = `sum((i + 1) * x_i)` in serve order,
//! `z` = dropped), and an in-process sink records every fire off SHM.
//!
//! Two host-driven writers share the listed topic `/tf`: writer A publishes
//! `x = 1, 2, 3, ...`, writer B `x = 101, 102, 103, ...`, both in the same
//! steps. The step is 10 ms and the reader fires every third step, so a fire
//! receives the frames of up to three steps.
//!
//! # Running (iceoryx2 SHM + the cdylib `NODES` registry: serial)
//!
//! ```bash
//! cargo build -p test_node_macro_accumulate_cdylib
//! cargo test -p cerulion_core --test cdylib_accumulate_parity_test -- --test-threads=1
//! ```

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

const STEP: Duration = Duration::from_millis(10);
const TOPIC: &str = "/tf";

static PREFIX_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_prefix(stem: &str) -> String {
    format!("{stem}{}", PREFIX_COUNTER.fetch_add(1, Ordering::Relaxed))
}

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

/// The in-process twin of the cdylib fixture: the same struct and tick.
#[cerulion_node(period_ms = 30)]
struct AccumulateNode {
    #[input(accumulate, cap = 8)]
    tf: Vector3,
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl AccumulateNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let mut count = 0.0;
        let mut weighted = 0.0;
        for frame in self.tf.iter() {
            count += 1.0;
            weighted += count * frame.x;
        }
        let dropped = self.tf.dropped() as f64;
        self.out.x = count;
        self.out.y = weighted;
        self.out.z = dropped;
        Ok(())
    }
}

/// `(count, weighted, dropped)` of each reader fire, in fire order, kept as
/// the `f64` the reader published so a fractional value cannot truncate into
/// a match.
type Fires = Vec<(f64, f64, f64)>;

#[cerulion_node]
#[derive(Default)]
struct Sink {
    #[input(trigger)]
    inp: Vector3,
    rec: Arc<Mutex<Fires>>,
}

#[cerulion_node_impl]
impl Sink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let fire = (self.inp.x, self.inp.y, self.inp.z);
        self.rec.lock().expect("rec lock").push(fire);
        Ok(())
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

/// The reader's fires, and its captured state at the end of the run.
fn run(reader: Box<dyn NodeEntry>, prefix: &str) -> (Fires, Vec<u8>) {
    let config = GraphConfig {
        sim_time: false,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: vec![TOPIC.to_string()],
        name: None,
        identity: "accumulate_parity".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            writer_def("writer_a"),
            writer_def("writer_b"),
            NodeDef {
                fuse: None,
                ros2: None,
                id: "acc".to_string(),
                node_type: "accumulate_node".to_string(),
                inputs: vec![InputDef {
                    name: "tf".to_string(),
                    source: TOPIC.to_string(),
                }],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "sink".to_string(),
                node_type: "sink".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "acc/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let rec = Arc::new(Mutex::new(Fires::new()));
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for (id, base) in [("writer_a", 0.0), ("writer_b", 100.0)] {
        factories.insert(
            id.to_string(),
            Box::new(WriterEntry::with_state(Writer {
                base,
                ..Default::default()
            })),
        );
    }
    factories.insert("acc".to_string(), reader);
    factories.insert(
        "sink".to_string(),
        Box::new(SinkEntry::with_state(Sink {
            rec: Arc::clone(&rec),
            ..Default::default()
        })),
    );
    let mut rt = GraphRuntime::build_for_test(config, factories, Arc::new(VirtualClock::new()), 8)
        .expect("build parity graph");
    // Five publishing steps, then silent steps until every frame is read.
    for _ in 0..5 {
        rt.trigger_external("writer_a").expect("trigger writer_a");
        rt.trigger_external("writer_b").expect("trigger writer_b");
        rt.step(STEP);
    }
    for _ in 0..7 {
        rt.step(STEP);
    }
    let fires = rec.lock().expect("rec lock").clone();
    let mut state = cerulion_core::state::VecSink::new();
    rt.node_entry_arc_for_test("acc")
        .expect("the reader")
        .lock()
        .expect("reader lock")
        .capture_state(&mut state)
        .expect("capture");
    (fires, state.into_inner())
}

/// The reader's captured state once every frame is served: no user field is
/// captured (the struct holds only ports), so it is the accumulate section
/// alone, by hand: one input, `tf`, no drops, no frame owed.
fn state_oracle() -> Vec<u8> {
    let mut want = Vec::new();
    want.extend_from_slice(&1u32.to_le_bytes());
    want.extend_from_slice(&2u32.to_le_bytes());
    want.extend_from_slice(b"tf");
    want.extend_from_slice(&0u64.to_le_bytes());
    want.extend_from_slice(&0u64.to_le_bytes());
    want.extend_from_slice(&0u32.to_le_bytes());
    want
}

fn cdylib_reader() -> Box<dyn NodeEntry> {
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_macro_accumulate_cdylib");
    Box::new(DylibNodeEntry::load(&path).expect("load accumulate cdylib"))
}

/// The hand oracle: every one of the 10 frames reaches exactly one fire,
/// collected at the step boundary of the reader's 30 ms grid,
/// oldest first, with ties on the publish stamp broken by the bytes
/// (`x = n` before `x = 100 + n`). Each `(count, weighted, dropped)` is
/// computed by hand from the frames the fire holds.
fn oracle() -> Fires {
    vec![
        // The first fire after publishing reads the frames of the two steps
        // before it: 1, 101, 2, 102. 1*1 + 2*101 + 3*2 + 4*102 = 617.
        (4.0, 617.0, 0.0),
        // The next fire, three steps on, reads the other three steps:
        // 3, 103, 4, 104, 5, 105.
        // 1*3 + 2*103 + 3*4 + 4*104 + 5*5 + 6*105 = 1292.
        (6.0, 1292.0, 0.0),
        // Nothing new.
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
    ]
}

#[test]
#[serial]
fn a_cdylib_accumulate_reader_receives_the_same_frames_as_the_in_process_twin() {
    // The in-process run goes first: on macOS a process that has run a graph
    // with a plugin node cannot create further iceoryx2 resources.
    let in_process = run(
        Box::new(AccumulateNodeEntry::new()),
        &unique_prefix("accin"),
    );
    let cdylib = run(cdylib_reader(), &unique_prefix("accdy"));
    assert_eq!(in_process.0, oracle(), "the in-process reader");
    assert_eq!(cdylib.0, oracle(), "the cdylib reader, through the FFI");
    assert_eq!(in_process.1, state_oracle(), "the in-process state capture");
    assert_eq!(
        cdylib.1,
        state_oracle(),
        "the cdylib state capture carries the accumulate section through the FFI"
    );
}

#[cfg_attr(
    target_os = "macos",
    ignore = "upstream iceoryx2 0.10.0 defect 2034: on macOS a process that has run one graph containing a plugin node cannot create any further iceoryx2 resource, so this arm cannot run here until that is fixed. Runs normally on Linux."
)]
#[test]
#[serial]
fn two_cdylib_runs_receive_identical_frames() {
    let a = run(cdylib_reader(), &unique_prefix("accdet"));
    let b = run(cdylib_reader(), &unique_prefix("accdet"));
    assert_eq!(a.0, oracle());
    assert_eq!(b.0, oracle());
}
