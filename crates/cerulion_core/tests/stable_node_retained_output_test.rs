// SPDX-License-Identifier: AGPL-3.0-only
//! Build test_node_cdylib first. A real data-trigger graph retains its first
//! output loan across a tick; input maintenance and later deliveries must work.

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

static PREFIX: AtomicU64 = AtomicU64::new(0);
const STEP: Duration = Duration::from_millis(1);
const VALUES: [[f64; 3]; 2] = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]];
const GRAPH: &str = r#"prefix: retainedoutput
nodes:
  - id: source
    type: source
    outputs:
      - name: output
        schema: geometry_msgs/Vector3
        max_slice_len: 64
  - id: transform
    type: transform
    inputs:
      - name: inp
        source: source/output
    outputs:
      - name: out
        schema: geometry_msgs/Vector3
        max_slice_len: 64
  - id: sink
    type: sink
    inputs:
      - name: input
        source: transform/out
"#;
struct Mode;
impl Drop for Mode {
    fn drop(&mut self) {
        std::env::remove_var("CER_RETAIN_OUTPUT");
    }
}
#[cerulion_node(external)]
#[derive(Default)]
struct RetainedSource {
    #[output]
    output: Vector3,
    value: Arc<Mutex<[f64; 3]>>,
}

#[cerulion_node_impl]
impl RetainedSource {
    fn tick(&mut self) -> Result<(), NodeError> {
        let value = *self.value.lock().expect("source state");
        self.output.x = value[0];
        self.output.y = value[1];
        self.output.z = value[2];
        Ok(())
    }
    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[cerulion_node]
#[derive(Default)]
struct RetainedSink {
    #[input(trigger)]
    input: Vector3,
    delivered: Arc<Mutex<Vec<[u8; 64]>>>,
}

#[cerulion_node_impl]
impl RetainedSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let mut frame = [0u8; 64];
        self.input.wire_header().write_to_buf(&mut frame[..40]);
        frame[40..48].copy_from_slice(&self.input.x.to_le_bytes());
        frame[48..56].copy_from_slice(&self.input.y.to_le_bytes());
        frame[56..64].copy_from_slice(&self.input.z.to_le_bytes());
        self.delivered.lock().expect("sink state").push(frame);
        Ok(())
    }
}

fn run() -> Vec<[u8; 64]> {
    std::env::set_var("CER_RETAIN_OUTPUT", "1");
    let _mode = Mode;
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_cdylib");
    let dll = DylibNodeEntry::load(&path).expect("stable retained-output fixture");
    let values = Arc::new(Mutex::new([0.0; 3]));
    let delivered = Arc::new(Mutex::new(Vec::new()));
    let mut graph = cerulion_core::graph::parse_graph(GRAPH).expect("real three-node graph");
    graph.prefix = format!("retainedoutput{}", PREFIX.fetch_add(1, Ordering::Relaxed));
    let mut entries: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    entries.insert(
        "source".into(),
        Box::new(RetainedSourceEntry::with_state(RetainedSource {
            value: Arc::clone(&values),
            ..Default::default()
        })),
    );
    entries.insert("transform".into(), Box::new(dll));
    entries.insert(
        "sink".into(),
        Box::new(RetainedSinkEntry::with_state(RetainedSink {
            delivered: Arc::clone(&delivered),
            ..Default::default()
        })),
    );
    let mut runtime =
        GraphRuntime::build_for_test(graph, entries, Arc::new(VirtualClock::new()), 8)
            .expect("isolated real transport");
    for _ in 0..16 {
        runtime.step(STEP);
    }
    assert!(delivered.lock().unwrap().is_empty());
    for (index, input) in VALUES.into_iter().enumerate() {
        *values.lock().unwrap() = input;
        runtime.trigger_external("source").expect("source frame");
        runtime.step(STEP);
        if index == 0 {
            assert!(
                delivered.lock().unwrap().is_empty(),
                "first loan remains retained"
            );
        }
    }
    for _ in 0..8 {
        runtime.step(STEP);
    }
    runtime.shutdown_all_nodes();
    let frames = delivered.lock().unwrap().clone();
    frames
}
fn oracle() -> Vec<[u8; 64]> {
    VALUES
        .into_iter()
        .enumerate()
        .map(|(index, values)| {
            let mut frame = [0u8; 64];
            frame[..8].copy_from_slice(&Vector3::SCHEMA_HASH.to_le_bytes());
            frame[8..12].copy_from_slice(&64u32.to_le_bytes());
            frame[12..16].copy_from_slice(&64u32.to_le_bytes());
            frame[20..24].copy_from_slice(&(index as u32).to_le_bytes());
            frame[24..32].copy_from_slice(&((17 + index as u64) * 1_000_000).to_le_bytes());
            for (field, value) in values.into_iter().enumerate() {
                frame[40 + field * 8..48 + field * 8].copy_from_slice(&value.to_le_bytes());
            }
            frame
        })
        .collect()
}
#[test]
#[serial]
fn retained_output_across_tick_keeps_data_trigger_draining_and_delivers_twice() {
    let first = run();
    let second = run();
    assert_eq!(first, oracle());
    assert_eq!(second, oracle());
    assert_eq!(first, second);
}
