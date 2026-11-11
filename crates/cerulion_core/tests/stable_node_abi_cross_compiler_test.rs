// SPDX-License-Identifier: AGPL-3.0-only
//! Compiler-independent node loading must deliver transformed wire frames to a
//! downstream node. Build test_node_stable_abi_cross_compiler first. The ignored
//! skew check requires CERULION_TEST_NODE_ABI_DYLIB naming the separately built
//! library; absence is an error. Run this binary alone with --test-threads=1.
#![cfg(any(target_os = "linux", target_os = "macos"))]

use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::node_abi::stabby::libloading::StabbyLibrary;
use cerulion_core::node_abi::{BootstrapFn, Bytes};
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

const STEP: Duration = Duration::from_millis(1);
const INPUTS: [[f64; 3]; 3] = [[1.0, 2.0, 3.0], [0.0, -0.0, 1.0], [-2.0, 4.0, 0.0]];
const OUTPUTS: [[f64; 3]; 3] = [[11.0, 4.0, 2.0], [10.0, -0.0, 0.0], [8.0, 8.0, -1.0]];
static NEXT_PREFIX: AtomicU64 = AtomicU64::new(0);

const GRAPH: &str = r#"prefix: stableabi
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
      - name: input
        source: source/output
    outputs:
      - name: output
        schema: geometry_msgs/Vector3
        max_slice_len: 64
  - id: sink
    type: sink
    inputs:
      - name: input
        source: transform/output
"#;

#[cerulion_node(external)]
#[derive(Default)]
struct Source {
    #[output]
    output: Vector3,
    value: Arc<Mutex<[f64; 3]>>,
}

#[cerulion_node_impl]
impl Source {
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
struct Sink {
    #[input(trigger)]
    input: Vector3,
    delivered: Arc<Mutex<Option<[u8; 64]>>>,
}

#[cerulion_node_impl]
impl Sink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let mut frame = [0u8; 64];
        self.input.wire_header().write_to_buf(&mut frame[..40]);
        frame[40..48].copy_from_slice(&self.input.x.to_le_bytes());
        frame[48..56].copy_from_slice(&self.input.y.to_le_bytes());
        frame[56..64].copy_from_slice(&self.input.z.to_le_bytes());
        let prior = self.delivered.lock().expect("sink state").replace(frame);
        assert!(
            prior.is_none(),
            "more than one delivery for a lockstep input"
        );
        Ok(())
    }
}

fn verify_compiler_and_bootstrap(path: &Path, skew: bool) {
    // SAFETY: this is the explicitly built fixture's library. Its constructors
    // and checked exports obey the frozen Stabby protocol; retain it through all
    // borrowed fingerprint bytes and function calls below.
    let library = unsafe { libloading::Library::new(path) }.expect("open fixture");
    // SAFETY: maintained Stabby discovery checks both known fixture exports'
    // signatures, and library ownership bounds the returned function pointers.
    let (probe, _bootstrap) = unsafe {
        (
            library
                .get_stabbied::<extern "C" fn() -> Bytes>(b"cerulion_test_compiler_fingerprint")
                .expect("compiler probe"),
            library
                .get_stabbied::<BootstrapFn>(b"cerulion_node_api_v1")
                .expect("stable node bootstrap"),
        )
    };
    let bytes = probe();
    assert!(!bytes.ptr.is_null());
    let length = usize::try_from(bytes.len).expect("fingerprint length");
    assert!(length > 0 && length <= isize::MAX as usize);
    // SAFETY: checked fixture export returns initialized library-static bytes;
    // its native owner remains live until after the comparison.
    let fingerprint = std::str::from_utf8(unsafe { std::slice::from_raw_parts(bytes.ptr, length) })
        .expect("UTF-8 fingerprint");
    if skew {
        assert_ne!(
            fingerprint,
            cerulion_core::RUSTC_FINGERPRINT,
            "node and host must use different actual compilers"
        );
    } else {
        assert_eq!(fingerprint, cerulion_core::RUSTC_FINGERPRINT);
    }
}

fn run_delivery(path: &Path, skew: bool) -> Vec<[u8; 64]> {
    verify_compiler_and_bootstrap(path, skew);
    let values = Arc::new(Mutex::new([0.0; 3]));
    let delivered = Arc::new(Mutex::new(None));
    let mut graph = cerulion_core::graph::parse_graph(GRAPH).expect("three-node YAML graph");
    graph.prefix = format!("stableabi{}", NEXT_PREFIX.fetch_add(1, Ordering::Relaxed));
    let mut entries: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    entries.insert(
        "source".into(),
        Box::new(SourceEntry::with_state(Source {
            value: Arc::clone(&values),
            ..Default::default()
        })),
    );
    entries.insert(
        "transform".into(),
        Box::new(DylibNodeEntry::load(path).expect("load compiler-independent transformer")),
    );
    entries.insert(
        "sink".into(),
        Box::new(SinkEntry::with_state(Sink {
            delivered: Arc::clone(&delivered),
            ..Default::default()
        })),
    );
    let mut runtime =
        GraphRuntime::build_for_test(graph, entries, Arc::new(VirtualClock::new()), 8)
            .expect("real SHM graph");
    for _ in 0..16 {
        runtime.step(STEP);
    }
    assert!(
        delivered.lock().expect("sink state").is_none(),
        "no invented delivery before any input"
    );
    let mut frames = Vec::new();
    for input in INPUTS {
        *values.lock().expect("source state") = input;
        runtime
            .trigger_external("source")
            .expect("one source publish");
        runtime.step(STEP);
        frames.push(
            delivered
                .lock()
                .expect("sink state")
                .take()
                .expect("transformed frame reached downstream sink"),
        );
    }
    for _ in 0..8 {
        runtime.step(STEP);
    }
    assert!(
        delivered.lock().expect("sink state").is_none(),
        "silent input cannot invent another output"
    );
    frames
}

fn oracle() -> Vec<[u8; 64]> {
    OUTPUTS
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
fn same_compiler_delivers_oracle_frames_in_two_identical_runs() {
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_stable_abi_cross_compiler");
    let first = run_delivery(&path, false);
    let second = run_delivery(&path, false);
    assert_eq!(first, oracle());
    assert_eq!(second, oracle());
    assert_eq!(first, second);
}

#[test]
#[serial]
#[ignore = "requires a node artifact built by a different compiler"]
fn different_compiler_delivers_oracle_frames_in_two_identical_runs() {
    let path = std::env::var_os("CERULION_TEST_NODE_ABI_DYLIB")
        .expect("separately built node artifact path");
    let first = run_delivery(Path::new(&path), true);
    let second = run_delivery(Path::new(&path), true);
    assert_eq!(first, oracle());
    assert_eq!(second, oracle());
    assert_eq!(first, second);
}
