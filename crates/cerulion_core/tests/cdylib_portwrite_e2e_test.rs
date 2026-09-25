// SPDX-License-Identifier: AGPL-3.0-only
//! Cdylib-parity cluster 2: the port-write surface exercised over the PRODUCTION
//! cdylib (`DylibNodeEntry`) path.
//!
//! The `#[cerulion_node_impl]` port-write rewriter (variable `= expr`, variable
//! READ accessors on inputs, `fill_from`, nested-leaf sugar, `with_<field>`
//! closures, and the loud `NestedWriteConflict` / `NestedChildIncomplete`
//! discard paths) is heavily covered IN-PROCESS by
//! `rewriter_var_field_assign_test.rs`, but was VERIFIED WIRED yet had ZERO
//! behavioral tests on the cdylib. This file closes that gap: it loads the
//! `test_node_macro_portwrite_cdylib` fixture through `DylibNodeEntry`, drives it
//! over real iceoryx2 (`GraphRuntime::build_for_test`), and asserts:
//!
//! 1. **healthy** — the delivered `Image` payload matches (a) a HAND-BUILT
//!    standalone-`HeaderShm` oracle on the nested `header` bytes (cribbing
//!    `rewriter_var_field_assign_test::header_payload_oracle` — never a
//!    self-compare) AND (b) known literal values for the top-level fixed +
//!    variable fields.
//! 2. **parity** — an in-process twin with byte-identical tick logic produces a
//!    byte-identical delivered payload (the FFI path vs the in-process path
//!    encode the same bytes).
//! 3. **conflict** — staged sugar + a whole-field write on `header` surfaces as
//!    a LOUD tick Err over the FFI (`node tick failed` + the
//!    `NestedWriteConflict` message reach the HOST log) and publishes nothing.
//! 4. **partial** — an incomplete staged child (`header.frame_id` unwritten) is
//!    discarded at `OutputProxy::Drop`; the node fires but publishes nothing.
//!
//! # Build requirement + serial
//!
//! Requires `cargo build -p test_node_macro_portwrite_cdylib`. All tests
//! `#[serial]` — the cdylib `NODES` singleton, the iceoryx2 SHM singleton, AND
//! the process-global `CER_FAIL_MODE` env (paired with an `EnvVarGuard` RAII).

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::wire::MaxPayloadCapacity;
use indexmap::IndexMap;
use native_ros2_messages::sensor_msgs::Image;
use native_ros2_messages::std_msgs::String as RosString;
use serial_test::serial;

/// The `fill_from` payload the fixture writes into `image.data` — must match the
/// fixture's `FILL_BYTES` const so the oracle knows the ground truth.
const FILL_BYTES: &[u8] = b"lidarframe";

// ---------------------------------------------------------------------------
// RAII env guard (panic-safe cleanup — see chunk_c_ffi_codes_3_4_test)
// ---------------------------------------------------------------------------

struct EnvVarGuard {
    key: &'static str,
}
impl EnvVarGuard {
    fn set(key: &'static str, val: &str) -> Self {
        std::env::set_var(key, val);
        Self { key }
    }
}
impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        std::env::remove_var(self.key);
    }
}

// ---------------------------------------------------------------------------
// Fixture locator (mirrors cdylib_depth_ffi_test::find_cdylib)
// ---------------------------------------------------------------------------

fn find_cdylib(crate_name: &str) -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib(crate_name)
}

// ---------------------------------------------------------------------------
// Hand oracle: standalone HeaderShm writer (crib of rewriter_var_field_assign)
// ---------------------------------------------------------------------------

/// A zeroed 8-byte-aligned buffer (backed by `Vec<u64>`; `from_bytes_mut`
/// asserts align 8 for f64-bearing fixed sections).
struct AlignedBuf(Vec<u64>);
impl AlignedBuf {
    fn new(n_bytes: usize) -> Self {
        Self(vec![0u64; n_bytes.div_ceil(8).max(1)])
    }
    fn bytes(&mut self) -> &mut [u8] {
        let n = self.0.len() * 8;
        // SAFETY: `Vec<u64>` is 8-aligned; all `n` bytes are initialized (zeroed).
        unsafe { std::slice::from_raw_parts_mut(self.0.as_mut_ptr() as *mut u8, n) }
    }
}

/// Build the hand-oracle `Header` payload via a STANDALONE `HeaderShm` writer
/// (NOT the staged rewrite path) — exactly the byte shape an Image view's
/// `header_bytes()` returns for a flushed staged header. Never a self-compare:
/// the oracle uses the ordinary writer; the assertion targets the cdylib's
/// staged/rewrite output.
fn header_payload_oracle(
    ops: impl FnOnce(&mut native_ros2_messages::std_msgs::HeaderShm<'_>),
) -> Vec<u8> {
    use native_ros2_messages::std_msgs::Header;
    let mut ab = AlignedBuf::new(2048);
    let bytes = ab.bytes();
    let mut w = Header::build_writer(bytes, MaxPayloadCapacity::const_new(2048), "oracle".into());
    ops(&mut w);
    let end = w.cursor() as usize;
    ab.bytes()[..end].to_vec()
}

// ---------------------------------------------------------------------------
// Driver (in-process String producer) + in-process parity twin
// ---------------------------------------------------------------------------

/// Publishes a `std_msgs::String` every 10 ms to DRIVE the port-write node's
/// data trigger. (The schema NAME on the OutputDef is cosmetic under
/// `build_for_test` — schema_hashes is None — so the real wire hash comes from
/// this node's `RosString` OutputMeta and matches the cdylib's `text_in`.)
// No `#[derive(Default)]` — a String-output node with only port fields
// constructs via the macro's `new()` without a struct Default (the proven
// `StringEcho` pattern in rewriter_var_field_assign_test).
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

/// In-process byte-identical twin of the cdylib fixture's HEALTHY tick — the
/// parity oracle for the FFI path. The tick body mirrors the healthy match arm
/// of `test_node_macro_portwrite_cdylib::PortWriteNode::tick` EXACTLY, so a byte
/// diff between the two delivered payloads is a real FFI-encoding bug.
#[cerulion_node]
struct PortWriteTwin {
    #[input(trigger)]
    text_in: RosString,
    #[output]
    image: Image,
}
#[cerulion_node_impl]
impl PortWriteTwin {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.text_in.data();
        self.image.encoding = "rgb8";
        self.image.data.fill_from(|buf: &mut [u8]| {
            buf[..FILL_BYTES.len()].copy_from_slice(FILL_BYTES);
            Ok(FILL_BYTES.len())
        })?;
        self.image.header.frame_id = "cam0";
        self.image.header.stamp.sec = 5;
        self.image.with_header(|h| {
            h.stamp.nanosec = 7;
            Ok(())
        })?;
        self.image.height = 1080;
        self.image.width = 1920;
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Graph harness — in-process capture consumer (avoids a raw create_subscriber,
// whose default slice-len could mismatch the small graph topic; the graph wires
// the capture input with a consistent topology config).
// ---------------------------------------------------------------------------

/// The delivered Image, decomposed into the fields the oracles compare.
#[derive(Clone, Debug, PartialEq, cerulion_core::state::CerulionState)]
struct Captured {
    header_bytes: Vec<u8>,
    data: Vec<u8>,
    encoding: String,
    height: u32,
    width: u32,
}

/// Data-triggered consumer of the portwrite's Image output: on each delivered
/// frame it captures the fields the oracles compare into a shared slot. If the
/// portwrite discards (conflict / partial), this node never fires ⇒ the slot
/// stays `None`.
#[cerulion_node]
#[derive(Default)]
struct ImageCaptureConsumer {
    #[input(trigger)]
    image: Image,
    captured: Arc<Mutex<Option<Captured>>>,
}
#[cerulion_node_impl]
impl ImageCaptureConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Read the delivered Image through the input view (header_bytes + the
        // variable/fixed accessors) — the same surface a subscriber's try_view
        // exposes, reached here via `InputView` Deref.
        let cap = Captured {
            header_bytes: self.image.header_bytes().to_vec(),
            data: self.image.data().to_vec(),
            encoding: self.image.encoding().expect("utf8 encoding").to_string(),
            height: self.image.height,
            width: self.image.width,
        };
        *self.captured.lock().unwrap() = Some(cap);
        Ok(())
    }
}

/// Build `producer(String) -> portwrite(text_in trigger, image out) ->
/// capture(image trigger)`, run `steps`, and return `(latest captured Image or
/// None, portwrite fire_count)`.
fn run_capture(
    portwrite: Box<dyn NodeEntry>,
    prefix: &str,
    steps: usize,
) -> (Option<Captured>, u64) {
    let captured: Arc<Mutex<Option<Captured>>> = Arc::new(Mutex::new(None));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("pw591_{prefix}"),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
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
                fuse: None,
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
                    max_slice_len: Some(4096),
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "capture".to_string(),
                node_type: "image_capture".to_string(),
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
    factories.insert("portwrite".to_string(), portwrite);
    let capture_node = ImageCaptureConsumer {
        captured: Arc::clone(&captured),
        ..Default::default()
    };
    factories.insert(
        "capture".to_string(),
        Box::new(ImageCaptureConsumerEntry::with_state(capture_node)),
    );

    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build portwrite graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(10));
    }

    let fire_count = runtime.node_handle("portwrite").unwrap().fire_count();
    let result = captured.lock().unwrap().clone();
    (result, fire_count)
}

fn portwrite_cdylib() -> Box<dyn NodeEntry> {
    Box::new(
        DylibNodeEntry::load(&find_cdylib("test_node_macro_portwrite_cdylib"))
            .expect("load portwrite fixture"),
    )
}

// ---------------------------------------------------------------------------
// 1. healthy: hand oracle (header bytes) + known field values
// ---------------------------------------------------------------------------

#[test]
#[serial]
fn healthy_delivered_payload_matches_header_oracle_and_known_fields() {
    let _guard = EnvVarGuard::set("CER_FAIL_MODE", "healthy");
    let (captured, fire_count) = run_capture(portwrite_cdylib(), "pwh", 4);
    let got = captured.expect("healthy mode must publish an Image frame");
    assert!(fire_count > 0, "the port-write node must have fired");

    // HAND ORACLE for the nested header: a STANDALONE HeaderShm executing the
    // SAME logical writes (frame_id + stamp.sec + stamp.nanosec). Byte equality
    // proves the staged/rewrite path over the FFI encodes the header
    // byte-identically to the ordinary writer.
    let oracle = header_payload_oracle(|w| {
        w.set_frame_id("cam0").expect("oracle frame_id");
        w.stamp.sec = 5;
        w.stamp.nanosec = 7;
    });
    assert_eq!(
        got.header_bytes, oracle,
        "nested-leaf sugar + with_header must serialize the header byte-identically \
         to the standalone oracle (over the cdylib FFI)"
    );

    // Top-level fixed + variable fields against known literals.
    assert_eq!(
        got.data, FILL_BYTES,
        "fill_from must write the exact bytes into image.data over the FFI"
    );
    assert_eq!(got.encoding, "rgb8", "variable `=` write of encoding");
    assert_eq!(got.height, 1080, "fixed-field write of height");
    assert_eq!(got.width, 1920, "fixed-field write of width");
}

// ---------------------------------------------------------------------------
// 2. parity: cdylib FFI path vs in-process twin → byte-identical payload
// ---------------------------------------------------------------------------

#[test]
#[serial]
fn cdylib_and_in_process_twin_produce_byte_identical_payloads() {
    let _guard = EnvVarGuard::set("CER_FAIL_MODE", "healthy");

    let (cdylib_cap, cdylib_fires) = run_capture(portwrite_cdylib(), "pwpa", 4);
    let (twin_cap, twin_fires) = run_capture(Box::new(PortWriteTwinEntry::new()), "pwpb", 4);

    assert!(cdylib_fires > 0 && twin_fires > 0, "both nodes must fire");
    let cdylib_cap = cdylib_cap.expect("cdylib publishes");
    let twin_cap = twin_cap.expect("twin publishes");
    assert_eq!(
        cdylib_cap, twin_cap,
        "the cdylib (FFI) path and the in-process twin must encode a \
         byte-identical Image payload — a diff is a real FFI port-write bug"
    );
}

// ---------------------------------------------------------------------------
// 3. conflict: staged sugar + whole-field write → LOUD tick Err, no publish
// ---------------------------------------------------------------------------

#[test]
#[serial]
#[tracing_test::traced_test]
fn conflict_mode_fails_tick_loudly_and_publishes_nothing() {
    let _guard = EnvVarGuard::set("CER_FAIL_MODE", "conflict");
    let (captured, fire_count) = run_capture(portwrite_cdylib(), "pwcf", 4);

    // The node fired (fire_count records even on a tick Err) but discarded the
    // frame — the conflict `?` propagated out of the FFI tick.
    assert!(fire_count > 0, "the node fires even though its tick errors");
    assert!(
        captured.is_none(),
        "a NestedWriteConflict tick must publish NO Image frame"
    );

    // LOUD error observable over the FFI: the HOST logs the failed tick
    // (runtime.rs `node tick failed`) and the DylibNodeEntry surfaces the
    // cdylib's rich LAST_ERROR (the conflict message) into it. (The image
    // topic wiring is proven live by the healthy test in this same file, so
    // the `is_none()` above is not vacuous.)
    assert!(
        logs_contain("node tick failed"),
        "a cdylib tick Err must be loudly logged by the host"
    );
    assert!(
        logs_contain("Conflicting writes to nested field"),
        "the host log must carry the cdylib's rich NestedWriteConflict message"
    );
}

// ---------------------------------------------------------------------------
// 4. partial: incomplete staged child → Drop discard, no publish
// ---------------------------------------------------------------------------

#[test]
#[serial]
#[tracing_test::traced_test]
fn partial_child_incomplete_discards_frame_and_publishes_nothing() {
    let _guard = EnvVarGuard::set("CER_FAIL_MODE", "partial");
    let (captured, fire_count) = run_capture(portwrite_cdylib(), "pwpc", 4);

    // The node fires, but OutputProxy::Drop fails the child gate
    // (NestedChildIncomplete: header.frame_id) and DISCARDS the frame.
    // Observable = fired-but-no-delivery (the healthy test is the positive
    // control that the same harness DOES deliver) PLUS the negative log pin
    // below, which separates THIS row's mechanism (tick Ok → Drop-discard)
    // from the conflict row's (tick Err): a fixture mutation that returned
    // Err instead of taking the Drop path would satisfy fired-but-no-delivery
    // too, but would emit the host's "node tick failed" error.
    //
    // NOTE: the discard breadcrumb itself is emitted by the cdylib's own,
    // process-local tracing subscriber, so it is NOT capturable by
    // the host's `#[traced_test]`; no-delivery + no-tick-error is the pin.
    assert!(
        fire_count > 0,
        "the node fires; the frame is dropped at Drop"
    );
    assert!(
        captured.is_none(),
        "an incomplete staged child must be discarded — no Image frame published"
    );
    assert!(
        !logs_contain("node tick failed"),
        "the partial arm must take the tick-Ok Drop-discard path, not a tick Err"
    );
}
