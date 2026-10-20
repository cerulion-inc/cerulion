// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-allocation gate for the `#[input(accumulate)]` receive path at steady
//! state: the step-boundary collect (one pop and one copy per frame), the
//! serve, and the per-frame typed read in the tick.
//!
//! # Topology
//!
//! ```text
//!  WriterA (period_ms = 1) ──┐
//!                            ├── /<unique>/tf ──▶ Reader (period_ms = 2,
//!  WriterB (period_ms = 1) ──┘                    #[input(accumulate, cap = 8)])
//! ```
//!
//! Both writers publish every step, so each reader fire collects four frames,
//! under the cap: no drop, no overflow warn. Once every slot has held a frame
//! (the warm-up), collecting and serving must allocate nothing. The count is
//! thread-scoped to the stepping thread, which is where the collect and the
//! tick run (see `step_zero_alloc_test.rs` for the counting modes).
//!
//! ```bash
//! cargo test -p cerulion_core --test accumulate_zero_alloc_test -- --test-threads=1
//! ```

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, SystemTime};

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::{parse_graph, validate_graph, GraphRuntime};
use cerulion_core::prelude::*;
use cerulion_core::transport::TransportManager;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

thread_local! {
    /// `true` only on the thread that opened the counting window. Const-init
    /// with a non-`Drop` payload, so touching it inside `alloc` cannot recurse.
    static MEASURED_THREAD: Cell<bool> = const { Cell::new(false) };
}

struct CountingAllocator {
    count: AtomicU64,
    enabled: AtomicBool,
}

impl CountingAllocator {
    const fn new() -> Self {
        Self {
            count: AtomicU64::new(0),
            enabled: AtomicBool::new(false),
        }
    }

    fn enable(&self) {
        MEASURED_THREAD.set(true);
        self.count.store(0, Ordering::SeqCst);
        self.enabled.store(true, Ordering::SeqCst);
    }

    fn disable(&self) -> u64 {
        self.enabled.store(false, Ordering::SeqCst);
        MEASURED_THREAD.set(false);
        self.count.load(Ordering::SeqCst)
    }
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if self.enabled.load(Ordering::Relaxed)
            && MEASURED_THREAD.try_with(Cell::get).unwrap_or(false)
        {
            self.count.fetch_add(1, Ordering::Relaxed);
        }
        unsafe { System.alloc(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator::new();

/// Frames the reader received, over all fires.
static FRAMES_SEEN: AtomicU64 = AtomicU64::new(0);
/// Frames the reader saw dropped, over all fires.
static FRAMES_DROPPED: AtomicU64 = AtomicU64::new(0);

#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct Writer {
    #[output]
    out: Vector3,
    n: u64,
}

#[cerulion_node_impl]
impl Writer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

#[cerulion_node(period_ms = 2)]
#[derive(Default)]
struct Reader {
    #[input(accumulate, cap = 8)]
    tf: Vector3,
    sum: f64,
}

#[cerulion_node_impl]
impl Reader {
    fn tick(&mut self) -> Result<(), NodeError> {
        let mut seen = 0u64;
        for frame in self.tf.iter() {
            self.sum += frame.x;
            seen += 1;
        }
        let dropped = self.tf.dropped();
        FRAMES_SEEN.fetch_add(seen, Ordering::Relaxed);
        FRAMES_DROPPED.fetch_add(dropped, Ordering::Relaxed);
        Ok(())
    }
}

/// Far past every window here, so the iceoryx2 liveliness sweep (an
/// allocating registry scan outside the step path) never runs in one.
const SWEEP_DISABLED_MS: u64 = 3_600_000;

fn build() -> GraphRuntime {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .expect("a clock after the epoch")
        .as_nanos();
    let topic = format!("/acc_alloc_{nanos}/tf");
    let yaml = format!(
        r#"
name: accumulate_alloc_gate
prefix: acc_alloc_{nanos}
multi_publisher_topics: [{topic}]
nodes:
  - id: writer_a
    type: writer
    outputs:
      - name: out
        schema: geometry_msgs/Vector3
        topic: {topic}
  - id: writer_b
    type: writer
    outputs:
      - name: out
        schema: geometry_msgs/Vector3
        topic: {topic}
  - id: reader
    type: reader
    inputs:
      - name: tf
        source: {topic}
"#
    );
    let config = parse_graph(&yaml).expect("parse graph");
    validate_graph(&config).expect("validate graph");
    let mgr = TransportManager::get_or_init().expect("init TransportManager");
    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("writer_a".to_string(), Box::new(WriterEntry::new()));
    nodes.insert("writer_b".to_string(), Box::new(WriterEntry::new()));
    nodes.insert("reader".to_string(), Box::new(ReaderEntry::new()));
    let mut rt = GraphRuntime::build(config, nodes, &mgr, Arc::new(VirtualClock::new()))
        .expect("build graph");
    rt.set_liveliness_sweep_period_for_test(SWEEP_DISABLED_MS);
    rt.set_liveliness_cleanup_period_for_test(SWEEP_DISABLED_MS);
    rt
}

#[test]
#[serial]
fn the_accumulate_receive_path_allocates_nothing_at_steady_state() {
    const MEASURE_STEPS: u64 = 200;
    let mut rt = build();
    // Warm-up: every slot holds a frame, the trace has grown its capacity.
    for _ in 0..600 {
        rt.step(Duration::from_millis(1));
    }
    rt.clear_trace();
    let seen_before = FRAMES_SEEN.load(Ordering::SeqCst);
    assert!(seen_before > 0, "the reader received frames during warm-up");

    ALLOCATOR.enable();
    for _ in 0..MEASURE_STEPS {
        rt.step(Duration::from_millis(1));
    }
    let allocs = ALLOCATOR.disable();
    let seen = FRAMES_SEEN.load(Ordering::SeqCst) - seen_before;
    rt.shutdown();

    // 2 writers x 200 steps: every frame published in the window is served
    // (the window starts and ends on a fire boundary of the period-2 reader).
    assert_eq!(seen, 2 * MEASURE_STEPS, "every frame reached the reader");
    assert_eq!(FRAMES_DROPPED.load(Ordering::SeqCst), 0, "under the cap");
    // Upstream iceoryx2 issue 2035: one allocation per publisher loan, which
    // the API cannot avoid. Exactly those, and nothing on the accumulate path.
    const PUBLISHING_PORTS: u64 = 2;
    let waived =
        cerulion_core::testing::upstream_2035_publish_allocs(PUBLISHING_PORTS * MEASURE_STEPS);
    assert_eq!(
        allocs, waived,
        "{allocs} allocations over {MEASURE_STEPS} steps, of which {waived} are the \
         per-loan upstream cost; the remainder is the accumulate receive path"
    );
}
