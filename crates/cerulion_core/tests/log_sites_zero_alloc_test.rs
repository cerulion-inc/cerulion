// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-allocation gate for a SUPPRESSED throttled and once-only log call.
//!
//! One period node ticks every step and logs through a `warn_throttle!` with
//! a 10 s period and a `warn_once!`. Both emit in the warm-up; in the measured
//! window (well inside the period) every call is suppressed. A subscriber
//! that formats each event is installed on the stepping thread, so a call
//! that wrongly emitted would allocate and be counted, and the message
//! arguments go through a counter, so a call that evaluated them is caught
//! too.
//!
//! ```bash
//! cargo test -p cerulion_core --test log_sites_zero_alloc_test -- --test-threads=1
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
use tracing_subscriber::layer::{Context, Layer};
use tracing_subscriber::prelude::*;

thread_local! {
    /// `true` only on the thread that opened the counting window.
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

/// Events the formatting subscriber saw from the node.
static NODE_EVENTS: AtomicU64 = AtomicU64::new(0);
/// Times a message argument was evaluated.
static ARG_EVALS: AtomicU64 = AtomicU64::new(0);

fn counted(v: u64) -> u64 {
    ARG_EVALS.fetch_add(1, Ordering::Relaxed);
    v
}

/// Formats every event into a `String`, as a real console subscriber would.
struct Formatting;

impl<S: tracing::Subscriber> Layer<S> for Formatting {
    fn on_event(&self, event: &tracing::Event<'_>, _ctx: Context<'_, S>) {
        let line = format!("{event:?}");
        if line.contains("lidar") {
            NODE_EVENTS.fetch_add(1, Ordering::Relaxed);
        }
    }
}

#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct Noisy {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl Noisy {
    fn tick(&mut self) -> Result<(), NodeError> {
        warn_throttle!(
            self,
            Duration::from_secs(10),
            dropped = counted(3),
            "lidar frames dropped: {}",
            counted(3)
        );
        warn_once!(self, first = counted(1), "lidar started {}", counted(1));
        self.out.x = 1.0;
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
    let yaml = format!(
        r#"
name: log_sites_alloc_gate
prefix: log_alloc_{nanos}
nodes:
  - id: noisy
    type: noisy
    outputs:
      - name: out
        schema: geometry_msgs/Vector3
"#
    );
    let config = parse_graph(&yaml).expect("parse graph");
    validate_graph(&config).expect("validate graph");
    let mgr = TransportManager::get_or_init().expect("init TransportManager");
    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("noisy".to_string(), Box::new(NoisyEntry::new()));
    let mut rt = GraphRuntime::build(config, nodes, &mgr, Arc::new(VirtualClock::new()))
        .expect("build graph");
    rt.set_liveliness_sweep_period_for_test(SWEEP_DISABLED_MS);
    rt.set_liveliness_cleanup_period_for_test(SWEEP_DISABLED_MS);
    rt
}

#[test]
#[serial]
fn a_tick_hitting_a_suppressed_throttle_and_once_allocates_nothing() {
    const MEASURE_STEPS: u64 = 200;
    let subscriber = tracing_subscriber::registry().with(Formatting);
    tracing::subscriber::with_default(subscriber, || {
        let mut rt = build();
        // Warm-up: both sites emit once, the trace grows its capacity.
        for _ in 0..600 {
            rt.step(Duration::from_millis(1));
        }
        rt.clear_trace();
        assert_eq!(
            NODE_EVENTS.load(Ordering::SeqCst),
            2,
            "the throttle and the once emitted in the warm-up"
        );
        assert_eq!(ARG_EVALS.load(Ordering::SeqCst), 4);

        ALLOCATOR.enable();
        for _ in 0..MEASURE_STEPS {
            rt.step(Duration::from_millis(1));
        }
        let allocs = ALLOCATOR.disable();
        rt.shutdown();

        assert_eq!(NODE_EVENTS.load(Ordering::SeqCst), 2, "nothing emitted");
        assert_eq!(ARG_EVALS.load(Ordering::SeqCst), 4, "no argument evaluated");
        // Upstream iceoryx2 issue 2035: one allocation per publisher loan,
        // which the API cannot avoid. Exactly those, and nothing from the
        // suppressed log calls.
        let waived = cerulion_core::testing::upstream_2035_publish_allocs(MEASURE_STEPS);
        assert_eq!(
            allocs, waived,
            "{allocs} allocations over {MEASURE_STEPS} steps, of which {waived} are the \
             per-loan upstream cost; the remainder is the suppressed log calls"
        );
    });
}
