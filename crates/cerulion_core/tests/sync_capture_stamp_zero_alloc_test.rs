// SPDX-License-Identifier: AGPL-3.0-only
//! Allocation gate for capture-time (`header.stamp`) Sync matching.
//!
//! Capture-time matching adds work to every head op of a Sync node: a stamp
//! decode, an order check, and the counters. None of it may allocate. This
//! binary runs the SAME steady-state stimulus through a publish-time node and
//! a capture-time node and requires the capture-time steps to allocate exactly
//! as often as the publish-time ones: no new allocation per message.
//!
//! The frames are stamped so both clocks fuse every pair, so the two runs fire
//! the same number of times and differ only in the matching clock.
//!
//! Counting is THREAD-SCOPED (only the stepping thread counts), the liveliness
//! sweep is parked, and the tick writes into pre-sized storage, so nothing
//! outside the receive-and-match path can land in the window.
//!
//! `#[serial]`: real iceoryx2 over the process-global SHM singleton.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::codegen::layout::LayoutResolver;
use cerulion_core::codegen::parse_rosmsg;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef};
use cerulion_core::graph::node::{
    BackpressurePolicy, ClosureNodeEntry, InputMeta, MacroPolicy, NodeEntry, NodeInfo,
};
use cerulion_core::graph::validation::resolve_capture_sync_stamps;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::message::ShmMessage;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::PoseStamped;
use serial_test::serial;

thread_local! {
    static MEASURED_THREAD: Cell<bool> = const { Cell::new(false) };
}

struct CountingAllocator {
    count: AtomicU64,
    enabled: AtomicBool,
}

impl CountingAllocator {
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
        // `try_with`: during thread teardown TLS is gone, and that thread is
        // not the measured one.
        if self.enabled.load(Ordering::Relaxed)
            && MEASURED_THREAD.try_with(Cell::get).unwrap_or(false)
        {
            self.count.fetch_add(1, Ordering::Relaxed);
        }
        // SAFETY: forwarding the caller's layout to the system allocator.
        unsafe { System.alloc(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // SAFETY: `ptr` came from `System.alloc` with this layout.
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator {
    count: AtomicU64::new(0),
    enabled: AtomicBool::new(false),
};

const TOPIC_CAM: &str = "/capza/cam";
const TOPIC_LIDAR: &str = "/capza/lidar";
const MS: u64 = 1_000_000;
const WARM_STEPS: u64 = 16;
const MEASURED_STEPS: u64 = 64;
/// Far beyond the window: parks the liveliness sweep and its reclaim walk.
const SWEEP_PARKED_MS: u64 = 3_600_000;

fn fusion_graph(
    prefix: &str,
    fires: Arc<Mutex<Vec<(f64, f64)>>>,
    capture: bool,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let meta = |name: &str| InputMeta {
        name: name.to_string(),
        schema_hash: <PoseStamped as ShmMessage>::SCHEMA_HASH,
        trigger: true,
        depth: cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
        backpressure: BackpressurePolicy::DropOldest,
        expect_within_ms: None,
    };
    let mut info = NodeInfo::with_meta(vec![meta("cam"), meta("lidar")], vec![])
        .with_policy(MacroPolicy::Sync { window_ms: 10 });
    if capture {
        let mut schemas = Vec::new();
        for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
            if let Ok(s) = parse_rosmsg(text, name, Some(pkg)) {
                schemas.push(s);
            }
        }
        let found = resolve_capture_sync_stamps(
            "fuse",
            &[
                ("cam", "geometry_msgs/PoseStamped"),
                ("lidar", "geometry_msgs/PoseStamped"),
            ],
            &mut LayoutResolver::new(schemas).0,
        )
        .expect("PoseStamped carries header.stamp");
        info = info.with_sync_capture_stamps(
            ["cam", "lidar"]
                .into_iter()
                .map(String::from)
                .zip(found)
                .collect(),
        );
    }
    let entry = ClosureNodeEntry::new(info, move |ctx| {
        let mut read = |input: &str| {
            ctx.subscriber_mut(input).and_then(|s| {
                s.try_view::<PoseStamped, _>(|view| view.pose.position.x)
                    .ok()
                    .flatten()
            })
        };
        if let (Some(cam), Some(lidar)) = (read("cam"), read("lidar")) {
            // Pre-sized: a push within capacity does not allocate.
            fires
                .lock()
                .expect("fires sink poisoned")
                .push((cam, lidar));
        }
        Ok(())
    })
    .with_label("capture_zero_alloc_fuse");
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("capza_{prefix}"),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            params: Default::default(),
            fuse: None,
            ros2: None,
            id: "fuse".to_string(),
            node_type: "capture_zero_alloc_fuse".to_string(),
            inputs: vec![
                InputDef {
                    name: "cam".to_string(),
                    source: TOPIC_CAM.to_string(),
                },
                InputDef {
                    name: "lidar".to_string(),
                    source: TOPIC_LIDAR.to_string(),
                },
            ],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fuse".to_string(), Box::new(entry));
    (config, factories)
}

/// A `capture_ns` that [`publish`] writes as `nanosec = 1_000_000_000`, a
/// stamp no `builtin_interfaces/Time` can hold.
const BAD_STAMP: u64 = u64::MAX;

fn publish(
    publisher: &mut cerulion_core::CerulionPublisher,
    clock: &VirtualClock,
    wire_ns: u64,
    capture_ns: u64,
) {
    clock.set(wire_ns);
    let mut p = publisher.loan_proxy::<PoseStamped>().expect("loan");
    let (sec, nanosec) = if capture_ns == BAD_STAMP {
        (0, 1_000_000_000)
    } else {
        (
            (capture_ns / 1_000_000_000) as i32,
            (capture_ns % 1_000_000_000) as u32,
        )
    };
    p.pose.position.x = (capture_ns / MS) as f64;
    p.with_header(|h| {
        h.set_frame_id("base_link")?;
        h.stamp.sec = sec;
        h.stamp.nanosec = nanosec;
        Ok(())
    })
    .expect("write header");
}

/// Steps and counts allocations on the stepping thread across the measured
/// steps; returns `(allocations, fires)`. `dirty` puts a backwards frame and
/// an undecodable frame ahead of every good camera frame of the MEASURED
/// steps, so each runs the discard loop (two discards, both counters, both
/// latches). The dirt starts at the first measured step, so the latches' first
/// transition (the loud warning that opens each regime) is counted too, not
/// only their steady state.
fn measure(prefix: &str, capture: bool, dirty: bool) -> (u64, usize) {
    let total = (WARM_STEPS + MEASURED_STEPS) as usize;
    let fires = Arc::new(Mutex::new(Vec::with_capacity(total + 1)));
    let (config, factories) = fusion_graph(prefix, Arc::clone(&fires), capture);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build fusion graph");
    runtime.set_liveliness_sweep_period_for_test(SWEEP_PARKED_MS);
    runtime.set_liveliness_cleanup_period_for_test(SWEEP_PARKED_MS);
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut cam = mgr
        .create_publisher(TOPIC_CAM, MaxSliceLen::const_new(256), 0)
        .expect("publisher on cam");
    let mut lidar = mgr
        .create_publisher(TOPIC_LIDAR, MaxSliceLen::const_new(256), 0)
        .expect("publisher on lidar");

    let mut allocations = 0u64;
    for k in 1..=(WARM_STEPS + MEASURED_STEPS) {
        // Both clocks fuse: wire stamps 1 ms apart, capture stamps 2 ms apart,
        // each well inside the 10 ms window.
        let base = k * 20 * MS;
        if dirty && k > WARM_STEPS {
            // 2 ms behind the previous good camera stamp (base - 25 ms):
            // backwards within the 10 ms window, so dropped, never an epoch.
            publish(&mut cam, &clock, base - 2 * MS, base - 27 * MS);
            publish(&mut cam, &clock, base - MS, BAD_STAMP);
        }
        publish(&mut cam, &clock, base, base - 5 * MS);
        publish(&mut lidar, &clock, base + MS, base - 3 * MS);
        clock.set(base + 2 * MS);
        if k > WARM_STEPS {
            ALLOCATOR.enable();
            runtime.step(Duration::from_millis(1));
            allocations += ALLOCATOR.disable();
        } else {
            runtime.step(Duration::from_millis(1));
        }
    }
    if dirty {
        // Anti-tautology: the dirty run really took both discard arms.
        let handle = runtime.node_handle("fuse").expect("node handle");
        assert_eq!(
            handle.sync_capture_backwards_drops(TOPIC_CAM),
            MEASURED_STEPS
        );
        assert_eq!(
            handle.sync_capture_decode_failures(TOPIC_CAM),
            MEASURED_STEPS
        );
    }
    let fired = fires.lock().expect("fires sink poisoned").len();
    (allocations, fired)
}

#[test]
#[serial]
fn capture_time_matching_adds_no_allocation_per_message() {
    let (publish_allocs, publish_fires) = measure("capzapub", false, false);
    let (capture_allocs, capture_fires) = measure("capzacap", true, false);
    let total = (WARM_STEPS + MEASURED_STEPS) as usize;
    assert_eq!(publish_fires, total, "every publish-time pair fuses");
    assert_eq!(capture_fires, total, "every capture-time pair fuses");
    assert_eq!(
        capture_allocs, publish_allocs,
        "capture-time matching must allocate exactly as often as publish-time \
         matching on the same stimulus ({capture_allocs} vs {publish_allocs} over \
         {MEASURED_STEPS} steps)"
    );
}

/// The discard loop on the real runtime: every step drops one backwards and
/// one undecodable camera frame before the good one. The dirty run must
/// allocate exactly as often as the clean capture-time run on the same good
/// frames, so neither a discard, a counter bump nor a latched log line adds an
/// allocation. (Both runs carry the same nonzero baseline the publish-time run
/// has too; it comes from the step loop and the receive path outside the
/// capture layer, which the pure-layer test below pins to zero by itself.)
#[test]
#[serial]
fn the_capture_time_discard_loop_adds_no_allocation_per_message() {
    let (clean_allocs, clean_fires) = measure("capzaclean", true, false);
    let (dirty_allocs, dirty_fires) = measure("capzadirty", true, true);
    let total = (WARM_STEPS + MEASURED_STEPS) as usize;
    assert_eq!(clean_fires, total, "every clean pair fuses");
    assert_eq!(
        dirty_fires, total,
        "the discarded frames never block a pair"
    );
    assert_eq!(
        dirty_allocs, clean_allocs,
        "discarding backwards and undecodable frames must not allocate \
         ({dirty_allocs} vs {clean_allocs} over {MEASURED_STEPS} steps)"
    );
}

/// The pure layer on its own: decoding a stamp and classifying and counting a
/// head op allocate nothing. The comparison above cannot see an allocation
/// both modes share; this pins the capture-time code to zero by itself.
#[test]
#[serial]
fn the_capture_stamp_decode_and_head_op_allocate_nothing() {
    use cerulion_core::scheduler::sync_stamp::{
        capture_head_op, capture_stamp_ns, CaptureIo, HeaderStampLocation, StampDecodeError,
        SyncFrameSlot,
    };
    use cerulion_core::scheduler::{SyncAlignSite, SyncHeadOp, SyncOpAnswer};

    /// A 64-byte frame (its wire header declares all 64) with
    /// `header.stamp` at fixed offset 0 after the wire header.
    fn frame(sec: i32, nanosec: u32) -> [u8; 64] {
        let mut f = [0u8; 64];
        f[8..12].copy_from_slice(&64u32.to_le_bytes());
        let at = cerulion_core::wire::WireHeader::SIZE;
        f[at..at + 4].copy_from_slice(&sec.to_le_bytes());
        f[at + 4..at + 8].copy_from_slice(&nanosec.to_le_bytes());
        f
    }
    const AT: HeaderStampLocation = HeaderStampLocation::Fixed { offset: 0 };

    /// A fixed ring of frames: in order, backwards, undecodable, in order.
    struct Ring {
        frames: [[u8; 64]; 4],
        head: usize,
    }
    impl CaptureIo for Ring {
        fn op(&mut self, op: SyncHeadOp) -> SyncOpAnswer {
            match op {
                SyncHeadOp::Void => SyncOpAnswer::Nothing,
                SyncHeadOp::ProbeNext => SyncOpAnswer::Present,
                SyncHeadOp::PeekNext { .. } => SyncOpAnswer::Stamp(0),
                _ => {
                    self.head = (self.head + 1) % self.frames.len();
                    SyncOpAnswer::Head(0)
                }
            }
        }
        fn stamp(&mut self, slot: SyncFrameSlot) -> Option<Result<u64, StampDecodeError>> {
            let i = match slot {
                SyncFrameSlot::Head => self.head,
                SyncFrameSlot::Next => (self.head + 1) % self.frames.len(),
            };
            Some(capture_stamp_ns(&self.frames[i], AT))
        }
    }

    let mut ring = Ring {
        frames: [
            frame(1, 0),
            frame(0, 999_000_000),
            frame(2, 1_000_000_000),
            frame(3, 0),
        ],
        head: 3,
    };
    let mut hw = None;
    let window = 2_000 * MS;
    let site = SyncAlignSite::Boundary;
    let ops = [
        SyncHeadOp::FillBoundary,
        SyncHeadOp::PeekNext { site },
        SyncHeadOp::Advance { site },
        SyncHeadOp::ProbeNext,
        SyncHeadOp::Void,
    ];
    let mut drops = 0u64;
    ALLOCATOR.enable();
    for _ in 0..256 {
        for op in ops {
            let out = capture_head_op(op, window, &mut hw, &mut ring);
            drops += u64::from(out.decode_failures + out.backwards_drops);
        }
    }
    let allocations = ALLOCATOR.disable();
    assert!(drops > 0, "the walk took the discard arms");
    assert_eq!(allocations, 0, "the capture-time pure layer allocated");
}
