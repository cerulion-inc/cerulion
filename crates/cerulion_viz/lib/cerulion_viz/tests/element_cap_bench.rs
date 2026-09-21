// SPDX-License-Identifier: AGPL-3.0-only
//! THE MEASUREMENT behind [`MAX_ELEMENT_INSTANCES`].
//!
//! The cap first shipped at 10 000 as a design-doc proposal with **no
//! performance measurement**. This file is the evidence: it sweeps the element
//! path over REAL wire frames decoded by the REAL built-in [`FrameWalker`] and
//! rendered through the REAL `dispatch_frame` into a Rerun memory sink, and
//! reports floor / p50 / p99 per element count (the house convention — never a
//! bare mean).
//!
//! # What is measured, and why the split matters
//!
//! Three stages, because they are NOT governed by the same thing:
//!
//! | Stage | What it is | Cap-governed? |
//! |---|---|---|
//! | `walk` | [`FrameWalker::walk_by_hash`] — decode the frame into `FrameValue`, including EVERY element of the array | **NO** — the walker decodes the whole array before the cap is consulted |
//! | `scan` | [`scan_element_arrays`] — extract `[f32; 3]`/box geometry from the elements | **YES** — inspects only the first `MAX_ELEMENT_INSTANCES` |
//! | `render` | `dispatch_frame` + `flush_blocking` — the above plus Rerun archetype construction, Arrow-ification and the sink write | **YES** (its archetype half) |
//!
//! The `walk` column is the load-bearing finding: raising the cap does **not**
//! add decode cost, because that cost is already paid unconditionally. Raising
//! the cap buys back the geometry the walker already decoded.
//!
//! `bytes/frame` is the serialized Rerun payload ([`MemorySinkStorage::drain_as_bytes`])
//! — the exact count of what crosses the link to the viewer, not an estimate.
//!
//! `alloc/frame` is what the MEASURED THREAD allocates to decode + extract one
//! frame (the thread-scoped counting allocator below). It is published because it
//! is the other axis a raised cap moves, and because it is the number that shows
//! the walk — not the cap — owns the cost: it tracks `walk`, not `scan`.
//!
//! # Result (M-series Mac, release, FLOORS — re-measured for the memo fix)
//!
//! | elements | `nav_msgs/Path` render | `Detection3DArray` render | cap-governed µs/element | alloc/frame (Path) |
//! |---:|---:|---:|---:|---:|
//! | 1 000 | 1.5 ms | 2.4 ms | 0.27 / 0.25 | 1.8 MiB |
//! | 10 000 (the OLD cap) | 14.9 ms | 20.3 ms | 0.27 / 0.24 | 19.2 MiB |
//! | 100 000 | 149.9 ms | 198.7 ms | 0.29 / 0.22 | 187.9 MiB |
//! | 300 000 (the cap) | 455.1 ms | 592.6 ms | 0.30 / 0.23 | 578.7 MiB |
//!
//! The cap-governed µs/element column is the `scan` floor ÷ elements — the DIRECT
//! measurement, rather than `render − walk`, whose two noisy terms make the small
//! rows unreadable (one 10 000-row `walk` floor in this run was visibly inflated
//! by ambient load, and `render − walk` reported an impossible 0.13 for it). Taken
//! as `render − walk` on the clean large rows it reads 0.29 / 0.29, consistent
//! with the published **0.32 µs/element** (polylines) and **0.27** (boxes)
//! to within run-to-run noise, so that derivation stands: 100 000 µs (a 10 Hz
//! frame) ÷ 0.32 = 312 500, which is where [`MAX_ELEMENT_INSTANCES`] = 300 000
//! comes from.
//!
//! TOTAL per-frame cost — dominated by the UNCAPPED walk — crosses a 30 Hz
//! budget at ~22 000 elements (polyline, 1.51 µs/element measured) / ~17 000
//! (boxes, 1.99) and a 10 Hz budget at ~66 000 / ~50 000. The old 10 000 cap was
//! therefore already spending 45-59 % of a 30 Hz frame. Raising the cap does not
//! move those crossings, because the cap never governed the decode that causes
//! them.
//!
//! The `500 000` row is the cap working in-band: `drawn` reads 300 000 and its
//! `scan` / `rerun KiB` columns equal the 300 000 row's exactly (90.6 vs 90.3 ms;
//! 2 561.1 KiB both), while `walk` keeps climbing (367.8 → 597.8 ms) and so does
//! `alloc` (578.7 → 888.8 MiB) — the uncapped and capped halves, side by side.
//!
//! # The UNMAPPED arm, and what it found
//!
//! Both rows above are NAME-mapped schemas, which never run the shape ladder — so
//! this harness was structurally blind to the "automagic" path the element ladder
//! exists for. On an UNMAPPED schema `infer_archetype_from_shape` runs the SAME
//! cap-governed scan and throws the geometry away, and then the render arm scans
//! again. [`bench_unmapped_element_array`] measures it, on the same
//! `PoseStamped` elements as the `nav_msgs/Path` row so the only difference is how
//! the archetype is decided (floors):
//!
//! | elements | render (memo warm) | render COLD (earlier) | dup-scan | `nav_msgs/Path` render |
//! |---:|---:|---:|---:|---:|
//! | 10 000 (the OLD cap) | 15.7 ms | 18.3 ms | 2.5 ms | 14.9 ms |
//! | 100 000 | 152.7 ms | 182.9 ms | 30.3 ms | 149.9 ms |
//! | 300 000 (the cap) | 458.7 ms | 552.3 ms | **93.6 ms** | 455.1 ms |
//!
//! Two readings, both load-bearing:
//!
//! - the **COLD** column is what an unseen vendor's `/plan`-shaped topic paid per
//!   frame before the memo fix — at the 300 000 cap, **93.6 ms of it discarded** (a
//!   second run of the same arm read 95.9 ms; the `scan` floor, which is what that
//!   delta is, reads 92.1 ms). The old 10 000 cap made the same waste ~2.5 ms,
//!   which is why it went unnoticed; the element-cap change raised the ceiling 30× and the waste
//!   with it.
//! - the **warm** column lands on the name-mapped row (458.7 vs 455.1 ms at the
//!   cap) — the memo makes the unmapped path cost exactly what the mapped one
//!   costs, which is the whole claim.
//!
//! # Running it
//!
//! The three sweeps are `#[ignore]`d so the normal suite stays fast (the
//! 500 000-element rows build ~40 MB frames; a full run is ~15 minutes):
//!
//! ```bash
//! cargo test -p cerulion_viz --test element_cap_bench --release -- --ignored --nocapture
//! ```
//!
//! The two NON-ignored tests in this file are guards, not measurements, and run in
//! the normal suite: the hostile-count refusal, and
//! [`the_unmapped_bench_arm_really_takes_the_shape_inferred_path`] — which fails
//! loudly if the arm ever stops exercising the unmapped path.
//!
//! Release mode is the representative build: the shipped sink is a release cdylib, and a
//! debug measurement would overstate the cost by roughly an order of magnitude.
//!
//! **Reproducing the cap CHOICE.** Rows above [`MAX_ELEMENT_INSTANCES`] report
//! the CLIPPED cost, so this sweep alone cannot re-derive the unclipped curve
//! past the current ceiling. To re-choose the cap, raise the constant
//! temporarily (the original sweep used 1 000 000) and re-run — the
//! per-element slopes above are what that sweep produced.

use std::alloc::{GlobalAlloc, Layout as AllocLayout, System};
use std::cell::Cell;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use cerulion_core::codegen::layout::{LayoutResolver, WireLayout};
use cerulion_core::codegen::{parse_rosmsg, MessageSchema};
use cerulion_core::message::ShmMessage;
use cerulion_core::shm_runtime::write_offset_entry;
use cerulion_core::wire::WireHeader;
use cerulion_viz::archetype::{scan_element_arrays, ElementArrayScan, MAX_ELEMENT_INSTANCES};
use cerulion_viz::schema_registry::builtin_walker;
use cerulion_viz::sink::{
    classify_schema, dispatch_frame, infer_archetype_from_shape, ArchetypeKind, SinkState,
};

// ============================================================================
// Thread-scoped counting allocator.
//
// Process-wide counting would fold in Rerun's batcher-thread allocations and
// libtest's own churn (the zero-alloc probes' lesson). Arming is a `thread_local!` `Cell`,
// so the count is exactly "what the MEASURED thread allocated inside the
// window" — which is precisely the per-frame element cost we are attributing.
// ============================================================================

static ALLOC_BYTES: AtomicUsize = AtomicUsize::new(0);

thread_local! {
    static ARMED: Cell<bool> = const { Cell::new(false) };
}

struct CountingAlloc;

unsafe impl GlobalAlloc for CountingAlloc {
    unsafe fn alloc(&self, layout: AllocLayout) -> *mut u8 {
        // `try_with` so an allocation during thread teardown (when the TLS slot
        // is already destroyed) can never panic inside the allocator.
        let armed = ARMED.try_with(|a| a.get()).unwrap_or(false);
        if armed {
            ALLOC_BYTES.fetch_add(layout.size(), Ordering::Relaxed);
        }
        unsafe { System.alloc(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: AllocLayout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAlloc = CountingAlloc;

/// Run `f`, returning its result and the bytes THIS thread allocated inside it.
fn measure_alloc<T>(f: impl FnOnce() -> T) -> (T, usize) {
    ALLOC_BYTES.store(0, Ordering::Relaxed);
    ARMED.with(|a| a.set(true));
    let out = f();
    ARMED.with(|a| a.set(false));
    (out, ALLOC_BYTES.load(Ordering::Relaxed))
}

// ============================================================================
// Percentiles — floor / p50 / p99, the house reporting convention.
// ============================================================================

#[derive(Clone, Copy)]
struct Stats {
    floor: Duration,
    p50: Duration,
    p99: Duration,
}

impl Stats {
    /// Nearest-rank percentiles over a sorted copy of `samples`. The FLOOR (min)
    /// is reported alongside p50/p99 because it is the jitter-resistant view of
    /// the real cost, while p99 shows what a loaded machine actually pays.
    fn of(mut samples: Vec<Duration>) -> Self {
        assert!(!samples.is_empty(), "no samples");
        samples.sort_unstable();
        let idx = |q: f64| -> usize {
            let n = samples.len();
            (((n as f64) * q).ceil() as usize).clamp(1, n) - 1
        };
        Self {
            floor: samples[0],
            p50: samples[idx(0.50)],
            p99: samples[idx(0.99)],
        }
    }
}

fn us(d: Duration) -> f64 {
    d.as_secs_f64() * 1e6
}

/// Time `f` `iters` times, discarding a warm-up run.
fn time_n(iters: usize, f: impl FnMut()) -> Stats {
    time_n_with_reset(iters, f, || {})
}

/// [`time_n`], plus a `reset` run OUTSIDE the timing window after every sample.
///
/// The render stage needs this: each `dispatch_frame` pushes a full point set
/// into the Rerun `MemorySinkStorage`, which NEVER self-drains — 50 samples at
/// 500 000 elements accumulated multi-GB of retained chunks, so the measurement
/// degraded into an allocator/page-fault benchmark (observed: 1.2 GB RSS and
/// climbing before this was added). Draining between samples keeps every sample
/// measuring the same steady state.
fn time_n_with_reset(iters: usize, mut f: impl FnMut(), mut reset: impl FnMut()) -> Stats {
    f(); // warm-up: first-touch page faults + branch-predictor priming
    reset();
    let mut samples = Vec::with_capacity(iters);
    for _ in 0..iters {
        let t0 = Instant::now();
        f();
        samples.push(t0.elapsed());
        reset();
    }
    Stats::of(samples)
}

// ============================================================================
// Wire-frame builders (crib: `sink_dispatch_test.rs`, whose builders assert the
// schema shapes they depend on so a codegen drift fails loudly here too).
// ============================================================================

fn all_schemas() -> Vec<MessageSchema> {
    let mut schemas = Vec::new();
    for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
        if let Ok(s) = parse_rosmsg(text, name, Some(pkg)) {
            schemas.push(s);
        }
    }
    schemas
}

fn layout_of(qname: &str) -> WireLayout {
    let (mut resolver, _) = LayoutResolver::new(all_schemas());
    resolver.layout_of(qname).expect("built-in schema")
}

/// `u32 count` + per element (`u32 len`, body) — canonical COUNTED framing.
fn counted_blob(elements: &[Vec<u8>]) -> Vec<u8> {
    let mut v = Vec::new();
    v.extend_from_slice(&(elements.len() as u32).to_le_bytes());
    for e in elements {
        v.extend_from_slice(&(e.len() as u32).to_le_bytes());
        v.extend_from_slice(e);
    }
    v
}

/// One `geometry_msgs/Pose` fixed section: seven f64 LE = 56 bytes.
fn pose_fixed_section(pos: [f64; 3], quat: [f64; 4]) -> Vec<u8> {
    let mut v = Vec::with_capacity(56);
    for c in pos.iter().chain(quat.iter()) {
        v.extend_from_slice(&c.to_le_bytes());
    }
    assert_eq!(v.len(), 56, "Pose fixed section drifted");
    v
}

/// A `std_msgs/Header` sub-frame: fixed `Time` (8 B) | entry[0] `frame_id`.
fn header_body(frame_id: &str) -> Vec<u8> {
    let l = layout_of("std_msgs/Header");
    assert_eq!(
        (l.fixed_size, l.offset_table_bytes()),
        (8, 8),
        "Header shape drifted — update this builder"
    );
    let mut v = vec![0u8; 16];
    write_offset_entry(&mut v, 8, 0, 16, frame_id.len() as u32);
    v.extend_from_slice(frame_id.as_bytes());
    v
}

/// A `geometry_msgs/PoseStamped` element body (the nav2 `/plan` element).
///
/// `hdr` is passed in (rather than resolved here) because these builders run
/// once PER ELEMENT — up to 500 000 times per row — and [`layout_of`] re-parses
/// the entire built-in corpus on every call. The shape assertions that used to
/// live here are hoisted to the frame builders, which run once per row.
fn pose_stamped_body(pos: [f64; 3], quat: [f64; 4], hdr: &[u8]) -> Vec<u8> {
    let mut v = pose_fixed_section(pos, quat);
    v.extend_from_slice(&[0u8; 8]);
    write_offset_entry(&mut v, 56, 0, 64, hdr.len() as u32);
    v.extend_from_slice(hdr);
    v
}

/// A `vision_msgs/Detection3D` element body (the detection-set element).
fn detection3d_body(
    center: [f64; 3],
    quat: [f64; 4],
    size: [f64; 3],
    id: &str,
    hdr: &[u8],
) -> Vec<u8> {
    let mut v = pose_fixed_section(center, quat);
    for c in size {
        v.extend_from_slice(&c.to_le_bytes());
    }
    assert_eq!(v.len(), 80);
    v.extend_from_slice(&[0u8; 24]);
    let var_start = 104u32;
    write_offset_entry(&mut v, 80, 0, var_start, hdr.len() as u32);
    write_offset_entry(&mut v, 80, 1, var_start + hdr.len() as u32, 0);
    write_offset_entry(&mut v, 80, 2, var_start + hdr.len() as u32, id.len() as u32);
    v.extend_from_slice(hdr);
    v.extend_from_slice(id.as_bytes());
    v
}

/// Build a `{header, <array>}` wire frame carrying `blob` verbatim in entry 1.
fn build_header_plus_array_frame(
    qname: &str,
    schema_hash: u64,
    array: &str,
    blob: &[u8],
) -> Vec<u8> {
    let l = layout_of(qname);
    assert_eq!(
        l.variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["header", array],
        "{qname} variable-field declaration order changed — update this builder"
    );
    assert_eq!(l.fixed_size, 0, "{qname} gained a fixed section");
    let table = l.offset_table_bytes();
    let mut payload = vec![0u8; table];
    write_offset_entry(&mut payload, 0, 1, table as u32, blob.len() as u32);
    payload.extend_from_slice(blob);
    let mut frame = vec![0u8; WireHeader::SIZE];
    let header = WireHeader {
        schema_hash,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: WireHeader::SIZE as u32,
        offset_table_count: l.variable_fields.len() as u32,
        sequence: 0,
        timestamp_ns: 42_000,
    };
    header.write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

/// A `nav_msgs/Path` of `n` stamped poses — the nav2 `/plan` shape
/// (`ElementGeometry::Path`, a polyline plus its vertices).
fn path_frame(n: usize) -> Vec<u8> {
    // Shape assertions hoisted OUT of the per-element builder (see its doc).
    let l = layout_of("geometry_msgs/PoseStamped");
    assert_eq!(
        (l.fixed_size, l.offset_table_bytes()),
        (56, 8),
        "PoseStamped shape drifted — update this builder"
    );
    let hdr = header_body("map");
    let bodies: Vec<Vec<u8>> = (0..n)
        .map(|i| pose_stamped_body([i as f64 * 0.05, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], &hdr))
        .collect();
    build_header_plus_array_frame(
        "nav_msgs/Path",
        <native_ros2_messages::nav_msgs::Path as ShmMessage>::SCHEMA_HASH,
        "poses",
        &counted_blob(&bodies),
    )
}

/// A `control_msgs/MotionPrimitive` of `n` stamped poses — the SAME
/// `PoseStamped` elements as [`path_frame`], on a schema that is NOT in
/// [`cerulion_viz::sink::classify_schema`]'s table.
///
/// This is the "unseen vendor" arm. It exists because BOTH earlier sweeps
/// used name-mapped schemas, and the name-mapped half is the CHEAP half: it
/// never runs the shape ladder, so the harness structurally could not see that
/// the unmapped path scanned the element array TWICE per frame (once in
/// `infer_archetype_from_shape`, which discards the geometry, then again in the
/// render arm). Same element type as the mapped `nav_msgs/Path` arm, so the two
/// rows differ ONLY in how the archetype is decided.
///
/// Its variable fields are `[additional_arguments, poses, joint_positions]`; the
/// two the sink does not read are written EMPTY.
fn motion_primitive_frame(n: usize) -> Vec<u8> {
    let pl = layout_of("geometry_msgs/PoseStamped");
    assert_eq!(
        (pl.fixed_size, pl.offset_table_bytes()),
        (56, 8),
        "PoseStamped shape drifted — update this builder"
    );
    let l = layout_of("control_msgs/MotionPrimitive");
    assert_eq!(
        l.variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["additional_arguments", "poses", "joint_positions"],
        "MotionPrimitive variable-field declaration order changed — update this builder"
    );
    assert_eq!(
        (l.fixed_size, l.offset_table_bytes()),
        (16, 24),
        "MotionPrimitive shape drifted — update this builder"
    );
    let hdr = header_body("map");
    let bodies: Vec<Vec<u8>> = (0..n)
        .map(|i| pose_stamped_body([i as f64 * 0.05, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], &hdr))
        .collect();
    let blob = counted_blob(&bodies);

    let (fixed, table) = (l.fixed_size, l.offset_table_bytes());
    let base = (fixed + table) as u32;
    let mut payload = vec![0u8; fixed + table];
    write_offset_entry(&mut payload, fixed, 0, base, 0); // additional_arguments: empty
    write_offset_entry(&mut payload, fixed, 1, base, blob.len() as u32); // poses
    write_offset_entry(&mut payload, fixed, 2, base + blob.len() as u32, 0); // joint_positions
    payload.extend_from_slice(&blob);

    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash:
            <native_ros2_messages::control_msgs::MotionPrimitive as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: WireHeader::SIZE as u32,
        offset_table_count: l.variable_fields.len() as u32,
        sequence: 0,
        timestamp_ns: 42_000,
    }
    .write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

/// A `vision_msgs/Detection3DArray` of `n` oriented boxes
/// (`ElementGeometry::Boxes` — the most expensive element shape we render).
fn detections_frame(n: usize) -> Vec<u8> {
    let l = layout_of("vision_msgs/Detection3D");
    assert_eq!(
        (l.fixed_size, l.offset_table_bytes()),
        (80, 24),
        "Detection3D shape drifted — update this builder"
    );
    assert_eq!(
        layout_of("vision_msgs/BoundingBox3D").fixed_size,
        80,
        "BoundingBox3D fixed size drifted"
    );
    let hdr = header_body("camera");
    let bodies: Vec<Vec<u8>> = (0..n)
        .map(|i| {
            detection3d_body(
                [i as f64 * 0.1, 1.0, 2.0],
                [0.0, 0.0, 0.0, 1.0],
                [0.5, 0.5, 0.5],
                "obj",
                &hdr,
            )
        })
        .collect();
    build_header_plus_array_frame(
        "vision_msgs/Detection3DArray",
        <native_ros2_messages::vision_msgs::Detection3DArray as ShmMessage>::SCHEMA_HASH,
        "detections",
        &counted_blob(&bodies),
    )
}

// ============================================================================
// The sweep.
// ============================================================================

/// Element counts spanning "an ordinary nav2 plan" to "a dense lidar sweep's
/// worth of elements", so the cap can be chosen against where the curve
/// actually crosses a frame budget rather than against a round number.
const SIZES: &[usize] = &[
    1_000,
    10_000, // the earlier cap
    50_000,
    100_000,
    200_000,
    MAX_ELEMENT_INSTANCES, // the chosen cap, measured exactly
    500_000,               // above the cap: the clipped cost, i.e. the cap working
];

/// Frame budgets the per-frame cost is judged against.
const BUDGET_30HZ_US: f64 = 33_333.0;
const BUDGET_10HZ_US: f64 = 100_000.0;

fn iters_for(n: usize) -> usize {
    if n <= 10_000 {
        200
    } else if n <= 100_000 {
        100
    } else {
        50
    }
}

/// How a sweep's schema reaches its archetype, the axis the memo fix added.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Classified {
    /// In [`cerulion_viz::sink::classify_schema`]'s table: a string match, no
    /// shape ladder, so nothing is memoized and there is no cold/warm split.
    NameMapped,
    /// Unmapped — the "automagic" path. The archetype comes from the shape
    /// ladder, whose element rung runs the cap-governed scan and DISCARDS its
    /// geometry before the render arm scans again. The memo fix memoizes that
    /// decision per input, so this arm reports BOTH per-frame costs: `render`
    /// (memo warm — the shipping steady state) and `render COLD` (a fresh
    /// `SinkState` per sample, which re-classifies every frame and is therefore
    /// the earlier cost, measured on the same frame in the same run).
    ShapeInferred,
}

fn sweep(label: &str, input: &str, classified: Classified, build: impl Fn(usize) -> Vec<u8>) {
    let walker = builtin_walker();
    let inferred = classified == Classified::ShapeInferred;
    println!("\n=== {label} ===");
    println!(
        "MAX_ELEMENT_INSTANCES = {MAX_ELEMENT_INSTANCES}  (rows above it are CLIPPED — \
         the scan/render columns then measure the clipped cost, which is the point)"
    );
    if inferred {
        println!(
            "UNMAPPED schema (memoized): `render` is the memo-WARM steady state, \
             `render COLD` re-classifies every frame (the pre-memo cost), and \
             `dup-scan` is their floor delta — the discarded duplicate scan, plus \
             one `SinkState::new()` per cold sample (a `Default` over a handful of \
             empty maps: immaterial at the large rows, a visible share of the 1 000 \
             one). Compare `dup-scan` against the `scan` column: they should agree."
        );
    }
    println!(
        "{:>9} {:>8} {:>10} | {:>26} | {:>26} | {:>26} | {:>10} {:>10}{}",
        "elements",
        "drawn",
        "wire KiB",
        "walk floor/p50/p99 us",
        "scan floor/p50/p99 us",
        "render floor/p50/p99 us",
        "alloc KiB",
        "rerun KiB",
        if inferred {
            " |     render COLD floor/p50/p99 us |  dup-scan ms"
        } else {
            ""
        },
    );

    for &n in SIZES {
        let frame = build(n);
        let wire_kib = frame.len() as f64 / 1024.0;

        // Stage 1: walker decode (NOT cap-governed — the whole array is decoded).
        let walk = time_n(iters_for(n), || {
            let fv = walker.walk_by_hash(&frame).expect("walk");
            std::hint::black_box(&fv);
        });

        // Stage 2: the cap-governed geometry extraction, on a pre-walked value.
        let fv = walker.walk_by_hash(&frame).expect("walk");
        let scan = time_n(iters_for(n), || {
            let s = scan_element_arrays(&fv);
            std::hint::black_box(&s);
        });

        // How many elements actually became geometry this frame.
        let drawn = match scan_element_arrays(&fv) {
            ElementArrayScan::Geometry(p) => match &p.geometry {
                cerulion_viz::archetype::ElementGeometry::Path(v)
                | cerulion_viz::archetype::ElementGeometry::Points(v) => v.len(),
                cerulion_viz::archetype::ElementGeometry::Boxes(b) => b.len(),
            },
            other => panic!("{label} n={n}: expected Geometry, got {other:?}"),
        };

        // Bytes THIS thread allocates to decode + extract one frame.
        let (_, alloc_bytes) = measure_alloc(|| {
            let fv = walker.walk_by_hash(&frame).expect("walk");
            let s = scan_element_arrays(&fv);
            std::hint::black_box(&s);
        });

        // Stage 3: the FULL per-frame viz cost — decode + extract + Rerun
        // archetype construction + Arrow-ification + sink write. Flushed so the
        // batcher's share is inside the window, not deferred off it.
        let (rec, storage) = rerun::RecordingStreamBuilder::new("bench")
            .recording_id("element_cap_bench")
            .memory()
            .expect("memory sink");
        let mut state = SinkState::new();
        let render = time_n_with_reset(
            iters_for(n),
            || {
                dispatch_frame(&rec, &walker, input, &frame, &mut state);
                rec.flush_blocking().expect("flush");
            },
            // Drop the logged chunks between samples (see `time_n_with_reset`).
            || {
                storage.take();
            },
        );
        // The classification cost this row actually paid, as a COUNT:
        // a name-mapped schema never runs the ladder; an unmapped one runs it
        // once for the whole row, warm-up sample included.
        let warm_inferences = state.inference_runs();
        assert_eq!(
            warm_inferences,
            u64::from(inferred),
            "{label} n={n}: expected {} shape inference(s) across the whole render \
             stage — the memo is what keeps the unmapped path off the per-frame \
             double scan",
            u64::from(inferred),
        );

        // Stage 3, COLD (UNMAPPED arms only): the SAME render with a fresh `SinkState`
        // per sample, so every frame re-classifies — i.e. the un-memoized cost,
        // measured on the same frame, on the same machine, in the same run.
        let cold = inferred.then(|| {
            time_n_with_reset(
                iters_for(n),
                || {
                    let mut cold_state = SinkState::new();
                    dispatch_frame(&rec, &walker, input, &frame, &mut cold_state);
                    rec.flush_blocking().expect("flush");
                },
                || {
                    storage.take();
                },
            )
        });

        // Serialized Rerun payload for ONE frame — what crosses the link.
        let (rec1, storage1) = rerun::RecordingStreamBuilder::new("bench_bytes")
            .recording_id("element_cap_bench_bytes")
            .memory()
            .expect("memory sink");
        let mut state1 = SinkState::new();
        dispatch_frame(&rec1, &walker, input, &frame, &mut state1);
        rec1.flush_blocking().expect("flush");
        let rerun_bytes = storage1.drain_as_bytes().map(|b| b.len()).unwrap_or(0);

        let cold_cols = match cold {
            Some(c) => format!(
                " | {:>8.1}{:>9.1}{:>9.1} | {:>12.1}",
                us(c.floor),
                us(c.p50),
                us(c.p99),
                (us(c.floor) - us(render.floor)) / 1000.0,
            ),
            None => String::new(),
        };
        println!(
            "{:>9} {:>8} {:>10.1} | {:>8.1}{:>9.1}{:>9.1} | {:>8.1}{:>9.1}{:>9.1} | \
             {:>8.1}{:>9.1}{:>9.1} | {:>10.1} {:>10.1}{}",
            n,
            drawn,
            wire_kib,
            us(walk.floor),
            us(walk.p50),
            us(walk.p99),
            us(scan.floor),
            us(scan.p50),
            us(scan.p99),
            us(render.floor),
            us(render.p50),
            us(render.p99),
            alloc_bytes as f64 / 1024.0,
            rerun_bytes as f64 / 1024.0,
            cold_cols,
        );
    }

    println!("budgets: 30 Hz = {BUDGET_30HZ_US:.0} us/frame, 10 Hz = {BUDGET_10HZ_US:.0} us/frame");
}

/// The nav2 `/plan` / `PoseArray` shape — ordered vertices, NAME-mapped.
#[test]
#[ignore = "measurement harness — run with --release -- --ignored --nocapture"]
fn bench_path_element_array() {
    sweep(
        "nav_msgs/Path (PoseStamped elements -> polyline) [name-mapped]",
        "plan",
        Classified::NameMapped,
        path_frame,
    );
}

/// The `Detection3DArray` shape — N oriented boxes, the heaviest element we
/// draw, NAME-mapped.
#[test]
#[ignore = "measurement harness — run with --release -- --ignored --nocapture"]
fn bench_detection_element_array() {
    sweep(
        "vision_msgs/Detection3DArray (Detection3D elements -> boxes) [name-mapped]",
        "detections",
        Classified::NameMapped,
        detections_frame,
    );
}

/// The UNMAPPED arm — the same `PoseStamped` elements as
/// [`bench_path_element_array`] on a schema the table does not carry, so the
/// archetype comes from the shape ladder.
///
/// This is the row both earlier sweeps were structurally blind to: they only
/// measured name-mapped schemas, which never infer, so the duplicated
/// cap-governed scan on the "automagic" path never appeared in any published
/// number. Its `render` vs `render COLD` columns are the memo's saving.
#[test]
#[ignore = "measurement harness — run with --release -- --ignored --nocapture"]
fn bench_unmapped_element_array() {
    sweep(
        "control_msgs/MotionPrimitive (PoseStamped elements -> polyline) [shape-inferred]",
        "motion",
        Classified::ShapeInferred,
        motion_primitive_frame,
    );
}

/// The structural pin for [`bench_unmapped_element_array`]: the arm really does
/// exercise the UNMAPPED path, and really does reach the element rung.
///
/// NOT `#[ignore]`d and deliberately tiny — if `control_msgs/MotionPrimitive`
/// ever gains a `classify_schema` entry, or its shape stops inferring to a path,
/// the unmapped sweep would silently degrade into a second name-mapped row
/// measuring nothing new. That is exactly the blindness the memo fix was filed for,
/// so it fails loudly here instead.
#[test]
fn the_unmapped_bench_arm_really_takes_the_shape_inferred_path() {
    let walker = builtin_walker();
    assert_eq!(
        classify_schema("control_msgs/MotionPrimitive"),
        None,
        "the unmapped bench arm needs a schema the name table does NOT carry"
    );
    assert_eq!(
        classify_schema("nav_msgs/Path"),
        Some(ArchetypeKind::Path3D),
        "and its mapped twin must still BE mapped, or the two arms measure the same path"
    );
    let frame = motion_primitive_frame(3);
    let fv = walker.walk_by_hash(&frame).expect("walk MotionPrimitive");
    assert_eq!(
        infer_archetype_from_shape(&fv),
        ArchetypeKind::Path3D,
        "the shape ladder must reach its ELEMENT rung — otherwise the arm measures \
         a cheap early rung, not the cap-governed scan"
    );
    // Hand oracle on the geometry: `motion_primitive_frame` lays element i at
    // x = i * 0.05, y = z = 0.
    match scan_element_arrays(&fv) {
        ElementArrayScan::Geometry(p) => {
            assert_eq!(p.field, "poses");
            assert_eq!(p.truncated, 0);
            assert_eq!(
                p.geometry,
                cerulion_viz::archetype::ElementGeometry::Path(vec![
                    [0.0, 0.0, 0.0],
                    [0.05, 0.0, 0.0],
                    [0.10, 0.0, 0.0],
                ])
            );
        }
        other => panic!("expected Geometry, got {other:?}"),
    }
    // And the memo really is what the sweep's `infer` assertion depends on.
    let (rec, _storage) = rerun::RecordingStreamBuilder::new("arm_probe")
        .recording_id("arm_probe")
        .memory()
        .expect("memory sink");
    let mut state = SinkState::new();
    for _ in 0..4 {
        dispatch_frame(&rec, &walker, "motion", &frame, &mut state);
    }
    assert_eq!(
        state.inference_runs(),
        1,
        "four frames, one classification — the steady state the `render` column measures"
    );
}

/// The hostile-input claim the cap is often credited with, measured instead of
/// assumed: a frame DECLARING a huge element count but carrying a short body is
/// refused by the WALKER's count budget (`count > remaining / 4` ⇒ opaque),
/// long before `MAX_ELEMENT_INSTANCES` is consulted. The cap is a RENDER
/// ceiling, not the allocation defense — so raising it cannot open a
/// memory-amplification hole.
#[test]
fn a_declared_billion_elements_is_refused_by_the_walker_not_the_render_cap() {
    let walker = builtin_walker();
    // A well-formed 3-pose plan, then the count word overwritten with 1e9.
    let frame = path_frame(3);
    let intact = walker.walk_by_hash(&frame).expect("walk");
    assert!(
        matches!(scan_element_arrays(&intact), ElementArrayScan::Geometry(_)),
        "control: the unmodified 3-pose frame decodes"
    );

    // The blob starts right after the wire header + the 2-entry offset table.
    let blob_start = WireHeader::SIZE + layout_of("nav_msgs/Path").offset_table_bytes();
    let mut hostile = frame.clone();
    hostile[blob_start..blob_start + 4].copy_from_slice(&1_000_000_000u32.to_le_bytes());

    let fv = walker.walk_by_hash(&hostile).expect("walk hostile");
    // Refused at the walker's byte budget: no 1e9-element Vec is ever sized.
    assert!(
        matches!(scan_element_arrays(&fv), ElementArrayScan::Absent),
        "a declared 1e9-element array must be refused as opaque by the walker's \
         count budget, NOT clipped by the render cap"
    );
}

// NOTE: the genuine cap-sized CLIP proof deliberately does NOT live here. It is
// inherently cap-sized (you cannot observe clipping below the ceiling), so it is
// kept in exactly ONE place rather than three:
//
// - `sink_dispatch_test::an_over_cap_element_array_renders_the_ceiling_and_warns_once_per_input`
//   — end-to-end over real wire frames, using the CHEAPEST over-cap shape
//   (fixed 56-byte `Pose` elements);
// - `archetype::tests::an_over_cap_element_array_truncates_and_reports_the_remainder`
//   — the unit-level twin;
// - `archetype::tests::element_render_split_is_the_one_ceiling_oracle` — the
//   arithmetic at every boundary including `usize::MAX`, allocation-free.
//
// Repeating it here with `nav_msgs/Path` (the most expensive element shape in
// this file) would add seconds to the suite for a claim already pinned.
