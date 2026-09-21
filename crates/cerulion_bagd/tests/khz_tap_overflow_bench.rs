// SPDX-License-Identifier: AGPL-3.0-only
//! The kHz × ~100-tap TAP-QUEUE OVERFLOW benchmark.
//!
//! A MEASUREMENT, not a gate (`#[ignore]`d, like its
//! [`go2_firehose_bench`](./go2_firehose_bench.rs) and
//! [`firehose_bench`](./firehose_bench.rs) siblings).
//!
//! The CI gates for the same class are elsewhere, and deliberately not
//! wall-clock: `cerulion_bagd`'s `the_pacing_table_never_sleeps_the_full_idle_tick_on_a_warm_recorder`
//! (the pacing decision as an oracle table — contention can only make drain gaps
//! LONGER, so any "cadence is quick enough" e2e is the load-inversion class) and
//! `cerulion_core/tests/ingress_route_depth_iox2_test.rs` (the provisioning
//! half, over real iceoryx2).
//!
//! ```text
//! cargo test -p cerulion_bagd --test khz_tap_overflow_bench -- --ignored --nocapture
//! ```
//!
//! # The shape it reproduces
//!
//! Measured on a Unitree Go2:
//! `graph run attach --single-process --record`, 102 taps, 73.9 s.
//!
//! | topic | lost | recorded | loss | implied commit rate |
//! |---|---|---|---|---|
//! | `/state_estimation` | 87,130 | 35,833 | **70.9 %** | ~1.66 kHz |
//! | `/tf` | 87,104 | 35,859 | **70.8 %** | ~1.66 kHz |
//! | `/lowcmd` | 65 | 51,479 | 0.13 % | ~696 Hz |
//! | ~99 others | <= 42 | — | ~0 | a few Hz |
//!
//! `dropped_unwritten = 0`, `prefix_lost = 0`, `headerless = 0` — the writer
//! path is clean, so every one of those 174,234 frames died in the topic's own
//! SHM receive queue between the bridge's commit and the recorder's next drain.
//!
//! # Why the sibling benches do not cover it
//!
//! `firehose_bench` is uniform-rate with discovery off. `go2_firehose_bench` is
//! Go2-faithful but tops out at 698 Hz, the fastest topic in the loss table it
//! was modelled on. Neither bench has a topic above 700 Hz,
//! and 700 Hz is precisely the rate that survives.
//!
//! # The arithmetic
//!
//! A tap's queue is `D` frames deep. Its topic commits at `R` Hz. The recorder
//! drains every tap to empty once per drive-loop pass, so a frame is lost iff
//! more than `D` of them arrive between two consecutive drains. Writing `g` for
//! that gap:
//!
//! ```text
//! loss(R, D) = E[max(0, R*g - D)] / (R * E[g])
//! ```
//!
//! `D = 16` (what `create_ingress_publisher` provisions) absorbs **9.6 ms** at
//! 1.66 kHz and **22.9 ms** at 700 Hz. That 2.4x is the whole difference between
//! the 70.9 % row and the 0.13 % row, and it says the robot's drain gaps cluster
//! BETWEEN those two numbers. Neither of the two gap models an earlier analysis fitted
//! reproduces that pair:
//!
//! * uniform cadence `C`: the 70.9 % row implies `C = 33.2 ms`, which would cost
//!   the 700 Hz row 31 % — it lost 0.13 %.
//! * rare stalls `(lambda, T)`: solving both rows gives `lambda*T = 1.22`, i.e.
//!   the recorder stalling more than 100 % of the time. Not physical.
//!
//! So the robot's distribution is neither, and no two-parameter fit will
//! recover it, which is exactly why the recorder ships the histogram
//! ([`DrainGapHistogram`]) rather than another scalar, and why this bench PRINTS
//! that histogram beside the loss table instead of inferring one.
//!
//! (Caveat, stated because it changes what the pair means: `/lowcmd` is one of
//! the run's four DECLARED topics, and a declared topic under `--record` is
//! provisioned by `recording_tap_buffer_depth`, not by
//! `create_ingress_publisher`. If its queue was deeper than 16, the 2.4x above
//! is a floor on the real separation and the "clusters between" reading is
//! weaker than it looks. The bench does not need to resolve this — it puts a
//! 700 Hz topic and a 1.66 kHz topic on IDENTICAL provisioning and reads the
//! separation directly.)
//!
//! # Modes
//!
//! [`khz_tap_overflow_at_the_go2_shape`] runs one of two shapes, selected by
//! `KHZ_TAP_MODE`. Anything else is a hard failure (see [`Mode::parse`]) — a
//! bench run under a typo'd mode must not silently measure a different shape
//! and be read as if it measured the one that was asked for.
//!
//! * `KHZ_TAP_MODE=repro` (default) — the 102-tap Go2 shape at production
//!   provisioning. Prints the per-topic loss table, the conservation identity,
//!   and the drain-gap histogram with the depth each topic's measured
//!   rate demands.
//! * `KHZ_TAP_MODE=depth` — the fit-free half: ONE 1.66 kHz stream tapped at
//!   depths 16..4096 in ONE run, on ONE drive loop, with the ~100-tap background
//!   present. Loss vs depth traces the gap distribution's tail directly and
//!   measures the absorbing depth with no model at all.
//! * `KHZ_TAP_MODE=burst`: the burst shape, which is the repro shape PLUS a
//!   large-payload topic that publishes in CLUMPS rather than steadily, so the
//!   recorder's drive loop takes an OUTLIER pass while the kHz victims keep
//!   committing. See the burst-shape section below.
//!
//! # The burst shape, and what it is for
//!
//! The `repro` and `depth` modes above hold the drive loop at a roughly UNIFORM
//! cadence, so they measure the loss a steady gap distribution causes. The Go2's
//! RESIDUAL loss is a different shape: with the steady loss gone (174,348 → 3,583
//! frames, −97.9 %), what remains is concentrated in TWO drive passes —
//! `max_pass_duration_us` = 107,898 with two drain-gap-histogram entries in the
//! 100–250 ms bucket, while the p50 stayed sub-millisecond. A ~1.66 kHz topic
//! commits 170+ frames across 100 ms; its queue holds 64
//! (`ingress_route_buffer_depth(1 MiB)`), which absorbs 38.6 ms; the rest are
//! reclaimed in SHM at commit before the drain arrives (the queue
//! depth IS the loss boundary).
//!
//! `burst` reproduces the CAUSE rather than the symptom: [`HEAVY_TOPICS`] heavy
//! rows ([`HEAVY_BODY`] bytes/frame, the `/utlidar/cloud` class) each publish
//! [`HEAVY_BURST`] frames back-to-back [`HEAVY_BURSTS_PER_SEC`] times a second,
//! so one pass has to stage and write a clump of large frames while every other
//! pass is cheap. That is a bimodal cadence produced by a real workload, not an
//! injected sleep. How MUCH heavy load a machine needs before its drive loop takes
//! outlier passes is a property of the machine — see [`HEAVY_TOPICS`] for the
//! measured ladder and for what the shipped default costs.
//!
//! The victims' SLICE is the variable under test, because the slice is what sets
//! the depth (`clamp(64 MiB / slice, 16, 1024)`) and the fix is to size
//! it per route from the route's schema instead of handing every route the
//! bridge-wide 1 MiB:
//!
//! * unset — victims get `KHZ_TAP_MSL` (1 MiB), i.e. EXACTLY what a `ros2 attach`
//!   route gets today. This is the BEFORE reading.
//! * `KHZ_TAP_VICTIM_TYPE=pkg/Type` — victims get the slice the PRODUCTION rule
//!   ([`cerulion_core::codegen::route_slice_budget_capped`]) derives for that ROS
//!   type against the same 1 MiB default. This is the AFTER reading, and it is
//!   the production function rather than a number retyped into the bench.
//! * `KHZ_TAP_VICTIM_MSL=<bytes>` — an explicit override, for sweeping the slice
//!   independently of any type.
//!
//! The tap-count sweep — cadence as a function of how many taps the recorder
//! holds, the term the robot has and a desk usually does not — is NOT a mode of
//! that test. It is the separate [`drive_loop_cadence_by_tap_count`] test, which
//! reads no mode at all and is swept with `KHZ_TAP_TAPS`.
//!
//! Knobs: `KHZ_TAP_SECS`, `KHZ_TAP_KHZ`, `KHZ_TAP_BODY`, `KHZ_TAP_MSL`,
//! `KHZ_TAP_BACKGROUND`, `KHZ_TAP_DISCOVERY`, `KHZ_TAP_DEPTHS`, `KHZ_TAP_TAPS`,
//! and (burst mode) `KHZ_TAP_VICTIM_TYPE`, `KHZ_TAP_VICTIM_MSL`,
//! `KHZ_TAP_HEAVY_BURST`, `KHZ_TAP_HEAVY_RATE`, `KHZ_TAP_HEAVY_TOPICS`.
//!
//! # Scope
//!
//! Wall-clock cadence is a property of the MACHINE. A desk with an NVMe and 16 idle
//! cores has a faster drive loop than a Jetson recording to eMMC, so the desk's
//! ABSOLUTE loss percentage is not the robot's and is not claimed to be. What
//! transfers, and what the fix is verified against, are the two load-immune
//! quantities:
//!
//! 1. the SEPARATION between a kHz topic and a sub-kHz one at the same depth
//!    (a ratio taken inside one run, against one drive loop), and
//! 2. the loss-vs-depth curve, whose knee is where a queue stops overflowing —
//!    a per-run measurement of that run's own gap tail.
//!
//! Every wall figure is printed beside the load average it was taken under.

mod common;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_bagd::{run_bagd, DrainGapHistogram, TapSpec};
use cerulion_core::codegen::{parse_rosmsg, CdrCodec, MessageSchema};
use cerulion_core::transport::INGRESS_ROUTE_BUFFER_DEPTH;
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::TransportManager;
use serial_test::serial;

use common::*;

/// The two Point-LIO outputs that lost 70.9 % / 70.8 %, at the rate the issue's
/// own arithmetic implies (`(recorded + lost) / 73.9 s`).
const KHZ_HZ: u64 = 1_660;

/// The contrast row: fast enough to be a real control stream, slow enough to
/// have survived on the robot. Same provisioning as the kHz rows here, which is
/// the one thing the robot run cannot promise (see the module caveat).
const CONTRAST_HZ: u64 = 700;

/// ~70 background topics at a few Hz each, exactly as in `go2_firehose_bench`.
const BACKGROUND_HZ: u64 = 3;

/// The bridge's per-route slice capacity: `dds_bridge`'s
/// `DEFAULT_RAW_MAX_SLICE_LEN`, which every `ros2 attach` route gets because the
/// generated bridge config never emits `max_slice_len:`. It is the multiplier in
/// the SHM cost of any depth raise, so the bench must carry the real one.
const BRIDGE_MAX_SLICE_LEN: u32 = 1 << 20;

/// The HEAVY topic's frame body — `/utlidar/cloud`'s ~46 KB/frame,
/// the prime suspect for the outlier passes in the measured Go2 run. It is
/// the STAGE-AND-WRITE cost of a clump of these that makes one drive pass long.
const HEAVY_BODY: usize = 46 * 1024;

/// How many heavy frames land back-to-back in one clump. 96 × 46 KiB = 4.3 MiB
/// per clump per row, so a clump crosses `CHUNK_TARGET` and the pass that sees
/// it has to hand a chunk to the writer as well as stage it.
const HEAVY_BURST: u64 = 96;

/// How many clumps a second.
const HEAVY_BURSTS_PER_SEC: u64 = 10;

/// How many heavy topics clump at once.
///
/// MEASURED on an M3 Max, and the reason this is not 1: an outlier drive pass is
/// a property of the recorder's per-pass WORK against the machine it runs on, and a
/// desk is not a Jetson recording to eMMC. The ladder, all at 15–20 s with the
/// same 102-tap shape and the victims at today's depth 64:
///
/// | heavy rows × clump × rate | offered | worst pass | victim loss |
/// |---|---|---|---|
/// | 1 × 24 × 4 | 5.5 MB/s | 57.6 ms | 0.00 % |
/// | 6 × 24 × 8 | 52.7 MB/s | 7.3 ms | 0.00 % |
/// | 16 × 64 × 8 | 368.6 MB/s | 38.8 ms | 0.00 % |
/// | 16 × 96 × 10 | 690.7 MB/s | 41.7 ms | 0.03 / 0.04 % |
/// | **24 × 96 × 10** | **1035.5 MB/s** | **68.8 ms** | **1.87 / 1.91 %** |
///
/// The last row is the shipped default because it lands on the Go2's own
/// reading — 1.4 / 1.5 % on the two kHz topics, with the drain-gap histogram
/// carrying an entry in the 100–250 ms bucket — which is what makes the AFTER
/// run a comparable measurement rather than a different experiment.
///
/// That last figure MOVES with the desk's ambient load, as any wall-derived one
/// does: three runs at the shipped default measured 1.46 / 1.56 %, 1.87 / 1.91 %
/// and 4.54 / 4.62 % at load averages of 7.5, 11.7 and 14.5. What does NOT move
/// is the direction and the control — the contrast row read 0.00 % in every one
/// of them, and so did the victims once their slice was derived. Read the
/// BEFORE/AFTER pair from ONE back-to-back pairing, never across sessions.
///
/// The knee is STEEP (0.04 % → 1.9 % between the last two rows), which is the
/// arithmetic doing what it says: loss is `E[max(0, R·g − D)] / (R·E[g])`, so it
/// is ~0 until the gap tail crosses `D/R` = 38.6 ms and then rises fast.
///
/// The cost of standing that far up the ladder, stated plainly: at 1 GB/s
/// offered the recorder is in a WRITER-SATURATED regime, so the heavy rows
/// themselves report large `staging_full_passes` and lose frames of their own.
/// That is collateral, not the reading — the reading is the VICTIM rows against
/// the CONTRAST row on identical provisioning, and the contrast row (700 Hz,
/// same depth 64, same recorder, same passes) stays at 0.00 %.
const HEAVY_TOPICS: usize = 24;

/// Which shape [`khz_tap_overflow_at_the_go2_shape`] runs, from `KHZ_TAP_MODE`.
///
/// An ENUM rather than the raw string, so every consumer below matches
/// exhaustively: a mode that exists in the doc but in no arm — which is exactly
/// how a `taps` value would fall through to the repro plan and
/// silently measure the wrong shape — cannot be written.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    /// The 102-tap Go2 shape at production provisioning.
    Repro,
    /// One kHz stream tapped at many depths in one run.
    Depth,
    /// The repro shape plus a clump-publishing heavy topic, so the
    /// drive loop takes OUTLIER passes while the kHz victims keep committing.
    Burst,
}

impl Mode {
    /// The accepted set, and the single source of truth for both parsing and
    /// the "accepted:" list in the rejection message.
    const ALL: [Mode; 3] = [Mode::Repro, Mode::Depth, Mode::Burst];

    fn as_str(self) -> &'static str {
        match self {
            Mode::Repro => "repro",
            Mode::Depth => "depth",
            Mode::Burst => "burst",
        }
    }

    /// Parse `KHZ_TAP_MODE`, LOUDLY.
    ///
    /// A bench is read for its numbers, so an unrecognised mode must stop the
    /// run rather than quietly select the default shape: the output of a
    /// `KHZ_TAP_MODE=dpeth` run is indistinguishable from a repro run, and would
    /// be reported as a depth sweep.
    fn parse(raw: &str) -> Mode {
        Mode::ALL
            .into_iter()
            .find(|m| m.as_str() == raw)
            .unwrap_or_else(|| {
                let accepted: Vec<&str> = Mode::ALL.iter().map(|m| m.as_str()).collect();
                panic!(
                    "KHZ_TAP_MODE={raw:?} is not a mode this bench has; accepted: {}. \
                     (A tap-count sweep is not a mode — it is the separate \
                     `drive_loop_cadence_by_tap_count` test, swept with KHZ_TAP_TAPS.)",
                    accepted.join(" | ")
                )
            })
    }
}

fn env_or(key: &str, default: u64) -> u64 {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

fn env_str(key: &str, default: &str) -> String {
    std::env::var(key).unwrap_or_else(|_| default.to_string())
}

/// Every wall figure is only interpretable beside the contention it was
/// taken under.
fn loadavg() -> String {
    if let Ok(s) = std::fs::read_to_string("/proc/loadavg") {
        return s.split_whitespace().take(3).collect::<Vec<_>>().join(" ");
    }
    std::process::Command::new("sysctl")
        .args(["-n", "vm.loadavg"])
        .output()
        .ok()
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map(|s| {
            s.trim()
                .trim_matches(|c| c == '{' || c == '}')
                .trim()
                .to_string()
        })
        .unwrap_or_else(|| "unknown".to_string())
}

/// One topic in the shape.
struct Plan {
    topic: String,
    label: String,
    /// Publish TICKS per second. With `burst == 1` this is the frame rate; with
    /// `burst > 1` the frame rate is `hz * burst` delivered in clumps.
    hz: u64,
    /// `None` = created by the PRODUCTION `create_ingress_publisher`, so the
    /// depth under test is whatever production provisions TODAY. `Some(d)` =
    /// hand-provisioned to `d` (the depth sweep).
    depth: Option<usize>,
    body_len: usize,
    /// This route's slice. It is per-plan: the whole point of the
    /// burst mode is that the VICTIMS carry a different slice — and therefore a
    /// different depth — from the heavy row beside them.
    msl: u32,
    /// Frames published back-to-back per tick. 1 = steady.
    burst: u64,
}

struct Row {
    label: String,
    hz_actual: f64,
    depth: usize,
    published: u64,
    recorded: u64,
    /// Gap-derived, from `record_health.json`. Under-reads a pre-baseline
    /// prefix; `published - recorded` is the reading that cannot miss anything.
    gap_lost: u64,
}

impl Row {
    fn missing(&self) -> u64 {
        self.published.saturating_sub(self.recorded)
    }
    fn loss_pct(&self) -> f64 {
        100.0 * self.missing() as f64 / self.published.max(1) as f64
    }
}

/// MEASURE (never assume) a tap's effective receive-queue depth.
///
/// A throwaway topic is filled far past any plausible ceiling with NO drain, so
/// iceoryx2 keeps exactly the newest `depth` samples; draining then counts them.
/// The anti-vacuity assert is what makes it a measurement: if the burst did not
/// overflow, the number returned is the burst size and means nothing.
fn probe_queue_depth(mgr: &TransportManager, depth: Option<usize>, msl: u32) -> usize {
    let topic = unique_topic("probe");
    let mut pubr = match depth {
        None => mgr
            .create_ingress_publisher(&topic, MaxSliceLen::try_new(msl).expect("msl"))
            .expect("create_ingress_publisher"),
        Some(d) => publisher_with_provisioning(mgr, &topic, 2, d, msl),
    };
    let mut tap = mgr
        .create_data_only_subscriber(&topic)
        .expect("data-only tap");
    // A production depth is now SLICE-DEPENDENT and can reach
    // `INGRESS_ROUTE_BUFFER_DEPTH`, so an unpinned probe must burst past the CAP:
    // the earlier `unwrap_or(16)` sized the burst for the stock ceiling and
    // tripped its own anti-vacuity assert the moment a small-slice route got a
    // deep queue.
    let burst = depth.unwrap_or(INGRESS_ROUTE_BUFFER_DEPTH) * 8 + 256;
    let body = vec![7u8; 64];
    for k in 0..burst {
        pubr.publish_raw(&build_frame(0xC1230, k as u32, k as u64, &body))
            .expect("publish");
    }
    let mut scratch = Vec::with_capacity(1);
    let mut drained = 0usize;
    loop {
        let n = tap.drain_owned(1, &mut scratch).expect("drain");
        scratch.clear();
        if n == 0 {
            break;
        }
        drained += n;
    }
    assert!(
        drained < burst,
        "probe never overflowed ({drained} of {burst} drained) — it measured the \
         burst, not the queue"
    );
    drained
}

/// Apparent SHM reservation of ONE publisher's data segment, in bytes, from
/// iceoryx2 0.9.1's own formula
/// (`static_config::publish_subscribe::required_amount_of_samples_per_data_segment`):
///
/// ```text
/// slots = max_subscribers * (subscriber_max_buffer_size + subscriber_max_borrowed_samples)
///       + history_size + publisher_max_loaned_samples
/// ```
///
/// With `create_ingress_publisher`'s config: `max_subscribers` and
/// `subscriber_max_borrowed_samples` and `publisher_max_loaned_samples` all left
/// at iceoryx2's defaults (8 / 2 / 2), `history_size = 0`.
fn apparent_pool_bytes(depth: usize, msl: u32) -> u64 {
    const IOX2_DEFAULT_MAX_SUBSCRIBERS: u64 = 8;
    const IOX2_DEFAULT_MAX_BORROWED: u64 = 2;
    const IOX2_DEFAULT_MAX_LOANED: u64 = 2;
    let slots = IOX2_DEFAULT_MAX_SUBSCRIBERS * (depth as u64 + IOX2_DEFAULT_MAX_BORROWED)
        + IOX2_DEFAULT_MAX_LOANED;
    slots * msl as u64
}

fn gib(bytes: u64) -> f64 {
    bytes as f64 / (1024.0 * 1024.0 * 1024.0)
}

/// The slice the kHz VICTIMS are created with, and a human-readable
/// account of where it came from (printed, because the whole BEFORE/AFTER
/// reading turns on it).
///
/// Precedence mirrors production's own (explicit > schema budget > default):
/// `KHZ_TAP_VICTIM_MSL` wins, then `KHZ_TAP_VICTIM_TYPE` through the PRODUCTION
/// rule, then the bridge-wide default.
fn victim_slice(default_msl: u32) -> (u32, String) {
    if let Ok(raw) = std::env::var("KHZ_TAP_VICTIM_MSL") {
        let n: u32 = raw
            .parse()
            .unwrap_or_else(|_| panic!("KHZ_TAP_VICTIM_MSL={raw:?} is not a byte count"));
        return (n, format!("KHZ_TAP_VICTIM_MSL={n}"));
    }
    let Ok(ros_type) = std::env::var("KHZ_TAP_VICTIM_TYPE") else {
        return (
            default_msl,
            format!("bridge default (today's production) = {default_msl} B"),
        );
    };

    // The SAME corpus + the SAME production function the bridge resolves
    // against — not a number retyped into the bench.
    let schemas: Vec<MessageSchema> = native_ros2_messages::BUILTIN_MSGS
        .iter()
        .filter_map(|(package, name, text)| parse_rosmsg(text, name, Some(package)).ok())
        .collect();
    let (codec, _warnings) = CdrCodec::new(schemas);
    let configured = MaxSliceLen::try_new(default_msl).expect("default msl");
    let derived = codec
        .route_slice_budget(&ros_type, configured)
        .unwrap_or_else(|| {
            panic!(
                "KHZ_TAP_VICTIM_TYPE={ros_type:?} is not in the built-in corpus — the bench \
                 cannot derive a slice for a type production could not resolve either"
            )
        });
    (
        derived.get(),
        format!(
            "route_slice_budget_capped({ros_type}, {default_msl}) = {} B",
            derived.get()
        ),
    )
}

fn plan_topics(
    mode: Mode,
    body_len: usize,
    background: usize,
    khz_n: usize,
    msl: u32,
    victim_msl: u32,
) -> Vec<Plan> {
    let heavy_burst = env_or("KHZ_TAP_HEAVY_BURST", HEAVY_BURST);
    let heavy_rate = env_or("KHZ_TAP_HEAVY_RATE", HEAVY_BURSTS_PER_SEC);
    let mut plans = Vec::new();
    match mode {
        Mode::Depth => {
            let depths: Vec<usize> = std::env::var("KHZ_TAP_DEPTHS")
                .ok()
                .map(|v| v.split(',').filter_map(|d| d.trim().parse().ok()).collect())
                .unwrap_or_else(|| vec![16usize, 32, 64, 96, 128, 192, 256, 512, 1024]);
            for d in depths {
                plans.push(Plan {
                    topic: unique_topic(&format!("khz_d{d}")),
                    label: format!("{KHZ_HZ}Hz/depth{d}"),
                    hz: KHZ_HZ,
                    depth: Some(d),
                    body_len,
                    msl,
                    burst: 1,
                });
            }
        }
        Mode::Repro => {
            for i in 0..khz_n {
                plans.push(Plan {
                    topic: unique_topic(&format!("khz{i}")),
                    label: format!("khz{i}@{KHZ_HZ}"),
                    hz: KHZ_HZ,
                    depth: None,
                    body_len,
                    msl,
                    burst: 1,
                });
            }
            plans.push(Plan {
                topic: unique_topic("contrast"),
                label: format!("contrast@{CONTRAST_HZ}"),
                hz: CONTRAST_HZ,
                depth: None,
                body_len,
                msl,
                burst: 1,
            });
        }
        Mode::Burst => {
            // A frame LARGER than its route's slice fails to publish (a loud,
            // counted drop — `publish_raw` loans exactly `frame.len()`), so a
            // victim whose derived slice is smaller than the configured body
            // publishes frames of the size that slice actually admits. Only
            // bites for a type whose budget is under `body_len` + 32; every
            // schema tier is >= 16 KiB, so it is the FIXED arm that reaches it.
            let victim_body = body_len.min(victim_msl as usize - 32);
            // The VICTIMS — the kHz rows, at the slice under test.
            for i in 0..khz_n {
                plans.push(Plan {
                    topic: unique_topic(&format!("khz{i}")),
                    label: format!("khz{i}@{KHZ_HZ}"),
                    hz: KHZ_HZ,
                    depth: None,
                    body_len: victim_body,
                    msl: victim_msl,
                    burst: 1,
                });
            }
            // The CAUSE — heavy topics publishing in clumps. They keep the
            // bridge-wide slice, exactly as a real point-cloud route does.
            let heavy_topics = env_or("KHZ_TAP_HEAVY_TOPICS", HEAVY_TOPICS as u64) as usize;
            for i in 0..heavy_topics {
                plans.push(Plan {
                    topic: unique_topic(&format!("heavy{i}")),
                    label: format!("heavy{i}@{heavy_rate}x{heavy_burst}"),
                    hz: heavy_rate,
                    depth: None,
                    body_len: HEAVY_BODY,
                    msl,
                    burst: heavy_burst,
                });
            }
            // The contrast row, on the victims' provisioning: a sub-kHz stream
            // that the same outlier passes do NOT cost, which is what makes the
            // victims' loss attributable to their RATE against their DEPTH.
            plans.push(Plan {
                topic: unique_topic("contrast"),
                label: format!("contrast@{CONTRAST_HZ}"),
                hz: CONTRAST_HZ,
                depth: None,
                body_len,
                msl: victim_msl,
                burst: 1,
            });
        }
    }
    for i in 0..background {
        plans.push(Plan {
            topic: unique_topic(&format!("bg{i}")),
            label: format!("bg{i}@{BACKGROUND_HZ}"),
            hz: BACKGROUND_HZ,
            depth: None,
            body_len,
            msl,
            burst: 1,
        });
    }
    plans
}

/// Print a drain-gap histogram as a table, plus the depth it implies for each
/// rate the run carried. This is the histogram doing its job:
/// the number that would otherwise need a two-parameter fit is read off
/// the recording.
fn print_gaps(h: &DrainGapHistogram, rates: &[(&str, f64)]) {
    println!("\n  drain-gap histogram ({} gaps):", h.total());
    if h.is_vacant() {
        println!("    VACANT — the loop recorded no gaps");
        return;
    }
    let total = h.total().max(1);
    let mut cum = 0u64;
    for (i, c) in h.counts.iter().enumerate() {
        if *c == 0 {
            continue;
        }
        cum += c;
        let edge = h
            .edges_us
            .get(i)
            .map(|e| format!("<={:>9.3} ms", *e as f64 / 1000.0))
            .unwrap_or_else(|| "  OVERFLOW   ".to_string());
        println!(
            "    {edge}  {c:>8}  ({:>6.2} %, cum {:>6.2} %)",
            100.0 * *c as f64 / total as f64,
            100.0 * cum as f64 / total as f64
        );
    }
    let q = |p: f64| {
        h.quantile_us(p)
            .map(|v| format!("{:.3} ms", v as f64 / 1000.0))
            .unwrap_or_else(|| "OVERFLOW (unbounded)".to_string())
    };
    println!(
        "    p50 {}   p90 {}   p99 {}   max {}",
        q(0.5),
        q(0.9),
        q(0.99),
        q(1.0)
    );
    println!("    absorbing depth (frames of tap queue) implied by this cadence:");
    for (label, hz) in rates {
        let d = |p: f64| {
            h.absorbing_depth(*hz, p)
                .map(|v| v.to_string())
                .unwrap_or_else(|| "unbounded".to_string())
        };
        println!(
            "      {label:<18} {hz:>7.1} Hz -> p50 {:>6}  p99 {:>6}  max {:>6}",
            d(0.5),
            d(0.99),
            d(1.0)
        );
    }
}

// ===========================================================================

/// The headline: the Go2's 102-tap shape, with the two kHz streams the measured
/// Go2 run lost 71 % of, and a 700 Hz contrast row on IDENTICAL provisioning.
#[test]
#[serial]
#[ignore = "benchmark: run explicitly with --ignored --nocapture"]
fn khz_tap_overflow_at_the_go2_shape() {
    let mode = Mode::parse(&env_str("KHZ_TAP_MODE", Mode::Repro.as_str()));
    let secs = env_or("KHZ_TAP_SECS", 15);
    let body_len = env_or("KHZ_TAP_BODY", 256) as usize;
    let msl = env_or("KHZ_TAP_MSL", BRIDGE_MAX_SLICE_LEN as u64) as u32;
    let khz_n = env_or("KHZ_TAP_KHZ", 2) as usize;
    let discovery = env_str("KHZ_TAP_DISCOVERY", "on") == "on";
    // 102 taps total on the robot: 2 kHz + 1 contrast + 99 background.
    let default_bg = match mode {
        Mode::Depth => 93,
        Mode::Repro => 99,
        // burst adds the heavy rows, so fewer background keeps the tap count.
        Mode::Burst => 99u64.saturating_sub(env_or("KHZ_TAP_HEAVY_TOPICS", HEAVY_TOPICS as u64)),
    };
    let background = env_or("KHZ_TAP_BACKGROUND", default_bg) as usize;
    let (victim_msl, victim_provenance) = victim_slice(msl);

    let plans = plan_topics(mode, body_len, background, khz_n, msl, victim_msl);
    let mgr = make_manager(16);

    println!("\n=== kHz tap-overflow bench ===");
    println!(
        "mode={} taps={} secs={secs} body={body_len}B max_slice_len={} KiB discovery={}",
        mode.as_str(),
        plans.len(),
        msl / 1024,
        if discovery { "on" } else { "off" }
    );
    if mode == Mode::Burst {
        println!(
            "burst victim slice: {victim_msl} B  <- {victim_provenance}\n\
             heavy rows: {} x {HEAVY_BODY} B/frame, clumps of {} at {}/s, slice {msl} B",
            env_or("KHZ_TAP_HEAVY_TOPICS", HEAVY_TOPICS as u64),
            env_or("KHZ_TAP_HEAVY_BURST", HEAVY_BURST),
            env_or("KHZ_TAP_HEAVY_RATE", HEAVY_BURSTS_PER_SEC)
        );
    }

    // Create every producer. Production ctor unless the plan pins a depth.
    let mut publishers = Vec::with_capacity(plans.len());
    for p in &plans {
        let pubr = match p.depth {
            None => mgr
                .create_ingress_publisher(&p.topic, MaxSliceLen::try_new(p.msl).expect("msl"))
                .expect("create_ingress_publisher"),
            Some(d) => publisher_with_provisioning(&mgr, &p.topic, 2, d, p.msl),
        };
        publishers.push(pubr);
    }

    // MEASURE the depths rather than trusting the constant. Per SLICE, because
    // per-route slice sizing makes different routes carry different ones.
    let production_depth = probe_queue_depth(&mgr, None, msl);
    println!(
        "\nMEASURED create_ingress_publisher tap depth = {production_depth} frames \
         at slice {msl} B (absorbs {:.2} ms at {KHZ_HZ} Hz, {:.2} ms at {CONTRAST_HZ} Hz)",
        1000.0 * production_depth as f64 / KHZ_HZ as f64,
        1000.0 * production_depth as f64 / CONTRAST_HZ as f64
    );
    let victim_depth = if victim_msl == msl {
        production_depth
    } else {
        let d = probe_queue_depth(&mgr, None, victim_msl);
        println!(
            "MEASURED victim tap depth = {d} frames at slice {victim_msl} B \
             (absorbs {:.2} ms at {KHZ_HZ} Hz)",
            1000.0 * d as f64 / KHZ_HZ as f64
        );
        d
    };
    let depths: Vec<usize> = plans
        .iter()
        .map(|p| {
            p.depth.unwrap_or(if p.msl == victim_msl {
                victim_depth
            } else {
                production_depth
            })
        })
        .collect();

    // SHM arithmetic at this shape, from iceoryx2's own formula.
    let apparent: u64 = depths
        .iter()
        .zip(plans.iter())
        .map(|(d, p)| apparent_pool_bytes(*d, p.msl))
        .sum();
    println!(
        "apparent SHM reservation: {:.2} GiB over {} routes ({:.0} MiB/route at depth {}) \
         — lazily demand-paged, so this is address space, not RAM",
        gib(apparent),
        plans.len(),
        apparent_pool_bytes(production_depth, msl) as f64 / (1024.0 * 1024.0),
        production_depth
    );

    let out = unique_out("khz");
    let ready = unique_out("khz_ready");
    let mut cfg = cerulion_bagd::BagdConfig::new(
        out.clone(),
        plans.iter().map(|p| TapSpec::attach(&p.topic)).collect(),
    );
    cfg.ready_file = Some(ready.clone());
    cfg.status_period = None;
    cfg.discover_live = discovery;
    cfg.discovery_settle = Duration::ZERO;

    let shutdown = Arc::new(AtomicBool::new(false));
    let shutdown_c = shutdown.clone();
    let mgr_c = mgr.clone();
    let bagd = std::thread::spawn(move || run_bagd(mgr_c, cfg, shutdown_c));
    assert!(
        wait_for_file(&ready, Duration::from_secs(30)),
        "ready-file never appeared"
    );
    settle();

    // Publishers: absolute-deadline pacing so jitter does not accumulate, and a
    // spin barrier so all N start together.
    let go = Arc::new(AtomicBool::new(false));
    let mut threads = Vec::with_capacity(plans.len());
    for (i, mut pubr) in publishers.drain(..).enumerate() {
        let go_c = go.clone();
        let hz = plans[i].hz;
        let body_len = plans[i].body_len;
        let burst = plans[i].burst;
        let period = Duration::from_nanos(1_000_000_000 / hz);
        let ticks = hz * secs;
        threads.push(std::thread::spawn(move || {
            while !go_c.load(Ordering::Relaxed) {
                std::hint::spin_loop();
            }
            let start = Instant::now();
            let body = vec![(i as u8).wrapping_add(1); body_len];
            let mut sent = 0u64;
            for k in 0..ticks {
                // `burst` frames back-to-back per tick. `burst == 1`
                // is the original steady loop, byte for byte.
                for b in 0..burst {
                    let seq = k * burst + b;
                    pubr.publish_raw(&build_frame(0xC1230, seq as u32, seq, &body))
                        .expect("publish");
                    sent += 1;
                }
                let due = start + period.mul_f64((k + 1) as f64);
                let now = Instant::now();
                if due > now {
                    std::thread::sleep(due - now);
                }
            }
            (sent, start.elapsed())
        }));
    }

    let load_before = loadavg();
    let wall_start = Instant::now();
    go.store(true, Ordering::Relaxed);
    let per_thread: Vec<(u64, Duration)> = threads
        .into_iter()
        .map(|t| t.join().expect("publisher thread"))
        .collect();
    let publish_wall = wall_start.elapsed();
    let load_after = loadavg();

    std::thread::sleep(Duration::from_millis(500));
    shutdown.store(true, Ordering::Relaxed);
    let summary = bagd
        .join()
        .expect("join bagd")
        .expect("run_bagd Ok — a firehose must not be an ERROR");

    let published: u64 = per_thread.iter().map(|(n, _)| *n).sum();
    let rows: Vec<Row> = plans
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let (sent, wall) = per_thread[i];
            Row {
                label: p.label.clone(),
                hz_actual: sent as f64 / wall.as_secs_f64(),
                depth: depths[i],
                published: sent,
                recorded: summary.per_topic.get(&p.topic).copied().unwrap_or(0),
                gap_lost: summary
                    .record_health
                    .topics
                    .get(&p.topic)
                    .map(|h| h.frames_lost)
                    .unwrap_or(0),
            }
        })
        .collect();

    // Per-plan, because the burst mode's heavy row has a different body size and
    // a uniform `body_len` would under-report the offered bandwidth ~40x.
    let bytes: u64 = per_thread
        .iter()
        .zip(plans.iter())
        .map(|((sent, _), p)| sent * (p.body_len as u64 + 32))
        .sum();
    println!(
        "\nload {load_before} -> {load_after} | publish wall {:.2} s | offered {:.2} MB/s",
        publish_wall.as_secs_f64(),
        bytes as f64 / publish_wall.as_secs_f64() / (1024.0 * 1024.0)
    );

    println!(
        "\n  {:<22} {:>9} {:>7} {:>10} {:>10} {:>9} {:>9}",
        "topic", "hz", "depth", "offered", "recorded", "gap_lost", "loss %"
    );
    let mut bg_pub = 0u64;
    let mut bg_rec = 0u64;
    for r in &rows {
        if r.label.starts_with("bg") {
            bg_pub += r.published;
            bg_rec += r.recorded;
            continue;
        }
        println!(
            "  {:<22} {:>9.1} {:>7} {:>10} {:>10} {:>9} {:>8.2}%",
            r.label,
            r.hz_actual,
            r.depth,
            r.published,
            r.recorded,
            r.gap_lost,
            r.loss_pct()
        );
    }
    if bg_pub > 0 {
        println!(
            "  {:<22} {:>9} {:>7} {:>10} {:>10} {:>9} {:>8.2}%",
            format!("{background} background"),
            format!("{BACKGROUND_HZ}"),
            production_depth,
            bg_pub,
            bg_rec,
            "-",
            100.0 * bg_pub.saturating_sub(bg_rec) as f64 / bg_pub.max(1) as f64
        );
    }

    // CONSERVATION — the reading that cannot miss anything, unlike the
    // gap-derived one, which is blind to a prefix lost before a tap's baseline.
    let missing = published.saturating_sub(summary.messages);
    println!(
        "\n  offered {published}  recorded {}  MISSING {missing} ({:.2} %)",
        summary.messages,
        100.0 * missing as f64 / published.max(1) as f64
    );
    println!(
        "  gap_lost {}  dropped_unwritten {}  staging_full {}  chunks {}",
        summary.frames_lost,
        summary.dropped_unwritten,
        summary
            .record_health
            .topics
            .values()
            .map(|h| h.staging_full_passes)
            .sum::<u64>(),
        summary.chunks
    );
    println!(
        "  drive_passes {}  span {:.2} s  mean cadence {:.3} ms  max pass (work) {:.3} ms",
        summary.drive_passes,
        summary.drive_span.as_secs_f64(),
        1000.0 * summary.drive_span.as_secs_f64() / summary.drive_passes.max(1) as f64,
        summary.max_pass_duration.as_secs_f64() * 1000.0
    );

    let mut rates: Vec<(&str, f64)> = vec![
        ("kHz class", KHZ_HZ as f64),
        ("contrast", CONTRAST_HZ as f64),
    ];
    if mode == Mode::Depth {
        rates.truncate(1);
    }
    print_gaps(&summary.drain_gaps, &rates);

    if mode == Mode::Depth {
        println!("\n  loss vs depth (ONE run, ONE drive loop — the fit-free tail measurement):");
        for r in rows.iter().filter(|r| !r.label.starts_with("bg")) {
            println!(
                "    depth {:>5}  absorbs {:>7.2} ms  loss {:>7.3} %",
                r.depth,
                1000.0 * r.depth as f64 / r.hz_actual,
                r.loss_pct()
            );
        }
    }

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

/// Drive-loop cadence as a function of TAP COUNT — the term a robot has and a
/// desk usually does not, and the reason the same recorder that keeps up with
/// 25 taps loses 71 % at 102.
///
/// No publishers: the taps are attached to SILENT topics, so the only thing
/// swept is the per-pass cost of HAVING them. That isolates the tap-count term
/// from the drain-work term, which a run with live producers cannot.
#[test]
#[serial]
#[ignore = "benchmark: run explicitly with --ignored --nocapture"]
fn drive_loop_cadence_by_tap_count() {
    let secs = env_or("KHZ_TAP_SECS", 5);
    let msl = env_or("KHZ_TAP_MSL", BRIDGE_MAX_SLICE_LEN as u64) as u32;
    let counts: Vec<usize> = std::env::var("KHZ_TAP_TAPS")
        .ok()
        .map(|v| v.split(',').filter_map(|d| d.trim().parse().ok()).collect())
        .unwrap_or_else(|| vec![1usize, 25, 50, 102, 200]);

    println!(
        "\n=== drive-loop cadence vs tap count (load {}) ===",
        loadavg()
    );
    println!(
        "  {:>6} {:>10} {:>12} {:>12} {:>12} {:>12}",
        "taps", "passes", "mean ms", "p50 ms", "p99 ms", "max ms"
    );

    for n in counts {
        let mgr = make_manager(16);
        let mut held = Vec::with_capacity(n);
        let mut taps = Vec::with_capacity(n);
        for i in 0..n {
            let topic = unique_topic(&format!("cadence{i}"));
            held.push(
                mgr.create_ingress_publisher(&topic, MaxSliceLen::try_new(msl).expect("msl"))
                    .expect("create_ingress_publisher"),
            );
            taps.push(TapSpec::attach(&topic));
        }
        let out = unique_out("cadence");
        let ready = unique_out("cadence_ready");
        let mut cfg = cerulion_bagd::BagdConfig::new(out.clone(), taps);
        cfg.ready_file = Some(ready.clone());
        cfg.status_period = None;
        cfg.discover_live = true;
        cfg.discovery_settle = Duration::ZERO;

        let shutdown = Arc::new(AtomicBool::new(false));
        let shutdown_c = shutdown.clone();
        let mgr_c = mgr.clone();
        let bagd = std::thread::spawn(move || run_bagd(mgr_c, cfg, shutdown_c));
        assert!(
            wait_for_file(&ready, Duration::from_secs(30)),
            "ready-file never appeared at {n} taps"
        );
        std::thread::sleep(Duration::from_secs(secs));
        shutdown.store(true, Ordering::Relaxed);
        let summary = bagd.join().expect("join bagd").expect("run_bagd Ok");

        let h = &summary.drain_gaps;
        let ms = |v: Option<u64>| {
            v.map(|x| format!("{:.3}", x as f64 / 1000.0))
                .unwrap_or_else(|| "OVERFLOW".to_string())
        };
        println!(
            "  {:>6} {:>10} {:>12.3} {:>12} {:>12} {:>12}",
            n,
            summary.drive_passes,
            1000.0 * summary.drive_span.as_secs_f64() / summary.drive_passes.max(1) as f64,
            ms(h.quantile_us(0.5)),
            ms(h.quantile_us(0.99)),
            ms(h.quantile_us(1.0))
        );
        cleanup(&out);
        let _ = std::fs::remove_file(&ready);
        drop(held);
    }
}

// ===========================================================================
// The mode-surface pins.
//
// These are the only NON-`#[ignore]`d tests in the file: they touch no
// transport, spawn no recorder, and run in the ordinary suite. They exist
// because a mode the module doc ADVERTISES and no arm implements
// (such as `taps`) falls through to the repro plan — so the
// bench answers a question nobody asked and prints it under the label of the
// one they did.
// ===========================================================================

/// Every accepted value parses to its OWN variant.
///
/// Anti-tautology for the rejection pins below: without this, a `parse` that
/// rejected EVERYTHING would satisfy them.
#[test]
fn every_accepted_mode_parses_to_its_own_variant() {
    for m in Mode::ALL {
        assert_eq!(
            Mode::parse(m.as_str()),
            m,
            "`{}` must parse back to the variant it names",
            m.as_str()
        );
    }
    // Hand oracle, not a round-trip: the two spellings are part of the
    // documented surface, so a rename must be deliberate.
    assert_eq!(Mode::Repro.as_str(), "repro");
    assert_eq!(Mode::Depth.as_str(), "depth");
}

/// A typo'd mode STOPS the run, naming the accepted set.
///
/// The whole hazard is that a bench's output is read for its numbers: a run
/// under `KHZ_TAP_MODE=dpeth` that silently fell back to the repro shape is
/// indistinguishable, in the printed table, from a depth sweep.
#[test]
#[should_panic(expected = "is not a mode this bench has")]
fn an_unknown_mode_stops_the_run() {
    let _ = Mode::parse("dpeth");
}

/// The RETIRED `taps` value is rejected, and the rejection points at the test
/// that actually does that sweep.
///
/// `taps` was a documented value that no arm implemented,
/// so somebody's shell history has it. Rejecting it silently would be a second
/// misdirection; the message has to say where the sweep went.
#[test]
#[should_panic(expected = "drive_loop_cadence_by_tap_count")]
fn the_retired_taps_value_is_rejected_and_names_its_replacement() {
    let _ = Mode::parse("taps");
}

/// Strip `//`-to-end-of-line and (depth-tracked, because Rust's nest) `/* */`
/// comments — everything EXCEPT the `//!` module doc, which is what the scan
/// below reads.
///
/// Kept deliberately small: the only consumer is the doc scan, and its input is
/// this file, whose shape is known.
fn module_doc_lines(src: &str) -> Vec<&str> {
    src.lines()
        .map(str::trim_start)
        .filter(|l| l.starts_with("//!"))
        .collect()
}

/// Extract every `KHZ_TAP_MODE=<value>` the module doc advertises.
fn advertised_modes(doc: &[&str]) -> Vec<String> {
    const KEY: &str = "KHZ_TAP_MODE=";
    let mut out = Vec::new();
    for line in doc {
        let mut rest = *line;
        while let Some(at) = rest.find(KEY) {
            rest = &rest[at + KEY.len()..];
            let value: String = rest
                .chars()
                .take_while(|c| c.is_ascii_alphanumeric() || *c == '_' || *c == '-')
                .collect();
            if !value.is_empty() {
                out.push(value);
            }
        }
    }
    out.sort();
    out.dedup();
    out
}

/// THE pin for the defect this section exists for: the set of modes the module
/// doc advertises must be EXACTLY the set [`Mode::parse`] accepts.
///
/// A documented value that no arm implements is the bug (`taps`); an accepted
/// value nobody documents is the same bug read the other way.
#[test]
fn the_documented_modes_are_exactly_the_accepted_ones() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/khz_tap_overflow_bench.rs"
    );
    let src = std::fs::read_to_string(path).expect("this test file must be readable");
    let doc = module_doc_lines(&src);
    assert!(
        doc.len() > 50,
        "anti-vacuity: the module doc scan found only {} `//!` lines — if the \
         scan reads nothing, every assertion below is empty",
        doc.len()
    );

    let advertised = advertised_modes(&doc);
    let mut accepted: Vec<String> = Mode::ALL.iter().map(|m| m.as_str().to_string()).collect();
    accepted.sort();
    assert_eq!(
        advertised, accepted,
        "the module doc advertises {advertised:?} but the bench accepts \
         {accepted:?} — a documented mode that falls through to another shape \
         is exactly the defect this pin exists for. Fix the \
         doc, or implement the mode; do not footnote it."
    );

    // The doc must also point at the tap sweep by NAME, so the removed value's
    // reader is sent somewhere real rather than left to grep.
    assert!(
        doc.iter()
            .any(|l| l.contains("drive_loop_cadence_by_tap_count")),
        "the module doc must name the test that owns the tap-count sweep"
    );
    assert!(
        doc.iter().any(|l| l.contains("KHZ_TAP_TAPS")),
        "the module doc must name the env var that sweeps it"
    );
}

/// The comment stripper the doc scan rests on really selects the module doc.
#[test]
fn module_doc_lines_selects_only_the_module_doc() {
    let src = "\
// SPDX
//! doc one
    //! indented doc
/// item doc
fn f() {} // trailing
//!not the module doc? it is
";
    assert_eq!(
        module_doc_lines(src),
        vec![
            "//! doc one",
            "//! indented doc",
            "//!not the module doc? it is"
        ],
        "only `//!` lines, indentation tolerated, item docs and code excluded"
    );
}

/// The extractor really finds every advertised value, and only values.
#[test]
fn advertised_modes_reads_values_not_prose() {
    let doc = vec![
        "//! * `KHZ_TAP_MODE=repro` (default) — the shape",
        "//! * `KHZ_TAP_MODE=depth` — the sweep",
        "//! duplicates collapse: KHZ_TAP_MODE=depth again",
        "//! a bare KHZ_TAP_MODE mention with no value contributes nothing",
        "//! KHZ_TAP_TAPS is a different knob entirely",
    ];
    assert_eq!(advertised_modes(&doc), vec!["depth", "repro"]);
    assert!(advertised_modes(&["//! nothing here"]).is_empty());
}
