// SPDX-License-Identifier: AGPL-3.0-only
//! The GO2-FAITHFUL firehose benchmark, and the drive-loop
//! measurement that goes with it.
//!
//! This is a MEASUREMENT, not a gate (`#[ignore]`d, like its
//! [`firehose_bench`](../firehose_bench.rs) sibling). It exists because that
//! sibling does NOT reproduce the shape the real robot fails at, and the issue
//! says so: 64 topics x 3000 Hz x 64 B uniform, on taps whose service ceilings
//! the bench itself chose, records everything on this desk while the Go2 loses
//! 23 %.
//!
//! # What the sibling bench is missing (three things, all structural)
//!
//! 1. **Discovery is OFF.** `BagdConfig::new` defaults `discover_live` to
//!    `false` and `firehose_bench` never sets it, so the sibling NEVER calls
//!    `TransportManager::list_topics()`. The robot ran `graph run --record`,
//!    which hands bagd `--topics-json` and therefore leaves discovery ON, so
//!    the robot's drive loop pays a full iceoryx2 service-directory enumeration
//!    every `DISCOVERY_RESCAN_INTERVAL` (250 ms). That enumeration runs INLINE
//!    on the drive loop, between two `drain_taps` calls.
//! 2. **Uniform rates.** The robot's loss is concentrated on five topics
//!    spanning 240-700 Hz while ~70 others (a few Hz each) lose nothing. A
//!    uniform sweep cannot show that, and the per-topic SPREAD is the signal:
//!    it is what identifies the gap distribution (below).
//! 3. **The depth is not the discovered depth.** `firehose_bench` sets
//!    `FH_QUEUE` (default 16) explicitly. That happens to equal the discovered
//!    depth, but the bench never derives it from the production constructor, so
//!    nothing pins the two together. Here every shallow topic is created by
//!    `TransportManager::create_ingress_publisher` — the EXACT call the
//!    `ros2 attach` bridge makes per DDS route — so the depth under test is
//!    whatever production actually provisions.
//!
//! # The arithmetic this bench is built to test
//!
//! A discovered topic's service is created by `create_ingress_publisher` with
//! `default_topic_config()`, whose `subscriber_max_buffer_size` is
//! `DEFAULT_SUBSCRIBER_BUFFER_SIZE` = **16** (`transport/mod.rs:93`, `:4104`).
//! bagd taps it with `create_data_only_subscriber` (`lib.rs:4860`), which passes
//! `buffer_size: None` and therefore inherits the service ceiling. So a
//! discovered topic's tap queue is **16 frames deep**, against a DECLARED
//! topic's `recording_tap_buffer_depth()` = up to 4096.
//!
//! `drain_taps` drains each tap TO EMPTY every pass (`lib.rs:5911` — the
//! `loop { drain_owned(1) .. if n == 0 break }`), and the drive loop skips its
//! sleep whenever a pass made progress. So steady-state loss cannot come from a
//! slow drain; it can only come from a GAP between two consecutive drains of a
//! topic that is longer than `D / R`. Writing `g` for that gap, per-topic loss
//! is `E[max(0, R*g - D)] / (R * E[g])`.
//!
//! Two gap models make DIFFERENT, falsifiable predictions:
//!
//! * **Uniform cadence `C`** (every pass costs the same): loss
//!   `= 1 - (D/R)/C` — linear in `D/R` with **intercept 1**.
//! * **Rare stalls**: gaps are normally negligible, but `lambda` times per
//!   second a pass takes `T`: loss `= lambda*T - lambda*(D/R)` — linear in
//!   `D/R` with intercept `lambda*T` **below 1**.
//!
//! Fitting the ISSUE's five per-topic rows (D = 16 throughout) to the second
//! model gives `lambda = 2.571 / s`, `T = 139.6 ms`, and reproduces all five
//! loss percentages to within 0.05 percentage points — where the uniform model
//! cannot fit them at all (it implies a per-topic cadence ranging 33-82 ms,
//! which one loop cannot have). `1/lambda = 389 ms` = `DISCOVERY_RESCAN_INTERVAL`
//! (250 ms) + T, which is exactly the period a 250 ms timer produces when the
//! work it gates takes T and the timer is re-armed AFTER that work
//! (`lib.rs:7492-7495`).
//!
//! That is an INFERENCE from a two-parameter fit. This bench tests it three
//! independent ways, none of which needs the recorder instrumented:
//!
//! * `GO2_MODE=repro` + `GO2_DISCOVERY=on|off` — an A/B whose ONLY difference
//!   is `cfg.discover_live`. Identical tap sets, identical publishers,
//!   identical everything else. If the rescan is the stall, OFF loses ~nothing.
//! * `GO2_MODE=depth` — the same 700 Hz stream tapped at depths 16..4096 IN ONE
//!   RUN. Loss vs depth traces the gap distribution's tail directly: it must
//!   reach zero at `D ~= R*T`, which MEASURES `T` with no fit at all.
//! * `list_topics_cost_by_service_count` — times the suspected blocker itself
//!   through its public API, against a swept service count.
//!
//! # MEASURED (macOS M3 Max, 16 cores, release, NVMe)
//!
//! **Read the LOAD column before any wall figure.** This desk carried 6
//! concurrent build/test agents throughout; 1-minute loadavg ranged 10-60 on 16
//! cores. Every wall-clock number below is therefore an upper bound on a quiet
//! box and is reported only as corroboration. The CONCLUSIONS rest on counts
//! and per-topic ratios taken WITHIN a run, or on A/B pairs run back-to-back at
//! matched load — quantities load can delay but not invert.
//!
//! ## The A/B, interleaved, 3 reps (75 topics, Go2 rates, identical tap set)
//!
//! | rep | loadavg (before) | discovery | published | recorded | /lowcmd loss |
//! |---|---|---|---|---|---|
//! | 1 | 18.83 | ON  | 42,780 | 42,060 | 4.4 % |
//! | 1 | 27.96 | OFF | 42,780 | 42,701 | 0.5 % |
//! | 2 | 60.16 | ON  | 42,780 | 42,093 | 4.2 % |
//! | 2 | 55.74 | OFF | 42,780 | 42,707 | 0.3 % |
//! | 3 | 41.97 | ON  | 42,780 | 42,042 | 4.3 % |
//! | 3 | 29.74 | OFF | 42,780 | **42,780** | **0.0 %** |
//!
//! The ON arm is 4.2-4.4 % across a 3x load swing; the OFF arm is 0-0.5 %. The
//! effect does not track load — which is the argument that it is not load.
//! (Other ON runs, load unrecorded, read 6.7 % and 8.3 %, so the full spread
//! on this desk is 4.2-8.3 %.)
//!
//! ## The bystander pair — the alternative-explanation killer
//!
//! 225 IDLE topics added (no frames, no taps, no drain work; enumeration cost
//! only), back to back at matched load (19.08 -> 15.68):
//!
//! | discovery | /lowcmd | /lowstate | /utlidar/imu | 70 background | MISSING |
//! |---|---|---|---|---|---|
//! | ON  | **21.1 %** | 18.4 % | 8.8 % | 0.0 % | 6,005 |
//! | OFF | **0.0 %** | 0.0 % | 0.0 % | 0.0 % | **0** |
//!
//! Idle services cost 0 % with discovery off and 21 % with it on, so the ONLY
//! path by which they hurt is the enumeration. `dropped_unwritten = 0` and
//! `staging_full_passes = 0` throughout — the robot's readings exactly.
//!
//! ## Loss vs queue depth, ONE run, one drive loop (700 Hz, 300 services)
//!
//! | depth | 16 | 24 | 32 | 40 | 48 | 56 | 64 | 96 |
//! |---|---|---|---|---|---|---|---|---|
//! | absorbs | 22.9 ms | 34.3 | 45.7 | 57.1 | 68.6 | 80.0 | 91.4 | 137 |
//! | loss | 11.3 % | 7.5 % | 3.7 % | 0.7 % | 0.2 % | 0.06 % | **0** | **0** |
//!
//! This is a FIT-FREE measurement of the drain-gap distribution's tail: loss
//! reaches zero between 80.0 ms and 91.4 ms of absorption, so the MAXIMUM gap
//! between two consecutive drains of a topic was 80-91 ms in that run. The
//! `list_topics()` median measured on that same namespace was 47.7 ms.
//!
//! ## `list_topics()` cost vs live service count
//!
//! TWO independent samples, because this is the file's most load-sensitive
//! number and one sample would misrepresent it. Sample A was taken at loadavg
//! ~40; sample B at loadavg ~101, after `#[serial]` was added.
//!
//! | topics | 1 | 25 | 75 | 150 | 200 | 300 |
//! |---|---|---|---|---|---|---|
//! | services | 2 | 50 | 150 | 300 | 400 | 600 |
//! | A min ms | 2.7 | 7.8 | 19.0 | 41.7 | 69.8 | 93.1 |
//! | A median ms | 3.3 | 8.4 | 20.7 | 74.4 | 115.1 | 111.1 |
//! | B min ms | 0.3 | 5.8 | 18.3 | 35.4 | 46.5 | 69.9 |
//! | B median ms | 0.3 | 6.0 | 19.9 | 36.8 | 48.9 | 74.6 |
//!
//! Both samples are LINEAR in service count and agree closely at the 150-service
//! robot scale (20.7 vs 19.9 ms median); they diverge up to ~1.5x at 600
//! services, so read the slope (~0.12-0.19 ms per service) rather than any
//! single cell. Under load, and with the two tests in this file running
//! CONCURRENTLY, the same call measured up to 551 ms at 600 services — which is
//! why they are now `#[serial]`, and why the tail is the part that matters: it
//! is exactly what a busy robot has.
//!
//! ```text
//! cargo test -p cerulion_bagd --test go2_firehose_bench -- --ignored --nocapture
//! ```
//!
//! Knobs: `GO2_MODE` (repro|depth), `GO2_DISCOVERY` (on|off), `GO2_SECS`,
//! `GO2_BODY`, `GO2_BACKGROUND` (background topic count), `GO2_DEEP_CONTROL`.

mod common;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_bagd::{run_bagd, TapSpec};
use cerulion_core::transport::recording_tap_buffer_depth;
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::TransportManager;
use serial_test::serial;

use common::*;

/// The five topics the issue measured, at the rates it measured them at.
/// Derived from the issue's own table: `(recorded + lost) / 74 s`.
const GO2_ROWS: &[(&str, u64)] = &[
    ("lowcmd", 698),
    ("lowstate", 499),
    ("utlidar_imu", 248),
    ("utlidar_robot_odom", 242),
    ("utlidar_mapping_cmd", 242),
];

/// The robot's aggregate offered rate was ~2115 frames/s over 75 topics; the
/// five above account for 1929, leaving ~186 f/s spread over ~70 background
/// topics. Modelled as a flat per-topic rate.
const BACKGROUND_HZ: u64 = 3;

/// The machine's 1/5/15-minute load averages, as a printable string.
///
/// Load discipline: every wall-clock number in this file is only
/// interpretable beside the contention it was taken under. A latency measured
/// on an oversubscribed machine measures the machine's scheduler, not the recorder.
/// Counts and per-topic RATIOS are load-immune and are what the conclusions
/// rest on; the wall figures are corroboration, and they carry their load with
/// them so a reader can discount them.
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

fn env_or(key: &str, default: u64) -> u64 {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

fn env_str(key: &str, default: &str) -> String {
    std::env::var(key).unwrap_or_else(|_| default.to_string())
}

/// One topic in the shape: what it publishes at, and how deep its tap queue is.
struct Plan {
    topic: String,
    label: String,
    hz: u64,
    /// The tap's receive-queue depth == the service's
    /// `subscriber_max_buffer_size`, because the tap opens with
    /// `buffer_size: None`.
    depth: usize,
    /// `true` when the service was created by the PRODUCTION
    /// `create_ingress_publisher` (the bridge's per-route constructor) rather
    /// than by hand — i.e. the depth is whatever production chose, not a number
    /// this bench picked.
    production_ctor: bool,
}

/// What one topic did, measured.
struct Row {
    label: String,
    hz_target: u64,
    hz_actual: f64,
    depth: usize,
    published: u64,
    recorded: u64,
    lost: u64,
}

fn plan_topics(mode: &str, body_len: usize, background: usize, deep_control: bool) -> Vec<Plan> {
    let msl = body_len + 32;
    let deep = recording_tap_buffer_depth(msl);
    let mut plans = Vec::new();

    match mode {
        "depth" => {
            // The SAME 700 Hz stream, tapped at a sweep of depths, in ONE run —
            // so every row shares one drive loop and therefore one gap
            // distribution. Loss vs depth is then a direct read of that
            // distribution's tail.
            let depths: Vec<usize> = std::env::var("GO2_DEPTHS")
                .ok()
                .map(|v| v.split(',').filter_map(|d| d.trim().parse().ok()).collect())
                .unwrap_or_else(|| vec![16usize, 32, 64, 128, 256, 512, 1024, 4096]);
            for d in depths {
                plans.push(Plan {
                    topic: unique_topic(&format!("d{d}")),
                    label: format!("700Hz/depth{d}"),
                    hz: 700,
                    depth: d,
                    production_ctor: false,
                });
            }
        }
        _ => {
            for (name, hz) in GO2_ROWS {
                plans.push(Plan {
                    topic: unique_topic(name),
                    label: (*name).to_string(),
                    hz: *hz,
                    // The DISCOVERED depth, taken from the production
                    // constructor rather than asserted here.
                    depth: 0,
                    production_ctor: true,
                });
            }
            if deep_control {
                // The in-body DEPTH CONTROL: the same 700 Hz stream on the
                // depth a DECLARED `graph run --record` topic gets. Same loop,
                // same pass cadence, same publisher code — only the queue
                // differs.
                plans.push(Plan {
                    topic: unique_topic("deep_control"),
                    label: format!("lowcmd-like/depth{deep}"),
                    hz: 698,
                    depth: deep,
                    production_ctor: false,
                });
            }
        }
    }

    for i in 0..background {
        plans.push(Plan {
            topic: unique_topic(&format!("bg{i}")),
            label: format!("background{i}"),
            hz: BACKGROUND_HZ,
            depth: 0,
            production_ctor: true,
        });
    }
    plans
}

/// MEASURE a provisioning class's effective tap-queue depth, rather than assert
/// it: create a throwaway topic exactly the way the class does, tap it with the
/// recorder's own `create_data_only_subscriber`, publish far more than any
/// plausible depth WITHOUT draining, then drain and count what survived.
///
/// iceoryx2 reclaims drop-oldest at SEND time, so what comes back IS the
/// queue's capacity — the `D` the arithmetic uses. No accessor needed, and the
/// number is a property of the shipped constructor rather than of this file.
fn probe_queue_depth(mgr: &TransportManager, body_len: usize, depth: Option<usize>) -> usize {
    let msl = MaxSliceLen::const_new((body_len + 32) as u32);
    let topic = unique_topic("depthprobe");
    let mut pubr = match depth {
        None => mgr
            .create_ingress_publisher(&topic, msl)
            .expect("create_ingress_publisher"),
        Some(d) => publisher_with_provisioning(mgr, &topic, 2, d, (body_len + 32) as u32),
    };
    let mut sub = mgr
        .create_data_only_subscriber(&topic)
        .expect("data-only tap");
    settle();
    let body = vec![7u8; body_len];
    // The burst must EXCEED the queue, or what comes back is the burst rather
    // than the capacity. For the `Some(d)` branch there is no guesswork — the
    // caller states the depth, so the burst is a function of it. The `None`
    // branch is the one that must ASSUME something about a constructor it
    // deliberately does not read, and that assumption is
    // `ASSUMED_INGRESS_DEPTH`.
    //
    // Today `create_ingress_publisher` provisions 16
    // (`DEFAULT_SUBSCRIBER_BUFFER_SIZE`), so the burst is 384 — 24x the depth,
    // and it stays correct until the real ceiling exceeds **384**, not 16. That
    // matters, because its own recommendation 2 is to raise this very
    // number: at the proposed `RECORDING_TAP_BUFFER_DEPTH_FLOOR` of 64 the
    // burst is still 6x the depth and the probe measures correctly.
    //
    // The headroom is nonetheless an ASSUMPTION, and this bench exists to
    // verify a change expected to move it — so the failure mode is what needs
    // fixing, not the constant. A too-small burst must not silently report a
    // plausible-looking number at the exact moment the thing under measurement
    // changed; the assertion below turns that into an attributable failure, and
    // it is correct at ANY future ceiling without a magic number to keep in
    // sync.
    const ASSUMED_INGRESS_DEPTH: usize = 16;
    let burst = depth.unwrap_or(ASSUMED_INGRESS_DEPTH) * 8 + 256;
    for k in 0..burst {
        pubr.publish_raw(&build_frame(0xF14E, k as u32, k as u64, &body))
            .expect("publish");
    }
    let mut scratch = Vec::with_capacity(1);
    let mut drained = 0usize;
    loop {
        let n = sub.drain_owned(1, &mut scratch).expect("drain");
        scratch.clear();
        if n == 0 {
            break;
        }
        drained += n;
    }
    // The measurement must have been bounded by the QUEUE, not by how much this
    // probe happened to publish. If the drain returned everything sent, the
    // queue was never filled and `drained` is the BURST SIZE — a number that
    // looks like a depth and is not one.
    assert!(
        drained < burst,
        "probe_queue_depth({}) drained ALL {burst} published frames, so the queue never \
         overflowed and {drained} is the BURST SIZE, not the queue capacity. The probe's burst \
         is too small for this topic's provisioning — raise it (the `None` branch assumes \
         ASSUMED_INGRESS_DEPTH = {ASSUMED_INGRESS_DEPTH}, giving {burst} frames of headroom, so \
         this fires once create_ingress_publisher's ceiling exceeds {burst}).",
        match depth {
            None => "None/create_ingress_publisher".to_string(),
            Some(d) => format!("Some({d})"),
        }
    );
    drained
}

/// Ordinary least squares of `y` on `x`, returning `(slope, intercept, r2)`.
fn ols(pts: &[(f64, f64)]) -> (f64, f64, f64) {
    let n = pts.len() as f64;
    let mx = pts.iter().map(|p| p.0).sum::<f64>() / n;
    let my = pts.iter().map(|p| p.1).sum::<f64>() / n;
    let sxy: f64 = pts.iter().map(|p| (p.0 - mx) * (p.1 - my)).sum();
    let sxx: f64 = pts.iter().map(|p| (p.0 - mx) * (p.0 - mx)).sum();
    let slope = if sxx == 0.0 { 0.0 } else { sxy / sxx };
    let intercept = my - slope * mx;
    let ss_tot: f64 = pts.iter().map(|p| (p.1 - my) * (p.1 - my)).sum();
    let ss_res: f64 = pts
        .iter()
        .map(|p| {
            let e = p.1 - (intercept + slope * p.0);
            e * e
        })
        .sum();
    let r2 = if ss_tot == 0.0 {
        1.0
    } else {
        1.0 - ss_res / ss_tot
    };
    (slope, intercept, r2)
}

/// `#[serial]` is a MEASUREMENT requirement, not a correctness one. Both tests
/// here are individually parallel-safe (each builds its own isolated per-test
/// SHM root), but they are TIMING probes of the same machine — run
/// concurrently, each one's publisher threads and service directory inflate the
/// other's numbers, and libtest interleaves their output line by line. Running
/// the file's tests together without this produced a `list_topics()` median of
/// 55 ms at 150 services against 20.7 ms measured alone.
#[test]
#[serial]
#[ignore = "benchmark: run explicitly with --ignored --nocapture"]
fn go2_firehose_shape() {
    let mode = env_str("GO2_MODE", "repro");
    let discovery = env_str("GO2_DISCOVERY", "on") == "on";
    let secs = env_or("GO2_SECS", 20);
    let body_len = env_or("GO2_BODY", 900) as usize;
    let deep_control = env_or("GO2_DEEP_CONTROL", 0) == 1;
    let background = env_or(
        "GO2_BACKGROUND",
        if mode == "depth" {
            67
        } else if deep_control {
            69
        } else {
            70
        },
    ) as usize;

    let msl = MaxSliceLen::const_new((body_len + 32) as u32);
    let mgr = make_manager(16);
    let plans = plan_topics(&mode, body_len, background, deep_control);

    // IDLE BYSTANDER SERVICES: topics that exist and publish NOTHING, are
    // tapped by nobody, and are never drained. They add exactly one thing to
    // the run — enumeration cost, since `list_topics` walks every service in
    // the namespace — and nothing else: no frames, no drain work, no writer
    // bytes. Sweeping them is therefore a clean isolation of the rescan.
    //
    // This is not a contrivance. The robot's recorder runs on the DEFAULT
    // `iox2_` data-plane namespace shared with every other tenant on the
    // machine (the co-tenancy residual recorded for live discovery), so its
    // enumeration cost is set by everything live on the machine, not by the graph.
    let extra_services = env_or("GO2_EXTRA_SERVICES", 0) as usize;
    let mut bystanders = Vec::with_capacity(extra_services);
    for i in 0..extra_services {
        let t = unique_topic(&format!("idle{i}"));
        bystanders.push(
            mgr.create_ingress_publisher(&t, msl)
                .expect("create_ingress_publisher"),
        );
    }

    // Create every producer the way production creates it. A `production_ctor`
    // topic goes through `create_ingress_publisher` — the bridge's own
    // per-DDS-route call — so its queue depth is whatever `default_topic_config`
    // provisions and this file asserts nothing about it.
    let mut publishers = Vec::with_capacity(plans.len());
    let mut depths = Vec::with_capacity(plans.len());
    for p in &plans {
        if p.production_ctor {
            publishers.push(
                mgr.create_ingress_publisher(&p.topic, msl)
                    .expect("create_ingress_publisher"),
            );
        } else {
            publishers.push(publisher_with_provisioning(
                &mgr,
                &p.topic,
                2,
                p.depth,
                (body_len + 32) as u32,
            ));
        }
        depths.push(0usize);
    }

    // MEASURE the effective tap-queue depth per provisioning class on throwaway
    // topics (never on the ones under test — a probe would prime their queues
    // and burn a subscriber slot). The production constructor's depth is
    // whatever it is; this file does not choose it.
    let ingress_depth = probe_queue_depth(&mgr, body_len, None);
    println!("MEASURED create_ingress_publisher tap depth = {ingress_depth} frames");
    let mut hand_depths: std::collections::BTreeMap<usize, usize> = Default::default();
    for p in &plans {
        if !p.production_ctor {
            hand_depths
                .entry(p.depth)
                .or_insert_with(|| probe_queue_depth(&mgr, body_len, Some(p.depth)));
        }
    }
    for (asked, got) in &hand_depths {
        println!("MEASURED hand-provisioned depth {asked} -> {got} frames");
    }
    for (i, p) in plans.iter().enumerate() {
        depths[i] = if p.production_ctor {
            ingress_depth
        } else {
            hand_depths[&p.depth]
        };
    }

    // The enumeration cost THIS run's drive loop will actually pay, measured on
    // the very namespace it will enumerate, right before it starts.
    for _ in 0..3 {
        let _ = mgr.list_topics().expect("list_topics");
    }
    let mut lt = Vec::new();
    for _ in 0..15 {
        let t0 = Instant::now();
        let _ = mgr.list_topics().expect("list_topics");
        lt.push(t0.elapsed());
    }
    lt.sort();
    let list_cost_ms = lt[lt.len() / 2].as_secs_f64() * 1000.0;
    let list_lo_ms = lt[0].as_secs_f64() * 1000.0;
    let list_hi_ms = lt[lt.len() - 1].as_secs_f64() * 1000.0;

    let out = unique_out("go2fh");
    let ready = unique_out("go2fh_ready");
    let mut cfg = cerulion_bagd::BagdConfig::new(
        out.clone(),
        plans.iter().map(|p| TapSpec::attach(&p.topic)).collect(),
    );
    cfg.ready_file = Some(ready.clone());
    cfg.status_period = None;
    // THE ONE VARIABLE between the two arms. Every tap is declared in both, so
    // the rescan finds nothing to attach either way — what differs is only
    // whether `list_topics()` runs at all.
    cfg.discover_live = discovery;
    // Take the bag-creation hold out of the comparison entirely: it
    // delays bag creation by the settle floor, which is a startup transient and
    // not the steady-state property under test.
    cfg.discovery_settle = Duration::ZERO;

    let shutdown = Arc::new(AtomicBool::new(false));
    let shutdown_c = shutdown.clone();
    let mgr_c = mgr.clone();
    let bagd = std::thread::spawn(move || run_bagd(mgr_c, cfg, shutdown_c));
    assert!(
        wait_for_file(&ready, Duration::from_secs(20)),
        "ready-file never appeared"
    );
    settle();

    let go = Arc::new(AtomicBool::new(false));
    let mut threads = Vec::with_capacity(plans.len());
    for (i, mut pubr) in publishers.drain(..).enumerate() {
        let go_c = go.clone();
        let hz = plans[i].hz;
        let period = Duration::from_nanos(1_000_000_000 / hz);
        let frames = hz * secs;
        threads.push(std::thread::spawn(move || {
            while !go_c.load(Ordering::Relaxed) {
                std::hint::spin_loop();
            }
            let start = Instant::now();
            let body = vec![(i as u8).wrapping_add(1); body_len];
            let mut sent = 0u64;
            for k in 0..frames {
                pubr.publish_raw(&build_frame(0xF14E, k as u32, k, &body))
                    .expect("publish");
                sent += 1;
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

    // Let the recorder drain the tail, then finalize.
    std::thread::sleep(Duration::from_millis(500));
    shutdown.store(true, Ordering::Relaxed);
    let summary = bagd
        .join()
        .expect("join bagd")
        .expect("run_bagd Ok — a firehose must not be an ERROR");

    let published: u64 = per_thread.iter().map(|(n, _)| *n).sum();
    let mut rows: Vec<Row> = Vec::new();
    for (i, p) in plans.iter().enumerate() {
        let (sent, wall) = per_thread[i];
        let health = summary.record_health.topics.get(&p.topic);
        rows.push(Row {
            label: p.label.clone(),
            hz_target: p.hz,
            hz_actual: sent as f64 / wall.as_secs_f64(),
            depth: depths[i],
            published: sent,
            recorded: summary.per_topic.get(&p.topic).copied().unwrap_or(0),
            lost: health.map(|h| h.frames_lost).unwrap_or(0),
        });
    }

    let bytes = published * (body_len as u64 + 32);
    let rate_mb_s = bytes as f64 / publish_wall.as_secs_f64() / (1024.0 * 1024.0);
    let missing = published.saturating_sub(summary.messages);

    println!("\n=== Go2-faithful firehose ===");
    println!(
        "mode={mode} discovery={} topics={} bystanders={extra_services} body={body_len}B window={secs}s",
        if discovery { "ON" } else { "OFF" },
        plans.len()
    );
    println!(
        "live services     : {} (each topic = /data + /event)",
        (plans.len() + extra_services) * 2
    );
    println!(
        "list_topics()     : {list_cost_ms:.2} ms median ({list_lo_ms:.2}..{list_hi_ms:.2}), \
         measured on THIS namespace before the run"
    );
    println!("loadavg 1/5/15    : [{load_before}] before -> [{load_after}] after  (16 cores)");
    println!("publish wall      : {publish_wall:?}");
    println!("published frames  : {published}");
    println!("offered rate      : {rate_mb_s:.2} MB/s");
    println!("recorded          : {}", summary.messages);
    println!("frames_lost (gap) : {}", summary.frames_lost);
    println!("MISSING (pub-rec) : {missing}");
    println!("dropped_unwritten : {}", summary.dropped_unwritten);
    println!("chunks            : {}", summary.chunks);
    println!(
        "staging_full      : {}",
        summary
            .record_health
            .topics
            .values()
            .map(|h| h.staging_full_passes)
            .sum::<u64>()
    );
    // The drive loop's own cadence, read off the recorder instead of
    // inferred from the loss curve. `max pass` is the number this whole bench
    // was built to establish indirectly — a pass longer than `depth / rate` is
    // what overflows a tap's queue, so on the discovery-ON arm it used to sit at
    // the `list_topics()` cost and should now sit near the OFF arm's.
    println!("drive passes      : {}", summary.drive_passes);
    println!(
        "max pass (work)   : {:.3} ms",
        summary.max_pass_duration.as_secs_f64() * 1000.0
    );
    println!(
        "\n{:<24} {:>7} {:>9} {:>6} {:>9} {:>9} {:>8} {:>7}",
        "topic", "tgt Hz", "act Hz", "depth", "published", "recorded", "lost", "loss %"
    );
    // Print the interesting rows in full and fold the quiet background into one
    // line — 70 zero rows is noise, and the claim that matters about them is
    // that they are ALL zero.
    let mut bg_pub = 0u64;
    let mut bg_rec = 0u64;
    let mut bg_lost = 0u64;
    let mut bg_n = 0usize;
    for r in &rows {
        if r.label.starts_with("background") {
            bg_pub += r.published;
            bg_rec += r.recorded;
            bg_lost += r.lost;
            bg_n += 1;
            continue;
        }
        let miss = r.published.saturating_sub(r.recorded);
        println!(
            "{:<24} {:>7} {:>9.1} {:>6} {:>9} {:>9} {:>8} {:>6.1}%",
            r.label,
            r.hz_target,
            r.hz_actual,
            r.depth,
            r.published,
            r.recorded,
            r.lost,
            100.0 * miss as f64 / r.published.max(1) as f64
        );
    }
    if bg_n > 0 {
        println!(
            "{:<24} {:>7} {:>9} {:>6} {:>9} {:>9} {:>8} {:>6.1}%",
            format!("[{bg_n} background topics]"),
            BACKGROUND_HZ,
            "-",
            "-",
            bg_pub,
            bg_rec,
            bg_lost,
            100.0 * bg_pub.saturating_sub(bg_rec) as f64 / bg_pub.max(1) as f64
        );
    }

    // The gap-distribution fit. Only rows that actually LOST anything carry
    // information about the tail; a row at zero loss only says `R*g < D` always,
    // which is a bound, not a point.
    let pts: Vec<(f64, f64)> = rows
        .iter()
        .filter(|r| !r.label.starts_with("background"))
        .filter(|r| r.published > r.recorded)
        .map(|r| {
            let miss = r.published - r.recorded;
            (
                r.depth as f64 / r.hz_actual,
                miss as f64 / r.published as f64,
            )
        })
        .collect();
    if pts.len() >= 2 {
        let (slope, intercept, r2) = ols(&pts);
        let lambda = -slope;
        let t = if lambda != 0.0 {
            intercept / lambda
        } else {
            0.0
        };
        println!("\n--- gap-distribution fit: loss = lambda*(T - D/R) ---");
        println!("points            : {}", pts.len());
        println!(
            "lambda (stalls/s) : {lambda:.3}   -> 1/lambda = {:.1} ms",
            1000.0 / lambda
        );
        println!("T (stall length)  : {:.1} ms", t * 1000.0);
        println!("r^2               : {r2:.4}");
        println!(
            "implied rescan    : 1/lambda - T = {:.1} ms (compare DISCOVERY_RESCAN_INTERVAL = 250 ms)",
            1000.0 / lambda - t * 1000.0
        );
    }
    println!("=====================================\n");

    drop(bystanders);
    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

/// Time the suspected blocker itself, through its public API, against a swept
/// service count — no recorder, no publishers, no instrumentation.
///
/// `rescan_discovery` (`lib.rs:5502`) calls `TransportManager::list_topics()`
/// INLINE on the drive loop every `DISCOVERY_RESCAN_INTERVAL`. `list_topics`
/// (`transport/mod.rs:3397`) is an `iceoryx2::service::Service::list` over the
/// node's whole config namespace — one directory walk plus a static-details
/// read per service. Every Cerulion topic contributes TWO services (`/data` and
/// `/event`), so a 75-topic robot enumerates ~150.
#[test]
#[serial] // see `go2_firehose_shape` — these are timing probes and must not overlap.
#[ignore = "benchmark: run explicitly with --ignored --nocapture"]
fn list_topics_cost_by_service_count() {
    let body = 900usize;
    let msl = MaxSliceLen::const_new((body + 32) as u32);
    let mgr = make_manager(16);
    let mut held = Vec::new();
    let mut created = 0usize;

    println!("\n=== list_topics() cost vs live service count ===");
    println!(
        "{:>8} {:>10} {:>12} {:>12} {:>12}",
        "topics", "services", "min ms", "median ms", "max ms"
    );
    for target in [1usize, 10, 25, 50, 75, 100, 150, 200, 300] {
        while created < target {
            let t = unique_topic(&format!("lt{created}"));
            held.push(
                mgr.create_ingress_publisher(&t, msl)
                    .expect("create_ingress_publisher"),
            );
            created += 1;
        }
        // Warm the page cache / directory entries first, then measure.
        for _ in 0..3 {
            let _ = mgr.list_topics().expect("list_topics");
        }
        let mut samples = Vec::new();
        for _ in 0..25 {
            let t0 = Instant::now();
            let listed = mgr.list_topics().expect("list_topics");
            samples.push(t0.elapsed());
            assert!(
                listed.len() >= created,
                "enumeration must see every live topic: saw {} of {created}",
                listed.len()
            );
        }
        samples.sort();
        println!(
            "{:>8} {:>10} {:>12.3} {:>12.3} {:>12.3}",
            created,
            created * 2,
            samples[0].as_secs_f64() * 1000.0,
            samples[samples.len() / 2].as_secs_f64() * 1000.0,
            samples[samples.len() - 1].as_secs_f64() * 1000.0,
        );
    }
    println!("========================================================\n");
}
