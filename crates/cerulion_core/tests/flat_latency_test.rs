// SPDX-License-Identifier: AGPL-3.0-only
//! Full-path flat-latency moat test.
//!
//! Cerulion's zero-copy headline property is that the FULL publish→receive
//! transport path is O(1) in payload size: the publisher loans a slot in
//! shared memory and writes fixed fields directly; the wire header is
//! finalized in place; the subscriber receives a pointer into the same
//! shared memory. None of that touches the variable payload, so end-to-end
//! one-way latency should be FLAT across a 1KB→16MiB sweep.
//!
//! # Why this test exists (gap it closes)
//!
//! * `latency_threshold_test.rs` measures end-to-end but puts the O(n)
//!   payload fill INSIDE the timed window (so it's O(n) by design) and only
//!   asserts catastrophe-only absolute thresholds.
//!
//! There WAS a third, `latency_scaling_test.rs`, gating the SUBSCRIBER-receive
//! path alone (`try_view`, the identical expression `recv_one` below calls) on a
//! LOOSE < 5× ratio of MEANS over 1 KB → 1 MB. It was deleted as strictly
//! contained by this file: the receive it timed is inside this file's window,
//! over a wider sweep (1 KB → 16 MiB), against a 3.3× tighter ceiling, on a
//! floor rather than a mean. The one class it could have caught and this gate's
//! floor cannot — an INTERMITTENT payload-proportional receive cost — it did not
//! actually catch either: a 1 MiB memcpy landing in 1 of its 100 iterations
//! moved its mean by ~1.3×, well under its own 5× ceiling. The allocating
//! variant of that class is pinned by `zero_alloc_test`; if a non-allocating one
//! ever wants a gate, the cheap form is a loose ceiling on the p50 this file
//! already computes, not a second flatness ratio.
//!
//! NEITHER of the two remaining gates tightly pins that the FULL
//! publish→receive path is O(1). An
//! accidental extra copy in the publish or wire-finalize path (a zero-copy
//! regression) would slip through both. This test closes the gap: it times
//! the full one-way path with the payload fill EXCLUDED and asserts a TIGHT
//! flatness ratio across a wide size range.
//!
//! # Scope: single-threaded one-way, not a 2-peer round-trip
//!
//! Before the service swap the iceoryx2 `ipc::Service` Node is `!Send`, so the
//! `TransportManager` singleton cannot be shared across two threads — a true
//! 2-peer RTT test lives in `cross_thread_rtt_test.rs`. The flatness property is the
//! same either way: the one-way path is what carries the payload-size cost,
//! so timing it one-way still detects an O(n) copy regression.
//!
//! # Window-health retry + interleaving + attributable stall red
//!
//! The gate was already robust to ONE VM-stalled per-size window (drop-one
//! outlier). On 2026-07-05 a macOS CI VM stall spanned FOUR of five sizes at
//! once (uniform ~43µs, size-INDEPENDENT), defeating drop-one. Two levers harden
//! this WITHOUT weakening the pin (the `FLATNESS_MAX` ceiling and the healthy
//! path assertion are byte-unchanged; a real O(n) copy still fails):
//!
//! * **(c) Per-size INTERLEAVING.** The sweep is measured in `MEASURE_ROUNDS`
//!   interleaved rounds (`ITERS_PER_ROUND` iters/size/round; total iters/size
//!   unchanged at `MEASURE_ITERS`), accumulating the per-size min across
//!   rounds. Because every round visits ALL sizes, a single contiguous VM
//!   stall can no longer own multiple sizes' floors — each size's floor is the
//!   min over windows spread across the whole sweep timeline, so a stall must
//!   span nearly the entire sweep to inflate even one floor.
//! * **(b) Window-health RETRY + attributable red.** After the sweep, any size
//!   whose floor exceeds `FLATNESS_MAX * min_floor` (the would-fail set) is
//!   RE-MEASURED in a fresh window (per-size budget of `MAX_RETRIES` total
//!   attempts, per-attempt accounting printed), keeping the better (min) floor.
//!   The would-fail set is RE-DERIVED each retry round from the CURRENT floors:
//!   a retry that finds a truer, LOWER floor shrinks the ratio
//!   denominator, so sizes newly elevated relative to the new min get their own
//!   re-measure budget instead of flipping a would-pass run red. Then the gate
//!   runs NORMALLY on the final floors. If it still fails AND the signature is
//!   size-INDEPENDENT (`is_uniform_stall_signature`: >= 3 elevated floors in a
//!   tight band — the uniform-stall shape, NOT a monotonic-in-size copy), the
//!   test panics with a loud NON-PROBATIVE message naming the signature, the
//!   floors, and the retry history — an ATTRIBUTABLE red instead of a mysterious
//!   flatness red. A monotonic (real-copy) violation does NOT take that arm; it
//!   fails the gate assertion normally. The pure floor-analysis helpers
//!   (`sizes_needing_retry`, `is_uniform_stall_signature`) live in
//!   `cerulion_core::testing` and are unit-tested there (oracle vectors).
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test flat_latency_test \
//!   -- --nocapture --test-threads=1
//! ```
//!
//! Must run with `--test-threads=1` (iceoryx2 singleton + shared memory
//! requires serial access). This test is RATIO-based and therefore
//! debug-OK — the flatness ratio is invariant to build mode (debug just
//! makes every size slower by the same factor), so it gates on every PR
//! WITHOUT a `cfg(not(debug_assertions))` guard. It deliberately does NOT
//! assert an absolute-µs ceiling (that would need release mode).

use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::subscriber::CerulionSubscriber;
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::wire::WireHeader;
use native_ros2_messages::sensor_msgs::Image;

/// Monotonic counter guaranteeing unique topics even within the same nanosecond.
static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

/// Generate a unique topic name for test isolation.
fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/flatfull/{}/{}/{}", base, nanos, id)
}

/// Warmup iterations before measurement (discarded — warms caches, the
/// iceoryx2 SHM pool, and the late-joiner/connection handshake).
const WARMUP_ITERS: usize = 50;
/// Measured iterations per payload size (the floor = min over these). Total is
/// UNCHANGED from the earlier single-pass count; the (c)
/// interleaving just splits it into `MEASURE_ROUNDS` rounds of `ITERS_PER_ROUND`
/// (`MEASURE_ROUNDS * ITERS_PER_ROUND == MEASURE_ITERS`).
const MEASURE_ITERS: usize = 200;
/// (c) Number of interleaved measurement rounds. Every round measures
/// EVERY size (for `ITERS_PER_ROUND` iters), so one contiguous VM stall cannot
/// own multiple sizes' floors — a size's floor is the min over 4 windows spread
/// across the whole sweep timeline.
const MEASURE_ROUNDS: usize = 4;
/// (c) Timed iters per size PER round. `MEASURE_ROUNDS *
/// ITERS_PER_ROUND` must equal `MEASURE_ITERS` (total iters/size unchanged).
const ITERS_PER_ROUND: usize = MEASURE_ITERS / MEASURE_ROUNDS;
/// (b) Max window-health RE-MEASUREMENTS per elevated size. A transient
/// VM stall almost always clears within a fresh window; 2 bounded retries give
/// every would-fail size a second (and third) chance without unbounded looping.
const MAX_RETRIES: usize = 2;

/// TIGHT flatness ceiling for the full publish→receive path.
///
/// This test measures the **minimum** (uncontended floor) latency per payload
/// size, NOT the p50. For a true zero-copy path the per-iteration work is O(1)
/// in payload size (loan a pre-allocated SHM slot, write fixed fields + the
/// wire header, hand a pointer to the receiver — none of it touches the
/// variable payload), so the FLOOR is payload-size-independent and the max/min
/// ratio across the 16384× size sweep should be ~1×.
///
/// **Why min, not p50 (commit after `879560d`):** on a noisy,
/// oversubscribed CI VM (the GitHub macOS runner) the *p50* of the largest
/// (16MiB) row balloons under scheduling/memory-pressure jitter — observed
/// 3.55× with p50 — even though no copy happens. That jitter only ever ADDS
/// latency; it can never lower the floor. The minimum strips the VM noise out
/// while PRESERVING the regression signal: a real memcpy regression makes
/// EVERY iteration O(n), so it inflates the floor too (a single 16MiB copy is
/// ~40×+ the ~µs floor). So min keeps the gate's signal BINARY (~1× clean vs
/// tens-of-× broken) AND robust to runner noise — strictly better than p50 for
/// a structural "is the path O(1)?" test.
///
/// MEASURED on an Apple M3 Max (200-iter min): **~1.0–1.1× release/debug**.
/// `1.5×` is deliberately TIGHT (vs the legacy 25× catastrophe threshold
/// `latency_threshold_test` uses, and the 5× of the deleted
/// `latency_scaling_test`) and is now robust on
/// the macOS CI runner because the metric is the floor. Any real zero-copy
/// break is tens-of-× — far above the ceiling. Do NOT loosen toward the
/// legacy 5×; if this ever flakes again, the floor genuinely differs by size
/// (a real finding — investigate, don't paper over).
const FLATNESS_MAX: f64 = 1.5;

/// Wait (without consuming the sample) until iceoryx2 has at least one
/// sample queued: there is no public peek API, and the publisher's send is
/// synchronous through
/// iceoryx2's lock-free queue, so a brief spin approximates "the sample has
/// landed". This sits OUTSIDE the timed window in the warmup helper but is
/// folded INTO the timed window for the measured receive (the receive is
/// part of the full path we are measuring).
fn brief_settle_spin() {
    for _ in 0..1000 {
        std::hint::spin_loop();
    }
}

/// Publish ONE `Image` of `data_len` payload bytes on `publisher`.
///
/// CRITICAL: the payload fill is EXCLUDED — we loan `data_len` bytes but touch
/// ONLY the first byte (so the loan is not optimized away), never iterating the
/// whole buffer. Charging the timed window with an O(n) fill would defeat the
/// test. The proxy publishes on `Drop` (the closing brace finalizes the wire
/// header and sends).
fn publish_one(publisher: &mut CerulionPublisher, data_len: usize) {
    let mut proxy = publisher.loan_proxy::<Image>().expect("loan");
    proxy.height = 1;
    proxy.width = data_len as u32;
    proxy.step = data_len as u32;
    proxy.is_bigendian = 0;
    proxy.set_header_bytes(&[]).expect("hdr");
    proxy.set_encoding("raw").expect("enc");
    let dst = proxy.loan_data(data_len).expect("loan_data");
    // Touch the first byte only — NOT the whole payload. This keeps the O(n)
    // fill out of the publish path we are timing.
    if let Some(first) = dst.first_mut() {
        *first = 0xAB;
    }
    // proxy drops here → wire header finalized → published.
}

/// Receive exactly one sample (drain-keep-latest). The callback does NOT read
/// the payload, so the receive cost is O(1). Bounded spin so a genuinely lost
/// sample fails loudly instead of hanging the suite.
fn recv_one(subscriber: &mut CerulionSubscriber) {
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        match subscriber.try_view::<Image, _>(|_view| ()) {
            Ok(Some(())) => return,
            Ok(None) => {
                if Instant::now() > deadline {
                    panic!("recv_one: timed out waiting for sample");
                }
                std::hint::spin_loop();
            }
            Err(e) => panic!("recv_one error: {e:?}"),
        }
    }
}

/// Build a fresh publisher+subscriber pair for `data_len` on a unique topic.
fn build_ports(mgr: &TransportManager, data_len: usize) -> (CerulionPublisher, CerulionSubscriber) {
    let topic = unique_topic("fullpath");
    // Image needs the WireHeader + fixed (13B) + 3 var-field offset entries +
    // variable payload (header_bytes empty, encoding short, data data_len).
    // 4096 bytes of slack covers the small fields (matches the sibling tests).
    let max_slice = (WireHeader::SIZE + 4096 + data_len) as u32;
    let publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(max_slice))
        .expect("create publisher");
    let subscriber = mgr.create_subscriber(&topic).expect("create subscriber");
    (publisher, subscriber)
}

/// Untimed warmup — publish + settle + receive, `WARMUP_ITERS` times. Warms
/// caches, the SHM pool, and the late-joiner/connection handshake before any
/// measurement.
fn warmup_ports(
    publisher: &mut CerulionPublisher,
    subscriber: &mut CerulionSubscriber,
    data_len: usize,
) {
    for _ in 0..WARMUP_ITERS {
        publish_one(publisher, data_len);
        brief_settle_spin();
        recv_one(subscriber);
    }
}

/// `(floor, p50, p99, max)` ns from per-iteration samples. The FLOOR (min) — the
/// uncontended transport cost — is the metric the flatness gate is built on (a
/// structural O(1)-in-payload-size proof; see `FLATNESS_MAX` for why min, not
/// p50). The p50/p99/max are reported for human insight only.
fn stats_from_samples(samples: &[f64]) -> (f64, f64, f64, f64) {
    // Floor stays byte-identical to the gate's prior input: `floor_ns` is the
    // min, which equals sorted[0]. The sort only adds p50/p99/max for reporting.
    let floor = cerulion_core::testing::floor_ns(samples);
    let mut sorted = samples.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).expect("latency sample is finite"));
    let p50 = sorted[sorted.len() / 2];
    let p99 = sorted[((sorted.len() as f64 * 0.99) as usize).min(sorted.len() - 1)];
    let max = sorted[sorted.len() - 1];
    (floor, p50, p99, max)
}

/// Time the FULL one-way publish→receive path once: publish block
/// (loan→write-fixed→drop→send) + receive (spin-drain one sample). The O(n)
/// payload fill is EXCLUDED (see `publish_one`) and the receive callback reads
/// nothing, so the window is pure payload-size-independent transport overhead
/// for a zero-copy path.
#[inline]
fn timed_full_path_iter(
    publisher: &mut CerulionPublisher,
    subscriber: &mut CerulionSubscriber,
    data_len: usize,
) -> f64 {
    let start = Instant::now();
    publish_one(publisher, data_len);
    recv_one(subscriber);
    start.elapsed().as_nanos() as f64
}

/// (b) Retry primitive: measure ONE size in a FRESH window (fresh ports,
/// full warmup, `MEASURE_ITERS` timed iters), returning `(floor, p50, p99, max)`.
/// Used to re-measure a size whose initial-sweep floor was elevated by a
/// transient VM stall — a fresh temporal window almost always clears it.
fn measure_one_size(data_len: usize) -> (f64, f64, f64, f64) {
    let mgr = TransportManager::get_or_init().expect("init");
    let (mut publisher, mut subscriber) = build_ports(&mgr, data_len);
    warmup_ports(&mut publisher, &mut subscriber, data_len);
    let mut samples: Vec<f64> = Vec::with_capacity(MEASURE_ITERS);
    for _ in 0..MEASURE_ITERS {
        samples.push(timed_full_path_iter(
            &mut publisher,
            &mut subscriber,
            data_len,
        ));
    }
    stats_from_samples(&samples)
}

/// Per-size measurement context held ALIVE simultaneously during the (c)
/// interleaved sweep (all sizes' ports coexist so every round can visit every
/// size). Keeping all five pools alive at once is cheap: iceoryx2 `Static` pools
/// are lazy/demand-paged and we touch only the first payload byte, so resident
/// memory stays at the working set (see `shm_footprint_probe`); the apparent
/// reservation grows only ~1MiB over the earlier single-largest-pool peak.
struct SizeMeasure {
    size: usize,
    publisher: CerulionPublisher,
    subscriber: CerulionSubscriber,
    samples: Vec<f64>,
}

/// (c) Measure ALL `sizes` in `MEASURE_ROUNDS` INTERLEAVED rounds,
/// returning `(size, floor, p50, p99, max)` per size. Every round runs
/// `ITERS_PER_ROUND` timed iters for EVERY size, accumulating into that size's
/// sample vector, so a single contiguous VM stall cannot own multiple sizes'
/// floors — each size's floor is the min over `MEASURE_ROUNDS` windows spread
/// across the whole sweep timeline. Total timed iters per size == `MEASURE_ITERS`
/// (unchanged from the earlier single-pass sweep). Each size owns its own
/// topic + ports, so there is no cross-size traffic; publish→recv stays strictly
/// 1:1 per iter (no queue accumulation between a size's rounds).
fn measure_all_sizes_interleaved(sizes: &[usize]) -> Vec<(usize, f64, f64, f64, f64)> {
    let mgr = TransportManager::get_or_init().expect("init");
    // Build + warm every size's ports up front (all held alive simultaneously).
    let mut ctxs: Vec<SizeMeasure> = sizes
        .iter()
        .map(|&size| {
            let (mut publisher, mut subscriber) = build_ports(&mgr, size);
            warmup_ports(&mut publisher, &mut subscriber, size);
            SizeMeasure {
                size,
                publisher,
                subscriber,
                samples: Vec::with_capacity(MEASURE_ITERS),
            }
        })
        .collect();

    // Interleave: each round measures ITERS_PER_ROUND iters for every size. A
    // contiguous stall inside one round hits at most the size(s) it overlaps,
    // and each size's floor is the min across all rounds — so a stall must span
    // nearly the entire sweep to inflate even one size's floor.
    for _round in 0..MEASURE_ROUNDS {
        for ctx in ctxs.iter_mut() {
            for _ in 0..ITERS_PER_ROUND {
                let sample =
                    timed_full_path_iter(&mut ctx.publisher, &mut ctx.subscriber, ctx.size);
                ctx.samples.push(sample);
            }
        }
    }

    ctxs.iter()
        .map(|ctx| {
            let (floor, p50, p99, max) = stats_from_samples(&ctx.samples);
            (ctx.size, floor, p50, p99, max)
        })
        .collect()
}

/// Print the per-size stats table (FLOOR gates flatness; p50/p99/max are for
/// human insight). Shared by the initial-sweep and post-retry prints.
fn print_stats_table(label: &str, stats: &[(usize, f64, f64, f64, f64)]) {
    println!(
        "=== full publish→receive path latency by payload size ({label}) (FLOOR gates flatness) ==="
    );
    for &(size, floor, p50, p99, max) in stats {
        println!(
            "  {:>10} bytes : floor = {:>10.1} ns ({:>8.3} µs)  p50 = {:>10.1} ns ({:>8.3} µs)  \
             p99 = {:>10.1} ns ({:>8.3} µs)  max = {:>10.1} ns ({:>8.3} µs)",
            size,
            floor,
            floor / 1000.0,
            p50,
            p50 / 1000.0,
            p99,
            p99 / 1000.0,
            max,
            max / 1000.0
        );
    }
}

/// The full publish→receive transport path is O(1) in payload size.
///
/// Sweeps 1KB → 16MiB (a 16384× span). For a true zero-copy path the FLOOR
/// (min) latency is flat across the sweep (the publisher writes a pointer into
/// SHM; the subscriber reads a pointer out — neither touches the payload), so
/// the max/min ratio of the per-size floors should be ~1×. A memcpy regression
/// in the publish or wire-finalize path would make the path O(n): the 16MiB
/// row's floor would be thousands of times slower than the 1KB row's and the
/// ratio would blow past `FLATNESS_MAX`.
///
/// Debug-OK: ratio-based, so it gates on every PR. No absolute-µs ceiling.
/// Metric is the FLOOR (min), not p50, for robustness to CI-VM noise — see
/// `FLATNESS_MAX`.
///
/// The sweep is (c) INTERLEAVED (so one contiguous stall can't own
/// multiple floors) and (b) any elevated size is RE-MEASURED before the gate;
/// an unhealed size-INDEPENDENT stall fails as an ATTRIBUTABLE non-probative
/// red, while a real monotonic-in-size copy still fails the gate normally. The
/// ceiling and the healthy-path assertion are unchanged (the pin is not
/// weakened). See the module header.
#[test]
fn test_full_path_flat_latency() {
    // Warm up the transport manager once up front.
    let _ = TransportManager::get_or_init().expect("init");

    // 1KB → 16MiB, a 16384× span. A memcpy regression would show ~16000×,
    // true zero-copy ~1×.
    let sizes: [usize; 5] = [1_024, 16_384, 262_144, 1_048_576, 16_777_216];

    // (c) INTERLEAVED sweep. `stats` carries (size, floor, p50, p99,
    // max) per size; the flatness-gate input is derived from it as the
    // (size, floor) pairs the gate always consumed.
    let mut stats = measure_all_sizes_interleaved(&sizes);

    // Print per-size stats so the human can read the real numbers to tune
    // FLATNESS_MAX (run serially in release first). The FLOOR is the gated
    // metric; p50/p99/max are informational.
    print_stats_table("initial interleaved sweep", &stats);

    // (b) Window-health RETRY. Re-measure any size whose floor exceeds
    // FLATNESS_MAX × min_floor (the would-fail set) in a fresh window, keeping
    // the better floor. A healthy sweep retries nothing (zero extra cost). The
    // orchestration is pure (shared, unit-tested in `cerulion_core::testing`);
    // the transport re-measurement (`measure_one_size`) is injected here.
    let retry_log = cerulion_core::testing::run_window_health_retries(
        &mut stats,
        FLATNESS_MAX,
        MAX_RETRIES,
        measure_one_size,
    );
    if retry_log.is_empty() {
        println!(
            "window-health: all per-size floors within {:.1}x of the min floor — no retries",
            FLATNESS_MAX
        );
    } else {
        println!(
            "window-health: {} re-measurement attempt(s) on elevated size(s) ((b) retry):",
            retry_log.len()
        );
        for r in &retry_log {
            println!(
                "  retry {} bytes round {} attempt {}/{}: old floor = {:.1} ns ({:.3} µs) -> \
                 measured = {:.1} ns ({:.3} µs), kept = {:.1} ns ({:.3} µs)",
                r.size,
                r.round,
                r.attempt,
                MAX_RETRIES,
                r.old_floor,
                r.old_floor / 1000.0,
                r.measured_floor,
                r.measured_floor / 1000.0,
                r.kept_floor,
                r.kept_floor / 1000.0
            );
        }
        print_stats_table("after window-health retries", &stats);
    }

    // Gate NORMALLY on the FINAL (post-retry) floors.
    let floors_ns: Vec<(usize, f64)> = stats
        .iter()
        .map(|&(size, floor, ..)| (size, floor))
        .collect();

    // Drop-one-outlier robust ratio: sort the per-size floors and
    // compare the 2nd-HIGHEST floor to the minimum, discarding the single worst
    // outlier. This tolerates ONE VM-stalled per-size window at ANY position
    // (a shared-runner stall inflates exactly one row's floor) while still
    // catching a real O(n) copy — which inflates MULTIPLE sizes monotonically,
    // so dropping the single max still leaves a 1000×+ ratio. The stats live
    // in the shared testing module (`drop_one_outlier_robust_ratio`), which
    // panics if given < 3 sizes (we sweep 5, so it never trips here).
    let rf = cerulion_core::testing::drop_one_outlier_robust_ratio(&floors_ns);
    let (min_size, min_floor) = rf.min;
    let (dropped_size, dropped_floor) = rf.dropped;
    let (robust_max_size, robust_max) = rf.robust_max;
    let ratio = rf.ratio;

    println!(
        "dropped outlier (worst floor): {} bytes @ {:.1} ns ({:.3} µs)",
        dropped_size,
        dropped_floor,
        dropped_floor / 1000.0
    );
    println!(
        "robust_max (2nd-highest) = {:.1} ns @ {} bytes, min_floor = {:.1} ns @ {} bytes, \
         drop-one-outlier ratio = {:.3}x (ceiling {:.1}x)",
        robust_max, robust_max_size, min_floor, min_size, ratio, FLATNESS_MAX
    );

    // (b) If the gate still fails AND the failure signature is
    // size-INDEPENDENT (the uniform-stall shape), fail as an ATTRIBUTABLE
    // non-probative red — NOT a mysterious flatness red — naming the signature,
    // the final floors, and the retry history. A monotonic-in-size copy does
    // NOT satisfy the signature (its elevated floors span the >= 4× payload-size
    // steps), so a real regression falls through to the normal gate assertion.
    if ratio >= FLATNESS_MAX
        && cerulion_core::testing::is_uniform_stall_signature(&floors_ns, FLATNESS_MAX)
    {
        let floors_str = stats
            .iter()
            .map(|&(s, f, ..)| format!("{s}B={:.1}ns", f))
            .collect::<Vec<_>>()
            .join(", ");
        let retry_str = if retry_log.is_empty() {
            "none".to_string()
        } else {
            retry_log
                .iter()
                .map(|r| {
                    format!(
                        "{}B a{}: {:.1}->{:.1} kept {:.1}ns",
                        r.size, r.attempt, r.old_floor, r.measured_floor, r.kept_floor
                    )
                })
                .collect::<Vec<_>>()
                .join("; ")
        };
        panic!(
            "NON-PROBATIVE: the one-way flatness gate failed with a \
             SIZE-INDEPENDENT uniform-stall signature, NOT a payload-size-dependent \
             copy. >= {} per-size floors are elevated (> {:.1}x the {:.1}ns min \
             floor @ {} bytes) yet fall within a tight {:.2}x band of EACH OTHER — \
             the shape of a shared-CI-VM stall spanning multiple measurement \
             windows, which a real O(n) copy (monotonic in size; adjacent floors \
             differ by the >= 4x payload-size step) can never produce. This is an \
             ATTRIBUTABLE infrastructure red, not a zero-copy regression — re-run \
             on a quiescent host. Final per-size floors: [{}]. Window-health retry \
             history ({} attempt(s)): [{}]. drop-one ratio = {:.3}x, ceiling = \
             {:.1}x.",
            cerulion_core::testing::UNIFORM_STALL_MIN_SIZES,
            FLATNESS_MAX,
            min_floor,
            min_size,
            cerulion_core::testing::UNIFORM_STALL_BAND,
            floors_str,
            retry_log.len(),
            retry_str,
            ratio,
            FLATNESS_MAX
        );
    }

    assert!(
        ratio < FLATNESS_MAX,
        "full publish→receive path should be FLAT across payload size \
         (zero-copy ⇒ O(1) ⇒ floor ratio ~1×). This is the DROP-ONE-OUTLIER \
         robust ratio: we discard the single worst per-size floor ({} bytes @ \
         {:.1}ns) to tolerate ONE VM-stalled measurement window on a shared CI \
         runner, then compare the 2nd-highest floor to the minimum. A real O(n) \
         copy inflates MULTIPLE sizes monotonically (1000×+), so dropping one \
         row still catches it — a ratio this high means the payload IS being \
         copied on the publish/wire-finalize/receive path. 2nd-max floor = \
         {:.1}ns @ {} bytes, min floor = {:.1}ns @ {} bytes, ratio = {:.3}x, \
         ceiling = {:.1}x",
        dropped_size,
        dropped_floor,
        robust_max,
        robust_max_size,
        min_floor,
        min_size,
        ratio,
        FLATNESS_MAX
    );
}
