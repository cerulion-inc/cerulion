// SPDX-License-Identifier: AGPL-3.0-only
//! Cross-thread round-trip-time (RTT) flat-latency test (the p2 swap).
//!
//! This is the Cerulion-layer end-to-end proof that the **p2 swap**
//! (`ipc::Service` → `ipc_threadsafe::Service`, see
//! `cerulion_core::transport::mod::CerService`) made the transport ports
//! (`CerulionPublisher` / `CerulionSubscriber`) `Send`, AND that the FULL
//! two-peer ROUND TRIP stays flat across a 1KB→16MiB payload sweep.
//!
//! It proves three things at once:
//!
//! 1. **Singleton sharing across threads.** Two worker threads each own and
//!    drive ports built from the SAME process-global `TransportManager`
//!    singleton (`Arc`-cloned per thread). Pre-swap the `ipc::Service` Node
//!    was `!Send`, so the singleton could not be shared this way.
//! 2. **Build-then-MOVE.** Every port is BUILT on the setup/main thread and
//!    then MOVED into a spawned worker thread via `thread::spawn(move || …)`.
//!    This is the swap's UNIQUE enablement: pre-swap the `!Send` ports could
//!    not cross a thread boundary at all, so this move would not compile. If
//!    this file ever fails to compile with a "cannot be sent between threads"
//!    error, the swap is INCOMPLETE — see the module header note in the test.
//! 3. **Round-trip latency is FLAT.** Peer A (timer) publishes a request and
//!    waits for peer B (echo) to publish a reply; the measured window is the
//!    full A→B→A RTT. For a true zero-copy path the RTT is O(1) in payload
//!    size, so the max/min ratio of the per-size FLOORS (min) should be ~1×.
//!    (Floor, not p50: it strips CI-VM jitter while keeping the regression
//!    signal — see `FLATNESS_MAX`.) This
//!    complements the SINGLE-THREAD ONE-WAY `flat_latency_test` (which only
//!    gates the publish→receive half) with a true 2-PEER RTT.
//!
//! # Window-health retry + attributable stall red
//!
//! The gate was already robust to ONE VM-stalled per-size window (drop-one
//! outlier). On 2026-07-05 a macOS CI VM stall spanned FOUR of five sizes at
//! once (this test's floors [16.9, 43.3, 42.8, 43.1, 43.7]µs — uniform ~43µs,
//! size-INDEPENDENT), defeating drop-one. This test now adds a window-health guard
//! WITHOUT weakening the pin (the `FLATNESS_MAX` ceiling and the healthy-path
//! assertion are byte-unchanged; a real O(n) copy still fails):
//!
//! * **(b) Window-health RETRY + attributable red.** After the sweep, any size
//!   whose floor exceeds `FLATNESS_MAX * min_floor` (the would-fail set) is
//!   RE-MEASURED in a fresh window (per-size budget of `MAX_RETRIES` total
//!   attempts, per-attempt accounting printed), keeping the better (min) floor
//!   — a transient stall almost always clears within a fresh window. The
//!   would-fail set is RE-DERIVED each retry round from the CURRENT floors:
//!   a retry that finds a truer, LOWER floor shrinks the ratio
//!   denominator, so sizes newly elevated relative to the new min get their own
//!   re-measure budget instead of flipping a would-pass run red. The gate then
//!   runs NORMALLY on the final floors. If it still fails AND the signature is size-INDEPENDENT
//!   (`is_uniform_stall_signature`: >= 3 elevated floors in a tight band — the
//!   uniform-stall shape, NOT a monotonic-in-size copy), the test panics with a
//!   loud NON-PROBATIVE message naming the signature, the floors, and the retry
//!   history — an ATTRIBUTABLE red instead of a mysterious flatness red. A
//!   monotonic (real-copy) violation does NOT take that arm; it fails the gate
//!   assertion normally. The pure floor-analysis helpers (`sizes_needing_retry`,
//!   `is_uniform_stall_signature`, `run_window_health_retries`) live in
//!   `cerulion_core::testing` and are unit-tested there (oracle vectors).
//!
//! **(c) Per-size interleaving is NOT applied here** (it IS applied to the
//! sibling `flat_latency_test`). Interleaving the sweep would require building
//! all five sizes' four ports (20 ports) up front, moving 10 into EACH worker
//! thread as `Vec`s, and round-robining the echo/timer loops across sizes —
//! materially enlarging the simultaneous SHM footprint (all five sizes' pools,
//! including two 16MiB legs, alive at once) and muddying the build-then-MOVE
//! Send proof (each thread currently moves exactly two named ports). The (b)
//! retry already re-measures a stalled size in a fresh temporal window (much of
//! what interleaving buys — temporal spreading — with none of the structural
//! cost), so (b) alone is the right scope for this test.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test cross_thread_rtt_test \
//!   -- --nocapture --test-threads=1
//! ```
//!
//! Must run with `--test-threads=1` (iceoryx2 singleton + shared memory
//! requires serial access). This test is RATIO-based and therefore debug-OK —
//! the flatness ratio is invariant to build mode (debug just makes every size
//! slower by the same factor), so it gates without a
//! `cfg(not(debug_assertions))` guard. It deliberately does NOT assert an
//! absolute-µs ceiling (that would need release mode + threshold tuning).

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Barrier};
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::transport::TransportManager;
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::wire::WireHeader;
use native_ros2_messages::sensor_msgs::Image;

/// Monotonic counter guaranteeing unique topics even within the same nanosecond.
static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

/// Generate a unique topic name for test isolation. (Mirrors `flat_latency_test`.)
fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/xthreadrtt/{}/{}/{}", base, nanos, id)
}

/// Warmup iterations before measurement (discarded — warms caches, the
/// iceoryx2 SHM pool, and the publisher↔subscriber connection handshake on
/// BOTH legs of the round trip).
const WARMUP_ITERS: usize = 50;
/// Measured iterations per payload size (the floor = min over these). Unchanged
/// by the window-health change (this test keeps the single-pass per-size sweep; (c) interleaving
/// is applied only to the sibling `flat_latency_test` — see the module header).
const MEASURE_ITERS: usize = 200;
/// (b) Max window-health RE-MEASUREMENTS per elevated size. A transient
/// VM stall almost always clears within a fresh window; 2 bounded retries give
/// every would-fail size a second (and third) chance without unbounded looping.
const MAX_RETRIES: usize = 2;

/// TIGHT flatness ceiling for the full cross-thread round-trip path.
///
/// This test measures the **minimum** (uncontended floor) RTT per payload size,
/// NOT the p50 — see the sibling `flat_latency_test::FLATNESS_MAX` for the full
/// rationale. Zero-copy ⇒ the full A→B→A round trip is O(1) in payload size ⇒
/// the floor is payload-size-independent and the max/min ratio across the
/// 16384× size sweep should be ~1×. A memcpy regression on EITHER leg (request
/// publish→receive or reply publish→receive) makes the RTT O(n) and inflates
/// the floor too (it copies on every iteration), blowing the ratio to tens-of-×.
///
/// **Why min, not p50:** on the noisy, oversubscribed GitHub macOS
/// CI runner the *p50* of the 16MiB row balloons under scheduling/memory-pressure
/// jitter even with no copy — the one-way `flat_latency_test` p50 was observed at
/// 3.55× and an earlier p50 ceiling here flaked too. That jitter only ever ADDS
/// latency; it can never lower the floor. min strips the VM noise while keeping
/// the regression signal BINARY (floor ~1× clean vs tens-of-× broken).
///
/// MEASURED on an Apple M3 Max (200-iter min): **~1.0–1.1×** (~3.6µs RTT, flat
/// 1KB→16MiB). Set to **2.0×**: a real TIGHT gate (vs the legacy 5×/25×),
/// slightly looser than the one-way `flat_latency_test`'s 1.5× because the RTT
/// floor crosses two transport legs + two worker threads (a touch more residual
/// floor variance than one-way). This test's UNIQUE value is also the
/// cross-thread `Send`-at-runtime proof (it compiles AND round-trips without
/// deadlock only because of the p2 swap). Any real zero-copy break is tens-of-×
/// — far above 2.0×. Do NOT loosen toward the legacy 5×/25×.
const FLATNESS_MAX: f64 = 2.0;

/// Touch only the first byte of a loaned payload so the loan is not optimized
/// away, WITHOUT iterating the buffer. The O(n) fill is EXCLUDED from every
/// timed window on purpose — charging the RTT with an O(n) memset would defeat
/// the flatness test (we want pure transport overhead, which is O(1) for a
/// true zero-copy path regardless of `data_len`).
#[inline]
fn touch_first_byte(dst: &mut [u8]) {
    if let Some(first) = dst.first_mut() {
        *first = 0xAB;
    }
}

/// Publish one `Image` of `data_len` payload bytes on `publisher`.
///
/// CRITICAL: the payload fill is EXCLUDED — we loan `data_len` bytes but touch
/// ONLY the first byte (via `touch_first_byte`). The fixed-field writes +
/// `set_*` var-field writes are O(1). The proxy publishes on `Drop` (the
/// closing brace finalizes the wire header and sends).
fn publish_image(
    publisher: &mut cerulion_core::transport::publisher::CerulionPublisher,
    data_len: usize,
) {
    let mut proxy = publisher.loan_proxy::<Image>().expect("loan image");
    proxy.height = 1;
    proxy.width = data_len as u32;
    proxy.step = data_len as u32;
    proxy.is_bigendian = 0;
    proxy.set_header_bytes(&[]).expect("set header");
    proxy.set_encoding("raw").expect("set encoding");
    let dst = proxy.loan_data(data_len).expect("loan_data");
    touch_first_byte(dst);
    // proxy drops here → wire header finalized → published.
}

/// Spin-receive exactly one `Image` sample (drain-keep-latest). The callback
/// does NOT read the payload, so the receive cost is O(1). Returns `true` once
/// a sample is observed, `false` if `deadline` passes first (so callers can
/// fail loudly / break their loop instead of hanging the suite).
fn try_recv_one(
    subscriber: &mut cerulion_core::transport::subscriber::CerulionSubscriber,
    deadline: Instant,
) -> bool {
    loop {
        match subscriber.try_view::<Image, _>(|_view| ()) {
            Ok(Some(())) => return true,
            Ok(None) => {
                if Instant::now() > deadline {
                    return false;
                }
                std::hint::spin_loop();
            }
            Err(e) => panic!("try_recv_one error: {e:?}"),
        }
    }
}

/// Measure the FULL cross-thread round-trip latency for one payload size,
/// returning `(floor, p50, p99, max)` ns from the per-iteration RTT samples.
/// The FLOOR (min) gates flatness; p50/p99/max are reported for human insight.
///
/// Builds all four ports on THIS (setup) thread, then MOVES them across the
/// thread boundary — the request publisher + reply subscriber into the timer
/// thread (peer A), the request subscriber + reply publisher into the echo
/// thread (peer B). The move only compiles because the p2 swap made
/// `CerulionPublisher` / `CerulionSubscriber` `Send`.
///
/// The timed window per iteration is the full RTT: peer A publishes `req` and
/// spins until it observes the matching `resp` that peer B echoed back. The
/// payload fill is excluded on BOTH legs and neither receive callback reads
/// the payload, so the window is pure payload-size-independent transport
/// overhead for a zero-copy round trip.
fn measure_rtt_floor_ns(data_len: usize) -> (f64, f64, f64, f64) {
    let mgr: Arc<TransportManager> = TransportManager::get_or_init().expect("init");
    let req_topic = unique_topic("req");
    let resp_topic = unique_topic("resp");

    // Image needs the WireHeader + fixed (13B) + 3 var-field offset entries +
    // variable payload (header_bytes empty, encoding short, data data_len).
    // 4096 bytes of slack covers the small fields (matches the sibling tests).
    let max_slice = (WireHeader::SIZE + 4096 + data_len) as u32;
    let slice_len = MaxSliceLen::const_new(max_slice);

    // --- Build ALL ports on the setup thread (the &self factory methods are
    // callable from any thread; we build here so the MOVE into the spawned
    // threads below is the Send demonstration). ---
    //
    // Peer A (timer): publishes `req`, subscribes `resp`.
    let mut a_req_pub = mgr
        .create_publisher_simple(&req_topic, slice_len)
        .expect("create A req publisher");
    let mut a_resp_sub = mgr
        .create_subscriber(&resp_topic)
        .expect("create A resp subscriber");
    // Peer B (echo): subscribes `req`, publishes `resp`.
    let mut b_req_sub = mgr
        .create_subscriber(&req_topic)
        .expect("create B req subscriber");
    let mut b_resp_pub = mgr
        .create_publisher_simple(&resp_topic, slice_len)
        .expect("create B resp publisher");

    // Two barriers OUTSIDE every timing window: one to gate startup (all ports
    // connected before any traffic), one to release the measured run together.
    let startup_barrier = Arc::new(Barrier::new(2));
    let start_barrier = Arc::new(Barrier::new(2));
    // Stop flag — set by the timer thread after its measured run so the echo
    // thread's loop terminates cleanly (no leaked threads).
    let stop = Arc::new(AtomicBool::new(false));

    // ===================== Peer B: the ECHO thread =====================
    // MOVE `b_req_sub` (subscriber) + `b_resp_pub` (publisher) across the
    // thread boundary. This `move ||` closure capturing the ports is the
    // Send demonstration — it only compiles because of the p2 swap.
    let echo_handle = {
        let startup_barrier = Arc::clone(&startup_barrier);
        let start_barrier = Arc::clone(&start_barrier);
        let stop = Arc::clone(&stop);
        std::thread::spawn(move || {
            // ^^^^^^^ b_req_sub + b_resp_pub MOVED into this worker thread.
            startup_barrier.wait();
            start_barrier.wait();
            // Echo loop: drain any pending `req`, reply on `resp`, until the
            // timer thread signals stop. A short per-attempt deadline keeps the
            // loop responsive to the stop flag (it is the consumer that must
            // notice termination, since the producer side has gone quiet).
            while !stop.load(Ordering::Relaxed) {
                let deadline = Instant::now() + Duration::from_millis(50);
                if try_recv_one(&mut b_req_sub, deadline) {
                    publish_image(&mut b_resp_pub, data_len);
                }
            }
            // Final drain-and-reply sweep: answer any request that landed
            // after the timer's last recv but before `stop` was observed, so a
            // genuinely in-flight request never strands the timer thread.
            let drain_deadline = Instant::now() + Duration::from_millis(50);
            while try_recv_one(&mut b_req_sub, Instant::now()) {
                publish_image(&mut b_resp_pub, data_len);
                if Instant::now() > drain_deadline {
                    break;
                }
            }
        })
    };

    // ===================== Peer A: the TIMER thread =====================
    // MOVE `a_req_pub` (publisher) + `a_resp_sub` (subscriber) across the
    // thread boundary — the second half of the Send demonstration.
    let timer_handle = {
        let startup_barrier = Arc::clone(&startup_barrier);
        let start_barrier = Arc::clone(&start_barrier);
        let stop = Arc::clone(&stop);
        std::thread::spawn(move || -> (f64, f64, f64, f64) {
            // ^^^^^^^ a_req_pub + a_resp_sub MOVED into this worker thread.
            startup_barrier.wait();

            // Warmup — full round trips, untimed. Warms caches, the SHM pool,
            // and the connection handshake on both legs.
            start_barrier.wait();
            for _ in 0..WARMUP_ITERS {
                publish_image(&mut a_req_pub, data_len);
                let deadline = Instant::now() + Duration::from_secs(2);
                if !try_recv_one(&mut a_resp_sub, deadline) {
                    panic!("warmup: timed out waiting for echo reply");
                }
            }

            // Measure: time the FULL round trip = publish `req`
            // (loan→write-fixed→drop→send) + spin-recv the echoed `resp`. The
            // payload fill is excluded on both legs and the receive callback
            // reads nothing, so this window is pure payload-size-independent
            // transport overhead for a zero-copy round trip. The FLOOR (min) —
            // the uncontended RTT — is what gates flatness: it is robust to
            // CI-VM scheduling jitter (noise only adds latency, never lowers the
            // floor) while still catching a real O(n) copy on either leg. See
            // `FLATNESS_MAX`. p50/p99/max are computed alongside for human
            // insight only.
            let mut samples: Vec<f64> = Vec::with_capacity(MEASURE_ITERS);
            for _ in 0..MEASURE_ITERS {
                let start = Instant::now();
                publish_image(&mut a_req_pub, data_len);
                let deadline = start + Duration::from_secs(2);
                if !try_recv_one(&mut a_resp_sub, deadline) {
                    panic!("measure: timed out waiting for echo reply");
                }
                let elapsed = start.elapsed().as_nanos() as f64;
                samples.push(elapsed);
            }

            // Signal the echo thread to stop, then return the stats. The floor
            // stays byte-identical to the gate's prior input: `floor_ns` is the
            // min, which equals sorted[0]. The sort only adds p50/p99/max for
            // reporting.
            stop.store(true, Ordering::Relaxed);
            let floor = cerulion_core::testing::floor_ns(&samples);
            let mut sorted = samples.clone();
            sorted.sort_by(|a, b| a.partial_cmp(b).expect("RTT sample is finite"));
            let p50 = sorted[sorted.len() / 2];
            let p99 = sorted[((sorted.len() as f64 * 0.99) as usize).min(sorted.len() - 1)];
            let max = sorted[sorted.len() - 1];
            (floor, p50, p99, max)
        })
    };

    // Join both worker threads (no leaked threads / panics propagate up).
    let stats = timer_handle.join().expect("timer thread panicked");
    echo_handle.join().expect("echo thread panicked");
    stats
}

/// Print the per-size RTT stats table (FLOOR gates flatness; p50/p99/max are for
/// human insight). Shared by the initial-sweep and post-retry prints.
fn print_rtt_stats_table(label: &str, stats: &[(usize, f64, f64, f64, f64)]) {
    println!(
        "=== cross-thread publish→echo→receive ROUND-TRIP latency by payload size ({label}) (FLOOR gates flatness) ==="
    );
    for &(size, floor, p50, p99, max) in stats {
        println!(
            "  {:>10} bytes : floor RTT = {:>10.1} ns ({:>8.3} µs)  p50 = {:>10.1} ns ({:>8.3} µs)  \
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

/// The full cross-thread publish→echo→receive round trip is O(1) in payload
/// size, AND the ports survive a build-on-one-thread / drive-on-another MOVE.
///
/// Sweeps 1KB → 16MiB (a 16384× span). For a true zero-copy path the FLOOR
/// (min) RTT is flat across the sweep (each leg writes a pointer into SHM and
/// reads a pointer out — neither touches the payload), so the max/min ratio of
/// the per-size floors should be ~1×. A memcpy regression on either leg would
/// make the RTT O(n) and blow the ratio past `FLATNESS_MAX`.
///
/// Ratio-based, no absolute-µs ceiling. Gated again with a
/// **drop-one-outlier robust ratio**. The per-size FLOOR is
/// jitter-*resistant* but not jitter-*immune*: on a shared CI VM a whole
/// per-size measurement window can be preempted, inflating even the min for
/// ONE size (observed on the macOS runner: 256KiB floor=42µs while the 16MiB
/// row stayed flat at 18µs — noise, NOT a copy: a real O(n) copy is monotonic
/// in size and would inflate the LARGEST row, not a middle one). So instead of
/// `max_floor / min_floor` over ALL sizes, we sort the per-size floors and
/// compare the **2nd-highest** floor to the minimum (`sorted[len-2] /
/// sorted[0]`), discarding the single worst outlier. This tolerates ONE
/// VM-stalled size at ANY position while still catching a real O(n) copy —
/// which inflates MULTIPLE sizes monotonically (1000×+), so dropping one row
/// leaves the regression plainly visible. Valid for `len >= 3`; the sweep is 5
/// sizes.
///
/// A MULTI-size uniform stall (four sizes at once, 2026-07-05) defeats
/// drop-one, so any elevated size is now (b) RE-MEASURED before the gate, and an
/// unhealed size-INDEPENDENT stall fails as an ATTRIBUTABLE non-probative red
/// while a real monotonic-in-size copy still fails the gate normally. The
/// ceiling and the healthy-path assertion are unchanged (the pin is not
/// weakened); (c) interleaving is NOT applied here — see the module header.
#[test]
fn test_cross_thread_rtt_flat() {
    // Warm up the transport manager once up front.
    let _ = TransportManager::get_or_init().expect("init");

    // 1KB → 16MiB, a 16384× span. A memcpy regression would show ~16000×,
    // true zero-copy ~1×.
    let sizes: [usize; 5] = [1_024, 16_384, 262_144, 1_048_576, 16_777_216];

    // `stats` carries (size, floor, p50, p99, max) for the print; the
    // flatness-gate input `floors_ns` is derived from it (post-retry) as exactly
    // the (size, floor) pairs the gate always consumed. This test keeps the
    // single-pass per-size sweep (the (c) interleaving is applied only to
    // the sibling `flat_latency_test` — see the module header for why).
    let mut stats: Vec<(usize, f64, f64, f64, f64)> = Vec::with_capacity(sizes.len());
    for &size in &sizes {
        let (floor, p50, p99, max) = measure_rtt_floor_ns(size);
        stats.push((size, floor, p50, p99, max));
    }

    // Print per-size RTT stats (ns + µs) so the human can read the real
    // numbers to tune FLATNESS_MAX (run serially in release first). The FLOOR
    // is the gated metric; p50/p99/max are informational.
    print_rtt_stats_table("initial sweep", &stats);

    // (b) Window-health RETRY. Re-measure any size whose floor exceeds
    // FLATNESS_MAX × min_floor (the would-fail set) in a fresh window, keeping
    // the better floor. A healthy sweep retries nothing (zero extra cost). The
    // orchestration is pure (shared, unit-tested in `cerulion_core::testing`);
    // the transport re-measurement (`measure_rtt_floor_ns`) is injected here.
    let retry_log = cerulion_core::testing::run_window_health_retries(
        &mut stats,
        FLATNESS_MAX,
        MAX_RETRIES,
        measure_rtt_floor_ns,
    );
    if retry_log.is_empty() {
        println!(
            "window-health: all per-size floors within {:.1}x of the min floor — no retries",
            FLATNESS_MAX
        );
    } else {
        println!(
            "window-health: {} re-measurement attempt(s) on elevated size(s):",
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
        print_rtt_stats_table("after window-health retries", &stats);
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
            "NON-PROBATIVE: the cross-thread RTT flatness gate failed \
             with a SIZE-INDEPENDENT uniform-stall signature, NOT a \
             payload-size-dependent copy. >= {} per-size floors are elevated \
             (> {:.1}x the {:.1}ns min floor @ {} bytes) yet fall within a tight \
             {:.2}x band of EACH OTHER — the shape of a shared-CI-VM stall \
             spanning multiple measurement windows (exactly the 2026-07-05 macOS \
             failure), which a real O(n) copy (monotonic in size; adjacent floors \
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
        "cross-thread round-trip path should be FLAT across payload size \
         (zero-copy ⇒ O(1) ⇒ floor ratio ~1×). This is the DROP-ONE-OUTLIER \
         robust ratio: we discard the single worst per-size floor ({} bytes @ \
         {:.1}ns) to tolerate ONE VM-stalled measurement window on a shared CI \
         runner, then compare the 2nd-highest floor to the minimum. A real O(n) \
         copy inflates MULTIPLE sizes monotonically (1000×+), so dropping one \
         row still catches it — a ratio this high means the payload IS being \
         copied on one of the round-trip legs. 2nd-max floor = {:.1}ns @ {} \
         bytes, min floor = {:.1}ns @ {} bytes, ratio = {:.3}x, ceiling = {:.1}x",
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
