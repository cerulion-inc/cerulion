//! SHM footprint findings — Linux-only (`#[ignore]`) regression pins.
//!
//! These began as throwaway measurement probes; they are now permanent
//! `#[ignore]` regression tests that ASSERT (not just print) the two load-bearing
//! footprint conclusions:
//!   1. iceoryx2 `Static` SHM pools are LAZY / demand-paged — resident
//!      (`/dev/shm` actual-block) usage tracks the working set, NOT the tier-max
//!      per-slot reservation. So the tier-max reservation is virtual/sparse (not
//!      real RAM), `PowerOfTwo` shrinks only address space, and the SHM-footprint
//!      motivation is illusory on Linux tmpfs. (`probe_static_pool_footprint`,
//!      `probe_multi_pool_fit`.)
//!   2. Oversizing the pool is LATENCY-FREE — a big-tier (128 MiB) pool serves the
//!      same 1 KiB payload with the same floor/p50/p99 as a small (16 KiB) pool,
//!      so we can oversize the tier freely. (`probe_latency_big_vs_small_tier`.)
//!
//! Linux-only: they read Linux `/dev/shm` apparent-vs-actual. On macOS `/dev/shm`
//! doesn't exist (`du` returns 0) so the residency asserts self-skip and the
//! signal degrades to the process RSS delta (print-only). Run on a Linux
//! host via:
//!   cargo test -p cerulion_core --test shm_footprint_probe -- --ignored --nocapture

use cerulion_core::transport::TransportManager;
use cerulion_core::wire::MaxSliceLen;
use native_ros2_messages::sensor_msgs::Image;
use serial_test::serial;
use std::time::{Duration, Instant};

/// `du -s` of `/dev/shm` in bytes. `apparent=true` → ftruncate/logical size;
/// `apparent=false` → actual disk blocks (sparse-aware = resident pages for tmpfs).
/// Returns 0 where `/dev/shm` / GNU `du` is absent (e.g. macOS).
fn du_dev_shm(apparent: bool) -> u64 {
    let mut cmd = std::process::Command::new("du");
    cmd.arg("-s").arg("--block-size=1");
    if apparent {
        cmd.arg("--apparent-size");
    }
    cmd.arg("/dev/shm");
    let out = match cmd.output() {
        Ok(o) => o,
        Err(_) => return 0,
    };
    String::from_utf8_lossy(&out.stdout)
        .split_whitespace()
        .next()
        .and_then(|n| n.parse().ok())
        .unwrap_or(0)
}

/// Does GNU `du --apparent-size --block-size=1 /dev/shm` actually WORK here?
/// False on macOS (no `/dev/shm`) AND on a Linux host with a BusyBox `du` that
/// rejects those flags — in both cases `du_dev_shm` returns a MISLEADING 0, so
/// the residency asserts must SKIP rather than misattribute a 0 delta to a
/// failed reservation. Keys on the child's EXIT STATUS, not its (swallowed-to-0)
/// output, so it distinguishes "tool unusable" (skip) from "usable, measured 0"
/// (a real reservation regression → let the assert FAIL).
fn du_apparent_usable() -> bool {
    std::process::Command::new("du")
        .args(["-s", "--block-size=1", "--apparent-size", "/dev/shm"])
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}

/// Process resident set size in bytes, cross-platform via `ps -o rss=` (KiB on
/// both Linux and macOS). This is the macOS lazy-vs-eager signal.
fn rss_bytes() -> u64 {
    let pid = std::process::id().to_string();
    let out = match std::process::Command::new("ps")
        .args(["-o", "rss=", "-p", &pid])
        .output()
    {
        Ok(o) => o,
        Err(_) => return 0,
    };
    let kb: u64 = String::from_utf8_lossy(&out.stdout)
        .split_whitespace()
        .next()
        .and_then(|n| n.parse().ok())
        .unwrap_or(0);
    kb * 1024
}

#[test]
#[ignore]
#[serial]
fn probe_static_pool_footprint() {
    const MAX: u32 = 16 * 1024 * 1024; // 16 MiB probe pool (an explicit size; TIER_HUGE is now 128 MiB, and any big pool proves laziness)
    const PAYLOAD: usize = 1024; // 1 KiB actual payload
    const N: usize = 40;

    let mgr = TransportManager::get_or_init().expect("init");
    let topic = format!("shm_probe/{}", std::process::id());

    let act_before = du_dev_shm(false);
    let app_before = du_dev_shm(true);
    let rss_before = rss_bytes();

    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(MAX))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    for i in 0..N {
        {
            let mut proxy = publisher.loan_proxy::<Image>().expect("loan");
            proxy.height = i as u32;
            proxy.width = 1;
            proxy.is_bigendian = 0;
            proxy.step = 0;
            proxy.set_header_bytes(&[]).expect("header");
            proxy.set_encoding("rgb8").expect("encoding");
            proxy.set_data(&vec![0x80u8; PAYLOAD]).expect("data");
        }
        std::thread::sleep(Duration::from_millis(5));
        let _ = subscriber.try_view::<Image, _>(|v| v.height);
    }

    // Measure WHILE the ports (and their SHM) are still alive.
    let act_after = du_dev_shm(false);
    let app_after = du_dev_shm(true);
    let rss_after = rss_bytes();

    let act_delta = act_after.saturating_sub(act_before);
    let app_delta = app_after.saturating_sub(app_before);
    let rss_delta = rss_after.saturating_sub(rss_before);
    let mib = |b: u64| b as f64 / (1024.0 * 1024.0);

    eprintln!("=== Static-pool footprint probe ===");
    eprintln!(
        "declared per-slot MAX = {} MiB; actual payload = {} KiB; N published = {}",
        MAX / 1048576,
        PAYLOAD / 1024,
        N
    );
    eprintln!(
        "/dev/shm ACTUAL (resident blocks) delta   = {} bytes = {:.2} MiB",
        act_delta,
        mib(act_delta)
    );
    eprintln!(
        "/dev/shm APPARENT (ftruncate size) delta  = {} bytes = {:.2} MiB",
        app_delta,
        mib(app_delta)
    );
    eprintln!(
        "process VmRSS delta                        = {} bytes = {:.2} MiB",
        rss_delta,
        mib(rss_delta)
    );
    eprintln!(
        "VERDICT: if ACTUAL ({:.1} MiB) << APPARENT ({:.1} MiB) → Static pool is LAZY \
         (tier-max reservation is sparse, PowerOfTwo shrink is illusory on Linux).",
        mib(act_delta),
        mib(app_delta)
    );

    // Skip only when the measurement TOOL is unusable — macOS (no `/dev/shm`)
    // or a BusyBox `du` that rejects our flags — keyed on `du`'s EXIT STATUS,
    // NOT `/dev/shm` existence: a working `du` reporting a 0 apparent delta is a
    // REAL reservation regression and must FAIL the assert, not silently skip.
    // (`du_dev_shm` swallows a broken tool to 0, which a bare `/dev/shm`-exists
    // gate would misattribute to a failed reservation.)
    if !du_apparent_usable() {
        eprintln!(
            "no usable GNU du for /dev/shm (non-Linux or BusyBox) — skipping residency assert"
        );
        drop(subscriber);
        drop(publisher);
        return;
    }
    assert!(
        app_delta >= 8 * 1024 * 1024,
        "a real >=8 MiB pool reservation must register as APPARENT /dev/shm (got {app_delta})"
    );
    assert!(
        act_delta * 8 <= app_delta,
        "Static pool must be LAZY: resident /dev/shm ({act_delta} B) must be <= 1/8 of the \
         apparent reservation ({app_delta} B) — eager backing would make them ~equal"
    );

    drop(subscriber);
    drop(publisher);
}

/// Does the `/dev/shm` `size=` limit enforce on APPARENT (ftruncate) or ACTUAL
/// (resident)? Create enough 16 MiB-tier pools to exceed the tmpfs `size=` in
/// APPARENT terms; if they all create and `df` stays tiny, the limit is on
/// resident (reservation is free → the footprint concern is fully moot). If creation fails, the
/// limit is on apparent (a real fit problem → PowerOfTwo helps fit, not RAM).
#[test]
#[ignore]
#[serial]
fn probe_multi_pool_fit() {
    const MAX: u32 = 16 * 1024 * 1024; // 16 MiB probe pool (an explicit size; TIER_HUGE is now 128 MiB, and any big pool proves laziness)

    // Pool count via env (default 8). Set CER_FIT_POOLS high enough that the
    // APPARENT reservation exceeds the machine's physical RAM while the actual data
    // written stays tiny — proves the virtual reservation is free even past RAM.
    let pools: usize = std::env::var("CER_FIT_POOLS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(8);

    let mgr = TransportManager::get_or_init().expect("init");
    let mib = |b: u64| b as f64 / (1024.0 * 1024.0);

    eprintln!("=== multi-pool FIT probe ===");
    eprintln!(
        "/dev/shm resident BEFORE = {:.1} MiB",
        mib(du_dev_shm(false))
    );

    let mut pubs = Vec::new();
    let mut created = 0usize;
    for i in 0..pools {
        let topic = format!("shm_fit/{}/{}", std::process::id(), i);
        match mgr.create_publisher_simple(&topic, MaxSliceLen::const_new(MAX)) {
            Ok(mut p) => {
                // Touch one sample so a page is written.
                if let Ok(mut proxy) = p.loan_proxy::<Image>() {
                    proxy.height = 1;
                    proxy.width = 1;
                    proxy.is_bigendian = 0;
                    proxy.step = 0;
                    let _ = proxy.set_header_bytes(&[]);
                    let _ = proxy.set_encoding("rgb8");
                    let _ = proxy.set_data(&[0x80u8; 1024]);
                }
                created += 1;
                pubs.push(p);
                eprintln!(
                    "  pool {} created OK — apparent so far ~{:.1} GiB; resident /dev/shm = {:.2} MiB",
                    i,
                    (i + 1) as f64 * 2.28,
                    mib(du_dev_shm(false))
                );
            }
            Err(e) => {
                eprintln!(
                    "  pool {} FAILED to create: {e} — apparent limit reached at ~{:.1} GiB (< pools' 2.3 GiB each)",
                    i,
                    i as f64 * 2.28
                );
                break;
            }
        }
    }

    eprintln!(
        "RESULT: {}/{} pools created; final resident /dev/shm = {:.2} MiB, apparent = {:.1} GiB",
        created,
        pools,
        mib(du_dev_shm(false)),
        mib(du_dev_shm(true)) / 1024.0
    );
    eprintln!(
        "VERDICT: all {pools} created + resident tiny even with apparent >> RAM → the virtual \
         reservation is FREE (demand-paged); the tmpfs/commit limit binds on RESIDENT, not the reservation."
    );

    assert_eq!(
        created, pools,
        "all {pools} lazy Static pools must create — apparent reservation is virtual/demand-paged, not real RAM"
    );
    let resident_after = du_dev_shm(false);
    if resident_after > 0 {
        assert!(
            resident_after < 128 * 1024 * 1024,
            "even with apparent reservation >> RAM, RESIDENT /dev/shm must stay tiny (<128 MiB); got {resident_after} B"
        );
    }

    drop(pubs);
}

/// Does oversizing the Static pool cost LATENCY (even if resident RAM is free)?
/// Measure pub→recv latency for the SAME 1 KiB payload through a BIG-tier pool
/// (128 MiB, the proposed generous TIER_HUGE) vs a SMALL pool (16 KiB). Same
/// working set → any delta is the big-sparse-mapping cost (page-table walk / TLB
/// / fault behavior). Reports floor(min)/p50/p99/max over a warmed window.
#[test]
#[ignore]
#[serial]
fn probe_latency_big_vs_small_tier() {
    fn publish_1kib(p: &mut cerulion_core::CerulionPublisher) {
        if let Ok(mut proxy) = p.loan_proxy::<Image>() {
            proxy.height = 1;
            proxy.width = 1;
            proxy.is_bigendian = 0;
            proxy.step = 0;
            let _ = proxy.set_header_bytes(&[]);
            let _ = proxy.set_encoding("rgb8");
            let _ = proxy.set_data(&[0x80u8; 1024]);
        }
    }

    fn measure(max: u32, label: &str) -> (u64, u64, u64) {
        const WARMUP: usize = 3000;
        const MEASURE: usize = 30000;
        let mgr = TransportManager::get_or_init().expect("init");
        let topic = format!("shm_lat/{}/{}", std::process::id(), label);
        let mut publisher = mgr
            .create_publisher_simple(&topic, MaxSliceLen::const_new(max))
            .expect("create publisher");
        let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

        for _ in 0..WARMUP {
            publish_1kib(&mut publisher);
            let _ = subscriber.try_view::<Image, _>(|v| v.height);
        }

        let mut lats: Vec<u64> = Vec::with_capacity(MEASURE);
        for _ in 0..MEASURE {
            let t0 = Instant::now();
            publish_1kib(&mut publisher);
            let mut got = false;
            for _ in 0..10_000 {
                if matches!(subscriber.try_view::<Image, _>(|v| v.height), Ok(Some(_))) {
                    got = true;
                    break;
                }
            }
            let ns = t0.elapsed().as_nanos() as u64;
            if got {
                lats.push(ns);
            }
        }
        lats.sort_unstable();
        let pct = |p: f64| lats[(((lats.len() - 1) as f64) * p) as usize];
        let (floor, p50, p99) = (lats[0], pct(0.50), pct(0.99));
        eprintln!(
            "[{label}] pool={:>4} : floor={:>7}ns  p50={:>7}ns  p99={:>8}ns  max={:>9}ns  (n={})",
            if max >= 1048576 {
                format!("{}M", max / 1048576)
            } else {
                format!("{}K", max / 1024)
            },
            floor,
            p50,
            p99,
            lats[lats.len() - 1],
            lats.len(),
        );
        (floor, p50, p99)
    }

    eprintln!("=== pub→recv latency: big-tier vs small-tier (same 1 KiB payload) ===");
    let (big_floor, big_p50, big_p99) = measure(128 * 1024 * 1024, "big-128M");
    let (small_floor, small_p50, small_p99) = measure(16 * 1024, "small-16K");
    let _ = measure(128 * 1024 * 1024, "big-128M-2"); // stability re-run, print-only

    // Oversizing a lazy Static pool must NOT cost steady-state latency: the big-tier
    // floor/p50/p99 track the small-tier within a generous 2x (measured ~identical
    // ~8/8.4/8.7us). Asserts on FLOOR/P50/P99 only — never max (a run-to-run outlier).
    assert!(
        big_floor <= small_floor * 2,
        "big-tier floor {big_floor} regressed vs small {small_floor} (>2x) — oversizing should be latency-free"
    );
    assert!(
        big_p50 <= small_p50 * 2,
        "big-tier p50 {big_p50} regressed vs small {small_p50} (>2x)"
    );
    assert!(
        big_p99 <= small_p99 * 2,
        "big-tier p99 {big_p99} regressed vs small {small_p99} (>2x)"
    );
}
