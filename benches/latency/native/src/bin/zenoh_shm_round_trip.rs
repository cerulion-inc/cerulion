//! zenoh-SHM round-trip latency benchmark — initiator (COMPARISON line).
//!
//! Mirrors zenoh's canonical
//! [`z_ping_shm.rs`](https://github.com/eclipse-zenoh/zenoh/blob/main/examples/examples/z_ping_shm.rs)
//! methodology:
//!
//! - One SHM buffer allocated per payload size, OUTSIDE the timing loop.
//! - Per iteration the buffer is `.clone()`'d (ZBytes is Arc-backed — no
//!   memcpy, no fill).
//! - Publisher uses `CongestionControl::Block` so backpressure stalls the
//!   sender rather than dropping samples.
//! - The pong responder is a separate process (`zenoh_shm_round_trip_pong`,
//!   spawned at startup behind an RAII guard that kills and reaps it on
//!   every exit path, panics included, and waited for by a bounded
//!   readiness poll on its listener) — same shape as zenoh's z_ping/z_pong
//!   split.
//!
//! Pacing per `CER_BENCH_PACING` (quiescent schedule by default; fixed100
//! uniform-rate or back-to-back on request) — identical iteration plan to
//! the Cerulion / iceoryx2 binaries so lines overlay apples-to-apples.
//!
//! Output: raw LE u64 ns samples per payload size, written to
//! `${CER_BENCH_RAW_DUMP_DIR}/${CER_BENCH_RAW_NAME}_<payloadbytes>.bin`
//! (fail-fast if either var is unset). Percentiles are computed offline.

use std::net::TcpStream;
use std::process::Child;
use std::time::{Duration, Instant};

use cerulion_latency_bench::{zenoh_shm_config, BenchPlan, ZENOH_BENCH_ENDPOINT};
use zenoh::{
    bytes::ZBytes, key_expr::keyexpr, qos::CongestionControl, shm::ShmProviderBuilder, Wait,
};

/// One full (warmup + measured) pass at the plan's rate; returns the
/// measured samples (dumping is the caller's job — `run_sized_bench`
/// owns the fixed100 fallback ladder and only dumps a SUSTAINED pass).
fn run_round_trip(
    publisher: &zenoh::pubsub::Publisher,
    sub: &zenoh::pubsub::Subscriber<zenoh::handlers::FifoChannelHandler<zenoh::sample::Sample>>,
    payload_size: usize,
    plan: &mut BenchPlan,
) -> Vec<u64> {
    // Provider sized 2x payload with a 1 MiB floor — zenoh 1.7.2's
    // default Talc backend rejects sub-MiB allocations on Linux
    // ("Error initializing Talc backend!" panic on small backends).
    // The floor doesn't change observed latency for the small payloads:
    // the SHM region is mmap'd lazily, so unused headroom is free.
    let backend_size = (payload_size * 2).max(1024 * 1024);
    let provider = ShmProviderBuilder::default_backend(backend_size)
        .wait()
        .expect("build SHM provider");
    let shm_buf = provider.alloc(payload_size).wait().expect("alloc SHM buf");
    // Convert ZShmMut → ZBytes. ZBytes is Arc-backed so .clone() per
    // iteration is just a refcount bump.
    let buf: ZBytes = shm_buf.into();

    for _ in 0..plan.warmup {
        plan.wait_next();
        publisher.put(buf.clone()).wait().expect("warmup put");
        let echo = sub.recv().expect("warmup recv");
        assert_eq!(
            echo.payload().len(),
            payload_size,
            "received a {} B sample while warming up {payload_size} B — the \
             subscriber is out of step",
            echo.payload().len()
        );
    }

    let mut samples = Vec::with_capacity(plan.measured as usize);
    for _ in 0..plan.measured {
        plan.wait_next();
        let to_send = buf.clone();
        let start_t = Instant::now();
        publisher.put(to_send).wait().expect("measurement put");
        let echo = sub.recv().expect("measurement recv");
        // STAMP FIRST, then check: the assert is the suite's own
        // bookkeeping and must not sit inside the measured window.
        samples.push(start_t.elapsed().as_nanos() as u64);
        // Key the receive on the payload it must be. Without this a
        // leftover 1-byte readiness echo — or a second responder on the
        // pong key — is accepted as a full round trip, and the sample
        // count stays exactly right so no downstream gate can see it.
        // The smallest swept payload is 64 B, so a probe can never alias.
        assert_eq!(
            echo.payload().len(),
            payload_size,
            "received a {} B sample while measuring {payload_size} B — the \
             subscriber is out of step (a leftover readiness-probe echo, or \
             a second responder on the pong key)",
            echo.payload().len()
        );
    }

    samples
}

/// Owns the pong subprocess for the whole run.
///
/// Dropping a `std::process::Child` does NOT kill the process, and every
/// step after the spawn (`expect` on the session, provider, publish or
/// receive) can unwind past the explicit kill at the end of `main`. The
/// orphan then keeps the fixed listener port and answers — or wedges —
/// the NEXT cell, which shows up as a hang or as stale-pong latency
/// rather than as a failure of the run that leaked it. This guard kills
/// and reaps on EVERY exit path, panics included.
struct PongGuard(Child);

impl Drop for PongGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// Bounded readiness poll for the pong process.
///
/// A fixed sleep is not a readiness check: a slow start silently becomes a
/// connect against nothing, and a child that died at startup (a bind
/// failure on the fixed locator, a missing SHM backend) is indistinguishable
/// from one that is merely slow. Poll the listener the child is supposed to
/// have bound, and check for child EXIT between polls so a dead pong is
/// reported as a dead pong.
fn wait_for_pong(guard: &mut PongGuard, timeout: Duration) {
    let deadline = Instant::now() + timeout;
    loop {
        match guard.0.try_wait() {
            Ok(Some(status)) => panic!(
                "pong process exited during startup ({status}) — it never \
                 bound {ZENOH_BENCH_ENDPOINT}; run zenoh_shm_round_trip_pong by hand \
                 to see its error"
            ),
            Ok(None) => {}
            Err(e) => panic!("cannot poll the pong process: {e}"),
        }
        if TcpStream::connect(ZENOH_BENCH_ENDPOINT).is_ok() {
            return;
        }
        if Instant::now() >= deadline {
            panic!(
                "pong did not accept a connection on {ZENOH_BENCH_ENDPOINT} within \
                 {timeout:?} — is the port already held by another run?"
            );
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

fn main() {
    let _lock = cerulion_latency_bench::acquire_dma_lock();
    zenoh::init_log_from_env_or("error");

    // Spawn the responder as a sibling subprocess. Both binaries live
    // in the same target/release/ dir.
    let pong_path = {
        let mut p = std::env::current_exe().expect("current_exe");
        p.pop();
        p.push("zenoh_shm_round_trip_pong");
        p
    };
    // Refuse BEFORE spawning if something is already accepting on the
    // endpoint. Without this the readiness poll below is satisfied by that
    // foreign listener on its first iteration — before our child has even
    // finished failing to bind — and the whole sweep would round-trip
    // against a pong this process does not own (a stale one from an earlier
    // cell answers happily, and the .bin it produces has the right sample
    // count and the wrong numbers). It also turns the timeout message's
    // speculative "already held by another run?" into something proved.
    if TcpStream::connect(ZENOH_BENCH_ENDPOINT).is_ok() {
        panic!(
            "{ZENOH_BENCH_ENDPOINT} is already accepting BEFORE the pong was \
             spawned — a stale zenoh_shm_round_trip_pong or another process \
             holds it. Kill it and re-run: measuring now would round-trip \
             against a peer this process does not own."
        );
    }

    let mut pong = PongGuard(
        std::process::Command::new(&pong_path)
            .spawn()
            .unwrap_or_else(|e| panic!("spawn {} failed: {}", pong_path.display(), e)),
    );

    // Wait for the child to actually bind its listener (or die trying).
    wait_for_pong(&mut pong, Duration::from_secs(30));

    // Ping connects to the pong's listener.
    let session = zenoh::open(zenoh_shm_config(/* listen */ false))
        .wait()
        .expect("open zenoh session (ping)");

    let key_ping = keyexpr::new("test/ping").expect("ping key");
    let key_pong = keyexpr::new("test/pong").expect("pong key");

    let publisher = session
        .declare_publisher(key_ping)
        .congestion_control(CongestionControl::Block)
        .wait()
        .expect("declare ping publisher");
    let sub = session
        .declare_subscriber(key_pong)
        .wait()
        .expect("declare pong subscriber");

    // Prove the WHOLE loop before measuring, rather than sleeping and
    // hoping the declarations propagated. This is not a nicety: the
    // warm-up below blocks on `sub.recv()`, so a put issued before the
    // peers matched is simply lost and the bench wedges with no
    // diagnostic.
    //
    // `matching_status()` covers only the forward half (a subscriber
    // matching the ping key exists). The REVERSE half — pong's publisher
    // against this process's `test/pong` subscriber — has no matching
    // query in this API, and it is the half that eats the first echo. So
    // the forward check is just a fast pre-condition, and readiness is
    // then established by an actual bounded round trip: put a probe, wait
    // a slice for the echo, repeat until one comes back or the deadline
    // passes. An echo proves both directions matched, by observation.
    let ready_deadline = Instant::now() + Duration::from_secs(30);
    loop {
        let matched = publisher
            .matching_status()
            .wait()
            .expect("query ping publisher matching status")
            .matching();
        if matched {
            break;
        }
        if Instant::now() >= ready_deadline {
            panic!(
                "no subscriber matched the ping key within 30s — the pong \
                 process is connected but never declared its subscriber"
            );
        }
        std::thread::sleep(Duration::from_millis(10));
    }

    // Readiness probe payload: a 1-byte SHM-free put is enough — this
    // exchange proves ROUTING, not the data path, and the measured loop
    // allocates its own SHM buffer per size.
    let mut probes = 0u32;
    loop {
        // Poll the child, as the startup wait does: a pong that died AFTER
        // binding its listener (a failed declare, or its republish
        // `expect` under CongestionControl::Block) would otherwise spin
        // the whole deadline and then blame a matching failure. The
        // forward put cannot report it either — with no matched
        // subscriber zenoh simply drops the sample.
        match pong.0.try_wait() {
            Ok(Some(status)) => panic!(
                "pong process exited ({status}) while waiting for the first \
                 echo, after {probes} probe(s) — it bound its listener and \
                 then died; run zenoh_shm_round_trip_pong by hand to see why"
            ),
            Ok(None) => {}
            Err(e) => panic!("cannot poll the pong process: {e}"),
        }
        publisher
            .put(ZBytes::from(vec![0u8; 1]))
            .wait()
            .expect("readiness probe put");
        probes += 1;
        match sub.recv_timeout(Duration::from_millis(200)) {
            Ok(Some(_)) => break,
            Ok(None) => {}
            Err(e) => panic!("readiness probe recv failed: {e}"),
        }
        if Instant::now() >= ready_deadline {
            panic!(
                "no echo came back within 30s after {probes} readiness \
                 probe(s), and the pong process is still alive. The ping key \
                 had matched a subscriber when this wait began, so the likely \
                 break is the reverse leg — pong's publisher against this \
                 process's subscriber on the pong key"
            );
        }
    }

    // Drain every echo the retries produced. Counted, not opportunistic:
    // `try_recv` is non-blocking, so a bare drain-until-empty stops at the
    // first MOMENTARILY empty queue while probe N's echo is still in
    // flight. One leftover echo shifts the whole sweep by one — the
    // measured loop is a synchronous put/recv over a subscriber shared
    // across every size, so every later sample would report put-time
    // instead of a round trip, with the sample COUNT still exactly right
    // and every count gate still green. Wait for the echoes we know we
    // provoked, then sweep opportunistically for any straggler.
    let mut drained: u32 = 1; // the probe loop consumed one
    while drained < probes {
        match sub.recv_timeout(Duration::from_millis(200)) {
            Ok(Some(_)) => drained += 1,
            // Never echoed: that probe was put before the peers matched.
            Ok(None) => break,
            Err(e) => panic!("draining the readiness probes' echoes failed: {e}"),
        }
    }
    loop {
        match sub.try_recv() {
            Ok(Some(_)) => {}
            Ok(None) => break,
            // A broken channel here would otherwise end the drain silently
            // and surface one line later as `warmup recv` — say which step
            // actually failed.
            Err(e) => panic!("draining the readiness probes' echoes failed: {e}"),
        }
    }

    // Pinned 10-size sweep, restrictable via CER_BENCH_PAYLOAD_SIZES
    // (partial re-sweeps — the suite-wide env contract).
    let payload_sizes = cerulion_latency_bench::sweep_payload_sizes();
    eprintln!(
        "zenoh-SHM round-trip benchmark [{}] — {} sizes",
        cerulion_latency_bench::PacingMode::from_env().as_str(),
        payload_sizes.len(),
    );
    eprintln!();

    for &size in &payload_sizes {
        // The shared driver owns the per-size progress line, the dump,
        // and (fixed100) the fallback ladder + .rate sidecar.
        cerulion_latency_bench::run_sized_bench(size, |plan| {
            run_round_trip(&publisher, &sub, size, plan)
        });
    }

    eprintln!();
    eprintln!("Done. Raw .bin samples written to CER_BENCH_RAW_DUMP_DIR.");

    // Clean shutdown. `PongGuard::drop` would do this anyway on every exit
    // path; dropping it explicitly keeps the ordering obvious.
    drop(pong);
}
