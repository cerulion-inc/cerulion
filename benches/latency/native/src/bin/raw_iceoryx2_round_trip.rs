//! Raw iceoryx2 round-trip latency benchmark — the transport FLOOR.
//!
//! Pure iceoryx2 ping-pong with NO Cerulion wrapping — establishes the
//! OS / hardware floor that any iceoryx2-based middleware (Cerulion or
//! otherwise) cannot beat. By comparing this against
//! `cerulion_user_round_trip` we can bound everything Cerulion adds on
//! top of iceoryx2's lock-free queue.
//!
//! # Methodology
//!
//! Mirrors the canonical iceoryx2 latency benchmark
//! ([`benchmarks/publish-subscribe/src/main.rs`](https://github.com/eclipse-iceoryx/iceoryx2/blob/main/benchmarks/publish-subscribe/src/main.rs))
//! as closely as possible:
//!
//! - Two threads on the same host, ping-pong over `publish_subscribe::<[u8]>()`.
//! - **Zero payload writes inside the timing window.** The publisher loans
//!   `loan_slice_uninit(N) → assume_init()` and never touches the bytes;
//!   the receiver drops samples without inspecting their contents. Ordering
//!   is iceoryx2's FIFO guarantee, full stop.
//! - **Pre-loaned-sample pattern.** Initiator loans the first sample before
//!   the timed loop starts, then per iteration: send the previously-loaned
//!   sample, loan the next one, wait for the echo. Matches upstream verbatim.
//! - **Service-builder options match upstream:** `max_publishers(1)`,
//!   `max_subscribers(1)`, `history_size(0)`, `subscriber_max_buffer_size(1)`,
//!   `enable_safe_overflow(true)`.
//! - **Barrier-synchronised startup.** No atomic-flag-plus-sleep guesswork.
//! - Same payload sweep (64 B → 16 MB) and per-payload iteration plan
//!   (`CER_BENCH_PACING`) as the other bench binaries for direct CSV overlay.
//!
//! The receive side is a spin-poll — this binary is spin-bound by design
//! (the suite runs it at chrt 0 only; pinning a busy-spinning pair with
//! SCHED_FIFO just measures scheduler starvation).
//!
//! Output: raw LE u64 ns samples per payload size, written to
//! `${CER_BENCH_RAW_DUMP_DIR}/${CER_BENCH_RAW_NAME}_<payloadbytes>.bin`
//! (fail-fast if either var is unset). Percentiles are computed offline.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Barrier};
use std::thread;
use std::time::Instant;

use cerulion_latency_bench::BenchPlan;
use iceoryx2::port::ReceiveError;
use iceoryx2::prelude::*;

/// One full (warmup + measured) pass at the plan's rate; returns the
/// measured samples (dumping is the caller's job — `run_sized_bench`
/// owns the fixed100 fallback ladder and only dumps a SUSTAINED pass).
/// Fresh services per call (unique run_id), so a ladder re-run starts
/// from a clean connection state.
fn run_round_trip(payload_size: usize, plan: &mut BenchPlan) -> Vec<u64> {
    // Unique topic names per (size, run) — iceoryx2 services are host-wide,
    // so collisions across runs are real.
    let run_id = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let ping_name: ServiceName = format!("rti/ping/{payload_size}/{run_id}")
        .as_str()
        .try_into()
        .expect("ping service name");
    let pong_name: ServiceName = format!("rti/pong/{payload_size}/{run_id}")
        .as_str()
        .try_into()
        .expect("pong service name");

    let stop = Arc::new(AtomicBool::new(false));
    // Two barriers — startup ("everyone has built their ports") and start
    // ("begin the timed work"). Main is also the initiator, so 2 participants.
    let startup = Arc::new(Barrier::new(2));
    let start = Arc::new(Barrier::new(2));

    let stop_b = Arc::clone(&stop);
    let startup_b = Arc::clone(&startup);
    let start_b = Arc::clone(&start);
    // `ServiceName` is `Copy` — plain bindings hand the responder thread
    // its own copies (a `.clone()` here trips clippy::clone_on_copy).
    let ping_name_b = ping_name;
    let pong_name_b = pong_name;

    let responder = thread::spawn(move || {
        let node_b = NodeBuilder::new()
            .create::<ipc::Service>()
            .expect("create iceoryx2 node B");
        let ping_service_b = node_b
            .service_builder(&ping_name_b)
            .publish_subscribe::<[u8]>()
            .max_publishers(1)
            .max_subscribers(1)
            .history_size(0)
            .subscriber_max_buffer_size(1)
            .enable_safe_overflow(true)
            .open_or_create()
            .expect("open ping service B");
        let pong_service_b = node_b
            .service_builder(&pong_name_b)
            .publish_subscribe::<[u8]>()
            .max_publishers(1)
            .max_subscribers(1)
            .history_size(0)
            .subscriber_max_buffer_size(1)
            .enable_safe_overflow(true)
            .open_or_create()
            .expect("open pong service B");

        let pub_b = pong_service_b
            .publisher_builder()
            .initial_max_slice_len(payload_size)
            .create()
            .expect("responder publisher");
        let sub_b = ping_service_b
            .subscriber_builder()
            .buffer_size(1)
            .create()
            .expect("responder subscriber");

        startup_b.wait();
        start_b.wait();

        // Hot loop: loan an outbound slot, wait for the inbound ping
        // (drop it without inspecting), echo. Same shape as iceoryx2's
        // canonical responder thread.
        'outer: loop {
            if stop_b.load(Ordering::Acquire) {
                break 'outer;
            }

            // SAFETY: `loan_slice_uninit` returns an iceoryx2 slot whose
            // payload bytes are uninitialised. The bench never reads
            // those bytes — it only forwards the slot back to the peer
            // via `send()`. iceoryx2's transport contract permits
            // sending uninitialised SHM as long as the receiver doesn't
            // interpret the contents, which the round-trip peer doesn't
            // (it `drop()`s the inbound sample without inspecting it).
            let sample = unsafe {
                pub_b
                    .loan_slice_uninit(payload_size)
                    .expect("responder loan_slice_uninit")
                    .assume_init()
            };

            let inbound = loop {
                match sub_b.receive().expect("responder receive") {
                    Some(s) => break s,
                    None => {
                        if stop_b.load(Ordering::Acquire) {
                            break 'outer;
                        }
                        std::hint::spin_loop();
                    }
                }
            };
            drop(inbound);

            sample.send().expect("responder send");
        }
    });

    // Main thread builds the initiator side.
    let node_a = NodeBuilder::new()
        .create::<ipc::Service>()
        .expect("create iceoryx2 node A");
    let ping_service_a = node_a
        .service_builder(&ping_name)
        .publish_subscribe::<[u8]>()
        .max_publishers(1)
        .max_subscribers(1)
        .history_size(0)
        .subscriber_max_buffer_size(1)
        .enable_safe_overflow(true)
        .open_or_create()
        .expect("open ping service A");
    let pong_service_a = node_a
        .service_builder(&pong_name)
        .publish_subscribe::<[u8]>()
        .max_publishers(1)
        .max_subscribers(1)
        .history_size(0)
        .subscriber_max_buffer_size(1)
        .enable_safe_overflow(true)
        .open_or_create()
        .expect("open pong service A");

    let pub_a = ping_service_a
        .publisher_builder()
        .initial_max_slice_len(payload_size)
        .create()
        .expect("initiator publisher");
    let sub_a = pong_service_a
        .subscriber_builder()
        .buffer_size(1)
        .create()
        .expect("initiator subscriber");

    startup.wait();

    // Pre-loan the first sample before the timing window opens. Per
    // iteration the loop sends this one, loans the next, then waits.
    // SAFETY: bench never reads the payload bytes — see the responder
    // thread above for the full uninitialised-loan rationale.
    let mut next_sample = unsafe {
        pub_a
            .loan_slice_uninit(payload_size)
            .expect("first loan_slice_uninit")
            .assume_init()
    };

    start.wait();

    for _ in 0..plan.warmup {
        plan.wait_next();
        let to_send = next_sample;
        to_send.send().expect("warmup send");
        // SAFETY: same uninitialised-loan rationale as the pre-loan above.
        next_sample = unsafe {
            pub_a
                .loan_slice_uninit(payload_size)
                .expect("warmup loan")
                .assume_init()
        };
        loop {
            match sub_a.receive().expect("warmup receive") {
                Some(_s) => break,
                None => std::hint::spin_loop(),
            }
        }
    }

    let mut samples = Vec::with_capacity(plan.measured as usize);
    for _ in 0..plan.measured {
        plan.wait_next();
        let start_t = Instant::now();
        let to_send = next_sample;
        to_send.send().expect("measurement send");
        // SAFETY: same uninitialised-loan rationale as the pre-loan above.
        next_sample = unsafe {
            pub_a
                .loan_slice_uninit(payload_size)
                .expect("measurement loan")
                .assume_init()
        };
        loop {
            match sub_a.receive().expect("measurement receive") {
                Some(_s) => break,
                None => std::hint::spin_loop(),
            }
        }
        samples.push(start_t.elapsed().as_nanos() as u64);
    }

    // Drop the unused pre-loaned sample to release its slot back to the
    // publisher pool.
    drop(next_sample);

    stop.store(true, Ordering::Release);
    responder.join().expect("responder thread panicked");

    // Drain any remaining samples to release SHM slots cleanly between
    // payload sizes (and between fixed100 ladder re-runs).
    let _ = drain(&sub_a);

    samples
}

fn drain(
    sub: &iceoryx2::port::subscriber::Subscriber<ipc::Service, [u8], ()>,
) -> Result<(), ReceiveError> {
    while sub.receive()?.is_some() {}
    Ok(())
}

fn main() {
    let _lock = cerulion_latency_bench::acquire_dma_lock();
    // Pinned 10-size sweep, restrictable via CER_BENCH_PAYLOAD_SIZES
    // (partial re-sweeps — the suite-wide env contract).
    let payload_sizes = cerulion_latency_bench::sweep_payload_sizes();
    eprintln!(
        "Raw iceoryx2 round-trip benchmark [{}] — {} sizes",
        cerulion_latency_bench::PacingMode::from_env().as_str(),
        payload_sizes.len(),
    );
    eprintln!();

    for &size in &payload_sizes {
        // The shared driver owns the per-size progress line, the dump,
        // and (fixed100) the fallback ladder + .rate sidecar.
        cerulion_latency_bench::run_sized_bench(size, |plan| run_round_trip(size, plan));
    }

    eprintln!();
    eprintln!("Done. Raw .bin samples written to CER_BENCH_RAW_DUMP_DIR.");
}
