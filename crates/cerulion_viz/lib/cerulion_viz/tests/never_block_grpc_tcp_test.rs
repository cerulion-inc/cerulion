// SPDX-License-Identifier: AGPL-3.0-only
//! THE decisive never-block pin over the REAL gRPC backpressure shape.
//!
//! The live incident behind this test was a gRPC wedge: the rerun server stopped draining the
//! sink's gRPC stream (a suspended browser tab). This reproduces that EXACT
//! shape — a local `TcpListener` that ACCEPTS connections but NEVER READS — and
//! points the sink's real gRPC sink at it. The gRPC client's send channel fills
//! (nothing is ever consumed), the batcher fills behind it, and `rec.log` blocks
//! on the worker thread. The assertion: the enqueue path stays bounded (never
//! blocks) while the drop counter climbs — the whole point of the never-block fix.
//!
//! Own binary, single `#[test]` (isolated). Uses a tiny always-flush batcher so
//! the pipeline wedges within a small number of frames.

mod common;

use std::io::Read;
use std::net::TcpListener;
use std::time::{Duration, Instant};

use cerulion_viz::sink::SinkState;
use cerulion_viz::worker::{InputFrames, VizLogWorker};
use common::{build_twist_at, builtin_walker};
use rerun::log::ChunkBatcherConfig;

/// `seq` advances the frame's WIRE timestamp by 10 ms, modelling a 100 Hz
/// publisher. Load-bearing since the plot rate gate landed: a plot topic is rate-gated on its
/// publisher's own clock, so a fixture stamping every frame identically would
/// render exactly one of them and the gRPC pipeline would never wedge — this
/// test's whole premise.
fn twist_batch(seq: u64) -> Vec<InputFrames> {
    vec![InputFrames {
        name: "twist".to_string(),
        frames: vec![build_twist_at(
            [1.0, 2.0, 3.0],
            [0.1, 0.2, 0.3],
            seq * 10_000_000,
        )],
    }]
}

/// The enqueue budget on an unloaded machine. `try_enqueue` is a bounded
/// channel `try_send` plus one `tracing` emission on the drop arm, so its real
/// cost is microseconds; the budget is three orders of magnitude above that and
/// three orders BELOW a genuine block, which never returns at all while the
/// pipeline stays wedged.
const ENQUEUE_BUDGET: Duration = Duration::from_millis(50);

/// The per-iteration breather, and the scheduler-delay reference it doubles as.
const PACING: Duration = Duration::from_millis(1);

#[test]
fn tick_never_blocks_while_grpc_server_accepts_but_never_reads() {
    // A server that ACCEPTS TCP connections but NEVER READS — the exact wedge
    // shape a suspended viewer tab produces. Hold accepted sockets so
    // the OS keeps the connection up; do a single zero-length peek that returns
    // immediately, then never read again.
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind ephemeral");
    let port = listener.local_addr().expect("addr").port();
    // Detached: this test process exits at the end of the test, reaping it.
    std::thread::spawn(move || {
        let mut held = Vec::new();
        for stream in listener.incoming() {
            match stream {
                Ok(mut s) => {
                    // One non-consuming touch, then hold forever without reading.
                    let mut buf = [0u8; 0];
                    let _ = s.read(&mut buf);
                    held.push(s);
                }
                Err(_) => break,
            }
        }
    });

    let url = format!("rerun+http://127.0.0.1:{port}/proxy");
    let tiny = ChunkBatcherConfig {
        max_bytes_in_flight: 1024,
        ..ChunkBatcherConfig::ALWAYS_TEST_ONLY
    };
    let rec = rerun::RecordingStreamBuilder::new("go2_test")
        .recording_id("never_block_grpc_tcp")
        .batcher_config(tiny)
        .connect_grpc_opts(url)
        .expect("gRPC sink to the never-reading listener");

    let mut worker =
        VizLogWorker::spawn(rec, builtin_walker(), SinkState::new()).expect("spawn worker");

    // Feed until drops appear (the wedge fills the client channel + batcher) or a
    // generous cap; time EVERY enqueue. The enqueue must never block.
    //
    // The pacing sleep below is timed too, and it is what makes the enqueue
    // budget load-proof. A ceiling on a wall can only be tripped by a machine
    // that is SLOWER than expected, so contention pushes this assert toward a
    // red with nothing wrong. The sleep is a pure scheduler wait in the same
    // loop under the same contention, so whatever delay a stalled runner adds
    // to the enqueue window it also adds here: subtracting the requested 1 ms
    // leaves the worst scheduling delay this loop actually paid, and the budget
    // carries that as slack. On a quiet machine the slack is ~0 and the gate is
    // exactly as tight as the bare budget. A genuine block is unbounded (the
    // wedged pipeline never drains), so no amount of slack can hide it.
    let mut max_enqueue = Duration::ZERO;
    let mut max_pacing = Duration::ZERO;
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut fed = 0u64;
    while worker.dropped_frames() == 0 && Instant::now() < deadline {
        // BUILT OUTSIDE THE TIMED WINDOW. `twist_batch` is not cheap:
        // `build_twist_at` calls `field_offset` twice and `build_fixed_frame_at`
        // once; EACH of those calls `all_schemas()`, which parses the whole
        // 254-message built-in corpus (`BUILTIN_MSGS`, embedded text — no disk
        // I/O), and each then builds a fresh `LayoutResolver` over it — so one
        // frame costs THREE parses and THREE resolvers. Timing that
        // alongside the enqueue made this assert a load-dependent coin flip
        // (MEASURED: the third of three local runs failed) while
        // measuring frame construction rather than the property under test.
        let batch = twist_batch(fed);
        let t0 = Instant::now();
        worker.try_enqueue(batch);
        max_enqueue = max_enqueue.max(t0.elapsed());
        fed += 1;
        // A small breather so the worker can advance into rec.log and wedge
        // (without this a very fast loop can outrun the batcher setup); this is
        // pacing only, NOT part of the timed enqueue window above. Its own wall
        // is the scheduler-delay reference (see the note at the loop head).
        let t1 = Instant::now();
        std::thread::sleep(PACING);
        max_pacing = max_pacing.max(t1.elapsed());
    }

    assert!(
        worker.dropped_frames() > 0,
        "the real gRPC sink to a never-reading server must wedge the worker so viz frames drop \
         (fed {fed}, dropped {})",
        worker.dropped_frames()
    );
    let scheduling_slack = max_pacing.saturating_sub(PACING);
    let budget = ENQUEUE_BUDGET + scheduling_slack;
    assert!(
        max_enqueue < budget,
        "the enqueue must never block even with the real gRPC pipeline wedged \
         (worst enqueue {max_enqueue:?} against {ENQUEUE_BUDGET:?} plus {scheduling_slack:?} \
          of measured scheduling delay; worst pacing sleep {max_pacing:?} for a {PACING:?} \
          request over {fed} iterations)"
    );
}
