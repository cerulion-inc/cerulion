// SPDX-License-Identifier: AGPL-3.0-only
//! The viz worker's live-reconnect orchestration.
//!
//! The Rerun 0.34 gRPC client does NOT auto-reconnect after a server bounce
//! (`ClientConnectionState::Disconnected` is terminal), so a bounced viz server
//! used to require a full graph restart. The [`cerulion_viz::worker::VizLogWorker`]
//! probes the sink on a timer and, on a genuine disconnect, swaps a fresh sink
//! and re-arms the scene-statics guards so the bounced (empty) server
//! re-receives the whole scene.
//!
//! This pins the orchestration deterministically WITHOUT a real bouncing gRPC
//! server, via injected hooks ([`ReconnectHooks::with_hooks`]): a fake probe
//! that reports the health we want + a fake reconnect that records its calls.
//! The scene-statics re-arm is proven behaviourally: with reconnects firing, the
//! (memory) sink KEEPS receiving statics (the guards were re-armed); a healthy
//! probe never reconnects and the statics are logged exactly once. The two parts
//! run in ONE `#[test]` body because the once-guards are process-global — one
//! guard-tripper at a time (no intra-binary race).
//!
//! (The pure reconnect DECISION — `Failed` ⇒ reconnect, `Timeout`/`Ok` ⇒ not —
//! is oracle-tested inline in `worker.rs`; this exercises the live loop.)

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::codegen::FrameWalker;
use cerulion_viz::sink::SinkState;
use cerulion_viz::worker::{ReconnectHooks, VizLogWorker};
use rerun::sink::SinkFlushError;

fn memory() -> (rerun::RecordingStream, rerun::sink::MemorySinkStorage) {
    rerun::RecordingStreamBuilder::new("go2_test")
        .recording_id("viz_reconnect")
        .memory()
        .expect("memory sink")
}

/// Bounded poll: run `cond` every 10 ms until it returns true or `deadline`
/// elapses. Returns whether it converged.
fn wait_until(deadline: Duration, mut cond: impl FnMut() -> bool) -> bool {
    let start = Instant::now();
    while start.elapsed() < deadline {
        if cond() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    cond()
}

#[test]
fn worker_reconnects_on_disconnect_and_re_logs_statics_but_not_when_healthy() {
    // --- PART A (control): a HEALTHY probe (Ok) → NEVER reconnect. ---
    {
        cerulion_viz::stream::reset_for_test(); // clean guard slate
        let (rec, _storage) = memory();
        let (walker, _w) = FrameWalker::new(Vec::new());
        let calls = Arc::new(AtomicU64::new(0));
        let c = Arc::clone(&calls);
        let hooks = ReconnectHooks::with_hooks(
            |_rec| Ok(()),
            move |_rec| {
                c.fetch_add(1, Ordering::Relaxed);
                Ok(())
            },
        );
        let worker = VizLogWorker::spawn_with_hooks(
            rec,
            walker,
            SinkState::new(),
            hooks,
            Duration::from_millis(10),
        )
        .expect("spawn worker");
        // Several probe intervals pass with a healthy sink.
        std::thread::sleep(Duration::from_millis(150));
        assert_eq!(
            calls.load(Ordering::Relaxed),
            0,
            "a healthy sink (probe Ok) must NEVER be reconnected"
        );
        assert_eq!(worker.reconnects(), 0);
        drop(worker); // drain + join
    }

    // --- PART B: a Failed probe → reconnect + statics re-log. ---
    {
        cerulion_viz::stream::reset_for_test(); // clean guard slate (re-arm the once-guards)
        let (rec, storage) = memory();
        let rec_flush = rec.clone(); // Arc-backed: flush the same stream the worker owns
        let (walker, _w) = FrameWalker::new(Vec::new());
        let calls = Arc::new(AtomicU64::new(0));
        let c = Arc::clone(&calls);
        let hooks = ReconnectHooks::with_hooks(
            |_rec| Err(SinkFlushError::failed("test: server bounced")),
            move |_rec| {
                c.fetch_add(1, Ordering::Relaxed);
                Ok(()) // fake swap succeeds without touching the memory sink
            },
        );
        let worker = VizLogWorker::spawn_with_hooks(
            rec,
            walker,
            SinkState::new(),
            hooks,
            Duration::from_millis(10),
        )
        .expect("spawn worker");

        // The worker's first `ensure_setup` logs the scene statics once.
        assert!(
            wait_until(Duration::from_secs(3), || {
                rec_flush.flush_blocking().expect("flush");
                storage.num_msgs() > 0
            }),
            "the initial scene statics were logged"
        );
        rec_flush.flush_blocking().expect("flush");
        let after_initial_setup = storage.num_msgs();

        // With reconnects firing, the guards are re-armed each time, so the
        // memory sink KEEPS receiving statics (num_msgs grows past the initial
        // setup) — the behavioural proof that `rearm_after_reconnect` ran. A
        // plateau would mean the guards were NOT re-armed (statics logged once).
        //
        // The wait gates on the WORKER's counter, not the test hook's.
        // `calls` is bumped INSIDE the hook and `counters.reconnects` strictly after
        // it returns (`worker.rs`, the `Ok(())` arm of `match (hooks.reconnect)(rec)`),
        // so `calls >= 3` can hold while `worker.reconnects() == 2` — a wait covering
        // a strict SUBSET of what the assertion below reads (the family).
        // `calls` stays in the message as the diagnostic that separates the two.
        assert!(
            wait_until(Duration::from_secs(3), || {
                rec_flush.flush_blocking().expect("flush");
                worker.reconnects() >= 3 && storage.num_msgs() > after_initial_setup
            }),
            "reconnects must fire (>=3) AND statics must re-log after re-arm \
             (num_msgs grew past the initial {after_initial_setup}; calls={}, \
             num_msgs={})",
            calls.load(Ordering::Relaxed),
            storage.num_msgs()
        );
        assert!(
            worker.reconnects() >= 3,
            "the worker's reconnect counter tracks the swaps ({} < 3)",
            worker.reconnects()
        );
        drop(worker);
    }
}

/// Strip Rust comments so a source probe cannot be satisfied by PROSE.
///
/// Handles `//`-to-end-of-line and `/* … */` blocks (depth-tracked, because Rust
/// block comments NEST). String literals are deliberately NOT modelled — this
/// file holds the probe words as literals, which is exactly why it never scans
/// ITSELF. (Crib: `cerulion_core`'s `cdylib_iox2_log_level_test::code_only`.)
fn code_only(src: &str) -> String {
    let bytes = src.as_bytes();
    let mut out = String::with_capacity(src.len());
    let (mut i, mut depth) = (0usize, 0usize);
    while i < bytes.len() {
        if depth == 0 && bytes[i..].starts_with(b"//") {
            while i < bytes.len() && bytes[i] != b'\n' {
                i += 1;
            }
            continue;
        }
        if bytes[i..].starts_with(b"/*") {
            depth += 1;
            i += 2;
            continue;
        }
        if depth > 0 && bytes[i..].starts_with(b"*/") {
            depth -= 1;
            i += 2;
            continue;
        }
        if depth == 0 {
            out.push(bytes[i] as char);
        }
        i += 1;
    }
    out
}

/// The worker's reconnect arm must reset EVERY per-viewer belief — the
/// `/tf_static` re-broadcast dedup AND the live marker set.
///
/// **Scope: this reads SOURCE, and it does so because the behaviour is
/// unobservable from outside.** `VizLogWorker` takes its `SinkState` BY VALUE and
/// exposes no accessor, so no test can watch the reconnect arm mutate it — which
/// is precisely the half a behavioural test on the method cannot reach. What
/// `reset_marker_state` DOES is pinned behaviourally by
/// `marker_array_test::a_reconnect_resets_the_live_marker_set_...`; what this
/// pins is that the reconnect arm CALLS it. (Same reason and same shape as
/// `cerulion_core`'s `every_hand_written_cdylib_init_applies_the_iox2_log_level`
/// source walk.)
///
/// It matters because the failure is SILENT and delayed: with the call missing, a
/// reconnected viewer starts empty while the sink still believes every marker is
/// live, so those markers are never re-drawn and a later `DELETEALL` emits clears
/// for entities the fresh server never saw.
#[test]
fn the_reconnect_arm_resets_every_per_viewer_belief() {
    let src = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/src/worker.rs"))
        .expect("read worker.rs");
    let code = code_only(&src);
    let anchor = code
        .find("rearm_after_reconnect();")
        .expect("the reconnect arm still calls stream::rearm_after_reconnect");
    // The whole arm is a handful of statements; bound the window generously and
    // require BOTH resets inside it (not merely somewhere in the file).
    let window = &code[anchor..(anchor + 600).min(code.len())];
    for required in ["clear_rebroadcast_dedup()", "reset_marker_state()"] {
        assert!(
            window.contains(required),
            "the reconnect arm must call {required}; window was:\n{window}"
        );
    }
    // ANTI-TAUTOLOGY: the probe really is comment-blind, so a commented-out call
    // could never satisfy it.
    assert_eq!(
        code_only("a /* x // y */ b // z\nc"),
        "a  b \nc",
        "code_only must strip BOTH comment syntaxes and nothing else"
    );
    // The needle here MUST be the same string the probe above requires. A rename
    // of the method once changed only the literal, leaving the
    // `.contains` looking for the OLD name — which made this assert trivially
    // true and the guard unable to fire. Both halves now derive from one binding.
    let call = "reset_marker_state()";
    assert!(
        !code_only(&format!("// state.{call};")).contains(call),
        "a commented-out call must not satisfy the probe"
    );
    assert!(
        code_only(&format!("state.{call};")).contains(call),
        "...but a REAL call must, or the probe above proves nothing"
    );
}
