// SPDX-License-Identifier: AGPL-3.0-only
//! HARD-GATE: the hosted proxy's LIVE path must not BUFFER image-class
//! frames behind a viewer that cannot keep up.
//!
//! The live-only-history change killed the proxy's per-client REPLAY history (`drop_temporal_history`
//! — a fresh viewer gets the scene skeleton and zero temporal backlog), and
//! `live_only_history_test.rs` pins it. This file covers the OTHER buffer on the
//! same path, which that work did not touch and could not reach: the **live
//! broadcast queue** every ALREADY-CONNECTED viewer reads from.
//!
//! `re_grpc_server`'s event loop hands every message to a byte-quota'd broadcast
//! channel (`CHANNEL_SIZE_MESSAGES` = 1024 messages, `CHANNEL_SIZE_BYTES` =
//! 128 MiB) and AWAITS space when it is full. Those are the crate's private
//! constants, not `ServerOptions` knobs. For a 30 Hz camera decoded to raw RGB8
//! (the desk decodes the frame and logs a `rerun::Image`; a 1280x720 frame is
//! 2.6 MiB) a viewer that renders slower than the robot publishes accumulates
//! frames there — measured at 27.8 MB / 10.1 frames in 2.4 s without the drop, headed for
//! ~47 frames ≈ 1.6 s of stale video it must play through before it shows the
//! present. That is the reported symptom ("h264 topic in the shell is still
//! really laggy") surviving a decode fix that made the decode itself
//! sub-millisecond.
//!
//! The decision, verbatim: *"we want no history wherever possible
//! (ofc things like plots should only start showing history etc when you start
//! vizing them, but not before). so there shouldnt be a buffer for clouds or
//! images etc."*
//!
//! # The oracle
//!
//! The proxy's OWN accounting, not a modelled consumer:
//! `MessageProxyHandle::capture_memory` reports a `MemUsageTree` whose children
//! are `broadcast` (bytes in the live queue), `live_dropped` (the running
//! total of dropped temporal bytes) and the `disposable` / `static` /
//! `persistent` history counts. A receiver that never drains is the SLOWEST
//! possible viewer, which is the true worst case and needs no sleep-tuning to
//! reproduce.
//!
//! # Arms
//!
//! | Test | Pins |
//! |---|---|
//! | `a_slow_viewer_does_not_accumulate_a_backlog_of_image_frames` | HEADLINE — the live queue stays within budget, AND drops really happened (anti-tautology) |
//! | `a_viewer_that_keeps_up_loses_nothing` | the CONTROL — a healthy stream drops zero, so the fix is not "drop always" |
//! | `a_fresh_viewer_still_gets_its_scene_while_the_live_queue_is_over_budget` | the SAFETY property — a skeleton message sent WHILE the budget is engaged still crosses |
//! | `a_plot_sample_survives_a_queue_full_of_camera_frames` | the SMALL-MESSAGE FLOOR e2e — control traffic is not collateral of an image backlog (decided by stream ORDER against a static sentinel, not by a delivery deadline) |
//! | `the_replay_history_still_holds_no_temporal_frames` | the `drop_temporal_history` contract in BYTES on the same stimulus: the backlog cannot be moved between the two buffers |
//! | `a_stream_of_small_messages_cannot_wedge_the_proxy` | LIVENESS across a burst past the 1024-message quota (see its own scope note — it does not drive the message-axis gate) |
//! | `the_budget_admits_a_whole_camera_frame_with_headroom` | sizing drift guard for the camera rendition (NOT the largest message vizd can emit — see `host.rs`) |
//! | `the_small_message_floor_sits_between_control_and_image_traffic` | the floor's sizing, MEASURED through the proxy's own accounting |
//! | `measure_*` (`#[ignore]`d) | print-only reproduction numbers |
//!
//! # This file really is a gate
//!
//! This file was added to the then-named `viz-tf-source-e2e` CI job, which ran two
//! named test files. Before that NO CI job ran any `cerulion_vizd` test —
//! `cargo clippy --workspace --all-targets` compiles them, so a compile break
//! was caught but a failing assertion was not. **A later CI change closed the rest of that
//! gap:** the `viz-tests` job now runs the WHOLE `cerulion_viz` +
//! `cerulion_vizd` suite on Linux and macOS. This file rides the vizd lane,
//! which is `--test-threads=1` — and its reason is one of the three that make
//! that lane serial: each arm hosts a real gRPC message proxy and streams
//! megabytes through it, so concurrent arms would make the byte-occupancy oracle
//! contend for CPU with its own siblings.

use std::collections::HashSet;
use std::net::{Ipv4Addr, SocketAddr, TcpListener};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use re_log_types::{LogMsg, SetStoreInfo, StoreId, StoreInfo, StoreKind, StoreSource};

use cerulion_vizd::host::LIVE_TEMPORAL_BUDGET_BYTES;
use re_grpc_server::LIVE_SMALL_MESSAGE_FLOOR_BYTES;

/// The two renditions the Go2 front camera interleaves on one topic
/// (`examples/go2/nodes/camera_jpeg/src/h264.rs` `KNOWN_RENDITION_HEIGHTS`), so a
/// single attach decodes and logs BOTH sizes every frame.
const RENDITIONS: [(u32, u32); 2] = [(640, 360), (1280, 720)];

/// The camera's per-rendition frame rate.
const FPS: f64 = 30.0;

/// Frames pushed per run. At ~30 Hz this is ~2 s — long enough that an undropped
/// queue is measurably climbing (10.1 frames at 2.4 s) while the whole test
/// stays fast.
const FRAMES: usize = 60;

fn rgb_bytes(w: u32, h: u32) -> usize {
    (w as usize) * (h as usize) * 3
}

/// Frames the UNDRAINED `run_stream` will feed before giving up on the budget
/// engaging.
///
/// A CEILING, not a target, set against the same worst case as
/// [`FLOOD_FRAME_CEILING`] and for the same reason: engagement costs ~52-55
/// frames when the receiver's forwarding task keeps perfect pace and absorbs its
/// whole 128 MiB sink first, and ~12-16 when it does not. The frame count
/// engagement needs is a property of the FORK's constants (a 128 MiB sink, an
/// 8 MiB budget, a ~2.78 MB encoded frame), not of the machine — so a run that
/// hits this should be read as one of those having moved, never as a slow runner.
const RUN_STREAM_FRAME_CEILING: usize = 300;

/// Liveness backstop for the same loop — a CEILING in seconds against ~2 s of
/// measured work (60 frames on a 33 ms cadence), never a threshold anything is
/// decided by. The 33 ms sleep is deliberate here (it is what lets the forwarding
/// task keep pace, which is the SLOW engagement path this bound must survive), so
/// this is generous against `RUN_STREAM_FRAME_CEILING` frames of it.
const RUN_STREAM_DEADLINE: Duration = Duration::from_secs(120);

/// An INCOMPRESSIBLE pixel buffer (xorshift-filled).
///
/// Load-bearing, not decoration: the wire payload is compressed, and an earlier
/// constant-filled buffer measured 5% of its raw size — which silently reported a
/// 2.6 MiB frame as a 140 KiB one and made the queue look empty. Camera pixels
/// are not compressible like that, so a synthetic buffer must not be either.
fn incompressible(n: usize, seed: u64) -> Vec<u8> {
    let mut s = seed | 1;
    (0..n)
        .map(|_| {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            (s >> 24) as u8
        })
        .collect()
}

fn probe_free_addr() -> SocketAddr {
    let l = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).expect("probe a free loopback port");
    let addr = l.local_addr().expect("probe addr");
    drop(l);
    addr
}

fn proxy_uri(addr: SocketAddr) -> re_uri::ProxyUri {
    re_uri::ProxyUri::new(re_uri::Origin::from_scheme_and_socket_addr(
        re_uri::Scheme::RerunHttp,
        addr,
    ))
}

fn set_store_info(store_id: &StoreId) -> LogMsg {
    LogMsg::SetStoreInfo(SetStoreInfo {
        row_id: *re_chunk::RowId::new(),
        info: StoreInfo::new(
            store_id.clone(),
            StoreSource::RustSdk {
                rustc_version: String::new(),
                llvm_version: String::new(),
            },
        ),
    })
}

/// A STATIC frame — part of the scene skeleton, so never eligible for the
/// live-budget drop.
fn static_frame(store_id: &StoreId, entity: &str) -> LogMsg {
    let chunk = re_chunk::Chunk::builder(entity)
        .with_archetype(
            re_chunk::RowId::new(),
            re_log_types::TimePoint::STATIC,
            &rerun::archetypes::Points2D::new([(0.0, 0.0), (1.0, 1.0)]),
        )
        .build()
        .expect("build static chunk");
    LogMsg::ArrowMsg(store_id.clone(), chunk.to_arrow_msg().expect("to arrow"))
}

/// The ORDER SENTINEL frame: a STATIC chunk, deliberately SMALL.
///
/// Wrapping [`static_frame`] rather than calling it directly is the whole point.
/// The sentinel's job is to be the one message in these arms that the live budget
/// can NEVER shed, and it holds that on the guard genuinely INDEPENDENT of the one
/// under test: `is_temporal` is false for a static, so `decide_live` returns Admit
/// before either axis is consulted. It is ALSO kept under
/// [`LIVE_SMALL_MESSAGE_FLOOR_BYTES`], but only as defence in depth on the BYTE
/// axis (that floor is itself the guard under test, and the fork's MESSAGE axis
/// exempts nothing), for the case where `is_temporal` misclassified it. Its
/// neighbour [`big_static_frame`] is sized OVER that floor on purpose, for the
/// opposite reason (it is a probe that must be droppable if misclassified), so
/// "harmonising" the sentinel onto it would quietly turn a liveness backstop into
/// a second thing under test. This function is where that must not happen.
fn order_sentinel(store_id: &StoreId, entity: &str) -> LogMsg {
    static_frame(store_id, entity)
}

/// Points enough to clear [`LIVE_SMALL_MESSAGE_FLOOR_BYTES`] with room to spare.
///
/// Load-bearing wherever a message must be DROPPABLE if it were misclassified:
/// the floor exempts sub-floor messages from the byte axis, so a small skeleton
/// probe would be delivered even if it were classified temporal, and
/// the arm asserting on it would be vacuous for a new reason. MEASURED through
/// the proxy's own accounting by
/// `the_small_message_floor_sits_between_control_and_image_traffic`.
const PROBE_POINTS: usize = 5000;

/// A STATIC frame big enough to be droppable if the `is_static` guard were gone —
/// the probe the fresh-viewer arm sends WHILE the budget is engaged.
fn big_static_frame(store_id: &StoreId, entity: &str) -> LogMsg {
    let chunk = re_chunk::Chunk::builder(entity)
        .with_archetype(
            re_chunk::RowId::new(),
            re_log_types::TimePoint::STATIC,
            &rerun::archetypes::Points3D::new(
                (0..PROBE_POINTS)
                    .map(|i| (i as f32, 0.0, 0.0))
                    .collect::<Vec<_>>(),
            ),
        )
        .build()
        .expect("build big static chunk");
    LogMsg::ArrowMsg(store_id.clone(), chunk.to_arrow_msg().expect("to arrow"))
}

/// A BLUEPRINT chunk big enough to be droppable if the blueprint guard were gone.
///
/// The archetype is irrelevant to the guard under test — `is_temporal` reads only
/// the store KIND and the `is_static` flag, never the payload — so this is sized
/// to clear the floor rather than shaped to look like a real Studio layout. It
/// carries a TEMPORAL timepoint deliberately: a static one would be exempt for
/// the OTHER reason and could not discriminate the blueprint guard.
fn big_blueprint_chunk(store_id: &StoreId) -> LogMsg {
    let tp = re_log_types::TimePoint::default().with(
        re_log_types::Timeline::new_sequence("blueprint"),
        re_log_types::TimeInt::new_temporal(1),
    );
    let chunk = re_chunk::Chunk::builder("viewport")
        .with_archetype(
            re_chunk::RowId::new(),
            tp,
            &rerun::archetypes::Points3D::new(
                (0..PROBE_POINTS)
                    .map(|i| (i as f32, 1.0, 0.0))
                    .collect::<Vec<_>>(),
            ),
        )
        .build()
        .expect("build blueprint chunk");
    LogMsg::ArrowMsg(store_id.clone(), chunk.to_arrow_msg().expect("to arrow"))
}

/// Frames one pressure phase will feed before giving up on the gate dropping.
///
/// A CEILING, not a target — the same shape as [`FLOOD_FRAME_CEILING`] and set
/// against the same worst case. Nothing in these arms ever CONSUMES, so the
/// receiver's forwarding task fills its own 128 MiB sink (`max_bytes_on_wire`)
/// and then stops pulling, after which every delivered frame stays resident in
/// the broadcast and a drop is arithmetically inevitable. At 2_779_552 encoded
/// bytes a frame that is 48.3 frames of absorption from COLD — and a pressure
/// phase never starts from cold, because the flood has already paid part of it
/// (MEASURED on this desk across loaded and idle runs, engagement lands anywhere
/// from 15 to 56 frames, so the worst case a phase inherits is ~33 frames of
/// remaining sink headroom).
///
/// 120 is ~2.5x the cold worst case, ~3.4x that inherited worst case, and ~25x
/// what a phase has actually been measured to need (3-5 frames, 70-90 ms).
const PRESSURE_FRAME_CEILING: usize = 120;

/// Liveness backstop for the same loop — a CEILING in seconds against ~0.1 s of
/// measured work, never a threshold anything is decided by. Load moves WHEN the
/// gate drops; it cannot move WHETHER it does (see [`Pressure::until_dropping`]).
///
/// **It is the wall the loop actually spends, and that needed enforcing.** A
/// deadline checked only at the TOP of an iteration bounds when the loop stops
/// STARTING work, not when it stops doing it: an iteration admitted with a
/// millisecond left could then spend its whole flush bound plus its whole
/// snapshot bound, taking the phase past the number its own name advertises. A
/// ceiling that lies is the load-sensitive class in miniature, so an iteration that
/// cannot fit in what is LEFT is not entered at all.
const PRESSURE_DEADLINE: Duration = Duration::from_secs(20);

/// The two BLOCKING calls one iteration of the pressure loop can sit in.
///
/// Both are LIVENESS backstops against work measured in MILLISECONDS (a phase
/// confirms in 3-5 frames / 70-90 ms), never pacing thresholds — a flush that
/// does not ack inside five seconds is counted and the loop simply continues,
/// so shortening these cannot change a verdict, only how long a stalled
/// iteration waits before giving up on it.
const PRESSURE_FLUSH_BOUND: Duration = Duration::from_secs(5);
const PRESSURE_SNAPSHOT_BOUND: Duration = Duration::from_secs(5);

/// The most one iteration can spend, and therefore the headroom
/// [`Pressure::until_dropping`] requires before entering one.
///
/// Derived from the two bounds rather than restated, so it cannot drift from
/// them.
const PRESSURE_ITERATION_BUDGET: Duration =
    Duration::from_secs(PRESSURE_FLUSH_BOUND.as_secs() + PRESSURE_SNAPSHOT_BOUND.as_secs());

/// An iteration must FIT inside the phase deadline, or the loop could never
/// enter one and every phase would report zero frames fed.
const _: () = assert!(
    PRESSURE_ITERATION_BUDGET.as_secs() < PRESSURE_DEADLINE.as_secs(),
    "one pressure iteration must fit inside PRESSURE_DEADLINE with room to enter it"
);

/// What one pressure phase observed on its way to the gate dropping again.
#[derive(Debug)]
struct Pressed {
    /// Whether the proxy's OWN accounting reported a further WHOLE camera frame
    /// dropped — the state this phase exists to establish, and the only thing
    /// its caller may assert on.
    confirmed: bool,
    /// Frames fed. Each was flushed, so this is also (up to a constant) the
    /// number the proxy INGESTED — the distinction this helper exists for.
    frames: usize,
    /// Flushes that did not ack. Reported, never asserted on.
    flush_failures: usize,
    /// Wall spent.
    elapsed: Duration,
    /// The last reading taken.
    last: Usage,
}

/// The camera stimulus one arm presses with — the pieces that do not vary across
/// its pressure phases, bundled so a phase call takes only what CHANGES.
///
/// Clippy's argument ceiling forced the bundle (the free function reached 8/7),
/// and it retires a real hazard on the way, exactly as `RecordingInputSpec` did
/// for the recorder: `w`/`h` are adjacent `u32`s and `tag`/`dropped_before` adjacent
/// integers, so a transposition at a call site would compile and quietly press
/// the wrong shape. Now the fixture is set once per arm and each phase names two
/// things.
struct Pressure<'a> {
    prod: &'a re_grpc_client::Client,
    handle: &'a re_grpc_server::MessageProxyHandle,
    store: &'a StoreId,
    rgb: &'a [u8],
    w: u32,
    h: u32,
}

impl Pressure<'_> {
    /// Feed camera frames — one at a time, each flushed — until the proxy's own
    /// accounting reports a further WHOLE camera frame DROPPED, or the bounds
    /// expire.
    ///
    /// `tag` is the wire sequence this phase starts numbering its frames from;
    /// callers space them so two phases of one arm cannot collide.
    ///
    /// # Why this is not a fixed burst any more
    ///
    /// It was — six unpaced `send_blocking` calls, then a passive poll of the drop
    /// counter — and that shape fired
    /// `a_fresh_viewer_still_gets_its_scene_while_the_live_queue_is_over_budget`
    /// twice in CI, reporting
    /// `dropped 2779553 -> 2779553` after twenty seconds of polling: the flood had
    /// ENGAGED (`engaged=true after 18 frames`) and then twelve further frames plus
    /// the probes produced NOT ONE further drop.
    ///
    /// The retired burst's own doc argued it was load-monotone because "a burst with
    /// no sleeps pegs the queue in the OVER-budget phase for as long as it is in
    /// flight". That is true of a burst that is IN FLIGHT, and the burst was six
    /// `send_blocking` calls, which only ENQUEUE — into a client command queue
    /// bounded at 100 MESSAGES with no byte term, so all six fit and the test thread
    /// walked straight past while the client's background encoder delivered them
    /// however it managed. On a loaded runner that is not a burst at the proxy at
    /// all; it is a trickle, and between two trickled frames the receiver's
    /// forwarding task drains the broadcast back under budget and admits every one
    /// of them.
    ///
    /// That last step is MEASURED, not inferred. The CI failure reproduces
    /// deterministically under 32 busy-loop processes on a 16-core desk, and the
    /// reading it fails on is `Usage { broadcast: 0, live_dropped: 2779551,
    /// persistent: 21830 }` — the queue is not merely under budget, it is EMPTY, so
    /// the forwarder's own sink still had room and the flood had only left it
    /// BEHIND rather than full. `persistent` moving from 212 to 21830 in the same
    /// reading is the probes landing: they crossed a queue with nothing in it, which
    /// is exactly the vacuity the precondition exists to forbid.
    ///
    /// So the retired precondition was a claim about the STIMULUS (twelve frames
    /// were pushed) standing in for a claim about the SUBJECT (the gate was
    /// dropping), and a slow runner INVERTED it. That is the flake whose report
    /// named this file as carrying the precondition twice.
    ///
    /// # Why the replacement cannot be inverted by load
    ///
    /// It runs until the SUBJECT says so. Nothing here ever consumes, so total
    /// buffered bytes are MONOTONE NON-DECREASING in frames delivered: the sink
    /// fills, stops pulling, and from there the broadcast only climbs — so a drop is
    /// not raced for, it is arithmetic. Load changes the RATE at which this loop
    /// advances and never the DIRECTION, exactly as
    /// [`flood_until_budget_engages`] argues for its own loop, and for the same
    /// reason: the per-frame `flush_blocking` is what makes SENT == DELIVERED, so an
    /// iteration is a frame the proxy really took rather than one the client still
    /// holds.
    ///
    /// The caller owns the assertion and its message; every field here is reported
    /// so a failure can be attributed rather than merely announced.
    fn until_dropping(&self, tag: i64, dropped_before: u64) -> Pressed {
        let start = Instant::now();
        let want = dropped_before.saturating_add(self.rgb.len() as u64);
        let mut p = Pressed {
            confirmed: false,
            frames: 0,
            flush_failures: 0,
            elapsed: Duration::ZERO,
            last: Usage::default(),
        };
        while p.frames < PRESSURE_FRAME_CEILING {
            // An iteration that cannot FIT in what is left is not entered, so the
            // wall this phase advertises is the wall it actually spends. Checking
            // only that the deadline has not passed would admit an iteration with
            // a millisecond left and then let it block for the whole iteration
            // budget.
            let left = PRESSURE_DEADLINE
                .checked_sub(start.elapsed())
                .unwrap_or_default();
            if left < PRESSURE_ITERATION_BUDGET {
                break;
            }
            // `send_blocking` cannot block here: it enqueues into a 100-message
            // client queue that the per-frame flush below leaves with at most one
            // message in it.
            self.prod.send_blocking(image_frame(
                self.store,
                "cam/video/rendition",
                tag + p.frames as i64,
                self.rgb,
                self.w,
                self.h,
            ));
            p.frames += 1;
            if self.prod.flush_blocking(PRESSURE_FLUSH_BOUND).is_err() {
                p.flush_failures += 1;
            }
            p.last = usage(self.handle, PRESSURE_SNAPSHOT_BOUND);
            if p.last.live_dropped >= want {
                p.confirmed = true;
                break;
            }
        }
        p.elapsed = start.elapsed();
        p
    }
}

/// The shared pressure precondition: the gate really was dropping whole camera
/// frames at the instant this phase ended, so a probe sent adjacent to it
/// crosses a gate that is actually deciding something.
///
/// Every exit is a FAILURE — this never converts a phase that did not drop into
/// a pass. What it adds over `assert!(p.confirmed)` is ATTRIBUTION, on the same
/// principle as [`assert_budget_engaged`].
fn assert_pressed_until_dropping(p: &Pressed, phase: &str, frame_bytes: usize) {
    if p.confirmed {
        return;
    }
    let why = if p.last.persistent == 0 {
        "the store handshake is not even in the proxy's accounting, so nothing this producer sent \
         reached it — the pipeline is broken upstream of the gate"
            .to_owned()
    } else if p.frames >= PRESSURE_FRAME_CEILING {
        format!(
            "it fed the whole {PRESSURE_FRAME_CEILING}-frame ceiling without the gate dropping a \
             further frame. Nothing drains these arms, so the receiver's 128 MiB forwarding sink \
             fills after ~48 frames and every frame after that is pure backlog — if that many \
             frames can still be absorbed, either the fork's sink or the encoded frame size moved"
        )
    } else {
        format!(
            "it spent its {PRESSURE_DEADLINE:?} liveness bound (it stops entering iterations \
             with under {PRESSURE_ITERATION_BUDGET:?} left, so the wall it advertises is the \
             wall it spends) after only {} frames, {} of whose flushes did not ack — so frames \
             were SENT and not DELIVERED, a starved stimulus rather than a gate that declined \
             to fire",
            p.frames, p.flush_failures
        )
    };
    panic!(
        "precondition: the gate must have been actively dropping {phase} — otherwise the probes \
         are admitted without `is_temporal` ever being consulted, which is exactly the vacuity \
         this arm exists to avoid. {why}. One camera frame is {frame_bytes} bytes; this phase \
         wanted the proxy's own dropped total to advance by at least that much. {p:?}"
    );
}

/// Frames the flood will feed before giving up on the live budget engaging.
///
/// A CEILING, not a target, and it is set against the WORST case rather than the
/// observed one because those differ by 4x and only one of them bounds anything.
/// MEASURED, this loop engages at **12-16 frames** (see
/// `flood_until_budget_engages` for the whole distribution). The worst case is
/// **~52-55**, reached when the receiver's forwarding task keeps perfect pace and
/// absorbs its entire 128 MiB sink before any backlog can exist — that is not a
/// hypothetical, it is exactly what the retired `sleep(33ms)` loop measured, and
/// a runner that schedules the forwarder generously reproduces it.
///
/// So 300 is ~5.5x the worst case and ~20x the observed. It is deliberately not
/// tighter: the frame count engagement needs is a property of the FORK's
/// constants (a 128 MiB sink, an 8 MiB budget, a 2_779_552-byte encoded frame),
/// not of the machine, so the only thing this bound has to survive is somebody
/// moving one of those — and a run that hits it should be read as exactly that,
/// never as a slow machine.
const FLOOD_FRAME_CEILING: usize = 300;

/// Liveness backstop for the same loop — a CEILING in seconds against ~0.1 s of
/// measured work (~2.3 s for the retired loop), never a threshold anything is
/// decided by. Load moves the engagement point in TIME; see
/// `flood_until_budget_engages` for why it cannot move it in DIRECTION. A bound
/// here has to be wall-generous or it is just the old frame cap wearing a clock.
const FLOOD_DEADLINE: Duration = Duration::from_secs(120);

/// What one flood of an UNDRAINED proxy observed on its way to the live budget.
#[derive(Debug)]
struct Flood {
    /// Whether the proxy's OWN accounting ever reported a temporal drop.
    engaged: bool,
    /// Frames fed. Each was flushed, so this is also (up to a constant) the
    /// number the proxy INGESTED — the distinction this whole helper exists for.
    frames: usize,
    /// Flushes that did not ack. Reported, never asserted on: a flush that times
    /// out under load is absorbed by the loop simply continuing, and the only
    /// question that matters is whether the budget engaged.
    flush_failures: usize,
    /// Wall spent.
    elapsed: Duration,
    /// Highest live-queue occupancy seen across the flood. The discriminator
    /// between "the stimulus never arrived" and "it arrived and the gate did not
    /// fire" — see `assert_budget_engaged`.
    peak_broadcast: u64,
    /// The last reading taken.
    last: Usage,
}

/// Feed camera frames into an UNDRAINED proxy until its own accounting reports
/// the live budget ENGAGED (`live_dropped > 0`).
///
/// # Why this is not a `for seq in 0..FRAMES` loop any more
///
/// It was, and that cost main a red `Viz tests (macos-latest)` (run 30767966485)
/// on `a_plot_sample_survives_a_queue_full_of_camera_frames`, reporting
/// `Usage { broadcast: 0, live_dropped: 0, disposable: 0, persistent: 212 }` —
/// every bucket at zero but the 212-byte store handshake, i.e. not a queue that
/// had drained but a queue that had never been fed. The `FRAMES`-bounded loop
/// was a LOAD-INVERTIBLE bound, and the margin it had was far thinner than it
/// looked.
///
/// MEASURED on this desk (debug build, 16 cores, load ~70), printing the proxy's
/// accounting at every frame of the old loop: `broadcast` is EXACTLY 0 through
/// frame 50, then climbs `2_779_552` -> `5_559_104` -> `8_338_656` ->
/// `11_118_208`, and the first drop lands on **frame 55 of the 60** the loop was
/// allowed. A margin of five frames — 8% — not the comfortable headroom the
/// constant's name suggests. (That `8_338_656` is the same number the previous
/// hardening of this file, `20b3e5a68`, recorded CI going red on: the same queue,
/// the same phase, one threshold earlier in the chain.)
///
/// The flat 50 frames are not slack, they are a SINK: nothing in these arms ever
/// drains `spawn_with_recv`'s receiver, so its forwarding task pulls from the
/// broadcast into its own `re_log_channel` quota channel — `max_bytes_on_wire`,
/// **128 MiB** — and only once THAT is full does it stop pulling and let the
/// broadcast accumulate. At 2_779_552 encoded bytes a frame that is 48.3 frames
/// of pure absorption before the first byte of backlog exists, then ~4 more to
/// cross the 8 MiB budget. So engagement costs ~52-55 frames REGARDLESS of the
/// machine, and the old loop's 60 was a coin toss the moment delivery fell
/// behind.
///
/// And it does fall behind, because SENT and DELIVERED were different numbers.
/// `re_grpc_client`'s command queue is bounded at **100 messages** with no byte
/// term, so all 60 of those 2.7 MB frames fit in it and `send_blocking` never
/// blocked; the test thread walked its 60 iterations on a `sleep(33ms)` while the
/// client's background encoder — a current-thread runtime, LZ4-compressing
/// deliberately incompressible pixels in a debug build — delivered however many
/// it managed. On this desk that was enough. On a loaded macOS runner it was
/// under 48, which is precisely why `broadcast` read a literal 0.
///
/// # Why the replacement cannot be inverted by load
///
/// Nothing here ever CONSUMES. The receiver is never drained, so the 128 MiB sink
/// fills and stays full, and every frame delivered after that stays resident in
/// the broadcast. Total buffered bytes are therefore MONOTONE NON-DECREASING in
/// frames delivered: load changes the RATE at which the flood advances and never
/// the DIRECTION. A loop that runs until the subject itself reports engagement
/// can be DELAYED by a slow runner and cannot be INVERTED by one — which is the
/// property a fixed count of frames SENT could never have, since load widens the
/// gap between sent and delivered.
///
/// The per-frame `flush_blocking` is what closes that gap, and it REPLACES the
/// 33 ms sleep rather than joining it. That sleep was an APPROXIMATION of exactly
/// this — the arm it came from documents it as needed because "sending faster
/// than ~30 Hz just backs frames up INSIDE the client and none of them reach the
/// proxy" — and an approximation of "the frame arrived" is the thing that fails
/// when the runner is slow. A flush acks once every earlier message has been
/// yielded into the outbound stream, which the server only drains through its own
/// 32-deep event queue, so one flush per frame bounds the client's backlog at
/// O(1) instead of O(FRAMES) and makes an iteration of this loop a frame the
/// proxy really took.
///
/// # It also got 4x cheaper, and the reason is worth writing down
///
/// MEASURED, this loop engages at **12-16 frames in ~100-140 ms**, against the
/// old loop's 55 frames in ~2.3 s. That is not the flush being fast; it is the
/// SLEEP having been expensive in a way nobody was counting. 33 ms of idle per
/// frame is 33 ms in which the receiver's forwarding task can decode and drain,
/// so it kept up and swallowed all 128 MiB of its sink before one byte of backlog
/// survived. Racing it instead lets the broadcast accumulate an order of
/// magnitude sooner. Which of the two the runner delivers is a scheduling
/// question and this loop does not care: both are bounded, both are monotone, and
/// `FLOOD_FRAME_CEILING` is set against the slower one.
fn flood_until_budget_engages(
    prod: &re_grpc_client::Client,
    handle: &re_grpc_server::MessageProxyHandle,
    store: &StoreId,
    rgb: &[u8],
    w: u32,
    h: u32,
) -> Flood {
    let start = Instant::now();
    let mut flood = Flood {
        engaged: false,
        frames: 0,
        flush_failures: 0,
        elapsed: Duration::ZERO,
        peak_broadcast: 0,
        last: Usage::default(),
    };
    while flood.frames < FLOOD_FRAME_CEILING && start.elapsed() < FLOOD_DEADLINE {
        prod.send_blocking(image_frame(
            store,
            "cam/video/rendition",
            flood.frames as i64,
            rgb,
            w,
            h,
        ));
        flood.frames += 1;
        if prod.flush_blocking(Duration::from_secs(20)).is_err() {
            flood.flush_failures += 1;
        }
        flood.last = usage(handle, Duration::from_secs(5));
        flood.peak_broadcast = flood.peak_broadcast.max(flood.last.broadcast);
        if flood.last.live_dropped > 0 {
            flood.engaged = true;
            break;
        }
    }
    flood.elapsed = start.elapsed();
    println!(
        "live-budget flood: engaged={} after {} frames in {:?} (peak broadcast {} bytes, budget \
         {LIVE_TEMPORAL_BUDGET_BYTES}, {} flush failures) {:?}",
        flood.engaged,
        flood.frames,
        flood.elapsed,
        flood.peak_broadcast,
        flood.flush_failures,
        flood.last
    );
    flood
}

/// The shared precondition: the flood really did drive the live queue over
/// budget, so the probes the caller is about to send cross a gate that is
/// actually deciding something.
///
/// Every exit is a FAILURE — this never converts a flood that did not flood into
/// a pass. What it adds over `assert!(f.engaged)` is ATTRIBUTION, and the two
/// classes it separates want opposite responses:
///
///  * `peak_broadcast == 0` — not one byte of backlog ever existed, so the
///    stimulus never reached the proxy in the volume the sink needs. That is a
///    HARNESS/environment failure (the shape an earlier flake fired on), and the fix is
///    upstream of the gate.
///  * `peak_broadcast > budget` with no drop — the queue genuinely went over
///    budget and `decide_live` did not fire. That is a REGRESSION, the
///    thing this whole file exists to catch, and reporting it as "the flood was
///    starved" would send a reader hunting a flake that is not there.
fn assert_budget_engaged(f: &Flood) {
    if f.engaged {
        return;
    }
    let why = if f.frames == 0 {
        "the flood fed ZERO frames — it never ran at all".to_owned()
    } else if f.last.persistent == 0 {
        "the store handshake never even landed (`persistent` is 0), so nothing this producer \
         sent reached the proxy — the pipeline is broken upstream of the gate"
            .to_owned()
    } else if f.peak_broadcast == 0 {
        format!(
            "the live queue never held a single byte across all {} frames. Nothing drains it \
             here, so that is not a queue that emptied — it is one that was never filled. Backlog \
             first appears at frame ~12 measured, ~48 worst case (the receiver's 128 MiB \
             forwarding sink absorbs that many first), and neither happened, so the frames were \
             SENT and not DELIVERED ({} flushes did not ack). A starved stimulus, not a broken \
             gate",
            f.frames, f.flush_failures
        )
    } else if f.peak_broadcast > LIVE_TEMPORAL_BUDGET_BYTES {
        format!(
            "the live queue reached {} bytes — genuinely OVER the {LIVE_TEMPORAL_BUDGET_BYTES}-byte \
             budget — and the gate dropped NOTHING. Read this as a live-budget regression rather than \
             a flake: the stimulus did its job and `decide_live` did not",
            f.peak_broadcast
        )
    } else {
        format!(
            "the live queue peaked at {} bytes, under the {LIVE_TEMPORAL_BUDGET_BYTES}-byte budget, \
             across {} frames. Delivery was progressing but never got the queue over the line — if \
             the fork's 128 MiB receiver sink or the encoded frame size moved, {FLOOD_FRAME_CEILING} \
             frames may no longer be enough",
            f.peak_broadcast, f.frames
        )
    };
    panic!(
        "precondition: the flood must have driven the live queue over budget, else the probes \
         below cross a gate that admits everything and this arm proves nothing. {why}. \
         Bounds were {FLOOD_FRAME_CEILING} frames / {FLOOD_DEADLINE:?}, spent {} frames in {:?}. \
         {f:?}",
        f.frames, f.elapsed
    );
}

/// A SMALL temporal message — the shape of a robot's ordinary traffic (a plot
/// sample, a `/tf` update, a telemetry scalar): kilobytes, not megabytes. 1024 of
/// these reach the channel's MESSAGE quota at a few MiB, far under any sane byte
/// budget, which is the axis the image arms cannot exercise.
fn small_frame(store_id: &StoreId, entity: &str, seq: i64) -> LogMsg {
    let tp = re_log_types::TimePoint::default().with(
        re_log_types::Timeline::new_sequence("frame"),
        re_log_types::TimeInt::new_temporal(seq),
    );
    let chunk = re_chunk::Chunk::builder(entity)
        .with_archetype(
            re_chunk::RowId::new(),
            tp,
            &rerun::archetypes::Points2D::new([(seq as f32, 0.0), (0.0, seq as f32)]),
        )
        .build()
        .expect("build small chunk");
    LogMsg::ArrowMsg(store_id.clone(), chunk.to_arrow_msg().expect("to arrow"))
}

/// Build + arrow-encode ONE decoded picture exactly as the path does:
/// `rerun::Image::from_rgb24(rgb.to_vec(), [w, h])` into a chunk, then to the
/// wire `ArrowMsg` the SDK hands the sink.
fn image_frame(store_id: &StoreId, entity: &str, seq: i64, rgb: &[u8], w: u32, h: u32) -> LogMsg {
    let tp = re_log_types::TimePoint::default().with(
        re_log_types::Timeline::new_sequence("frame"),
        re_log_types::TimeInt::new_temporal(seq),
    );
    let chunk = re_chunk::Chunk::builder(entity)
        .with_archetype(
            re_chunk::RowId::new(),
            tp,
            // `to_vec()` is the production path's own copy (archetype.rs).
            &rerun::archetypes::Image::from_rgb24(rgb.to_vec(), [w, h]),
        )
        .build()
        .expect("build image chunk");
    LogMsg::ArrowMsg(store_id.clone(), chunk.to_arrow_msg().expect("to arrow"))
}

/// Run the body UNDER a multi-thread runtime (entered, not `block_on`) so the
/// server + client `tokio::spawn` find a runtime while the test thread stays free
/// to block. Mirrors `live_only_history_test.rs`.
fn with_runtime<R>(f: impl FnOnce() -> R) -> R {
    let rt = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()
        .expect("build tokio runtime");
    let _guard = rt.enter();
    f()
}

/// The proxy's byte accounting for one instant.
#[derive(Debug, Default, Clone, Copy)]
struct Usage {
    /// Bytes sitting in the LIVE broadcast queue.
    broadcast: u64,
    /// Running total of TEMPORAL bytes dropped for being over budget.
    live_dropped: u64,
    /// Retained REPLAY history (`drop_temporal_history` keeps this at 0 for temporal).
    disposable: u64,
    /// Retained skeleton.
    persistent: u64,
}

fn child_bytes(tree: &re_byte_size::MemUsageTree, label: &str) -> Option<u64> {
    match tree {
        re_byte_size::MemUsageTree::Bytes(_) => None,
        re_byte_size::MemUsageTree::Node(node) => node
            .children()
            .iter()
            .find(|c| c.name == label)
            .map(|c| c.size_bytes()),
    }
}

/// `capture_memory()` REQUESTS a snapshot from the event loop and returns the
/// LATEST one it has produced — `MemUsageTree::default()` (a childless
/// `Bytes(0)`) until the loop has handled at least one request. So a caller must
/// POLL for the first real snapshot; reading a childless tree as "zero bytes
/// buffered" would make every assertion below silently vacuous.
///
/// Panics rather than returning a default if no snapshot arrives within `bound`,
/// because that means the event loop is not running — itself a failure, never a
/// reading of zero.
fn usage(handle: &re_grpc_server::MessageProxyHandle, bound: Duration) -> Usage {
    let deadline = Instant::now() + bound;
    loop {
        if let Some(tree) = handle.capture_memory() {
            if let (Some(broadcast), Some(live_dropped), Some(disposable), Some(persistent)) = (
                child_bytes(&tree, "broadcast"),
                child_bytes(&tree, "live_dropped"),
                child_bytes(&tree, "disposable"),
                child_bytes(&tree, "persistent"),
            ) {
                return Usage {
                    broadcast,
                    live_dropped,
                    disposable,
                    persistent,
                };
            }
        }
        assert!(
            Instant::now() < deadline,
            "the proxy event loop did not produce a memory snapshot within {bound:?} — it is not \
             running (wedged?), so no measurement below would mean anything"
        );
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// What one run observed.
struct Run {
    /// Highest live-queue occupancy seen during the run.
    peak_broadcast: u64,
    /// Final accounting.
    last: Usage,
    /// Per-frame live-queue occupancy.
    series: Vec<u64>,
    /// The frame size used.
    frame_bytes: usize,
    /// The LARGEST thing the live queue ACCOUNTS for any ONE frame this run
    /// streamed — the encoded, LZ4-compressed proto, which is the quantity
    /// `decide_live` compares against the budget and is ~0.5% LARGER than
    /// `frame_bytes` (the shed-starvation probe D measurement).
    ///
    /// A RUNNING MAXIMUM over the frames actually sent, never a measurement of one
    /// of them, and the reason is that the size is **not monotone in sequence**.
    /// Each frame carries its own timeline sequence AND a fresh random `RowId`,
    /// both of which ride the compressed arrow buffer, so the accounted size
    /// JITTERS by a handful of bytes frame to frame. MEASURED over one 60-frame
    /// run of this fixture: 2_779_471 at seq 0, a maximum of 2_779_483 at **seq
    /// 17**, and back down to 2_779_474 by seq 41 — a 12-byte spread whose peak is
    /// neither the first frame nor the last.
    ///
    /// So neither shortcut is sound: pricing the term at frame 0 (or at the run's
    /// highest sequence) can land BELOW a message the gate really did admit, and
    /// the ceiling is an upper bound, so being short by any amount is being wrong
    /// in the only direction that matters. Only a max over what was really sent
    /// bounds every admissible message.
    ///
    /// Measured from the SAME message object the run sends (built once, priced,
    /// then handed to `send_blocking`), so it is this run's own frame rather than
    /// a reconstruction of one.
    encoded_frame_bytes: u64,
    /// Messages the receiver actually consumed (0 when undrained).
    drained_count: usize,
}

/// Stream `FRAMES` image frames of the given rendition through the REAL
/// production proxy, with the in-process receiver either drained each iteration
/// (a viewer that keeps up) or not drained at all (the slowest possible viewer —
/// the reported lag condition). The ONLY variable between the two is consumption.
fn run_stream(drained: bool, w: u32, h: u32) -> Run {
    let frame_bytes = rgb_bytes(w, h);
    let rgb = incompressible(frame_bytes, 0x5A);

    let addr = probe_free_addr();
    let (rx, handle) = re_grpc_server::spawn_with_recv(
        addr,
        cerulion_vizd::host::server_options(),
        re_grpc_server::shutdown::never(),
    );
    let prod =
        re_grpc_client::Client::new(proxy_uri(addr), re_grpc_client::write::Options::default());
    let store = StoreId::random(StoreKind::Recording, "backlog");
    prod.send_blocking(set_store_info(&store));
    prod.send_blocking(static_frame(&store, "scene/axes"));

    let mut series = Vec::with_capacity(FRAMES);
    let mut peak_broadcast = 0;
    // How many messages the receiver actually took. Load-bearing for the control
    // arm: without it, "dropped 0" is also satisfied by a proxy that received
    // nothing at all.
    let mut drained_count = 0usize;
    // The UNDRAINED arm runs until the SUBJECT reports engagement; the DRAINED
    // control runs exactly `FRAMES` (its `drained_count >= FRAMES` assertion is
    // what makes its `dropped 0` non-vacuous, so its bound must not move).
    //
    // The per-frame flush below made SENT == DELIVERED, and its own
    // comment records what that left standing — engagement MEASURED at frame 55
    // of the 60 this loop was allowed, "a margin of five frames — 8%". That is a
    // fixed count of frames standing in for a claim about the SUBJECT, which is
    // exactly the load-invertible bound `flood_until_budget_engages` was
    // rewritten to retire, on the one loop that never got the rewrite. It cost
    // main two `a_slow_viewer…` firings reporting `Usage { broadcast: 0,
    // live_dropped: 0, disposable: 0, persistent: 212 }` — not a queue that
    // drained but one that never filled, the earlier flake's signature verbatim.
    //
    // Nothing here ever consumes, so total buffered bytes are MONOTONE
    // NON-DECREASING in frames delivered: load changes the RATE at which this
    // loop advances and never the DIRECTION, so a slow runner takes more frames
    // rather than reporting a different verdict. The floor stays at `FRAMES` so
    // the occupancy series — which `observed_message` is read off — is never
    // SHORTER than it was.
    let frame_ceiling = if drained {
        FRAMES
    } else {
        RUN_STREAM_FRAME_CEILING
    };
    let stream_start = Instant::now();
    let mut seq = 0usize;
    let mut encoded_frame_bytes = 0u64;
    while seq < frame_ceiling && stream_start.elapsed() < RUN_STREAM_DEADLINE {
        // Build ONCE, price it, then send that same object — so the byte count is
        // the frame this run really put on the wire, and the running max covers
        // every sequence the run reached rather than standing in for them with
        // frame 0's (see `Run::encoded_frame_bytes`).
        let frame = image_frame(&store, "cam/video/rendition", seq as i64, &rgb, w, h);
        encoded_frame_bytes = encoded_frame_bytes.max(accounted_size(&frame));
        prod.send_blocking(frame);
        if drained {
            // LOCKSTEP, not "drain periodically and hope". `send_blocking` only
            // ENQUEUES — the client's background task does the wire encode — so a
            // send/sleep/drain cadence lets the client burst several frames into
            // the proxy while this thread sleeps, and on a loaded runner that
            // burst goes over budget and drops. CI hit exactly that (2 frames
            // dropped against an asserted 0), and it reproduced locally under 48
            // busy-loop processes.
            //
            // Flushing the frame to the proxy and then WAITING for the receiver to
            // take it makes "the viewer keeps up" true BY CONSTRUCTION: at most
            // one frame is ever in flight, so the queue cannot reach a budget that
            // is three frames deep, at any runner speed. The zero this arm asserts
            // then stays a zero under load instead of becoming a threshold — the
            // repo's rule for load-sensitive assertions.
            let _ = prod.flush_blocking(Duration::from_secs(20));
            let deadline = Instant::now() + Duration::from_secs(20);
            let mut took_one = false;
            while !took_one {
                while rx.try_recv().is_ok() {
                    drained_count += 1;
                    took_one = true;
                }
                assert!(
                    took_one || Instant::now() < deadline,
                    "the draining receiver never consumed frame {seq} — the pipeline is stuck, so \
                     `dropped 0` below would be meaningless"
                );
                if !took_one {
                    std::thread::sleep(Duration::from_millis(5));
                }
            }
        } else {
            // FLUSH even though nothing drains — an earlier flake, and the reason is the
            // same defect that took `a_plot_sample_...` red on main, sitting in
            // the arm that carries this file's headline.
            //
            // `send_blocking` only ENQUEUES, into a client command queue bounded
            // at 100 MESSAGES with no byte term, so all `FRAMES` of these 2.7 MB
            // frames fit in it and this thread walks the whole loop while the
            // client's background encoder delivers however many it manages. That
            // made the `live_dropped > 0` anti-tautology below a claim about the
            // RUNNER rather than about the gate: the budget only engages once the
            // receiver's 128 MiB forwarding sink is full (~48 frames) plus ~4
            // more to cross the 8 MiB budget — MEASURED at frame 55 of the 60
            // this loop sends — so a runner that delivered under 48 in the loop's
            // ~2 s inverted it, reporting the queue empty because it had never
            // been filled.
            //
            // One flush per frame makes SENT == DELIVERED, so the engagement
            // point stops being a function of the runner. What remains is fixed
            // by the fork's byte constants, and load can only move it EARLIER: a
            // forwarding task that falls behind lets the backlog build sooner
            // (measured at frame 12 when it is denied the 33 ms of idle below).
            // That is the monotone direction, so the residual margin here is a
            // constant to be re-measured if the fork's sink or the encoded frame
            // size moves, never a flake to be re-run.
            //
            // It also TIGHTENS the ceiling this arm asserts. `observed_message`
            // is read off the largest single-sample JUMP in occupancy as "one
            // admitted message"; without a flush several frames could land
            // between two 33 ms-spaced reads and inflate it, and the ceiling with
            // it. At most one frame is now in flight per read.
            let _ = prod.flush_blocking(Duration::from_secs(20));
        }
        std::thread::sleep(Duration::from_millis(33));
        let u = usage(&handle, Duration::from_secs(2));
        peak_broadcast = peak_broadcast.max(u.broadcast);
        series.push(u.broadcast);
        seq += 1;
        // Bounded on the SUBJECT's own report, never on a frame count — and only
        // once the series is at least as long as it used to be.
        if !drained && seq >= FRAMES && u.live_dropped > 0 {
            break;
        }
    }
    // Let the event loop settle so the final read is not mid-send.
    std::thread::sleep(Duration::from_millis(300));
    let last = usage(&handle, Duration::from_secs(2));
    drop(prod);
    drop(rx);
    Run {
        peak_broadcast,
        last,
        series,
        frame_bytes,
        encoded_frame_bytes,
        drained_count,
    }
}

#[test]
fn a_slow_viewer_does_not_accumulate_a_backlog_of_image_frames() {
    with_runtime(|| {
        let (w, h) = RENDITIONS[1];
        let r = run_stream(false, w, h);

        // The live queue may hold up to the budget plus the ONE message that was
        // admitted while the queue was still within it (the gate compares against
        // the CURRENT occupancy, so a message always fits an empty queue).
        //
        // The "+1 message" term must be the ENCODED message size the channel
        // actually accounts (`total_size_bytes` of the proto), NOT the raw pixel
        // count: measured, the encoded message is ~0.5% LARGER than the raw RGB,
        // which left this assertion 0.3% from red — a rerun patch bump or a
        // compression change would have failed it for a reason unrelated to the
        // fix. So derive it from the run: the largest single-sample JUMP in
        // occupancy is one admitted message.
        //
        // The FLOOR under that derivation must be the encoded size too, and
        // pricing it at the raw payload was the residual the shed-starvation probe D
        // measured. The derivation is sound only while the 33 ms series happens to
        // REVEAL a full encoded-size jump; on a runner whose samples straddle a
        // drain the largest observed jump is SMALLER than one message, the floor
        // binds, and a raw-priced floor understates the ceiling by exactly the
        // compression delta — ~14.7 KB, 0.53% of a frame.
        //
        // That is not slack being trimmed. `decide_live` admits whenever
        // `occ.bytes <= budget`, so the worst case it PERMITS is `budget +
        // ENCODED`, and a ceiling of `budget + RAW` sits BELOW that by the delta:
        // a healthy gate at its permitted peak would be a red. Measured, the peak
        // is four encoded frames (~11.12 MB) against a raw-floored ceiling of
        // ~11.15 MB — 0.3% of headroom for a bound that has no business being
        // that tight.
        //
        // WHICH encoded frame is the second half of that, and it is not frame 0.
        // The accounted size varies frame to frame (the timeline sequence and a
        // fresh random `RowId` both ride the compressed buffer), by 12 bytes over
        // a 60-frame run measured here — and NOT monotonically, so the largest is
        // neither the first frame nor the last. Pricing the term at any single
        // frame can therefore land under a message the gate really did admit,
        // which is small and still on the wrong side of a bound whose whole job is
        // to be an upper one. `encoded_frame_bytes` is the running MAXIMUM over
        // the frames the run actually streamed, so the ceiling is at or above
        // every message this run could have had admitted. See its own doc for the
        // measured series.
        //
        // Byte totals are deliberately not spelled out here: per-message encoded
        // sizes vary by tens of bytes (see `a_plot_sample_...`), so the numbers to
        // read are `probe_the_slow_viewer_ceiling_against_the_encoded_message`'s,
        // which prints raw / encoded / observed jump / both ceilings for the run
        // in front of you.
        let observed_message = r
            .series
            .windows(2)
            .map(|w| w[1].saturating_sub(w[0]))
            .max()
            .unwrap_or(0)
            .max(r.encoded_frame_bytes);
        let ceiling = LIVE_TEMPORAL_BUDGET_BYTES + observed_message;
        assert!(
            r.peak_broadcast <= ceiling,
            "the LIVE queue must not accumulate image frames — a viewer that cannot keep \
             up would play through the backlog before seeing the present. Peak {} bytes ({:.1} \
             frames of {w}x{h} RGB8); ceiling {ceiling} bytes ({:.1} frames). Occupancy series (in \
             frames): {:?}",
            r.peak_broadcast,
            r.peak_broadcast as f64 / r.frame_bytes as f64,
            ceiling as f64 / r.frame_bytes as f64,
            r.series
                .iter()
                .map(|b| format!("{:.1}", *b as f64 / r.frame_bytes as f64))
                .collect::<Vec<_>>()
        );

        // ANTI-TAUTOLOGY: the bound above would also be satisfied by a proxy that
        // received nothing at all. Drops must actually have happened, and the
        // stream must actually have reached the proxy.
        assert!(
            r.last.live_dropped > 0,
            "the run must actually have exercised the budget (dropped bytes > 0) — otherwise the \
             bound above is vacuous. Got {:?}",
            r.last
        );
        assert!(
            r.last.persistent > 0,
            "the run must actually have reached the proxy (the SetStoreInfo is persistent)"
        );
    });
}

#[test]
fn a_viewer_that_keeps_up_loses_nothing() {
    with_runtime(|| {
        // The CONTROL for the headline arm: the fix must be "do not BUFFER", not
        // "drop always". A receiver draining each iteration never pushes the queue
        // over budget, so nothing may be dropped — a healthy viewer sees every
        // frame it could have displayed. This is the SAME stimulus, the same
        // frame size and the same budget as the headline arm; the ONLY difference
        // is that this receiver consumes. So the pair discriminates "drops when
        // there is a backlog" from "drops always".
        let (w, h) = RENDITIONS[1];
        let r = run_stream(true, w, h);
        assert_eq!(
            r.last.live_dropped, 0,
            "a viewer that keeps up must lose NOTHING — the budget may only bite on a backlog. \
             Got {:?}",
            r.last
        );
        // Not vacuous — and `persistent > 0` alone would NOT establish this: the
        // `SetStoreInfo` is sent BEFORE the frame loop, so a pipeline that wedged
        // and delivered none of the frames would still satisfy it. The load-bearing
        // assertion is that the receiver actually CONSUMED frames.
        assert!(
            r.drained_count >= FRAMES,
            "the receiver must actually have consumed the stream — otherwise `dropped 0` is \
             satisfied by a proxy that delivered nothing. Consumed {} messages, sent {FRAMES} \
             frames (+ the store handshake and a static)",
            r.drained_count
        );
        assert!(
            r.last.persistent > 0,
            "the run must actually have reached the proxy (the SetStoreInfo is persistent)"
        );
        assert!(
            r.peak_broadcast < r.frame_bytes as u64,
            "the drained receiver must actually have kept up (peak live queue under one frame), \
             got {} bytes ({:.1} frames)",
            r.peak_broadcast,
            r.peak_broadcast as f64 / r.frame_bytes as f64
        );
    });
}

#[test]
fn a_fresh_viewer_still_gets_its_scene_while_the_live_queue_is_over_budget() {
    with_runtime(|| {
        // THE safety property. Two things could go wrong with a live budget and
        // both would leave a viewer staring at nothing:
        //   1. dropping SKELETON messages (a viewer cannot render without its
        //      SetStoreInfo / statics / blueprint);
        //   2. blocking the event loop, which also serves `Event::NewClient`, so a
        //      new viewer could not connect at all — the reason the fix DROPS
        //      rather than shrinking the byte quota.
        // So: flood an undrained proxy past its budget, send the skeleton probes
        // WHILE it is over budget, THEN connect a fresh viewer and require it to
        // receive them.
        //
        // THE ORDER IS THE WHOLE ARM. A skeleton sent
        // BEFORE the flood lands on an EMPTY queue — where `decide_live` returns Admit
        // under ANY classification (`is_temporal` is never consulted for a
        // within-budget queue), the message lands in the replay history, and the
        // fresh viewer reads it back from `Event::NewClient`'s connect-history
        // rather than from the live queue the budget governs. That shape passes
        // with EITHER skeleton guard deleted from `is_temporal`, while asserting
        // "the skeleton is never eligible for the drop" from a stimulus in which
        // nothing of any class was eligible. Sent while the budget is engaged, a
        // misclassified skeleton message is DROPPED — and the drop returns before
        // `history.add_msg`, so it reaches neither buffer and the viewer never
        // sees it.
        let (w, h) = RENDITIONS[1];
        let frame_bytes = rgb_bytes(w, h);
        let rgb = incompressible(frame_bytes, 0x5A);

        let addr = probe_free_addr();
        let (rx, handle) = re_grpc_server::spawn_with_recv(
            addr,
            cerulion_vizd::host::server_options(),
            re_grpc_server::shutdown::never(),
        );
        let uri = proxy_uri(addr);
        let prod =
            re_grpc_client::Client::new(uri.clone(), re_grpc_client::write::Options::default());
        let store = StoreId::random(StoreKind::Recording, "s");
        prod.send_blocking(set_store_info(&store));
        // Flood until the budget ENGAGES — bounded on the SUBJECT's own report
        // rather than on a fixed count of frames the producer managed to enqueue.
        // See `flood_until_budget_engages` for the measurement that retired the
        // old `for seq in 0..FRAMES` loop.
        let flood = flood_until_budget_engages(&prod, &handle, &store, &rgb, w, h);
        assert_budget_engaged(&flood);
        let dropped_before = flood.last.live_dropped;

        // The probes: a STATIC in the recording store and a BLUEPRINT chunk in its
        // own store, both above the small-message floor so misclassification
        // really would drop them — BRACKETED by two pressure phases that
        // each run until the SUBJECT reports a further whole camera frame dropped.
        // Sampling occupancy and then sending would read whichever phase of the
        // admit/drop oscillation it landed in; feeding a FIXED burst and then
        // polling is what CI inverted twice — see
        // `Pressure::until_dropping`.
        let bp_store = StoreId::random(StoreKind::Blueprint, "bp");
        let pressure = Pressure {
            prod: &prod,
            handle: &handle,
            store: &store,
            rgb: &rgb,
            w,
            h,
        };
        let before = pressure.until_dropping(1000, dropped_before);
        assert_pressed_until_dropping(&before, "when the probes were sent", frame_bytes);

        prod.send_blocking(big_static_frame(&store, LATE_STATIC_ENTITY));
        prod.send_blocking(set_store_info(&bp_store));
        prod.send_blocking(big_blueprint_chunk(&bp_store));

        // The over-budget PRECONDITION, restated as an EVENT rather than a sample
        // and DRIVEN rather than awaited: whole camera frames were DROPPED on
        // BOTH sides of the probes, which can only happen while the queue is over
        // budget, and the proxy handles messages in send order — so the probes
        // crossed between two confirmed drops. Load-monotone: a slower runner
        // needs more frames or more wall, never fewer drops.
        let during = pressure.until_dropping(2000, before.last.live_dropped);
        assert_pressed_until_dropping(&during, "after the probes crossed", frame_bytes);

        let consumer = re_grpc_client::stream(uri);
        let deadline = Instant::now() + Duration::from_secs(10);
        let mut saw_store_info = false;
        let mut saw_late_static = false;
        let mut saw_blueprint = false;
        while Instant::now() < deadline && !(saw_store_info && saw_late_static && saw_blueprint) {
            match consumer.recv_timeout(Duration::from_millis(200)) {
                Ok(sm) => match sm.into_data() {
                    Some(re_log_channel::DataSourceMessage::LogMsg(LogMsg::SetStoreInfo(info))) => {
                        if info.info.store_id.kind() == StoreKind::Recording {
                            saw_store_info = true;
                        }
                    }
                    Some(re_log_channel::DataSourceMessage::LogMsg(LogMsg::ArrowMsg(
                        sid,
                        arrow,
                    ))) => {
                        if sid.kind() == StoreKind::Blueprint {
                            saw_blueprint = true;
                        } else if let Ok(chunk) = re_chunk::Chunk::from_arrow_msg(&arrow) {
                            // Compare EntityPath to EntityPath: `to_string()`
                            // renders a leading `/`, so a raw `== "scene/..."`
                            // never matches and the arm would fail for a reason
                            // that has nothing to do with the budget.
                            if chunk.is_static()
                                && chunk.entity_path() == &entity_path(LATE_STATIC_ENTITY)
                            {
                                saw_late_static = true;
                            }
                        }
                    }
                    _ => {}
                },
                Err(_) => continue,
            }
        }
        assert!(
            saw_store_info && saw_late_static && saw_blueprint,
            "the LATE static and the blueprint chunk — both sent WHILE the live queue was \
             over budget — must still reach a fresh viewer: the skeleton is never eligible for the \
             drop, and the event loop must not be wedged. (`store_info` is the PRE-flood recording \
             `SetStoreInfo`, served from the replay history; it rides along as a liveness check, \
             and it is the other two that carry the over-budget claim.) \
             store_info={saw_store_info} late_static={saw_late_static} blueprint={saw_blueprint}"
        );
        drop(prod);
        drop(rx);
    });
}

/// The entity the fresh-viewer arm's LATE static is logged under, so the arm can
/// tell it from anything sent before the flood.
const LATE_STATIC_ENTITY: &str = "scene/late_axes";

fn entity_path(s: &str) -> re_log_types::EntityPath {
    re_log_types::EntityPath::from(s)
}

#[test]
fn a_plot_sample_survives_a_queue_full_of_camera_frames() {
    with_runtime(|| {
        // THE SMALL-MESSAGE FLOOR, end to end. The budget is compared against the
        // queue's OCCUPANCY, not against the arriving message, and this daemon
        // logs every topic into ONE `RecordingStream` — so without a floor, a
        // camera holding the queue over budget makes collateral of every plot
        // sample, `/tf` update, `TextLog` line and marker `Clear` sharing it.
        //
        // That is not a latency trade for those classes. A frame is a
        // latest-value sample and the next one supersedes it; a `rerun::Clear` is
        // a one-shot STATE TRANSITION, and vizd never re-sends it (the resolver's
        // `cleared` set exists to suppress the ROS repeat-DELETE idiom), so a
        // dropped one leaves a deleted marker rendered for the rest of the
        // session — a persistently WRONG picture, which is worse than the lag the
        // budget removes.
        //
        // So: flood past the budget with camera frames, then send a plot sample
        // and require a viewer to RECEIVE it.
        //
        // # The verdict is an ORDERING, not a deadline
        //
        // It used to be a deadline: send the sample, then give the viewer 5 s to
        // show it. That measures the RUNNER. On a loaded CI runner it failed with
        // every round's pressure confirmed, `broadcast` moving by EXACTLY one plot
        // sample (1353 bytes) per round — i.e. the gate had ADMITTED the sample
        // and the viewer had simply not been handed it yet — so the arm reported
        // a data-loss regression that had not happened. That is the load-sensitive class.
        //
        // The replacement asks the same question through the ORDER of the stream
        // instead. Immediately after the sample this arm logs a STATIC chunk on
        // its own entity — the SENTINEL. Two facts make it decisive:
        //
        //  * `is_temporal` is false for a static chunk, so `decide_live` returns
        //    Admit BEFORE either axis is consulted — that is the guard that is
        //    genuinely independent of the one under test. The sentinel is ALSO
        //    kept under the small-message floor, but as defence in depth on the
        //    byte axis only (the fork's MESSAGE axis exempts nothing), for the
        //    case where `is_temporal` itself misclassified it;
        //  * `re_quota_channel`'s broadcast hands each receiver the messages sent
        //    after it subscribed, IN ORDER, and BACKPRESSURES rather than evicting
        //    — so one receiver's stream is a total order over admitted messages.
        //
        // Therefore the sentinel's arrival proves the viewer's stream has passed
        // the position the sample would occupy. If the sentinel is in and the
        // sample is not, the sample was SHED — and no amount of load can produce
        // that reading, because load cannot make a later message overtake an
        // earlier one. If NEITHER is in, the viewer is merely behind: wait. Load
        // can only DELAY this arm, never invert it.
        //
        // MEASURED over 32 rounds (idle desk and `taskpolicy -b`, see
        // `measure_plot_sample_delivery_under_pressure`): the sentinel arrived in
        // 32/32 rounds, 1.3 us - 219.5 ms after the round's flush, and NEVER
        // before the sample. The ordering itself is not left to that sample,
        // though — it is ASSERTED on every round that delivers both, because a
        // measurement cannot cover a guarantee and this verdict rests on one.
        //
        // The discriminator this replaces was "the viewer saw other entities while
        // missing ours". That was MEASURED and REFUTED: 25 of those 32 rounds saw
        // ZERO other entities, because the queue is over budget for the whole run
        // so every camera frame is shed and never broadcast at all. The 7 that saw
        // any were seeing a PREVIOUS round's traffic land late, not a witness for
        // the round that was asking. "The viewer received nothing" is the NORMAL
        // reading here, so gating on it would have passed a genuinely shed
        // sample.
        //
        // Ordering is the constraint that shapes this arm. The viewer must connect
        // AFTER the backlog is built but BEFORE the sample is sent: a viewer
        // present during the flood keeps the queue from ever going over budget
        // (MEASURED on this harness — `broadcast: 0, live_dropped: 0` across all
        // 60 frames), and one connecting after the sample could never see it,
        // since `drop_temporal_history` leaves no replay history for temporal data. In between,
        // both hold: the queue is already over budget and a live subscriber gets
        // everything published from its subscribe point on.
        let (w, h) = RENDITIONS[1];
        let frame_bytes = rgb_bytes(w, h);
        let rgb = incompressible(frame_bytes, 0x5A);

        let addr = probe_free_addr();
        let (rx, handle) = re_grpc_server::spawn_with_recv(
            addr,
            cerulion_vizd::host::server_options(),
            re_grpc_server::shutdown::never(),
        );
        let uri = proxy_uri(addr);
        let prod =
            re_grpc_client::Client::new(uri.clone(), re_grpc_client::write::Options::default());
        let store = StoreId::random(StoreKind::Recording, "p");
        prod.send_blocking(set_store_info(&store));

        // Flood until the budget ENGAGES — bounded on the SUBJECT's own report
        // rather than on a fixed count of frames the producer managed to enqueue.
        // This is the arm an earlier flake fired on; see `flood_until_budget_engages` for
        // the measurement that retired the old `for seq in 0..FRAMES` loop.
        let flood = flood_until_budget_engages(&prod, &handle, &store, &rgb, w, h);
        assert_budget_engaged(&flood);

        // QUIESCE before taking the baseline. `send_blocking` only ENQUEUES — the
        // client's background task is still pushing flood frames when the loop
        // breaks, and each of those is dropped at the proxy. Attributing one of
        // them to the plot sample is a false failure (MEASURED: the baseline moved
        // by exactly one frame). The flood now flushes each frame, so at most one
        // is ever in flight — this is kept because "at most one" is not "none",
        // and it costs a single 250 ms pass when there is nothing to settle.
        let _ = prod.flush_blocking(Duration::from_secs(20));
        let mut u = usage(&handle, Duration::from_secs(5));
        for _ in 0..40 {
            std::thread::sleep(Duration::from_millis(250));
            let next = usage(&handle, Duration::from_secs(5));
            if next.live_dropped == u.live_dropped {
                break;
            }
            u = next;
        }

        // The viewer joins NOW — after the backlog, before the sample — and is
        // drained CONTINUOUSLY by its own thread, which is both what a real
        // viewer does and what this arm needs to be measuring.
        //
        // It used to be drained only INSIDE each round's 5 s window,
        // discarding every message that was not that round's sample. The two
        // together are cumulative: a viewer that ends a window still holding
        // frames starts the next one further behind, and because the window
        // discards non-matching messages it spends the next 5 s draining the
        // PREVIOUS round's sample, which cannot match. Once a round is lost the
        // arm can never recover, and it fails with every round's pressure
        // CONFIRMED and no round delivered — the observed failure signature.
        //
        // MEASURED (`shed_starvation_test`, per-message stall injected
        // into the windowed drain, on an idle desk): windowed misses 0/2/5/7 of 8
        // rounds at 0/200/500/1200 ms while a viewer drained continuously
        // receives 8 of 8 at EVERY level. The proxy's own accounting shows the
        // samples were ADMITTED throughout — `broadcast` climbs by ~1351 bytes a
        // round, one plot sample each — so what the old shape measured was the
        // test thread's drain rate, not the byte-axis guarantee.
        //
        // This changes only WHO reads the stream. The claim is unchanged: the
        // viewer received that round's sample.
        let consumer = re_grpc_client::stream(uri);
        // ORDERED, and it records `is_static` beside the entity, because the
        // verdict rests on two properties a `HashSet<String>` cannot observe:
        // that the sentinel really arrived AFTER the sample on this receiver's
        // stream, and that what arrived under the sentinel's entity really was a
        // STATIC. Both are asserted below on the chunks that were RECEIVED rather
        // than assumed from the ones that were sent.
        let seen: Arc<Mutex<Vec<(String, bool)>>> = Arc::new(Mutex::new(Vec::new()));
        let stop = Arc::new(AtomicBool::new(false));
        // A swallowed decode failure or a dead stream both look exactly like a
        // shed sample to the predicate below, so they are COUNTED and named in
        // every failure message rather than left to be misattributed. `stream`
        // does not reconnect, so a disconnect is permanent and terminal.
        let decode_failures = Arc::new(AtomicUsize::new(0));
        let stream_dead = Arc::new(AtomicBool::new(false));
        let drain_seen = Arc::clone(&seen);
        let drain_stop = Arc::clone(&stop);
        let drain_decode_failures = Arc::clone(&decode_failures);
        let drain_stream_dead = Arc::clone(&stream_dead);
        let drainer = std::thread::spawn(move || {
            while !drain_stop.load(Ordering::Relaxed) {
                match consumer.recv_timeout(Duration::from_millis(50)) {
                    Ok(sm) => {
                        if let Some(re_log_channel::DataSourceMessage::LogMsg(LogMsg::ArrowMsg(
                            _,
                            arrow,
                        ))) = sm.into_data()
                        {
                            match re_chunk::Chunk::from_arrow_msg(&arrow) {
                                Ok(chunk) => drain_seen
                                    .lock()
                                    .expect("the seen log is not poisoned")
                                    .push((chunk.entity_path().to_string(), chunk.is_static())),
                                Err(_) => {
                                    drain_decode_failures.fetch_add(1, Ordering::Relaxed);
                                }
                            }
                        }
                    }
                    // A timeout is the normal idle case; anything else means the
                    // source has run dry, which `is_connected` reports without
                    // this file having to name the channel's error type.
                    Err(_) => {
                        if !consumer.is_connected() {
                            drain_stream_dead.store(true, Ordering::Relaxed);
                            // `recv_timeout` returns Disconnected IMMEDIATELY and
                            // `is_connected` never comes back, so continuing here
                            // would spin a core until the round loop ends — on a
                            // `--test-threads=1` lane that starves the very
                            // pressure phases whose failure would then be
                            // reported instead of this one.
                            break;
                        }
                    }
                }
            }
        });
        std::thread::sleep(Duration::from_millis(500));

        // ROUNDS, because the DROP half must hold in the same window as the
        // sample, and a pressure phase can exhaust its wall on a loaded runner.
        //
        // Each round brackets one plot sample (on its own entity, so arrivals are
        // attributable to the round that sent them) and its sentinel between two
        // pressure phases that each run until the SUBJECT reports a further whole
        // camera frame dropped. A round in which BOTH phases confirm DECIDES —
        // there is no delivery race left to retry, so that round's sample must
        // arrive and its absence is a failure rather than an uncounted round. The
        // loop does NOT stop there: it runs every round it can confirm and
        // convicts on the first that sheds. What the remaining rounds retry is a
        // phase that ran out of wall. The bound
        // is on rounds rather than on time, so a slow runner takes longer rather
        // than reporting a different verdict.
        //
        // Sampling occupancy once and then sending — the shape CI went red on —
        // reads whichever phase of the admit/drop oscillation it lands in; so does
        // feeding a FIXED burst and then passively polling the drop counter, which
        // is what fired twice on the sibling arm. The pressure is
        // DRIVEN until the gate reports a drop; see `Pressure::until_dropping`. The
        // exact "not one byte of the sample was counted" reading is not available
        // under continuous pressure (the dropped total moves by many frames and
        // per-message encoded sizes vary by tens of bytes), so it is not claimed;
        // delivery is the stronger statement anyway, and it is what dies when the
        // floor is deleted.
        let plot_bytes = accounted_size(&small_frame(&store, PLOT_ENTITY, 1));
        let pressure = Pressure {
            prod: &prod,
            handle: &handle,
            store: &store,
            rgb: &rgb,
            w,
            h,
        };
        // EVERY confirmed round must deliver, not just one.
        //
        // The old shape stopped at the first round that delivered, so a gate that
        // shed seven samples out of eight passed green. Ordering removed the
        // delivery race, which makes the strict form affordable: a confirmed
        // round's reading is now a verdict, so the arm takes ALL of them and
        // convicts on the first shed one.
        //
        // That also shrinks the arm's one residual. `pressure_held` is a
        // SANDWICH — the gate was confirmed dropping on each side of the sample —
        // and between the confirming drop and the sample's own admission the
        // queue can in principle fall back under budget (nothing here reads
        // occupancy AT the instant the sample is handled, and a post-hoc sample
        // of an oscillating queue is exactly the shape this arm retired). A
        // floor-deleted gate could therefore admit one sample by luck. It cannot
        // do so in every confirmed round, and each round is an independent draw.
        let mut delivered_rounds = 0usize;
        let mut failed: Option<Verdict> = None;
        // Checked after the drainer is retired, never inside the loop — an
        // assertion there would abandon a thread still holding a gRPC stream.
        // (`usage` keeps its own liveness panic on that path; it fires only when
        // the proxy's event loop has stopped, which no verdict could survive.)
        let mut min_dropped: Option<(usize, u64)> = None;
        // (round, sample entity, sentinel entity) for every round that DECIDED, so
        // the ordering premise can be checked over the FINAL log rather than over
        // whatever happened to have landed at the poll that ended each wait.
        let mut deciding: Vec<(usize, String, String)> = Vec::new();
        let mut last_pressure: Option<(Pressed, Pressed)> = None;
        // Reported rather than derived from `PLOT_ROUNDS`: a run can stop early,
        // and a message claiming eight rounds ran when two did is a claim about
        // work nobody did.
        let mut rounds_run = 0usize;
        let mut unconfirmed_streak = 0usize;
        for round in 0..PLOT_ROUNDS {
            let entity = format!("{PLOT_ENTITY}/r{round}");
            let sentinel_entity = format!("{PLOT_SENTINEL_ENTITY}/r{round}");
            let want = entity_path(&entity).to_string();
            let want_sentinel = entity_path(&sentinel_entity).to_string();
            let base = round as i64 * 1000;
            let d0 = usage(&handle, Duration::from_secs(5)).live_dropped;

            let before = pressure.until_dropping(base, d0);
            prod.send_blocking(small_frame(&store, &entity, round as i64));
            // The ORDER SENTINEL, immediately after the sample and before any
            // further pressure, so nothing this arm sends can come between them.
            prod.send_blocking(order_sentinel(&store, &sentinel_entity));
            let after_probe = pressure.until_dropping(base + 500, before.last.live_dropped);
            // A TRAILING flush. `after_probe` flushes per frame, so its FIRST
            // iteration already pushed the sample and the sentinel — which is why
            // `Pressed::flush_failures` is the stronger producer signal and this
            // is the weaker one. It is still carried rather than dropped, because
            // a producer that could not deliver otherwise reads downstream as a
            // wedged viewer. Note a flush failure cannot manufacture a false
            // SHED: the two messages ride one ordered stream, so "sentinel in,
            // sample out" still requires a server-side drop.
            let flush_failed = prod.flush_blocking(Duration::from_secs(20)).is_err();

            let dropped_in_round = after_probe.last.live_dropped.saturating_sub(d0);
            let pressure_held = before.confirmed && after_probe.confirmed;
            last_pressure = Some((before, after_probe));
            rounds_run += 1;

            // A round can only DECIDE when the gate was confirmed dropping on BOTH
            // sides of the sample.
            //
            // `pressure_held` is the load-bearing term, and it SUBSUMES the
            // aggregate: two confirmed phases mean the proxy's own dropped total
            // advanced by at least one whole frame EACH. `dropped_in_round` is
            // kept, and ASSERTED below, precisely so that subsumption is checked
            // rather than trusted to this `continue` staying where it is.
            //
            // Requiring only the aggregate is
            // strictly WEAKER, and weaker in exactly the direction this arm
            // exists to close: the AFTER phase's drops alone satisfy it, so a
            // round in which the BEFORE phase never confirmed could qualify, i.e.
            // the sample crossed a queue nothing had established was over budget.
            // That is the vacuity `Pressure::until_dropping` exists to
            // remove, re-opened one conjunct later.
            if !pressure_held {
                unconfirmed_streak += 1;
                if unconfirmed_streak >= PRESSURE_UNCONFIRMED_ROUNDS_TOLERATED {
                    // Two rounds in a row could not confirm the gate. Since
                    // the DROP half is DRIVEN, repeated failure there
                    // is the apparatus having changed rather than a race lost —
                    // and burning the remaining `PLOT_ROUNDS` phases on it turns
                    // one attributable failure into a several-minute one.
                    break;
                }
                continue;
            }
            unconfirmed_streak = 0;

            // The gate was dropping on both sides of this sample, so this round
            // DECIDES.
            //
            // The drainer never stops reading, so this waits on the RECORD rather
            // than racing the stream for it.
            //
            // EITHER arrival ends the wait. The sample arriving IS the property,
            // so it needs no witness; the sentinel exists only to make an ABSENCE
            // final. Waiting for the sentinel even after the sample had landed
            // would put a wall back in front of a healthy run — the two are
            // ADJACENT messages, so any stream break between them would spend the
            // whole liveness wall and then report a regression that did not
            // happen, which is the shape this arm exists to stop reporting.
            //
            // Both positions are read under ONE guard, and that is what makes the
            // reading exact: the drainer appends in ARRIVAL order and never holds
            // the lock across two messages, so a sentinel visible here implies
            // every message the broadcast DELIVERED before it is visible here too
            // — which is what makes the sample's absence under this SAME guard
            // mean "never delivered" rather than "not yet read". Two separate
            // acquisitions could straddle an append; one cannot.
            let deadline = Instant::now() + SENTINEL_DELIVERY_DEADLINE;
            let started = Instant::now();
            let v = loop {
                let (sample_at, sentinel_at) = {
                    let seen = seen.lock().expect("the seen log is not poisoned");
                    (
                        seen.iter().position(|(e, _)| *e == want),
                        seen.iter().position(|(e, _)| *e == want_sentinel),
                    )
                };
                if sample_at.is_some() || sentinel_at.is_some() || Instant::now() >= deadline {
                    break Verdict {
                        round,
                        dropped_in_round,
                        waited: started.elapsed(),
                        flush_failed,
                        sentinel_at,
                        outcome: Outcome::classify(Sightings {
                            sample: sample_at.is_some(),
                            sentinel: sentinel_at.is_some(),
                        }),
                    };
                }
                std::thread::sleep(Duration::from_millis(20));
            };
            // Recorded on EVERY deciding round, delivered or not, so the guards
            // after the loop read the whole run rather than its last round. They
            // are reached only when nothing failed — a failing round panics — so
            // on a green run they cover every round, and on a red one that
            // round's own `Outcome` message carries its numbers instead.
            if min_dropped.is_none_or(|(_, least)| v.dropped_in_round < least) {
                min_dropped = Some((v.round, v.dropped_in_round));
            }
            deciding.push((v.round, want.clone(), want_sentinel.clone()));
            if v.outcome == Outcome::Delivered {
                delivered_rounds += 1;
                continue;
            }
            failed = Some(v);
            break;
        }

        // THE ORDER PREMISE, checked over the FINAL log.
        //
        // Checking it per round as each wait ended would have covered almost
        // nothing: a healthy wait ends the moment the SAMPLE lands, so the
        // sentinel one message behind it is usually not in the log yet, and an
        // `order_violation.is_none()` computed from that is satisfied identically
        // by "never violated" and "never looked at". Pressure has stopped here,
        // so the queue drains and the sentinels of every deciding round arrive
        // within a settle that decides nothing.
        //
        // SKIPPED after a `ViewerNeverReached` round, which has ALREADY spent the
        // full `SENTINEL_DELIVERY_DEADLINE` on the very sentinel this loop would
        // wait for. Waiting again cannot change the answer; it only delays the
        // diagnostic by another wall on a `--test-threads=1` lane. The coverage
        // asserts this settle feeds sit BELOW that round's panic, so nothing
        // downstream of here is reached on that path anyway.
        let liveness_exhausted = failed
            .as_ref()
            .is_some_and(|v| v.outcome == Outcome::ViewerNeverReached);
        let settle = Instant::now() + SENTINEL_PREMISE_SETTLE;
        if !liveness_exhausted {
            loop {
                let missing = {
                    let seen = seen.lock().expect("the seen log is not poisoned");
                    deciding
                        .iter()
                        .filter(|(_, _, sentinel)| !seen.iter().any(|(e, _)| e == sentinel))
                        .count()
                };
                if missing == 0 || Instant::now() >= settle {
                    break;
                }
                std::thread::sleep(Duration::from_millis(20));
            }
        }
        let final_seen = seen.lock().expect("the seen log is not poisoned").clone();

        let after = usage(&handle, Duration::from_secs(5));
        // Retire the drainer BEFORE the verdict: an assertion that fires here
        // would otherwise abandon a thread still holding a gRPC stream to a proxy
        // this body is about to drop, and this file's lane is `--test-threads=1`,
        // so that thread would spin against a dead server for the rest of the run.
        stop.store(true, Ordering::Relaxed);
        drainer.join().expect("the viewer drainer does not panic");
        let decode_failures = decode_failures.load(Ordering::Relaxed);
        let stream_dead = stream_dead.load(Ordering::Relaxed);
        let viewer_health = format!(
            "(viewer: {decode_failures} chunk(s) failed to decode, stream {})",
            if stream_dead { "DIED" } else { "alive" }
        );

        let at = |want: &str| final_seen.iter().position(|(e, _)| e == want);
        let sentinels_seen = deciding
            .iter()
            .filter(|(_, _, sentinel)| at(sentinel).is_some())
            .count();
        let order_checked = deciding
            .iter()
            .filter(|(_, sample, sentinel)| at(sample).is_some() && at(sentinel).is_some())
            .count();
        let order_violation = deciding.iter().find_map(|(round, sample, sentinel)| {
            match (at(sample), at(sentinel)) {
                (Some(a), Some(b)) if a >= b => Some((*round, a, b)),
                _ => None,
            }
        });
        let non_static_sentinel = deciding.iter().find_map(|(round, _, sentinel)| {
            final_seen
                .iter()
                .find(|(e, _)| e == sentinel)
                .and_then(|(_, is_static)| (!is_static).then_some(*round))
        });

        // APPARATUS first: both of these make an absence in the log mean
        // something other than a shed sample, so neither may reach a verdict.
        assert!(
            !stream_dead,
            "apparatus: the viewer's gRPC stream DIED during the run, so the log stopped being a \
             record of what the gate admitted — and `re_grpc_client::stream` does not reconnect. \
             {delivered_rounds} round(s) delivered before that, {rounds_run} ran. \
             {viewer_health}. {after:?}"
        );
        assert_eq!(
            decode_failures, 0,
            "apparatus: {decode_failures} chunk(s) reached the viewer and failed to decode. A \
             sample that arrives and fails to decode is indistinguishable here from one that was \
             SHED, so no verdict below would be attributable while this is nonzero. {after:?}"
        );

        // PREMISES next, before any verdict is announced: a verdict that rests on
        // a premise already observed broken must not be reported as a data-loss
        // regression.
        assert!(
            order_violation.is_none(),
            "the ORDER SENTINEL arrived at or before the sample it is supposed to follow (round, \
             sample position, sentinel position) = {order_violation:?}. The sentinel is sent one \
             `send_blocking` after the sample on the same client, and the proxy broadcasts \
             admitted messages in order to each receiver, so this cannot happen unless that \
             stopped being true — and if it has, this arm's verdict (sentinel in + sample out == \
             SHED) is no longer sound and must be re-derived before it is trusted. {after:?}"
        );
        assert!(
            non_static_sentinel.is_none(),
            "the chunk that arrived under round {non_static_sentinel:?}'s sentinel entity was NOT \
             static. The sentinel decides anything only because `is_temporal` is false for a \
             static, which makes the gate return Admit before either axis is consulted — a \
             temporal sentinel would instead be riding the small-message floor, i.e. the very \
             guard under test, and this arm would go vacuous. Check `order_sentinel`. {after:?}"
        );
        // The vacuity guard, checked rather than left to the position of one
        // `continue`. Two confirmed pressure phases mean the proxy's own dropped
        // total advanced by at least one whole frame EACH, so any deciding round
        // must show at least two — and a round that decided without the gate
        // dropping is precisely the shape `Pressure::until_dropping` exists to
        // prevent.
        assert!(
            min_dropped.is_none_or(|(_, least)| least >= 2 * frame_bytes as u64),
            "a round DECIDED while the proxy's own accounting showed less than two whole camera \
             frames shed across it — (round, bytes) = {min_dropped:?} against {frame_bytes} \
             bytes a frame. Two confirmed phases imply two frames, so either the pressure guard \
             stopped gating which rounds decide, or `Pressure::until_dropping` stopped requiring \
             a whole frame. Either way that round's sample crossed a queue nothing had \
             established was over budget. {last_pressure:?}. {after:?}"
        );
        if let Some(v) = failed {
            match v.outcome {
                // Unreachable: a delivered round `continue`s rather than being
                // recorded as the failure. Spelled out so the match stays
                // exhaustive over the classifier rather than over today's flow.
                Outcome::Delivered => unreachable!(
                    "a delivered round is not a failure (round {}, sentinel_at {:?})",
                    v.round, v.sentinel_at
                ),
                Outcome::Shed => panic!(
                    "a {plot_bytes}-byte plot sample must reach a viewer across a queue \
                     that megabyte-sized camera frames filled. The byte axis exists to shed \
                     those; a plot sample or a marker `Clear` is not what filled it, shedding \
                     one buys the viewer well under a millisecond, and it is not recoverable — \
                     `drop_temporal_history` leaves no replay history and vizd never re-sends a `Clear`. In \
                     round {} the gate was CONFIRMED dropping on BOTH sides of the sample ({} \
                     bytes of whole camera frames, one frame being {frame_bytes}), and the \
                     STATIC sentinel — logged immediately AFTER the sample — reached the \
                     viewer (the wait resolved after {:?}, at stream position {:?}) while the \
                     sample itself never did. The broadcast hands one receiver its messages in order, \
                     so this is not a viewer that is behind — a message sent after the sample \
                     is already in. The sample was SHED. Read the BYTE axis first, since that \
                     is the one the small-message floor exempts; the fork's MESSAGE axis \
                     exempts nothing and would shed a sample too, but it triggers near 1024 \
                     in-flight messages and an {LIVE_TEMPORAL_BUDGET_BYTES}-byte budget holds \
                     this queue at a handful. {} rounds delivered before this one. \
                     Round flush failed: {}. {viewer_health}. Pressure phases: \
                     {last_pressure:?}. {after:?}",
                    v.round,
                    v.dropped_in_round,
                    v.waited,
                    v.sentinel_at,
                    delivered_rounds,
                    v.flush_failed
                ),
                Outcome::ViewerNeverReached => panic!(
                    "liveness: round {}'s plot sample did not reach the viewer, and neither \
                     did the STATIC sentinel logged immediately after it, within \
                     {SENTINEL_DELIVERY_DEADLINE:?} (waited {:?}), so this run never got far \
                     enough to have an opinion about the shed decision. `is_temporal` is false \
                     for a static, so the gate returns Admit before EITHER axis is consulted — \
                     that is the guard independent of the one under test, and the sentinel is \
                     additionally kept under the {LIVE_SMALL_MESSAGE_FLOOR_BYTES}-byte floor as \
                     defence in depth on the byte axis. {} round(s) delivered before this one: \
                     read a zero there as a viewer that never got scheduled at all, and a \
                     non-zero one as a viewer that STOPPED mid-run (the wall is ~270x the \
                     219.5 ms worst case behind `SENTINEL_DELIVERY_DEADLINE`, so a merely slow \
                     runner does not reach it) — or as the skeleton exemption itself breaking, \
                     which \
                     `a_fresh_viewer_still_gets_its_scene_while_the_live_queue_is_over_budget` \
                     pins directly. Round flush failed: {} (a TRAILING flush; the phase counts \
                     in the pressure phases below are the stronger signal). Pressure phases: \
                     {last_pressure:?}. {viewer_health}. {after:?}",
                    v.round, v.waited, delivered_rounds, v.flush_failed
                ),
            }
        }

        // COVERAGE last: these say how much of the run the premises above could
        // examine, so they must never pre-empt the verdict — a round that shed
        // its sample is a conviction, not a coverage problem, and a run in which
        // no round could confirm pressure is the `delivered_rounds` failure
        // below, whose message names the pressure phases.
        assert!(
            delivered_rounds == 0 || sentinels_seen > 0,
            "the ORDER SENTINEL never reached the viewer in ANY of the {} deciding round(s), \
             even after a {SENTINEL_PREMISE_SETTLE:?} settle with all pressure stopped. The \
             sentinel is what makes a sample's ABSENCE final rather than merely not-yet — \
             without one this arm is a delivery deadline again, which is what the order sentinel retired. \
             {delivered_rounds} round(s) delivered. {viewer_health}. {after:?}",
            deciding.len()
        );
        // Gated on a delivered round existing for the same reason: a run whose
        // only deciding round shed its sample legitimately has no round carrying
        // BOTH messages.
        assert!(
            delivered_rounds == 0 || order_checked > 0,
            "the ORDER PREMISE went UNCHECKED: no deciding round ended with BOTH its sample and \
             its sentinel in the viewer's log, even after a {SENTINEL_PREMISE_SETTLE:?} settle \
             with all pressure stopped, so the check above would be vacuous rather than true. \
             {} round(s) decided, {delivered_rounds} delivered. {viewer_health}. {after:?}",
            deciding.len()
        );
        // The TRIAL STRUCTURE, checked only once no round has convicted — a
        // conviction is itself a legitimate reason for the loop to have stopped
        // early. `PLOT_ROUNDS`' doc claims every confirmed round is an
        // independent trial the gate must pass; without this, turning the
        // `continue` after a delivered round back into a `break` would revert the
        // arm to the earlier "one of eight is enough" shape in silence.
        // Load-monotone: a healthy run exhausts the rounds, and a starved one
        // exhausts the unconfirmed tolerance.
        assert!(
            rounds_run == PLOT_ROUNDS
                || unconfirmed_streak >= PRESSURE_UNCONFIRMED_ROUNDS_TOLERATED,
            "the arm stopped after {rounds_run} of {PLOT_ROUNDS} round(s) without a verdict and \
             without exhausting its {PRESSURE_UNCONFIRMED_ROUNDS_TOLERATED}-round unconfirmed \
             tolerance (streak was {unconfirmed_streak}). Every CONFIRMED round is an \
             independent trial the gate must pass; stopping at the first delivery is the \
             pre-sentinel shape, under which a gate shedding seven samples in eight passed \
             green. {delivered_rounds} round(s) delivered. {after:?}"
        );
        assert!(
            delivered_rounds > 0,
            "precondition: no round could establish the gate as dropping on BOTH sides of its \
             sample, so none of them could decide anything. {rounds_run} round(s) ran (of a \
             permitted {PLOT_ROUNDS}; a run stops after \
             {PRESSURE_UNCONFIRMED_ROUNDS_TOLERATED} consecutive rounds whose pressure could \
             not be confirmed). One whole camera frame is {frame_bytes} bytes; each phase \
             wanted the proxy's own dropped total to advance by at least that much. Read the \
             last phases for the attribution: {last_pressure:?}. {viewer_health}. {after:?}"
        );
        // NOTE: there is deliberately no trailing `broadcast > budget` assertion.
        // That is a SAMPLE of an oscillating queue taken after the fact, which is
        // precisely the shape that went red in CI, and it is redundant: every
        // deciding round already established the stronger, load-monotone fact
        // that whole camera frames were being dropped in the window its sample
        // crossed, which cannot happen below budget.
        drop(prod);
        drop(rx);
    });
}

/// What ONE DECIDING round of [`a_plot_sample_survives_a_queue_full_of_camera_frames`]
/// observed — a round in which the gate was confirmed dropping on both sides of
/// its sample, so its reading is a verdict rather than a retry. One is built per
/// deciding round; only the first that fails to deliver is kept and reported.
///
/// A `Verdict` EXISTING is itself the proof that the pressure held: the round
/// loop `continue`s before the wait otherwise, so there is no field for it and no
/// reading of one to get wrong.
struct Verdict {
    round: usize,
    /// Whole-camera-frame bytes the gate shed across this round.
    ///
    /// ASSERTED, not merely reported. Two confirmed pressure phases imply at
    /// least two whole frames, so checking it is checking that the round really
    /// was gated — an invariant that otherwise lives only in the position of one
    /// `continue`, which is exactly the kind a refactor breaks in silence.
    dropped_in_round: u64,
    /// Wall spent waiting for one of the two messages to land.
    waited: Duration,
    /// The round's own flush — the one that gets the sample AND the sentinel to
    /// the proxy — did not ack. Reported in every message, because a producer
    /// that could not deliver reads downstream exactly like a wedged viewer.
    flush_failed: bool,
    /// Position of the ORDER SENTINEL in the viewer's arrival order, if it had
    /// arrived by the time this round stopped waiting. Reported in the `Shed`
    /// message so a reader can see WHERE in the stream the proof landed.
    ///
    /// The ordering PREMISE itself (`sample` before `sentinel`) is not checked
    /// from here — see the settle after the loop, which examines every deciding
    /// round over one final snapshot instead of over whatever each wait happened
    /// to catch.
    sentinel_at: Option<usize>,
    outcome: Outcome,
}

/// Whether the viewer's stream reached the point at which the sample's ABSENCE
/// becomes evidence: the order-not-deadline distinction, carried by the type rather than
/// by the order of two assertions.
///
/// The retired shape read "did the sample arrive?" on its own, which is a
/// question about the runner's speed. Reading it again without first establishing
/// that the stream passed the sample's position would restore exactly that, so
/// the answer is not reachable except through the variant that gives it meaning.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Outcome {
    /// The sample reached the viewer. The property held, and NOTHING about when
    /// it arrived is asserted — which is why this arm does not wait for the
    /// sentinel once the sample is in. The sentinel exists to make an ABSENCE
    /// final, so a presence needs no witness.
    Delivered,
    /// The ORDER SENTINEL — sent immediately AFTER the sample, on the same
    /// ordered stream — reached the viewer while the sample did not. That is the
    /// live-budget regression, and load cannot manufacture it: a message sent after
    /// the sample is already in, so this is not a viewer that is behind.
    Shed,
    /// Neither arrived inside the liveness wall, so the stream never reached the
    /// point at which the sample's absence means anything.
    ViewerNeverReached,
}

/// What the viewer had been handed by the time a deciding round stopped waiting.
///
/// A STRUCT, not two positional `bool`s, for the reason [`Pressure`] is a struct:
/// they are adjacent, same-typed, and a transposition at the call site compiles.
/// Here that transposition is not a wrong stimulus but a WRONG VERDICT — swap
/// them and a shed sample (sentinel in, sample out) classifies `Delivered`, so
/// the arm goes green on the exact regression it exists to catch.
#[derive(Debug, Clone, Copy)]
struct Sightings {
    /// The round's plot sample reached the viewer.
    sample: bool,
    /// The round's ORDER SENTINEL reached the viewer.
    sentinel: bool,
}

impl Outcome {
    /// The ONE place the two observations are classified — shared with
    /// [`measure_plot_sample_delivery_under_pressure`], so the measurement that
    /// justified this design cannot drift from the pin that enforces it.
    fn classify(seen: Sightings) -> Self {
        if seen.sample {
            Self::Delivered
        } else if seen.sentinel {
            Self::Shed
        } else {
            Self::ViewerNeverReached
        }
    }
}

/// The entity the floor arms log their control-class sample under.
const PLOT_ENTITY: &str = "telemetry/plot";

/// The entity `a_plot_sample_...` logs its ORDER SENTINEL under.
///
/// A STATIC chunk, on its own entity, sent immediately after each round's plot
/// sample. `is_temporal` is false for a static, so `decide_live` returns Admit
/// BEFORE either axis is consulted and the live budget may never shed it; the
/// broadcast hands one receiver the messages sent after it subscribed IN ORDER
/// and backpressures rather than evicting; so the sentinel arriving proves the
/// viewer's stream has passed the position the sample would occupy. That turns
/// "did the sample arrive?" from a question about the runner's speed into a
/// question about the gate's decision.
const PLOT_SENTINEL_ENTITY: &str = "scene/plot_sentinel";

/// The ONE catastrophe-scale wall in `a_plot_sample_...`: how long a deciding
/// round waits for EITHER its plot sample or its ORDER SENTINEL before reporting
/// that the viewer stopped receiving altogether.
///
/// A LIVENESS backstop, never a threshold anything is decided by — nothing in
/// this arm's verdict depends on WHEN the sentinel lands, only on the fact that
/// it landed. MEASURED (`measure_plot_sample_delivery_under_pressure`, 32 rounds
/// across an idle desk and `taskpolicy -b`): 1.3 us - 219.5 ms, 32/32 rounds. 60 s
/// is ~270x that worst case.
///
/// A healthy round never approaches it: the wait ends as soon as EITHER the
/// sample or the sentinel lands, so it is bounded above by the sentinel figures
/// quoted above — 219.5 ms worst case over those 32 rounds. Only a run in which
/// the viewer has stopped receiving altogether can spend it, and such a run stops
/// at the first round that does — so the whole arm pays it at most once, which is
/// what lets it be this generous. The retired shape spent its 5 s deadline in
/// every round that did NOT deliver — all eight, in the CI signature this
/// replaces — and reported a VERDICT from it; nothing is reported from this one
/// but liveness.
///
/// Provenance for the figures above: measured 2026-09-11 on a 16-core arm64
/// macOS desk by `measure_plot_sample_delivery_under_pressure`,
/// against a subject (`cerulion_vizd` plus the pinned `re_grpc_server` fork)
/// — FOUR runs of `PLOT_ROUNDS`, two on an idle desk and two under
/// `taskpolicy -b`. Each figure is timed from that round's flush (i.e. after its
/// `after_probe` phase), which is the same instant the arm's own wait starts.
const SENTINEL_DELIVERY_DEADLINE: Duration = Duration::from_secs(60);

/// A wall near the measured worst case would decide on the runner rather than on
/// the gate, which is the whole defect the order sentinel removed. This is the same
/// drift-guard shape the pressure bounds use two hundred lines up.
const _: () = assert!(
    SENTINEL_DELIVERY_DEADLINE.as_secs() >= 10,
    "SENTINEL_DELIVERY_DEADLINE must stay orders of magnitude above the 219.5 ms worst case \
     measured behind it, or it stops being a liveness backstop and starts being a deadline \
     that decides"
);

/// How long the arm waits, with ALL pressure stopped, for the ORDER SENTINELs of
/// the rounds that already decided — so the ordering premise is checked over the
/// whole run instead of over whatever had landed at the poll that ended each
/// wait.
///
/// Decides nothing about the GATE: every round's verdict is already fixed before
/// this runs, and the coverage asserts it feeds sit BELOW the verdict so they can
/// never pre-empt one. What it does decide is whether those coverage asserts have
/// anything to look at, so it is sized as a liveness backstop rather than as a
/// window — it is derived from [`SENTINEL_DELIVERY_DEADLINE`] for exactly that
/// reason, and is paid at most once per run and only when something is missing.
/// A 5 s window was tried first and REPRODUCIBLY false-RED a run whose eight
/// rounds had all delivered: the viewer was simply further behind than that, and
/// a wall that decides on how far behind a viewer is is the load-sensitive class this
/// arm exists to remove. With nothing pressing the queue the sentinels of
/// delivered rounds are one message behind their samples; measured, they arrive
/// 1.3 us - 219.5 ms after their round's flush even WITH pressure running.
const SENTINEL_PREMISE_SETTLE: Duration = SENTINEL_DELIVERY_DEADLINE;

/// How many pressure rounds `a_plot_sample_...` runs. Bounded on rounds, not on
/// wall time, so a slow runner takes longer rather than reporting a different
/// verdict.
///
/// The DROP half of a round is now DRIVEN rather than raced (each
/// round presses until the proxy's own accounting reports one), and now
/// the DELIVERY half is not raced either: it waits for an ORDER
/// SENTINEL instead of a deadline (see [`PLOT_SENTINEL_ENTITY`]). So these rounds
/// are no longer RETRIES of a race. They are independent trials: EVERY round in
/// which the gate is confirmed dropping on both sides of its sample must deliver
/// that sample, and the first one that does not fails the arm.
///
/// That is why the count still earns its keep. The old shape passed as soon as
/// ONE of eight rounds delivered, so a gate shedding seven samples in eight went
/// green; now every confirmed round must deliver, so a gate that sheds with
/// per-round probability p passes only by getting lucky in ALL of them (p^n,
/// where the old shape needed luck just once, 1-(1-p)^n).
///
/// The cost is wall: a healthy run now spends all eight rounds' pressure phases
/// instead of breaking at the first, which on an idle desk is ~2 s in total and
/// is bounded by rounds rather than by time — so a slow runner takes longer
/// rather than reporting a different verdict, which is this file's standing rule.
const PLOT_ROUNDS: usize = 8;

/// CONSECUTIVE rounds whose pressure phases may fail to confirm before
/// `a_plot_sample_...` gives up.
///
/// The DROP half of a round is driven, so an unconfirmed phase is normally the
/// apparatus having changed rather than a race lost — which is why this is not
/// simply `PLOT_ROUNDS`. But a phase can also exhaust its WALL, and on a
/// pathologically slow runner ONE such round is a transient rather than a
/// verdict, so one retry is granted before the arm reports.
///
/// Consecutive, not cumulative: a confirmed round is evidence the apparatus is
/// fine, so an earlier transient must not be held against a later one. And now
/// every confirmed round is also a TRIAL the gate must pass, so
/// resetting the streak keeps the trials coming.
///
/// The bound that buys: an all-unconfirmed run ends after two rounds rather than
/// eight, i.e. ~2 x (2 x `PRESSURE_DEADLINE`). It does NOT bound an ALTERNATING
/// run, which resets the streak each time and can pay that wall once per
/// unconfirmed round — bounded by `PLOT_ROUNDS`, not by this.
const PRESSURE_UNCONFIRMED_ROUNDS_TOLERATED: usize = 2;

#[test]
fn a_stream_of_small_messages_cannot_wedge_the_proxy() {
    with_runtime(|| {
        // The live channel is bounded on TWO axes — `CHANNEL_SIZE_BYTES` (128 MiB)
        // and `CHANNEL_SIZE_MESSAGES` (1024) — and `send_async` awaits when EITHER
        // is reached. Every other arm in this file streams 2.6 MiB image frames,
        // where the BYTE budget always trips first, so none of them can see the
        // message axis.
        //
        // A robot's ordinary traffic is the opposite shape: plots, `/tf` and
        // telemetry are kilobytes each, so 1024 of them reach the message quota at
        // a few MiB — well under an 8 MiB byte budget. With a byte-only gate the
        // budget never fires, the send awaits, and the event loop stalls; because
        // that loop also serves `Event::NewClient`, a NEW viewer could then never
        // connect. This arm pins that it does not happen.
        let addr = probe_free_addr();
        let (rx, handle) = re_grpc_server::spawn_with_recv(
            addr,
            cerulion_vizd::host::server_options(),
            re_grpc_server::shutdown::never(),
        );
        let uri = proxy_uri(addr);
        let prod =
            re_grpc_client::Client::new(uri.clone(), re_grpc_client::write::Options::default());
        let store = StoreId::random(StoreKind::Recording, "m");
        prod.send_blocking(set_store_info(&store));

        // Well past the 1024-message quota. `send_blocking` only ENQUEUES (the
        // client's background task does the wire encode), so flush before reading.
        for seq in 0..2000 {
            prod.send_blocking(small_frame(&store, "telemetry/scalar", seq));
        }
        let _ = prod.flush_blocking(Duration::from_secs(20));

        // SCOPE — this arm pins LIVENESS, not the message-axis gate.
        //
        // It cannot drive the gate: `spawn_with_recv`'s `LogReceiver` runs its own
        // forwarding task that pulls from the broadcast into a byte-bounded
        // channel of its own. With 2.6 MiB image frames that channel fills and
        // stops pulling (which is how every other arm here builds a backlog), but
        // 2000 kilobyte-sized messages fit in it comfortably, so the broadcast
        // stays drained and `live_dropped` legitimately reads 0 — MEASURED
        // (`broadcast: 0, live_dropped: 0`), which is why this arm does NOT assert
        // a drop. The message-axis decision itself is pinned where it can be
        // driven exactly: the fork's `the_message_quota_is_guarded_not_just_the_byte_budget`
        // oracle over `decide_live`.
        //
        // What it DOES pin is the consequence that matters: the proxy stays
        // RESPONSIVE across a burst far larger than the 1024-message quota, and a
        // fresh viewer can still connect. `usage()` panics if the event loop never
        // answers, which is the wedge signature itself.
        let _ = usage(&handle, Duration::from_secs(5));

        // And the end-to-end consequence: a FRESH viewer still connects.
        let consumer = re_grpc_client::stream(uri);
        let deadline = Instant::now() + Duration::from_secs(6);
        let mut connected = false;
        while Instant::now() < deadline && !connected {
            if let Ok(sm) = consumer.recv_timeout(Duration::from_millis(200)) {
                if let Some(re_log_channel::DataSourceMessage::LogMsg(LogMsg::SetStoreInfo(_))) =
                    sm.into_data()
                {
                    connected = true;
                }
            }
        }
        assert!(
            connected,
            "a fresh viewer must still connect after 2000 small messages backed up behind \
             an undrained receiver — the event loop must not be wedged on the message quota"
        );
        drop(prod);
        drop(rx);
    });
}

#[test]
fn the_replay_history_still_holds_no_temporal_frames() {
    with_runtime(|| {
        // `drop_temporal_history`'s contract, re-asserted in BYTES over the live-backlog stimulus.
        // Without this arm, a "fix" that moved the backlog from the live queue
        // into the retained replay history would pass the headline pin.
        let (w, h) = RENDITIONS[1];
        let r = run_stream(false, w, h);
        assert_eq!(
            r.last.disposable, 0,
            "the per-client REPLAY history must hold NO temporal frames \
             (drop_temporal_history), got {:?}",
            r.last
        );
        assert!(
            r.last.persistent > 0,
            "the run must actually have reached the proxy (the SetStoreInfo is persistent)"
        );
    });
}

#[test]
fn the_budget_admits_a_whole_camera_frame_with_headroom() {
    // Sizing drift guard for the CAMERA rendition — the high-rate stream the
    // budget was sized against, NOT "the largest message vizd can emit". `host.rs`
    // says so explicitly in the same PR: an occupancy grid reaches ~32 MB and
    // `Points3D` is uncapped, both far above this. Naming this arm after the
    // largest message would promise coverage it does not have (raising
    // `MAX_OCCUPANCY_PIXELS` leaves it green).
    //
    // The budget is compared against the queue's CURRENT occupancy, so one message
    // always crosses however large it is — but a budget BELOW one frame would drop
    // on ordinary scheduling jitter, costing frames a viewer that IS keeping up
    // could have displayed.
    let largest = rgb_bytes(RENDITIONS[1].0, RENDITIONS[1].1) as u64;
    assert!(
        LIVE_TEMPORAL_BUDGET_BYTES > largest,
        "the live budget ({LIVE_TEMPORAL_BUDGET_BYTES}) must exceed one camera frame \
         ({largest} = 1280x720 RGB8)"
    );
    // And not so large that it re-admits a visible backlog: at 30 Hz, the budget
    // must be worth well under half a second of video. The `+ 1` is the message
    // the gate admits into a within-budget queue — omitting it understated the
    // real bound (the measured peak is 4.0 frames, not 3).
    let frames_in_flight = (LIVE_TEMPORAL_BUDGET_BYTES / largest) + 1;
    let seconds_of_video = frames_in_flight as f64 / FPS;
    assert!(
        seconds_of_video < 0.5,
        "the live budget must be worth well under half a second of video, got {seconds_of_video:.2}s"
    );
}

/// What the live queue ACCOUNTS for one message — the exact quantity the budget
/// and the floor are compared against.
///
/// NOT the payload this daemon built: the encoded, LZ4-compressed proto measures
/// ~0.5% LARGER than a raw image and is mostly schema overhead for a one-row chunk
/// (a 1-value `Scalars` sample encodes to ~1.2 KB). The fork pins this helper
/// against the server's OWN accounting in
/// `the_live_budget_gate_is_wired_to_the_real_broadcast_queue`, so the numbers
/// asserted below are the numbers `decide_live` sees.
///
/// It is NOT measured through a live proxy here, and that is a real constraint
/// rather than a preference: `spawn_with_recv`'s receiver runs a forwarding task
/// that pulls from the broadcast into a byte-bounded channel of its own, and a
/// kilobyte-sized message fits in it comfortably — so the broadcast occupancy of
/// exactly the class under test here never rises above 0 (MEASURED; it is the
/// same mechanism `a_stream_of_small_messages_cannot_wedge_the_proxy` documents).
fn accounted_size(msg: &LogMsg) -> u64 {
    re_grpc_server::live_queue_size_bytes(msg).expect("the message encodes")
}

#[test]
fn the_small_message_floor_sits_between_control_and_image_traffic() {
    // The floor's whole justification is a SIZE BAND: vizd's control traffic is
    // kilobytes and its image traffic is megabytes, so exempting the former from
    // the byte axis costs the viewer nothing measurable while saving one-shot
    // state transitions that never come again. That is a claim about THIS
    // daemon's real message classes, so it is measured rather than asserted —
    // `host.rs` cites this arm for it.
    let store = StoreId::random(StoreKind::Recording, "f");

    // CONTROL CLASS — the archetypes vizd logs temporally that are not sensor
    // frames: a plot sample (`archetype.rs` `log_scalar`), a marker delete
    // (`marker.rs` `log_marker_clear` — the one-shot that never self-heals), a
    // rosout line (`log_text`), and a live `/tf` pose (`sink.rs`, temporal for
    // `/tf` and `log_static` only for `/tf_static`).
    let control: Vec<(&str, LogMsg)> = vec![
        (
            "Scalars (a plot sample)",
            temporal_chunk(&store, "plot/x", 1, &rerun::archetypes::Scalars::new([1.5])),
        ),
        (
            "Clear::recursive (a marker delete)",
            temporal_chunk(
                &store,
                "markers/ns/3",
                1,
                &rerun::archetypes::Clear::recursive(),
            ),
        ),
        (
            "TextLog (a rosout line)",
            temporal_chunk(
                &store,
                "log",
                1,
                &rerun::archetypes::TextLog::new(
                    "a fairly typical rosout line about something happening",
                ),
            ),
        ),
        (
            "Transform3D (a live /tf pose)",
            temporal_chunk(
                &store,
                "tf/base_link",
                1,
                &rerun::archetypes::Transform3D::from_translation([1.0, 2.0, 3.0]),
            ),
        ),
        (
            "Points2D (the small_frame helper)",
            small_frame(&store, PLOT_ENTITY, 1),
        ),
        // The ORDER SENTINEL. Its own doc says it must stay under the floor as
        // defence in depth on the byte axis, and nothing else measures that —
        // which is how a "harmonisation" onto `big_static_frame` would slip
        // through and quietly make the sentinel a second thing under test.
        (
            "the order sentinel",
            order_sentinel(&store, PLOT_SENTINEL_ENTITY),
        ),
    ];
    for (label, msg) in control {
        let n = accounted_size(&msg);
        assert!(
            n < LIVE_SMALL_MESSAGE_FLOOR_BYTES,
            "{label} accounts {n} bytes, which is NOT under the small-message floor \
             ({LIVE_SMALL_MESSAGE_FLOOR_BYTES}). The floor is what makes `host.rs`'s claim true — \
             that vizd's control traffic is never collateral of an image backlog. If this class \
             really has grown past the floor, the floor is what must move, and the worst-case \
             bound in the fork's `LIVE_SMALL_MESSAGE_FLOOR_BYTES` doc must be recomputed with it."
        );
    }

    // IMAGE CLASS — the SMALLER of the two renditions a Go2 attach interleaves. If
    // even this were under the floor, the byte axis would be exempting the very
    // traffic it exists to shed.
    let (w, h) = RENDITIONS[0];
    let n = accounted_size(&image_frame(
        &store,
        "cam",
        1,
        &incompressible(rgb_bytes(w, h), 0x5A),
        w,
        h,
    ));
    assert!(
        LIVE_SMALL_MESSAGE_FLOOR_BYTES <= n,
        "a {w}x{h} RGB8 frame accounts {n} bytes, which is UNDER the small-message floor \
         ({LIVE_SMALL_MESSAGE_FLOOR_BYTES}) — the floor would then exempt image frames from the \
         byte axis, disarming the budget entirely"
    );

    // The ORDER SENTINEL's OTHER defining property, pinned here rather than left
    // to the arm's own runtime check — which can only look at a sentinel that
    // ARRIVED. This one is unconditional, and it reads the CHUNK's `is_static`;
    // the gate reads the PROTO flag (`arrow.is_static`), so a change that lost
    // the flag in transport would leave both pins green while the sentinel fell
    // back to the small-message floor, i.e. the guard under test.
    let LogMsg::ArrowMsg(_, arrow) = order_sentinel(&store, PLOT_SENTINEL_ENTITY) else {
        panic!("the order sentinel must be an ArrowMsg");
    };
    assert!(
        re_chunk::Chunk::from_arrow_msg(&arrow)
            .expect("the order sentinel decodes")
            .is_static(),
        "the ORDER SENTINEL must be a STATIC chunk. That is the guard that makes it \
         decisive — `is_temporal` is false for a static, so `decide_live` returns Admit before \
         either axis is consulted, INDEPENDENTLY of the small-message floor this file exists to \
         pin. A temporal sentinel would ride the floor instead, i.e. the very guard under test, \
         and `a_plot_sample_survives_a_queue_full_of_camera_frames` would go vacuous."
    );

    // The PROBE shapes the skeleton arm relies on must clear the floor too, or
    // that arm goes vacuous for a NEW reason: a misclassified skeleton message
    // would be admitted by the floor instead of by its exemption.
    for (label, msg) in [
        (
            "the late static probe",
            big_static_frame(&store, LATE_STATIC_ENTITY),
        ),
        (
            "the blueprint probe",
            big_blueprint_chunk(&StoreId::random(StoreKind::Blueprint, "fbp")),
        ),
    ] {
        let n = accounted_size(&msg);
        assert!(
            LIVE_SMALL_MESSAGE_FLOOR_BYTES <= n,
            "{label} accounts {n} bytes, UNDER the floor \
             ({LIVE_SMALL_MESSAGE_FLOOR_BYTES}) — \
             `a_fresh_viewer_still_gets_its_scene_while_the_live_queue_is_over_budget` would then \
             pass under a skeleton-guard mutant because of the floor, not because of the exemption \
             it claims to pin. Raise PROBE_POINTS."
        );
    }
}

/// A temporal chunk for an arbitrary archetype — the shape every one of vizd's
/// non-sensor `rec.log` calls produces.
fn temporal_chunk(
    store_id: &StoreId,
    entity: &str,
    seq: i64,
    archetype: &dyn rerun::AsComponents,
) -> LogMsg {
    let tp = re_log_types::TimePoint::default().with(
        re_log_types::Timeline::new_sequence("frame"),
        re_log_types::TimeInt::new_temporal(seq),
    );
    let chunk = re_chunk::Chunk::builder(entity)
        .with_archetype(re_chunk::RowId::new(), tp, archetype)
        .build()
        .expect("build temporal chunk");
    LogMsg::ArrowMsg(store_id.clone(), chunk.to_arrow_msg().expect("to arrow"))
}

// ---------------------------------------------------------------------------
// Measurement (print-only, `#[ignore]`d) — the reproduction numbers quoted above.
// ---------------------------------------------------------------------------

fn percentile(sorted: &[Duration], p: f64) -> Duration {
    if sorted.is_empty() {
        return Duration::ZERO;
    }
    sorted[(((sorted.len() - 1) as f64) * p).round() as usize]
}

/// Total CPU (user + system) this PROCESS has consumed, across all threads.
///
/// `send_blocking` only ENQUEUES — the wire encode (`to_transport`: IPC
/// serialize + compression) and the gRPC write happen on the client's background
/// task, and the proxy's work on its event loop — so a per-call wall timing
/// measures none of the cost this issue is about. Process CPU over a window
/// captures all of it regardless of which thread paid.
fn process_cpu() -> Duration {
    // SAFETY: `getrusage` fills a caller-owned `rusage`; RUSAGE_SELF is valid.
    let mut ru: libc::rusage = unsafe { std::mem::zeroed() };
    let rc = unsafe { libc::getrusage(libc::RUSAGE_SELF, &mut ru) };
    assert_eq!(rc, 0, "getrusage(RUSAGE_SELF) failed");
    let secs = |t: libc::timeval| Duration::new(t.tv_sec as u64, (t.tv_usec as u32) * 1000);
    secs(ru.ru_utime) + secs(ru.ru_stime)
}

#[test]
#[ignore = "measurement, not a pin — run with --ignored --nocapture"]
fn measure_encode_cost_per_decoded_picture() {
    const ITERS: usize = 40;
    let store = StoreId::random(StoreKind::Recording, "enc");
    println!("\nlive-backlog (1) ENCODE cost per decoded picture (log_raw_image → arrow wire msg)");
    println!(
        "  {:<12} {:>12} {:>10} {:>10} {:>12} {:>14}",
        "rendition", "raw bytes", "p50", "p99", "MB/s raw", "core @30Hz"
    );
    let mut total_frac = 0.0;
    for (w, h) in RENDITIONS {
        let n = rgb_bytes(w, h);
        let rgb = incompressible(n, 0xA5);
        for i in 0..4 {
            let _ = std::hint::black_box(image_frame(&store, "cam", i, &rgb, w, h));
        }
        let mut samples = Vec::with_capacity(ITERS);
        for i in 0..ITERS {
            let t = Instant::now();
            let msg = image_frame(&store, "cam", i as i64, &rgb, w, h);
            samples.push(t.elapsed());
            drop(std::hint::black_box(msg));
        }
        samples.sort();
        let frac = percentile(&samples, 0.50).as_secs_f64() * FPS;
        total_frac += frac;
        println!(
            "  {:<12} {:>12} {:>10.2?} {:>10.2?} {:>12.1} {:>13.1}%",
            format!("{w}x{h}"),
            n,
            percentile(&samples, 0.50),
            percentile(&samples, 0.99),
            n as f64 * FPS / 1e6,
            frac * 100.0
        );
    }
    println!(
        "  BOTH renditions (one attach, interleaved on one topic): {:.1}% of a core to ENCODE, \
         {:.1} MB/s raw RGB into the pipe",
        total_frac * 100.0,
        RENDITIONS
            .iter()
            .map(|&(w, h)| rgb_bytes(w, h) as f64)
            .sum::<f64>()
            * FPS
            / 1e6
    );
}

#[test]
#[ignore = "measurement, not a pin — run with --ignored --nocapture"]
fn measure_live_queue_backlog() {
    for drained in [true, false] {
        with_runtime(|| {
            let (w, h) = RENDITIONS[1];
            let cpu0 = process_cpu();
            let started = Instant::now();
            let r = run_stream(drained, w, h);
            let wall = started.elapsed();
            let cpu = process_cpu() - cpu0;
            let label = if drained {
                "DRAINED (viewer keeps up)"
            } else {
                "UNDRAINED (viewer cannot keep up)"
            };
            println!("\nlive-backlog (2) LIVE-QUEUE — {label}");
            println!("  frame        : {w}x{h} RGB8 = {} bytes", r.frame_bytes);
            println!("  frames sent  : {FRAMES} at ~30 Hz (wall {wall:.2?})");
            println!(
                "  PROCESS CPU  : {cpu:.2?} = {:.1}% of a core (encode + IPC + compression + gRPC + proxy)",
                cpu.as_secs_f64() / wall.as_secs_f64() * 100.0
            );
            println!(
                "  live queue   : peak {} bytes ({:.1} frames), budget {LIVE_TEMPORAL_BUDGET_BYTES}",
                r.peak_broadcast,
                r.peak_broadcast as f64 / r.frame_bytes as f64
            );
            println!(
                "  dropped      : {} bytes ({:.1} frames)",
                r.last.live_dropped,
                r.last.live_dropped as f64 / r.frame_bytes as f64
            );
            print!("  occupancy    :");
            for (seq, b) in r.series.iter().enumerate().step_by(6) {
                print!(" {seq}:{:.1}", *b as f64 / r.frame_bytes as f64);
            }
            println!();
        });
    }
}

// ---------------------------------------------------------------------------
// Measurement: what a viewer actually RECEIVES while the gate sheds.
// ---------------------------------------------------------------------------

/// What one round of [`measure_plot_sample_delivery_under_pressure`] observed.
#[derive(Debug)]
struct DeliveryProbe {
    round: usize,
    pressure_held: bool,
    dropped_in_round: u64,
    /// Messages the viewer received between this round's start and its verdict,
    /// by class. `other` is the discriminator the sentinel arm needs measured: entities
    /// that are neither this round's sample nor its sentinel.
    plot_msgs: usize,
    sentinel_msgs: usize,
    other_msgs: usize,
    /// Wall from the round's flush until this round's sample appeared.
    sample_after: Option<Duration>,
    /// Same for the ORDER SENTINEL (a STATIC logged immediately after the
    /// sample, so `is_temporal` is false and the gate may never shed it).
    sentinel_after: Option<Duration>,
    /// The SAME classification the shipped arm convicts on, so this
    /// measurement's SHED column cannot drift from the pin's verdict — DEMOTED
    /// to `Apparatus` when this round could not observe cleanly. Note `Shed` here
    /// is NOT the arm's `order_violation`, which names the opposite problem (a
    /// sentinel that overtook its sample).
    verdict: ProbeVerdict,
}

/// What one measured round is allowed to CLAIM.
///
/// `Outcome::Shed` is inferred from "the sample is absent and the sentinel is
/// present", and a chunk that arrived but failed to decode is absent for a reason
/// that has nothing to do with the gate. So an observation failure DEMOTES the
/// round rather than being classified: the measurement may report that it could
/// not see, and may never report shedding it did not observe.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ProbeVerdict {
    Observed(Outcome),
    /// This round saw a receive or decode failure, so its absence evidence is
    /// worthless. Carries what the (untrustworthy) classification would have
    /// been, because a reader debugging the apparatus wants both.
    Apparatus {
        would_have_been: Outcome,
        decode_failures: usize,
        stream_dead: bool,
    },
}

#[test]
#[ignore = "measurement, not a pin — run with --ignored --nocapture"]
fn measure_plot_sample_delivery_under_pressure() {
    with_runtime(|| {
        let (w, h) = RENDITIONS[1];
        let frame_bytes = rgb_bytes(w, h);
        let rgb = incompressible(frame_bytes, 0x5A);

        let addr = probe_free_addr();
        let (rx, handle) = re_grpc_server::spawn_with_recv(
            addr,
            cerulion_vizd::host::server_options(),
            re_grpc_server::shutdown::never(),
        );
        let uri = proxy_uri(addr);
        let prod =
            re_grpc_client::Client::new(uri.clone(), re_grpc_client::write::Options::default());
        let store = StoreId::random(StoreKind::Recording, "m");
        prod.send_blocking(set_store_info(&store));

        let flood = flood_until_budget_engages(&prod, &handle, &store, &rgb, w, h);
        assert_budget_engaged(&flood);

        // QUIESCE exactly as the arm does. This function is cited as the evidence
        // for the arm's design, so it has to model the apparatus it justifies —
        // without this the first round starts from a different queue state and
        // its `other` column measures the settling rather than the steady state.
        let _ = prod.flush_blocking(Duration::from_secs(20));
        let mut u = usage(&handle, Duration::from_secs(5));
        for _ in 0..40 {
            std::thread::sleep(Duration::from_millis(250));
            let next = usage(&handle, Duration::from_secs(5));
            if next.live_dropped == u.live_dropped {
                break;
            }
            u = next;
        }

        // An ORDERED log of every entity path the viewer received, so a round can
        // ask "what had arrived by then", not merely "did ours".
        let seen: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let stop = Arc::new(AtomicBool::new(false));
        // Recorded for the same reason the acceptance drainer records them: a
        // chunk that arrives and fails to DECODE never reaches `seen`, and
        // "absent from `seen`" is exactly what `Outcome::Shed` is inferred from.
        // Left unrecorded, an undecodable delivered sample would be published in
        // this table as evidence that the gate SHED it — a measurement asserting
        // the opposite of what happened, in the function whose numbers justify
        // the pin.
        let decode_failures = Arc::new(AtomicUsize::new(0));
        let stream_dead = Arc::new(AtomicBool::new(false));
        let consumer = re_grpc_client::stream(uri);
        let drain_seen = Arc::clone(&seen);
        let drain_stop = Arc::clone(&stop);
        let drain_decode_failures = Arc::clone(&decode_failures);
        let drain_stream_dead = Arc::clone(&stream_dead);
        let drainer = std::thread::spawn(move || {
            while !drain_stop.load(Ordering::Relaxed) {
                match consumer.recv_timeout(Duration::from_millis(50)) {
                    Ok(sm) => {
                        if let Some(re_log_channel::DataSourceMessage::LogMsg(LogMsg::ArrowMsg(
                            _,
                            arrow,
                        ))) = sm.into_data()
                        {
                            match re_chunk::Chunk::from_arrow_msg(&arrow) {
                                Ok(chunk) => drain_seen
                                    .lock()
                                    .expect("the seen log is not poisoned")
                                    .push(chunk.entity_path().to_string()),
                                Err(_) => {
                                    drain_decode_failures.fetch_add(1, Ordering::Relaxed);
                                }
                            }
                        }
                    }
                    Err(_) => {
                        if !consumer.is_connected() {
                            drain_stream_dead.store(true, Ordering::Relaxed);
                            break;
                        }
                    }
                }
            }
        });
        std::thread::sleep(Duration::from_millis(500));

        let pressure = Pressure {
            prod: &prod,
            handle: &handle,
            store: &store,
            rgb: &rgb,
            w,
            h,
        };
        let snapshot = |from: usize| -> Vec<String> {
            seen.lock().expect("the seen log is not poisoned")[from..].to_vec()
        };

        let mut probes = Vec::new();
        for round in 0..PLOT_ROUNDS {
            let sample = entity_path(&format!("{PLOT_ENTITY}/r{round}")).to_string();
            let sentinel = entity_path(&format!("{PLOT_SENTINEL_ENTITY}/r{round}")).to_string();
            let from = seen.lock().expect("the seen log is not poisoned").len();
            // Snapshotted per round so an observation failure is attributed to
            // the round it could have mis-described, not to the whole run.
            let errors_before = decode_failures.load(Ordering::Relaxed);
            let base = round as i64 * 1000;
            let d0 = usage(&handle, Duration::from_secs(5)).live_dropped;

            let before = pressure.until_dropping(base, d0);
            prod.send_blocking(small_frame(&store, &format!("{PLOT_ENTITY}/r{round}"), 1));
            prod.send_blocking(order_sentinel(
                &store,
                &format!("{PLOT_SENTINEL_ENTITY}/r{round}"),
            ));
            let after_probe = pressure.until_dropping(base + 500, before.last.live_dropped);
            let _ = prod.flush_blocking(Duration::from_secs(20));

            // The RETIRED 5 s question (what the arm used to decide on) and the
            // order-aware one, so one run reports both and the table shows which
            // rounds the old shape would have failed.
            let t0 = Instant::now();
            let mut sample_after = None;
            let mut sentinel_after = None;
            // The shipped constant, so the measurement can observe the whole
            // range the arm is allowed to wait over rather than half of it.
            let wall = SENTINEL_DELIVERY_DEADLINE;
            while t0.elapsed() < wall && (sample_after.is_none() || sentinel_after.is_none()) {
                let log = snapshot(from);
                if sample_after.is_none() && log.contains(&sample) {
                    sample_after = Some(t0.elapsed());
                }
                if sentinel_after.is_none() && log.contains(&sentinel) {
                    sentinel_after = Some(t0.elapsed());
                }
                if sample_after.is_none() || sentinel_after.is_none() {
                    std::thread::sleep(Duration::from_millis(20));
                }
            }
            let log = snapshot(from);
            probes.push(DeliveryProbe {
                round,
                pressure_held: before.confirmed && after_probe.confirmed,
                dropped_in_round: after_probe.last.live_dropped.saturating_sub(d0),
                plot_msgs: log.iter().filter(|e| **e == sample).count(),
                sentinel_msgs: log.iter().filter(|e| **e == sentinel).count(),
                other_msgs: log
                    .iter()
                    .filter(|e| **e != sample && **e != sentinel)
                    .count(),
                sample_after,
                sentinel_after,
                verdict: {
                    let would_have_been = Outcome::classify(Sightings {
                        sample: sample_after.is_some(),
                        sentinel: sentinel_after.is_some(),
                    });
                    let failed_decodes = decode_failures.load(Ordering::Relaxed) - errors_before;
                    let dead = stream_dead.load(Ordering::Relaxed);
                    if failed_decodes > 0 || dead {
                        ProbeVerdict::Apparatus {
                            would_have_been,
                            decode_failures: failed_decodes,
                            stream_dead: dead,
                        }
                    } else {
                        ProbeVerdict::Observed(would_have_been)
                    }
                },
            });
        }

        // Pressure has stopped. Anything ADMITTED is still in the queue, so a
        // sample missing HERE was shed rather than merely late.
        std::thread::sleep(Duration::from_secs(5));
        let tail = seen.lock().expect("the seen log is not poisoned").clone();
        stop.store(true, Ordering::Relaxed);
        drainer.join().expect("the viewer drainer does not panic");

        println!("\nlive-backlog (3) PLOT-SAMPLE DELIVERY under a shedding gate");
        println!(
            "  frame {frame_bytes} B, budget {LIVE_TEMPORAL_BUDGET_BYTES} B, floor \
             {LIVE_SMALL_MESSAGE_FLOOR_BYTES} B"
        );
        println!(
            "  {:>5} {:>8} {:>12} {:>6} {:>8} {:>6} {:>12} {:>12} {:>10}",
            "round",
            "pressure",
            "dropped",
            "plot",
            "sentinel",
            "other",
            "sample@",
            "sentinel@",
            "verdict"
        );
        for p in &probes {
            println!(
                "  {:>5} {:>8} {:>12} {:>6} {:>8} {:>6} {:>12} {:>12} {:>10}",
                p.round,
                p.pressure_held,
                p.dropped_in_round,
                p.plot_msgs,
                p.sentinel_msgs,
                p.other_msgs,
                p.sample_after
                    .map_or_else(|| "-".to_owned(), |d| format!("{d:.2?}")),
                p.sentinel_after
                    .map_or_else(|| "-".to_owned(), |d| format!("{d:.2?}")),
                match p.verdict {
                    ProbeVerdict::Observed(Outcome::Delivered) => "delivered".to_owned(),
                    ProbeVerdict::Observed(Outcome::Shed) => "SHED".to_owned(),
                    ProbeVerdict::Observed(Outcome::ViewerNeverReached) => "no-show".to_owned(),
                    ProbeVerdict::Apparatus {
                        decode_failures,
                        stream_dead,
                        ..
                    } => format!("APPARATUS({decode_failures},dead={stream_dead})"),
                }
            );
        }
        let within_5s = probes
            .iter()
            .filter(|p| p.sample_after.is_some_and(|d| d < Duration::from_secs(5)))
            .count();
        let eventually = probes
            .iter()
            .filter(|p| {
                tail.iter()
                    .any(|e| *e == entity_path(&format!("{PLOT_ENTITY}/r{}", p.round)).to_string())
            })
            .count();
        println!(
            "  samples within the RETIRED 5 s deadline: {within_5s}/{PLOT_ROUNDS}; ever \
             delivered: {eventually}/{PLOT_ROUNDS}; SHED (sentinel arrived, sample did \
             not): {}; rounds this drainer could NOT observe cleanly: {}",
            probes
                .iter()
                .filter(|p| p.verdict == ProbeVerdict::Observed(Outcome::Shed))
                .count(),
            probes
                .iter()
                .filter(|p| matches!(p.verdict, ProbeVerdict::Apparatus { .. }))
                .count()
        );
        // The numbers above are only evidence if the drainer saw cleanly. Said
        // out loud rather than left to a reader to notice a zero: this function
        // is cited by the arm's design comment and by the PR that re-pinned it.
        println!(
            "  drainer: {} chunk(s) failed to decode, stream {} — any nonzero here means the \
             SHED column is NOT usable as gate evidence",
            decode_failures.load(Ordering::Relaxed),
            if stream_dead.load(Ordering::Relaxed) {
                "DIED"
            } else {
                "alive"
            }
        );
        println!(
            "  total messages the viewer received: {} (distinct entities {})",
            tail.len(),
            tail.iter().collect::<HashSet<_>>().len()
        );
        drop(prod);
        drop(rx);
    });
}
