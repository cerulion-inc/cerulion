// SPDX-License-Identifier: AGPL-3.0-only
//! PHASE-1 DIAGNOSTIC HARNESS — is the `live_backlog_test` flake
//! family a REAL load-sensitive gap in the byte-axis guarantee, or an ORACLE that
//! over-claims under starvation?
//!
//! **Nothing in this file is a pin.** Every test is `#[ignore]`d and prints a
//! machine-readable `shed-starvation …` line; the sibling `live_backlog_test.rs`
//! is the gate and is deliberately untouched. This exists so the question is
//! settled by MEASUREMENT rather than by another tolerance widening — its
//! own standing instruction is that a fix which loosens an oracle is refused
//! unless the mechanism analysis proves the guarantee itself is load-relative.
//!
//! # The subject, restated from the fork
//!
//! `re_grpc_server`'s `decide_live` (the patch) is an ARRIVAL gate on a
//! SINGLE-THREADED event loop:
//!
//! ```text
//! let over_bytes    = LIVE_SMALL_MESSAGE_FLOOR_BYTES <= msg_bytes && budget < occ.bytes;
//! let over_messages = occ.max_messages <= occ.messages + LIVE_MESSAGE_RESERVE;
//! if over_bytes || over_messages { Drop } else { Admit }
//! ```
//!
//! Two facts fall straight out of that and shape every probe below:
//!
//!  * The BYTE axis exempts anything under the 8 KiB floor, so a 1286-byte plot
//!    sample can only be shed by the MESSAGE axis (`messages_in_flight >= 960`),
//!    which a stream of 2.6 MiB camera frames cannot reach — the byte quota caps
//!    the queue at a handful of messages long before 960.
//!  * `occ.bytes` is `bytes_in_flight`, and a message's bytes are freed only when
//!    EVERY subscriber's `recv()` has taken it (`TrackedInner::drop` in
//!    `re_quota_channel`). So a subscriber that stops calling `recv()` pins the
//!    queue PERMANENTLY — occupancy cannot fall, and every camera frame after that
//!    is dropped for the rest of the run.
//!
//! # What the CI dumps actually show, which refuted the obvious guess
//!
//! The obvious reading is a delivery race: admitted camera frames pile up ahead of
//! the plot sample, and the arm's 5 s per-round window cannot chew through them.
//! **The failing runs refute it.** In the `feat/flashback-anchor-first` firing both
//! pressure phases confirmed after **1 and 2 frames**, in 4.6 ms and 19.3 ms — so
//! essentially nothing was being admitted and the viewer owed nothing.
//!
//! What the same dump does show is the queue PINNED and the samples ADMITTED:
//! `broadcast` reads 11 118 204 at flood end — exactly four encoded camera frames —
//! and 11 129 016 at the end of the eighth and last probe round. That delta is **10 812 bytes ≈ eight
//! 1286-byte plot samples**. Camera frames are all being dropped, so the only thing
//! that can grow that number is the small messages the floor admitted.
//!
//! **The byte-axis guarantee HELD. The samples were admitted and never arrived.**
//!
//! # The candidate mechanisms these probes separate
//!
//! | # | Claim | The probe that decides it |
//! |---|---|---|
//! | A | the shed really can drop a sub-floor sample | `probe_a_small_sample_crosses_a_pinned_over_budget_queue` — load-free, deterministic |
//! | A′ | admitted but UNDELIVERABLE while a peer subscriber is stalled | the same probe, plus `SHED_DRAIN_RX=1` |
//! | B | the arm's DELIVERY oracle loses a race | `probe_the_delivery_oracle_under_a_starved_consumer` — the shadow viewer is the discriminator |
//! | C | `usage()` reads a STALE snapshot, inflating the stimulus | `probe_usage_snapshot_staleness_under_pressure` |
//!
//! (A′) is the leading candidate and it is a question about the HARNESS as much as
//! the product: the gate file never drains `spawn_with_recv`'s receiver, so the
//! over-budget state it measures under is manufactured by a stalled peer rather
//! than by a slow viewer — a condition a production vizd never enters, because it
//! drains its receiver. `SHED_DRAIN_RX` is what tells those two apart, and it is
//! the one knob whose answer decides whether anything in the product must change.
//!
//! (D) is the `a_slow_viewer` arm's own, DIFFERENT question — its two firings carry
//! `Usage { broadcast: 0, live_dropped: 0, persistent: 212 }`, i.e. not one of the
//! 60 frames ever reached the proxy. That is the starved-stimulus class,
//! and `run_stream` still uses the `send_blocking` + `sleep(33 ms)` shape with a
//! fixed 60-frame bound that the flood loop was rewritten to retire. It gets
//! `probe_the_slow_viewer_ceiling_against_the_encoded_message`.
//!
//! # Why the starvation is INJECTED, not ambient
//!
//! This repo's standing lesson is that "load" is not a stressor: background CPU
//! spin does not reproduce a CI starvation, because it does not starve the
//! particular thread whose starvation produces the signature. The signature here
//! is `Pressed { confirmed: true } … delivered = false`, i.e. the gate kept
//! deciding while the VIEWER stopped keeping up — so the thread to starve is the
//! test's own drain loop, and `SHED_CONSUMER_STALL_MS` starves exactly it, on
//! an idle desk, deterministically.
//!
//! # Knobs
//!
//! | Env | Default | What it does |
//! |---|---|---|
//! | `SHED_CONSUMER_STALL_MS` | 0 | per-message stall injected into the FOREGROUND drain — the (B) stressor |
//! | `SHED_ROUNDS` | 8 | rounds, mirroring `PLOT_ROUNDS` |
//! | `SHED_SETTLE_MS` | 250 | wall a "settled" read waits before believing a snapshot |
//! | `SHED_PRESSURE_CEILING` | 120 | frames one pressure phase may feed, mirroring `PRESSURE_FRAME_CEILING` |
//! | `SHED_DRAIN_RX` | 0 | drain `spawn_with_recv`'s receiver — the (A′) discriminator |
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_vizd --test shed_starvation_test -- \
//!     --ignored --nocapture --test-threads=1
//! ```

use std::net::{Ipv4Addr, SocketAddr, TcpListener};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::mpsc;
use std::sync::Arc;
use std::time::{Duration, Instant};

use re_log_types::{LogMsg, SetStoreInfo, StoreId, StoreInfo, StoreKind, StoreSource};

use cerulion_vizd::host::LIVE_TEMPORAL_BUDGET_BYTES;
use re_grpc_server::LIVE_SMALL_MESSAGE_FLOOR_BYTES;

/// The rendition the failing arms stream — the larger of the two a Go2 attach
/// interleaves, and the one whose encoded message is ~2.78 MB.
const RENDITION: (u32, u32) = (1280, 720);

/// The entity the plot sample rides, mirroring `PLOT_ENTITY` in the gate file.
const PLOT_ENTITY: &str = "telemetry/plot";

/// The camera entity, mirroring the gate file.
const CAM_ENTITY: &str = "cam/video/rendition";

// ───────────────────────────── knobs ─────────────────────────────

fn env_u64(key: &str, default: u64) -> u64 {
    std::env::var(key)
        .ok()
        .and_then(|v| v.trim().parse::<u64>().ok())
        .unwrap_or(default)
}

fn consumer_stall() -> Duration {
    Duration::from_millis(env_u64("SHED_CONSUMER_STALL_MS", 0))
}

fn rounds() -> usize {
    env_u64("SHED_ROUNDS", 8) as usize
}

fn settle() -> Duration {
    Duration::from_millis(env_u64("SHED_SETTLE_MS", 250))
}

fn pressure_ceiling() -> usize {
    env_u64("SHED_PRESSURE_CEILING", 120) as usize
}

/// Whether to DRAIN `spawn_with_recv`'s receiver in a background thread.
///
/// **This is the knob that separates "the queue is over budget" from "a peer
/// subscriber stopped receiving".** The gate file never drains that receiver, and
/// the CI dumps show the consequence: `bytes_in_flight` is PINNED (11 118 204 =
/// exactly four encoded camera frames) and never falls for the rest of the run.
/// A quota-channel message's bytes are freed only when EVERY subscriber's `recv()`
/// has taken it (`TrackedInner::drop`), so a subscriber that stops calling `recv()`
/// pins the queue permanently — which is what holds it over budget, and is a
/// condition a production vizd never enters, because it drains its receiver.
///
/// So the arm's over-budget state is manufactured by a stalled peer rather than by
/// a slow viewer, and the two are not the same claim.
fn drain_rx() -> bool {
    env_u64("SHED_DRAIN_RX", 0) != 0
}

// ─────────────────────────── shared fixture ───────────────────────────

fn rgb_bytes(w: u32, h: u32) -> usize {
    (w as usize) * (h as usize) * 3
}

/// An INCOMPRESSIBLE pixel buffer. Load-bearing for the same reason it is in the
/// gate file: the wire payload is LZ4-compressed, and a constant-filled buffer
/// measures ~5% of its raw size, which would make every occupancy number here a
/// fiction.
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

/// One decoded picture, built exactly as the production path does.
fn image_frame(store_id: &StoreId, seq: i64, rgb: &[u8], w: u32, h: u32) -> LogMsg {
    let tp = re_log_types::TimePoint::default().with(
        re_log_types::Timeline::new_sequence("frame"),
        re_log_types::TimeInt::new_temporal(seq),
    );
    let chunk = re_chunk::Chunk::builder(CAM_ENTITY)
        .with_archetype(
            re_chunk::RowId::new(),
            tp,
            &rerun::archetypes::Image::from_rgb24(rgb.to_vec(), [w, h]),
        )
        .build()
        .expect("build image chunk");
    LogMsg::ArrowMsg(store_id.clone(), chunk.to_arrow_msg().expect("to arrow"))
}

/// A SMALL temporal message — the control class the floor exists to protect.
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

/// What the LIVE queue ACCOUNTS for one message — the exact quantity `decide_live`
/// compares against, which is NOT the payload size (the encoded proto measures
/// ~0.5% larger than raw RGB, and that 0.5% is the whole of probe D).
fn accounted_size(msg: &LogMsg) -> u64 {
    re_grpc_server::live_queue_size_bytes(msg).expect("the message encodes")
}

fn with_runtime<R>(f: impl FnOnce() -> R) -> R {
    let rt = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()
        .expect("build tokio runtime");
    let _guard = rt.enter();
    f()
}

/// The proxy's own byte accounting for one instant.
#[derive(Debug, Default, Clone, Copy)]
struct Usage {
    broadcast: u64,
    live_dropped: u64,
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

/// The SAME read the gate file makes: fire a `CaptureMemory` request and take
/// whatever snapshot is already in the mutex.
///
/// It is deliberately NOT settled, because that staleness is itself under test —
/// `MessageProxyHandle::capture_memory` does a `try_send` on a 32-slot event mpsc
/// and returns the previous turn's numbers, silently dropping the refresh request
/// when that queue is full. Probe C measures the gap between this and
/// [`usage_settled`].
fn usage_now(handle: &re_grpc_server::MessageProxyHandle, bound: Duration) -> Usage {
    let deadline = Instant::now() + bound;
    loop {
        if let Some(tree) = handle.capture_memory() {
            if let (Some(broadcast), Some(live_dropped), Some(persistent)) = (
                child_bytes(&tree, "broadcast"),
                child_bytes(&tree, "live_dropped"),
                child_bytes(&tree, "persistent"),
            ) {
                return Usage {
                    broadcast,
                    live_dropped,
                    persistent,
                };
            }
        }
        assert!(
            Instant::now() < deadline,
            "the proxy event loop produced no memory snapshot within {bound:?} — it is not \
             running, so nothing measured here would mean anything"
        );
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// A read the event loop has DEMONSTRABLY serviced: ask, wait out `SHED_SETTLE_MS`,
/// ask again. On an idle loop the second read reflects a turn taken after the
/// first request; under a saturated 32-slot event queue it still may not, which is
/// exactly what probe C reports.
fn usage_settled(handle: &re_grpc_server::MessageProxyHandle) -> Usage {
    let _ = usage_now(handle, Duration::from_secs(5));
    std::thread::sleep(settle());
    usage_now(handle, Duration::from_secs(5))
}

// ─────────────────────── the shadow viewer ───────────────────────

/// The entity the shadow's readiness handshake rides.
///
/// SMALL by construction (a `small_frame`), so it is below
/// `LIVE_SMALL_MESSAGE_FLOOR_BYTES` and the byte axis cannot shed it — the
/// handshake must work against an ALREADY over-budget queue, which is exactly
/// the state probe B's shadow joins in. Distinct from [`PLOT_ENTITY`] and
/// carrying no `video` substring, so it contributes to neither the plot-sample
/// nor the camera-frame count.
const SHADOW_SENTINEL_ENTITY: &str = "telemetry/shadow_sentinel";

/// Consecutive empty reads that count as "this viewer's queue is drained".
///
/// Three at a 50 ms read timeout is 150 ms of silence — an order of magnitude
/// past the pressure loop's measured per-frame cadence, so a shadow that stops
/// here really has nothing left rather than having lost a race with its own
/// producer.
const SHADOW_QUIESCE_EMPTY_READS: usize = 3;

/// What the shadow viewer saw, keyed by entity path.
#[derive(Debug, Default)]
struct ShadowReport {
    /// Every entity path it received, in arrival order.
    entities: Vec<String>,
    /// Whether the receiver reported itself connected at the end.
    connected: bool,
    /// Whether the final drain reached silence rather than its wall ceiling.
    ///
    /// Load-bearing for every MISS the shadow reports: a shadow stopped at the
    /// ceiling was still owed messages, so its misses are its own read position
    /// and say nothing about what the proxy delivered.
    quiesced: bool,
    /// Sentinel frames received — the proof the gRPC subscription was really
    /// attached, not merely that a local receiver object existed.
    sentinels_seen: usize,
}

impl ShadowReport {
    fn saw(&self, entity: &str) -> bool {
        self.entities.iter().any(|e| e == entity)
    }

    fn camera_frames(&self) -> usize {
        self.entities.iter().filter(|e| e.contains("video")).count()
    }
}

/// A SECOND viewer on the same proxy, drained CONTINUOUSLY by its own thread and
/// never blocked on matching anything.
///
/// **This is the discriminator between (A) and (B).** The failing arm's viewer is
/// drained only inside a 5 s per-round window and DISCARDS every message that is
/// not that round's sample, so a viewer running one message behind can never
/// match — a self-sustaining failure that looks identical to the sample having
/// been shed. A viewer that never falls behind cannot confuse the two: if it
/// receives every plot sample, the byte-axis guarantee held and the arm's oracle
/// is what failed; if it misses them, the sample really was dropped.
struct Shadow {
    stop: Arc<AtomicBool>,
    /// Sentinels seen SO FAR, readable while the thread runs — what makes the
    /// closing handshake in [`Shadow::close`] an ordering proof rather than a
    /// post-mortem count.
    seen_sentinels: Arc<AtomicUsize>,
    handle: Option<std::thread::JoinHandle<ShadowReport>>,
}

impl Shadow {
    fn spawn(uri: re_uri::ProxyUri) -> (Self, mpsc::Receiver<()>) {
        let stop = Arc::new(AtomicBool::new(false));
        let stop_thread = Arc::clone(&stop);
        let seen_sentinels = Arc::new(AtomicUsize::new(0));
        let seen_thread = Arc::clone(&seen_sentinels);
        // The shadow signals ITS OWN readiness rather than the caller sleeping a
        // fixed wall for it — a fixed sleep is the class of precondition two earlier
        // flakes both fired on.
        let (ready_tx, ready_rx) = mpsc::channel();
        // `re_grpc_client::stream` calls `tokio::spawn`, which panics outside a
        // runtime context. `with_runtime` enters the runtime on the TEST thread
        // only — a plain `std::thread` does NOT inherit it — so the handle is
        // carried across and entered here. (Measured: without this the shadow
        // thread panics with "there is no reactor running".)
        let rt = tokio::runtime::Handle::current();
        let handle = std::thread::spawn(move || {
            let _guard = rt.enter();
            let rx = re_grpc_client::stream(uri);
            // READINESS IS AN ACKED SENTINEL, NOT A CONSTRUCTED RECEIVER.
            //
            // `re_grpc_client::stream` returns a local `LogReceiver` immediately
            // and dials on a background task, so signalling here would mean
            // "an object exists", not "the server has this client subscribed".
            // Every message published in that window reaches a subscription that
            // does not exist yet — and because the shadow's whole job is to say
            // whether the proxy DELIVERED a sample, a sample lost to that window
            // is reported as a delivery MISS, i.e. as evidence for the very
            // hypothesis (A) this instrument exists to discriminate against.
            //
            // The same class as the quiescence defect at the other end of the
            // shadow's life, and the same fix: signal on OBSERVED EVIDENCE. The
            // caller publishes sentinels until one comes back, so `ready` means
            // a frame really crossed the wire into this subscription.
            let mut report = ShadowReport::default();
            let sentinel = format!("/{SHADOW_SENTINEL_ENTITY}");
            let mut announced = false;
            while !stop_thread.load(Ordering::Relaxed) {
                match rx.recv_timeout(Duration::from_millis(50)) {
                    Ok(sm) => {
                        if let Some(re_log_channel::DataSourceMessage::LogMsg(LogMsg::ArrowMsg(
                            _,
                            arrow,
                        ))) = sm.into_data()
                        {
                            if let Ok(chunk) = re_chunk::Chunk::from_arrow_msg(&arrow) {
                                let path = chunk.entity_path().to_string();
                                if path == sentinel {
                                    report.sentinels_seen += 1;
                                    seen_thread.fetch_add(1, Ordering::Relaxed);
                                    if !announced {
                                        announced = true;
                                        let _ = ready_tx.send(());
                                    }
                                } else {
                                    report.entities.push(path);
                                }
                            }
                        }
                    }
                    Err(_) => continue,
                }
            }
            // DRAIN TO QUIESCENCE before reporting. Stopping the instant the
            // flag flips measures the shadow's READ POSITION, not what the
            // proxy delivered to it: a shadow still holding queued messages
            // counts every unread sample as a miss, which is the windowed
            // viewer's own defect reproduced inside the instrument built to
            // discriminate against it.
            //
            // MEASURED, and it is why this exists: one run of the 200 ms sweep
            // reported `shadow_hits=1 shadow_total_msgs=38` where its two
            // siblings on the identical stimulus reported 8 and 51 — the
            // shadow had read 37 camera frames and one sample and was stopped
            // owing seven more.
            //
            // Bounded on BOTH sides: consecutive empty reads (the queue is
            // genuinely drained) and a wall ceiling, so a proxy that keeps
            // producing cannot hold the join open.
            let quiesce_deadline = Instant::now() + Duration::from_secs(20);
            let mut empty_in_a_row = 0;
            while empty_in_a_row < SHADOW_QUIESCE_EMPTY_READS && Instant::now() < quiesce_deadline {
                match rx.recv_timeout(Duration::from_millis(50)) {
                    Ok(sm) => {
                        empty_in_a_row = 0;
                        if let Some(re_log_channel::DataSourceMessage::LogMsg(LogMsg::ArrowMsg(
                            _,
                            arrow,
                        ))) = sm.into_data()
                        {
                            if let Ok(chunk) = re_chunk::Chunk::from_arrow_msg(&arrow) {
                                let path = chunk.entity_path().to_string();
                                if path == sentinel {
                                    report.sentinels_seen += 1;
                                    seen_thread.fetch_add(1, Ordering::Relaxed);
                                } else {
                                    report.entities.push(path);
                                }
                            }
                        }
                    }
                    Err(_) => empty_in_a_row += 1,
                }
            }
            report.quiesced = empty_in_a_row >= SHADOW_QUIESCE_EMPTY_READS;
            report.connected = rx.is_connected();
            report
        });
        (
            Self {
                stop,
                seen_sentinels,
                handle: Some(handle),
            },
            ready_rx,
        )
    }

    /// Close the shadow behind a CLOSING SENTINEL, then report.
    ///
    /// The proxy handles messages in SEND ORDER, so a sentinel published after
    /// the last probe message can only arrive after it. Waiting for that
    /// sentinel therefore PROVES every earlier message has either arrived or
    /// been shed — an ordering fact — where the quiescence drain alone infers it
    /// from 150 ms of silence, which is a timing guess.
    ///
    /// MEASURED, and it is why this exists: at stall 0, where the foreground
    /// viewer received all 8 samples (`fg_hits=8`, so all 8 were admitted AND
    /// delivered), one run in three reported `shadow_hits=7` — the instrument
    /// under-reporting a delivery it was built to certify, which is evidence in
    /// the direction of hypothesis (A). The silence bar had ended while the
    /// server still owed this subscriber.
    ///
    /// The quiescence drain is KEPT as the backstop for the case this cannot
    /// cover — a closing sentinel that is itself lost — and `quiesced` still
    /// gates every reading.
    fn close(
        self,
        prod: &re_grpc_client::Client,
        store: &StoreId,
        bound: Duration,
    ) -> ShadowReport {
        let before = self.seen_sentinels.load(Ordering::Relaxed);
        prod.send_blocking(small_frame(store, SHADOW_SENTINEL_ENTITY, i64::MAX));
        let _ = prod.flush_blocking(Duration::from_secs(5));
        let deadline = Instant::now() + bound;
        while Instant::now() < deadline && self.seen_sentinels.load(Ordering::Relaxed) <= before {
            std::thread::sleep(Duration::from_millis(20));
        }
        self.finish()
    }

    fn finish(mut self) -> ShadowReport {
        self.stop.store(true, Ordering::Relaxed);
        self.handle
            .take()
            .expect("the shadow thread is joined exactly once")
            .join()
            .expect("the shadow thread does not panic")
    }
}

/// Drive the shadow's subscription to PROVEN-ATTACHED, and report whether it got
/// there.
///
/// Publishes sentinels until one comes back or the bound expires. Sentinels are
/// REPEATED rather than sent once because the window this closes is exactly the
/// window in which a published frame is lost: a single sentinel raced the dial
/// and would leave the handshake waiting forever on a subscription that is by
/// then perfectly healthy.
fn await_shadow_subscription(
    ready: &mpsc::Receiver<()>,
    prod: &re_grpc_client::Client,
    store: &StoreId,
    bound: Duration,
) -> bool {
    let deadline = Instant::now() + bound;
    let mut seq = 0i64;
    while Instant::now() < deadline {
        prod.send_blocking(small_frame(store, SHADOW_SENTINEL_ENTITY, seq));
        seq += 1;
        let _ = prod.flush_blocking(Duration::from_secs(5));
        if ready.recv_timeout(Duration::from_millis(200)).is_ok() {
            return true;
        }
    }
    false
}

/// Report a probe whose PREREQUISITE never held, and fail.
///
/// A causal reading is a claim about what the shed did; a probe that never
/// established the state it was going to observe has evidence for no such claim,
/// and printing one anyway is how an instrument manufactures its own answer.
///
/// It PANICS rather than printing and returning, and that is a deliberate call
/// for an `#[ignore]`d diagnostic. These probes exist to produce evidence a
/// human reads out of the `VERDICT` lines; a run that exits 0 having printed
/// INCONCLUSIVE is one skim away from being entered in a table as a result. A
/// nonzero exit cannot be skimmed past.
fn inconclusive(probe: &str, prerequisite: &str, detail: &str) -> ! {
    println!("shed-starvation {probe} INCONCLUSIVE: {prerequisite} — {detail}");
    panic!(
        "shed-starvation {probe}: PREREQUISITE NOT ESTABLISHED — {prerequisite}. {detail}. No causal \
         reading is printed, because this run observed no state one could be about."
    );
}

/// Optionally drain `spawn_with_recv`'s receiver on its own thread.
///
/// Returns the join handle and a stop flag. When [`drain_rx`] is off this consumes
/// the receiver and parks it exactly as the gate file does (holding it, never
/// reading it), so the two configurations differ in ONE thing.
fn spawn_rx_drainer(
    rx: re_log_channel::LogReceiver,
) -> (Arc<AtomicBool>, std::thread::JoinHandle<usize>) {
    let stop = Arc::new(AtomicBool::new(false));
    let stop_thread = Arc::clone(&stop);
    let drain = drain_rx();
    let handle = std::thread::spawn(move || {
        let mut taken = 0usize;
        while !stop_thread.load(Ordering::Relaxed) {
            if drain {
                if rx.recv_timeout(Duration::from_millis(50)).is_ok() {
                    taken += 1;
                }
            } else {
                // Hold it undrained — the gate file's own configuration.
                std::thread::sleep(Duration::from_millis(50));
            }
        }
        taken
    });
    (stop, handle)
}

// ─────────────────────── the pressure driver ───────────────────────

/// One round's outcome, kept per round because only CONFIRMED rounds may
/// contribute to probe B's reading.
#[derive(Debug)]
struct RoundOutcome {
    round: usize,
    /// Pressure confirmed on BOTH sides of this round's sample.
    confirmed: bool,
    /// The windowed foreground viewer matched this round's sample.
    fg_got: bool,
}

/// What one pressure phase observed, mirroring the gate file's `Pressed` so the
/// two can be read side by side against a CI failure dump.
#[derive(Debug, Default)]
struct Pressed {
    confirmed: bool,
    frames: usize,
    flush_failures: usize,
    elapsed: Duration,
    last: Usage,
}

/// Feed camera frames — one at a time, each flushed — until the proxy's own
/// accounting reports a further WHOLE camera frame dropped.
///
/// Byte-for-byte the gate file's `Pressure::until_dropping`, reproduced here
/// rather than shared because the point of this file is to instrument what that
/// loop DOES (how many frames it feeds, how many the viewer then owes) without
/// touching the gate.
fn press_until_dropping(
    prod: &re_grpc_client::Client,
    handle: &re_grpc_server::MessageProxyHandle,
    store: &StoreId,
    rgb: &[u8],
    tag: i64,
    dropped_before: u64,
) -> Pressed {
    let start = Instant::now();
    let want = dropped_before.saturating_add(rgb.len() as u64);
    let ceiling = pressure_ceiling();
    let mut p = Pressed::default();
    while p.frames < ceiling && start.elapsed() < Duration::from_secs(20) {
        prod.send_blocking(image_frame(
            store,
            tag + p.frames as i64,
            rgb,
            RENDITION.0,
            RENDITION.1,
        ));
        p.frames += 1;
        if prod.flush_blocking(Duration::from_secs(5)).is_err() {
            p.flush_failures += 1;
        }
        p.last = usage_now(handle, Duration::from_secs(5));
        if p.last.live_dropped >= want {
            p.confirmed = true;
            break;
        }
    }
    p.elapsed = start.elapsed();
    p
}

// ══════════════════════════ PROBE A ══════════════════════════

/// **(A) — load-free, deterministic.** Does the shed ever drop a sub-floor sample?
///
/// The gate file cannot answer this: under continuous pressure the dropped total
/// moves by many frames and per-message sizes vary, so it explicitly declines to
/// claim "not one byte of the sample was counted". This probe removes the
/// pressure instead of measuring through it.
///
/// The queue is PINNED over budget rather than held there by a racing producer:
/// `spawn_with_recv`'s receiver is never drained, so its forwarding task fills its
/// own 128 MiB `re_log_channel` sink and then STOPS pulling, after which
/// `bytes_in_flight` cannot fall. That state is entered once, verified, and then
/// the stimulus is a SINGLE 1286-byte sample with nothing else in flight — so
/// `live_dropped` moving at all is unambiguous.
///
/// If the delta is 0, hypothesis (A) is refuted for this shape and the flake is
/// downstream of the gate. If the delta is the sample's accounted size, (A) is
/// confirmed with no load, no timing and no oracle in the way.
#[test]
#[ignore = "diagnostic, not a pin — run with --ignored --nocapture"]
fn probe_a_small_sample_crosses_a_pinned_over_budget_queue() {
    with_runtime(|| {
        let (w, h) = RENDITION;
        let rgb = incompressible(rgb_bytes(w, h), 0x5A);
        let addr = probe_free_addr();
        let (rx, handle) = re_grpc_server::spawn_with_recv(
            addr,
            cerulion_vizd::host::server_options(),
            re_grpc_server::shutdown::never(),
        );
        let uri = proxy_uri(addr);
        let prod =
            re_grpc_client::Client::new(uri.clone(), re_grpc_client::write::Options::default());
        let store = StoreId::random(StoreKind::Recording, "a");
        let (rx_stop, rx_join) = spawn_rx_drainer(rx);
        prod.send_blocking(set_store_info(&store));

        // THE STALLED PEER. Measured on the first run of this probe: with only
        // the `spawn_with_recv` receiver present, `broadcast` climbs to
        // 11_118_212 under load and then falls to EXACTLY 0 the moment the
        // producer quiesces — the queue is not pinned at all, because a message's
        // bytes are freed as soon as every subscriber's `recv()` has taken it and
        // that receiver's forwarding task keeps pulling into its own 128 MiB sink.
        //
        // A viewer that NEVER drains does not merely pin the queue, it WEDGES the
        // daemon: its bytes are never freed, `bytes_in_flight` climbs past
        // `CHANNEL_SIZE_BYTES` (128 MiB) — which the shed cannot prevent, because
        // the frames admitted before its sink filled are already in flight — and
        // the event loop then AWAITS in `send_async`, stopping `capture_memory`
        // and every flush with it. MEASURED: the pin loop ran 400 x
        // `flush_blocking(5s)` and was killed at ten minutes without printing a
        // single line. So this probe does NOT use one.
        //
        // The queue is held over budget by PRESSURE instead, which is also what
        // the CI state actually is: the failing runs report `broadcast` steady at
        // 11_118_204 = exactly FOUR encoded frames, a queue held just over the
        // 8 MiB budget while frames keep arriving — not a 128 MiB pin.
        let (shadow, ready) = Shadow::spawn(uri);
        let attached = await_shadow_subscription(&ready, &prod, &store, Duration::from_secs(15));
        if !attached {
            inconclusive(
                "probeA",
                "the shadow viewer's subscription was never proven attached",
                "no sentinel came back within 15 s, so a sample this probe fails to observe \
                 cannot be told apart from one the shed dropped",
            );
        }

        // Drive the queue over budget, then send the sample WHILE it is there.
        // Bounded by frames AND wall, and it REPORTS whether it got there rather
        // than assuming it (the precondition rule).
        let start = Instant::now();
        let mut fed = 0usize;
        let mut peak = 0u64;
        let mut over = false;
        while fed < 300 && start.elapsed() < Duration::from_secs(60) {
            prod.send_blocking(image_frame(&store, fed as i64, &rgb, w, h));
            fed += 1;
            let _ = prod.flush_blocking(Duration::from_secs(5));
            let u = usage_now(&handle, Duration::from_secs(5));
            peak = peak.max(u.broadcast);
            if u.live_dropped > 0 && u.broadcast > LIVE_TEMPORAL_BUDGET_BYTES {
                over = true;
                break;
            }
        }
        let before = usage_now(&handle, Duration::from_secs(5));
        println!(
            "shed-starvation probeA pressure: fed={fed} peak={peak} over_budget={over} \
             before={before:?} budget={LIVE_TEMPORAL_BUDGET_BYTES}"
        );

        // The sample crosses a queue the line above reported the state of.
        let sample = small_frame(&store, PLOT_ENTITY, 1);
        let sample_bytes = accounted_size(&sample);
        prod.send_blocking(sample);
        let _ = prod.flush_blocking(Duration::from_secs(20));
        std::thread::sleep(Duration::from_secs(2));
        let after = usage_settled(&handle);
        // Closed behind a sentinel, AFTER the final accounting read, so the
        // closing message cannot move the numbers the verdict is read from.
        let report = shadow.close(&prod, &store, Duration::from_secs(10));

        let delta = after.live_dropped.saturating_sub(before.live_dropped);
        let over_when_sent = before.broadcast > LIVE_TEMPORAL_BUDGET_BYTES;
        println!(
            "shed-starvation probeA VERDICT: over_budget={over_when_sent} sample_bytes={sample_bytes} \
             floor={LIVE_SMALL_MESSAGE_FLOOR_BYTES} dropped_delta={delta} \
             shadow_saw_sample={} shadow_connected={} shadow_camera_frames={} \
             shadow_quiesced={} shadow_sentinels={} before={before:?} after={after:?}",
            report.saw(&format!("/{PLOT_ENTITY}")),
            report.connected,
            report.camera_frames(),
            report.quiesced,
            report.sentinels_seen,
        );
        // EVERY prerequisite is checked BEFORE the causal reading, because each
        // one is a state the reading is ABOUT. Over-budget is what makes the
        // shed a decision at all; quiescence is what makes a shadow MISS mean
        // the proxy did not deliver rather than that the shadow had not read yet.
        if !over_when_sent {
            inconclusive(
                "probeA",
                "the queue was NOT over budget when the sample crossed",
                &format!(
                    "fed {fed} frames, peak {peak} against a {LIVE_TEMPORAL_BUDGET_BYTES}-byte \
                     budget, occupancy {} at the send — the shed admits everything below the \
                     budget, so this sample crossed a gate that decided nothing",
                    before.broadcast
                ),
            );
        }
        if !report.quiesced {
            inconclusive(
                "probeA",
                "the shadow hit its drain ceiling still owing messages",
                "its read position, not the proxy's delivery, decides what it saw",
            );
        }
        println!(
            "shed-starvation probeA READING: dropped_delta==0 && shadow_saw_sample ⇒ (A) REFUTED for \
             this shape — the guarantee holds under a pinned over-budget queue. dropped_delta \
             ~= sample_bytes ⇒ (A) CONFIRMED, load-free. dropped_delta==0 && \
             !shadow_saw_sample ⇒ (A') — ADMITTED but never DELIVERED, which is the promise \
             the live budget actually makes; re-run with SHED_DRAIN_RX=1 to see whether a peer that \
             keeps receiving restores delivery."
        );
        rx_stop.store(true, Ordering::Relaxed);
        let rx_taken = rx_join.join().expect("the rx drainer does not panic");
        println!(
            "shed-starvation probeA rx: drain_rx={} taken={rx_taken}",
            drain_rx()
        );
        drop(prod);
    });
}

// ══════════════════════════ PROBE B ══════════════════════════

/// **(B) — the delivery oracle under an injected consumer stall.**
///
/// Reproduces the failing arm's exact round structure and adds the one thing it
/// lacks: a viewer that never falls behind. The FOREGROUND viewer is drained the
/// way the gate file drains its own (a bounded window per round, discarding
/// everything that is not that round's sample) with `SHED_CONSUMER_STALL_MS`
/// injected per message; the SHADOW viewer is drained continuously.
///
/// The stall is the stressor because it starves the RIGHT thread. Ambient CPU load
/// does not reproduce this class — it slows everything, including the producer,
/// which merely makes the run longer. What produces the CI failure signature is the
/// VIEWER falling behind while the gate keeps deciding, and a per-message stall in
/// the drain loop is that condition, on an idle desk, with no scheduler in the
/// way.
///
/// Predictions:
///
///  * (B) — the foreground misses samples the shadow receives, and its miss count
///    grows with the stall. The guarantee held; the oracle lost a race.
///  * (A) — the shadow misses them too, at stall 0 as well as under stall.
#[test]
#[ignore = "diagnostic, not a pin — run with --ignored --nocapture"]
fn probe_the_delivery_oracle_under_a_starved_consumer() {
    with_runtime(|| {
        let (w, h) = RENDITION;
        let frame_bytes = rgb_bytes(w, h);
        let rgb = incompressible(frame_bytes, 0x5A);
        let stall = consumer_stall();
        let addr = probe_free_addr();
        let (rx, handle) = re_grpc_server::spawn_with_recv(
            addr,
            cerulion_vizd::host::server_options(),
            re_grpc_server::shutdown::never(),
        );
        let uri = proxy_uri(addr);
        let prod =
            re_grpc_client::Client::new(uri.clone(), re_grpc_client::write::Options::default());
        let store = StoreId::random(StoreKind::Recording, "b");
        let (rx_stop, rx_join) = spawn_rx_drainer(rx);
        prod.send_blocking(set_store_info(&store));

        // Engage the budget, bounded on the SUBJECT's own report.
        let mut engaged = false;
        let mut flood_frames = 0usize;
        while flood_frames < 300 {
            prod.send_blocking(image_frame(&store, flood_frames as i64, &rgb, w, h));
            flood_frames += 1;
            let _ = prod.flush_blocking(Duration::from_secs(20));
            if usage_now(&handle, Duration::from_secs(5)).live_dropped > 0 {
                engaged = true;
                break;
            }
        }
        println!("shed-starvation probeB flood: engaged={engaged} frames={flood_frames}");
        if !engaged {
            inconclusive(
                "probeB",
                "the flood never drove the queue over budget",
                &format!(
                    "{flood_frames} frames fed and the proxy reported no drop, so every round \
                     below would press a gate that admits everything"
                ),
            );
        }

        let (shadow, ready) = Shadow::spawn(uri.clone());
        if !await_shadow_subscription(&ready, &prod, &store, Duration::from_secs(15)) {
            inconclusive(
                "probeB",
                "the shadow viewer's subscription was never proven attached",
                "no sentinel came back within 15 s, so its misses would be indistinguishable \
                 from the shed dropping the samples — which is the discrimination this probe IS",
            );
        }
        let consumer = re_grpc_client::stream(uri);
        std::thread::sleep(Duration::from_millis(500));

        // Per-round detail, because only CONFIRMED rounds may contribute to the
        // reading. A round whose pressure never confirmed observed a gate nothing
        // proved was deciding, so its delivery outcome — hit OR miss — is evidence
        // of nothing, and folding it into the totals is the same vacuity the
        // inconclusive gates exist to remove, one aggregate later.
        //
        // This is the per-round form of the review note at the `confirmed_rounds`
        // guard. The suggested `confirmed_rounds != n` is SOUND — every round
        // contributing would then be confirmed — but it discards a whole run over
        // one transient, and an unconfirmed round is legitimate under load (a
        // pressure phase can exhaust its WALL on a slow runner, which is why the
        // gate file itself tolerates two in a row). Dropping only the unearned
        // rounds keeps the earned evidence and is strictly stronger per round.
        let mut rounds_detail: Vec<RoundOutcome> = Vec::new();
        let n = rounds();
        for round in 0..n {
            let entity = format!("{PLOT_ENTITY}/r{round}");
            let want = re_log_types::EntityPath::from(entity.as_str());
            let base = round as i64 * 1000;
            let d0 = usage_now(&handle, Duration::from_secs(5)).live_dropped;

            let before = press_until_dropping(&prod, &handle, &store, &rgb, base, d0);
            prod.send_blocking(small_frame(&store, &entity, round as i64));
            let after = press_until_dropping(
                &prod,
                &handle,
                &store,
                &rgb,
                base + 500,
                before.last.live_dropped,
            );
            let _ = prod.flush_blocking(Duration::from_secs(20));

            // The gate file's own delivery loop, with the stall injected.
            let deadline = Instant::now() + Duration::from_secs(5);
            let mut got = false;
            let mut drained = 0usize;
            while Instant::now() < deadline && !got {
                if let Ok(sm) = consumer.recv_timeout(Duration::from_millis(100)) {
                    drained += 1;
                    if !stall.is_zero() {
                        std::thread::sleep(stall);
                    }
                    if let Some(re_log_channel::DataSourceMessage::LogMsg(LogMsg::ArrowMsg(
                        _,
                        arrow,
                    ))) = sm.into_data()
                    {
                        if let Ok(chunk) = re_chunk::Chunk::from_arrow_msg(&arrow) {
                            if chunk.entity_path() == &want {
                                got = true;
                            }
                        }
                    }
                }
            }
            rounds_detail.push(RoundOutcome {
                round,
                confirmed: before.confirmed && after.confirmed,
                fg_got: got,
            });
            println!(
                "shed-starvation probeB round {round}: fg_got={got} fg_drained={drained} \
                 pressure_confirmed={} before_frames={} after_frames={} \
                 dropped_in_round={} last={:?}",
                before.confirmed && after.confirmed,
                before.frames,
                after.frames,
                after.last.live_dropped.saturating_sub(d0),
                after.last,
            );
        }

        let report = shadow.close(&prod, &store, Duration::from_secs(10));

        // ONLY CONFIRMED ROUNDS CONTRIBUTE. The unconfirmed ones are reported so
        // a reader can see how much of the run was spent, but they score nothing.
        let confirmed: Vec<&RoundOutcome> = rounds_detail.iter().filter(|r| r.confirmed).collect();
        let confirmed_rounds = confirmed.len();
        let unconfirmed: Vec<usize> = rounds_detail
            .iter()
            .filter(|r| !r.confirmed)
            .map(|r| r.round)
            .collect();
        let fg_hits = confirmed.iter().filter(|r| r.fg_got).count();
        let fg_misses = confirmed_rounds - fg_hits;
        let saw_round = |r: usize| report.saw(&format!("/{PLOT_ENTITY}/r{r}"));
        let shadow_hits = confirmed.iter().filter(|r| saw_round(r.round)).count();
        // A bare count is not attributable — WHICH rounds a shadow missed is what
        // separates "the last one was still in flight" from "the shed took one".
        let shadow_missed: Vec<usize> = confirmed
            .iter()
            .filter(|r| !saw_round(r.round))
            .map(|r| r.round)
            .collect();
        println!(
            "shed-starvation probeB VERDICT: stall={stall:?} rounds={n} confirmed_rounds={confirmed_rounds} \
             unconfirmed={unconfirmed:?} \
             fg_hits={fg_hits} fg_misses={fg_misses} shadow_hits={shadow_hits} \
             shadow_missed={shadow_missed:?} \
             shadow_camera_frames={} shadow_connected={} shadow_total_msgs={} \
             shadow_quiesced={} shadow_sentinels={}",
            report.camera_frames(),
            report.connected,
            report.entities.len(),
            report.quiesced,
            report.sentinels_seen,
        );
        if !report.quiesced {
            inconclusive(
                "probeB",
                "the shadow hit its drain ceiling still owing messages",
                &format!(
                    "its misses are its own read position rather than the proxy's delivery, so \
                     shadow_hits={shadow_hits} is not evidence about the byte-axis guarantee \
                     (saw {} messages, {} of them camera frames)",
                    report.entities.len(),
                    report.camera_frames(),
                ),
            );
        }
        // With the unconfirmed rounds already dropped from the tally, what is left
        // to refuse is a run that contributed NOTHING. The DROP half of a round is
        // DRIVEN, so no round confirming means no over-budget gate was ever
        // observed. (`SHED_PRESSURE_CEILING=0` reaches exactly this state.)
        if confirmed_rounds == 0 {
            inconclusive(
                "probeB",
                "no round confirmed pressure on BOTH sides of its sample",
                &format!(
                    "{n} round(s) ran and none established the gate was dropping, so there is no \
                     round whose delivery outcome describes a queue anything proved was over \
                     budget"
                ),
            );
        }
        println!(
            "shed-starvation probeB READING: every count below is over the CONFIRMED subset, so the \
             denominator is confirmed_rounds — never `rounds`, which counts rounds this probe \
             refused to score. shadow_hits==confirmed_rounds while fg_misses>0 ⇒ (B) — the \
             byte-axis guarantee HELD and the arm's delivery oracle lost a race it re-loses every \
             round (each window discards the previous round's sample, so a viewer one message \
             behind never matches again). shadow_hits==0 with the samples ADMITTED (watch \
             `broadcast` creep by ~1286 bytes a round) ⇒ (A') — admitted but undeliverable while \
             a peer subscriber is stalled. Re-run with SHED_DRAIN_RX=1: if shadow_hits jumps \
             to confirmed_rounds, the stalled peer is the cause and the gate arm is asserting a \
             delivery guarantee under a condition production never enters."
        );
        rx_stop.store(true, Ordering::Relaxed);
        let rx_taken = rx_join.join().expect("the rx drainer does not panic");
        println!(
            "shed-starvation probeB rx: drain_rx={} taken={rx_taken}",
            drain_rx()
        );
        drop(prod);
    });
}

// ══════════════════════════ PROBE C ══════════════════════════

/// **(C) — how STALE is the reading the pressure loop steers on?**
///
/// `MessageProxyHandle::capture_memory` fires a `try_send` at a 32-slot event mpsc
/// and returns whatever snapshot is already in the mutex — on `Full` it drops the
/// refresh request silently and hands back the previous turn's numbers as if they
/// were fresh. The gate file's `usage()` guards only the FIRST snapshot (the
/// childless-tree case), never staleness thereafter.
///
/// That matters because it is an AMPLIFIER, not a competing cause: a pressure
/// phase steers on `live_dropped`, so a stale reading makes it keep feeding frames
/// after the drop it caused has already happened. Every extra frame is another
/// admission opportunity, and every admitted frame is another 2.78 MB the viewer
/// owes before it can reach the sample. This probe measures that gap directly:
/// the divergence between an immediate read and a settled one, in bytes and in
/// whole camera frames.
#[test]
#[ignore = "diagnostic, not a pin — run with --ignored --nocapture"]
fn probe_usage_snapshot_staleness_under_pressure() {
    with_runtime(|| {
        let (w, h) = RENDITION;
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
        let store = StoreId::random(StoreKind::Recording, "c");
        prod.send_blocking(set_store_info(&store));

        let encoded = accounted_size(&image_frame(&store, 0, &rgb, w, h));
        let mut worst_lag_bytes = 0u64;
        let mut worst_lag_frames = 0.0f64;
        let mut samples = 0usize;

        for seq in 0..60i64 {
            prod.send_blocking(image_frame(&store, seq, &rgb, w, h));
            let _ = prod.flush_blocking(Duration::from_secs(5));
            let immediate = usage_now(&handle, Duration::from_secs(5));
            let settled = usage_settled(&handle);
            let lag = settled.live_dropped.saturating_sub(immediate.live_dropped);
            if lag > worst_lag_bytes {
                worst_lag_bytes = lag;
                worst_lag_frames = lag as f64 / encoded as f64;
            }
            samples += 1;
            if seq % 10 == 0 {
                println!(
                    "shed-starvation probeC seq={seq}: immediate={immediate:?} settled={settled:?} \
                     lag_bytes={lag}"
                );
            }
        }

        let final_usage = usage_settled(&handle);
        println!(
            "shed-starvation probeC VERDICT: samples={samples} encoded_frame={encoded} \
             worst_lag_bytes={worst_lag_bytes} worst_lag_frames={worst_lag_frames:.2} \
             settle={:?} persistent={} final={final_usage:?}",
            settle(),
            final_usage.persistent,
        );
        // Staleness is measured against the drops the stimulus caused; with no
        // drop, `worst_lag_bytes` is trivially 0 and the reading below would
        // report "no amplifier" on a probe that never ran its own stimulus.
        if final_usage.live_dropped == 0 {
            inconclusive(
                "probeC",
                "the stimulus never drove the queue over budget",
                &format!(
                    "{samples} frames fed and the proxy reported no drop, so a zero lag measures \
                     an idle event loop rather than a stale snapshot"
                ),
            );
        }
        println!(
            "shed-starvation probeC READING: worst_lag_frames >= 1 ⇒ a pressure phase can feed at least \
             that many frames PAST the drop it already caused, each an admission opportunity the \
             viewer then owes. That is the amplifier (B) rides on; it cannot fabricate a \
             confirmation (staleness only DELAYS one), so it is not a cause on its own."
        );
        drop(prod);
        drop(rx);
    });
}

// ══════════════════════════ PROBE D ══════════════════════════

/// **(D) — the macOS arm's ceiling, against the ENCODED message rather than the
/// payload.**
///
/// `a_slow_viewer_does_not_accumulate_a_backlog_of_image_frames` bounds peak
/// occupancy at `budget + observed_message`, where `observed_message` is the
/// largest single-sample JUMP in its 33 ms occupancy series, FLOORED at the raw
/// RGB byte count. But `decide_live` admits whenever `occ.bytes <= budget`, so the
/// true worst case is `budget + ENCODED`, and the encoded proto measures ~0.5%
/// larger than the raw payload — the arm's own comment records it as having been
/// 0.3% from red.
///
/// So the ceiling is sound only while the sampled series happens to reveal a full
/// encoded-size jump. On a runner whose 33 ms samples straddle a drain, the
/// largest observed jump is SMALLER than one message, the floor falls back to the
/// raw size, and the ceiling is understated by exactly the compression delta —
/// a load-sensitive oracle, not a load-sensitive guarantee.
///
/// This probe reports all three numbers so the gap is a measurement rather than an
/// argument.
#[test]
#[ignore = "diagnostic, not a pin — run with --ignored --nocapture"]
fn probe_the_slow_viewer_ceiling_against_the_encoded_message() {
    with_runtime(|| {
        let (w, h) = RENDITION;
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
        let store = StoreId::random(StoreKind::Recording, "d");
        prod.send_blocking(set_store_info(&store));

        let encoded = accounted_size(&image_frame(&store, 0, &rgb, w, h));
        let mut series = Vec::new();
        let mut peak = 0u64;
        for seq in 0..60i64 {
            prod.send_blocking(image_frame(&store, seq, &rgb, w, h));
            std::thread::sleep(Duration::from_millis(33));
            let u = usage_now(&handle, Duration::from_secs(2));
            peak = peak.max(u.broadcast);
            series.push(u.broadcast);
        }
        std::thread::sleep(Duration::from_millis(300));
        let last = usage_now(&handle, Duration::from_secs(2));

        let observed_jump = series
            .windows(2)
            .map(|s| s[1].saturating_sub(s[0]))
            .max()
            .unwrap_or(0);
        let arm_ceiling = LIVE_TEMPORAL_BUDGET_BYTES + observed_jump.max(frame_bytes as u64);
        let true_ceiling = LIVE_TEMPORAL_BUDGET_BYTES + encoded;
        println!(
            "shed-starvation probeD VERDICT: peak={peak} raw_frame={frame_bytes} encoded_frame={encoded} \
             observed_jump={observed_jump} arm_ceiling={arm_ceiling} true_ceiling={true_ceiling} \
             headroom={} last={last:?}",
            arm_ceiling as i64 - peak as i64,
        );
        println!("shed-starvation probeD series: {series:?}");
        // The whole reading compares a PEAK against two ceilings; a queue that
        // never held a byte has no peak, and `peak=0` sits under every ceiling
        // for a reason that says nothing about either.
        if peak == 0 {
            inconclusive(
                "probeD",
                "the live queue never held a single byte",
                "nothing was ever backlogged, so comparing a zero peak against either ceiling \
                 measures the stimulus rather than the bound",
            );
        }
        println!(
            "shed-starvation probeD READING: arm_ceiling < true_ceiling ⇒ the arm's bound is understated \
             by the compression delta whenever the sampled series does not reveal a full \
             encoded-size jump, which is (B) — an oracle that load can invert, not a guarantee \
             that load can break. peak > true_ceiling ⇒ (A), a genuine over-admission."
        );
        drop(prod);
        drop(rx);
    });
}
