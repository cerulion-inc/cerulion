// SPDX-License-Identifier: AGPL-3.0-only
//! The testable transcode loop between the Cerulion `/go2/camera/h264` input
//! and the GStreamer decode→JPEG pipeline
//! (shaped for the topic-fed `appsrc`).
//!
//! The H.264 stream arrives on a Cerulion topic, so the node is DATA
//! TRIGGERED and the whole cycle happens inside one tick: an access unit
//! arrives, is fed to the decoder, and whatever JPEG the decoder has ready is
//! published. The node owns no source of its own: there is no helper thread,
//! no queue and no doorbell. The failure policy needs a home that can be
//! tested without GStreamer. That is this module:
//!
//! - [`JpegTranscoder`] — one running decode→encode pipeline attempt
//!   (implemented by `pipeline::GstCamera` with real gst + bus monitoring, and
//!   by a scripted fake in the unit tests below — no gst at unit-test time);
//! - [`TranscodeLoop`] — the gst-free state machine the tick drives. It owns
//!   the lazy build, the rebuild backoff, the [`crate::h264::AuGate`] admission
//!   policy, and the newest-wins drain of the encoder's output.
//!
//! # Time is the NODE clock, never the wall clock
//!
//! [`TranscodeLoop::feed`] takes `now_ns` — the value the tick reads from
//! `self.now_ns()` (the scheduler's clock: `RealClock` in production,
//! `VirtualClock` under test). NOTHING here calls `Instant::now`. That is not
//! only a determinism nicety: `#[cerulion_node_impl]` DENIES `Instant::now` in
//! a tick body outright, so a wall-clock backoff is not reachable from the
//! tick at all: it would have to live on a separate thread, and this node
//! runs none.
//!
//! # Failure philosophy
//!
//! The camera NEVER busy-spins and NEVER gives up:
//!
//! - a **fatal pipeline error** (a GStreamer bus `Error` — e.g. the decoder
//!   dying on a corrupt IDR, or an `appsrc` push refused because the pipeline
//!   is flushing) is logged LOUDLY (`error!`), the pipeline is torn down, and a
//!   REBUILD is attempted after [`REBUILD_BACKOFF_NS`];
//! - a **failed (re)build** warns once per down-regime (repeats demoted to
//!   `debug!` with the attempt count — the standard flood-latch pattern) and
//!   retries on the same backoff; a successful rebuild after failures logs a
//!   recovery `info!`;
//! - **EOS** keeps its once-per-pipeline-instance warn (a decoder ending is
//!   unusual but not a rebuild trigger). **Known limitation:** because EOS is
//!   not a rebuild trigger, a pipeline that reaches
//!   end-of-stream is PERMANENTLY non-producing — it keeps accepting pushes
//!   and never yields another JPEG, and nothing tears it down. The observable
//!   is unambiguous in the teardown summary: `aus_pushed` keeps climbing while
//!   `published` stops, with exactly one `reached end-of-stream` warn in the
//!   log. This is deliberate: on a live `appsrc` fed by an infinite
//!   topic there is no legitimate EOS, so treating one as a rebuild trigger
//!   would be guessing at a condition that does not arise there;
//! - a **map failure** (a sample with no readable buffer) is counted
//!   ([`TranscodeLoop::map_failure_count`]) and warned once per regime
//!   (re-armed by the next good frame), repeats demoted to `debug!`.
//!
//! Because the loop is driven by arriving frames rather than by a thread of
//! its own it never sleeps and never asks the caller to: a backoff is simply a
//! window during which arriving access units are counted
//! ([`TranscodeLoop::aus_dropped_while_down`]) and dropped.

// Pure by construction (no gst, no FFI). Same module-scope forbid as `frame`.
#![forbid(unsafe_code)]

use crate::frame::JpegFrame;
use crate::h264::{AuGate, AuVerdict, FloodLog, DEFAULT_TARGET_HEIGHT};

/// What one [`JpegTranscoder::poll`] observed.
#[derive(Debug, Clone, PartialEq)]
pub enum Pull {
    /// An encoded JPEG frame was ready.
    Frame(JpegFrame),
    /// Nothing ready right now — the decoder has not produced output yet.
    NoFrame,
    /// A sample arrived but carried no readable buffer (map failure) — counted
    /// + rate-limit-warned by the loop, then treated like [`Pull::NoFrame`].
    MapFailure,
    /// The pipeline reached end-of-stream — no more frames from THIS instance.
    Eos,
    /// The pipeline reported a FATAL error (a GStreamer bus `Error` message —
    /// already formatted as `source: error (debug)`). The loop tears the
    /// instance down and schedules a rebuild.
    Fatal(String),
}

/// One running decode→encode pipeline attempt. `pipeline::GstCamera` is the
/// real implementation (`appsrc` push + `appsink` pull + bus monitoring); the
/// unit tests below script a fake.
///
/// `Send` because the node that owns it is moved across threads by the runtime.
pub trait JpegTranscoder: Send {
    /// Feed ONE complete H.264 Annex-B access unit to the decoder. `Err` is a
    /// display string; the loop treats it as fatal (a refused push means the
    /// pipeline is flushing, unlinked or dead — not something a retry of the
    /// same buffer fixes).
    fn push_au(&mut self, au: &[u8]) -> Result<(), String>;

    /// NON-BLOCKING poll for the next encoded JPEG. Must return promptly:
    /// this runs inside the node's tick, so a blocking wait here would stall
    /// the whole graph step while the decoder works.
    fn poll(&mut self) -> Pull;
}

/// Builds a fresh [`JpegTranscoder`] — called on the first fed access unit and
/// on every rebuild attempt. `Err` carries a display string (house types often
/// don't carry `Debug`; the loop only logs it).
pub type TranscoderFactory =
    Box<dyn FnMut() -> Result<Box<dyn JpegTranscoder>, String> + Send + 'static>;

/// Backoff between pipeline (re)build attempts after a fatal error or a failed
/// build — loud-but-patient recovery, never a rebuild storm. Expressed in NODE
/// clock nanoseconds (see the module docs).
pub const REBUILD_BACKOFF_NS: u64 = 1_000_000_000;

/// How many encoder outputs one [`TranscodeLoop::feed`] will drain before
/// giving up and returning the newest it has.
///
/// At steady state the decoder emits one JPEG per access unit, so a single
/// poll suffices and the second returns [`Pull::NoFrame`]. The bound exists
/// for the transient where a burst of buffers has queued up (pipeline
/// start-up, a scheduling hiccup): the loop takes the NEWEST and counts the rest as
/// stale rather than publishing week-old video or looping unboundedly inside
/// one tick.
pub const MAX_POLLS_PER_FEED: usize = 8;

/// The gst-free transcode state machine. See the module docs.
///
/// Capturable. The decoder pipeline is REBUILT on restore (it is a
/// handle, and a decoder's internal state is not ours to carry anyway) while
/// every counter and latch around it is carried, so a resumed run's teardown
/// summary still reports the whole run rather than the tail.
#[derive(cerulion_core::state::CerulionState)]
pub struct TranscodeLoop {
    /// Builds a fresh pipeline instance (first feed + every rebuild).
    ///
    /// `#[cerulion(reconstruct)]` is written out here rather than inferred:
    /// `TranscoderFactory` is a type ALIAS over `Box<dyn FnMut(..)>`, and a
    /// proc macro sees tokens, so the structural `dyn` rule that recognises
    /// `source` below cannot see through the alias. That direction fails
    /// CLOSED (a compile error naming the field, never a silent capture),
    /// which is why one line here is the whole cost.
    #[cerulion(reconstruct)]
    factory: TranscoderFactory,
    /// The live pipeline, when up.
    source: Option<Box<dyn JpegTranscoder>>,
    /// The admission gate (rendition filter + wait-for-SPS).
    gate: AuGate,
    /// When down: the next (re)build attempt is gated to not-before this node
    /// clock instant (`None` = attempt immediately — the initial state).
    retry_at_ns: Option<u64>,
    /// Consecutive failed build attempts in the CURRENT down-regime (0 once a
    /// build succeeds). The first failure of a regime warns; repeats demote to
    /// `debug!`; recovery logs `info!` with the count.
    failed_builds: u64,
    /// EOS warned for the current pipeline instance (reset on rebuild).
    eos_warned: bool,
    /// Map-failure warn latch: armed on the first failure of a regime,
    /// re-armed (cleared) by the next successfully pulled frame.
    map_fail_warned: bool,
    /// Lifetime map-failure count.
    map_failures: u64,
    /// Access units that arrived while the pipeline was down (dropped).
    ///
    /// Counted BEFORE the admission gate runs, so it includes the sibling
    /// rendition — i.e. roughly TWICE the decodable loss on a two-rendition
    /// topic. That is deliberate (classifying while down would advance the
    /// gate's own counters and SPS latch for frames no decoder could ever have
    /// received), and it is why the teardown summary reports this under a name
    /// that says ARRIVED rather than one that implies decodable frames lost.
    aus_dropped_while_down: u64,
    /// Access units the decoder ACCEPTED — incremented only after `push_au`
    /// returned `Ok`. Distinct from `AuGate::admitted` (which counts the
    /// gate's DECISION, taken before the push is attempted): a push refused by
    /// a dying pipeline is admitted-but-not-pushed, and the gap between the
    /// two counters in the teardown summary is exactly that count.
    aus_pushed: u64,
    /// JPEGs discarded because a newer one was ready in the SAME feed
    /// (latest-wins — stale video is worthless).
    stale_jpegs_dropped: u64,
    /// JPEGs handed back to the caller (lifetime).
    jpegs_produced: u64,
    /// Successful pipeline builds (1 on a healthy run; >1 means it recovered
    /// from at least one fatal).
    pipeline_builds: u64,
}

impl Default for TranscodeLoop {
    /// A loop with NO usable factory.
    ///
    /// It exists only because `#[cerulion_node]` derives `Default` on the node
    /// struct, so every field must have one — and a factory closure has no
    /// meaningful default. The node REPLACES this in `init()` (with the
    /// production GStreamer factory, or with an injected one), so it is never
    /// fed in practice; if it ever were, it fails LOUDLY on every rebuild
    /// attempt rather than looking like a decoder that produces nothing.
    fn default() -> Self {
        Self::new(
            Box::new(|| {
                Err(
                    "no transcoder factory installed — the node's init() did not run \
                     (this is a wiring bug, not a GStreamer condition)"
                        .to_string(),
                )
            }),
            DEFAULT_TARGET_HEIGHT,
        )
    }
}

impl TranscodeLoop {
    /// A fresh loop: no pipeline yet; the first [`feed`](Self::feed) attempts
    /// the build immediately. `target_height` selects which of the topic's two
    /// interleaved renditions is decoded (see [`crate::h264`]).
    pub fn new(factory: TranscoderFactory, target_height: u32) -> Self {
        Self {
            factory,
            source: None,
            gate: AuGate::new(target_height),
            retry_at_ns: None,
            failed_builds: 0,
            eos_warned: false,
            map_fail_warned: false,
            map_failures: 0,
            aus_dropped_while_down: 0,
            aus_pushed: 0,
            stale_jpegs_dropped: 0,
            jpegs_produced: 0,
            pipeline_builds: 0,
        }
    }

    /// The admission gate (its per-class counters are the skip observability).
    pub fn gate(&self) -> &AuGate {
        &self.gate
    }

    /// Lifetime count of samples that arrived without a readable buffer.
    pub fn map_failure_count(&self) -> u64 {
        self.map_failures
    }

    /// Lifetime count of access units that ARRIVED while the pipeline was down
    /// (build failed / rebuilding on backoff) and were therefore dropped.
    ///
    /// BOTH renditions, un-filtered — see the field docs. On the Go2's
    /// two-rendition topic the decodable loss is about half this number.
    pub fn aus_dropped_while_down(&self) -> u64 {
        self.aus_dropped_while_down
    }

    /// Lifetime count of access units the decoder ACCEPTED (`push_au` returned
    /// `Ok`) — the actual push tally, as opposed to `gate().admitted()`, which
    /// is the gate's decision taken one step earlier.
    pub fn aus_pushed(&self) -> u64 {
        self.aus_pushed
    }

    /// Lifetime count of JPEGs discarded as stale (a newer one was ready in
    /// the same feed).
    pub fn stale_jpegs_dropped(&self) -> u64 {
        self.stale_jpegs_dropped
    }

    /// Lifetime count of JPEGs returned to the caller.
    pub fn jpegs_produced(&self) -> u64 {
        self.jpegs_produced
    }

    /// Successful pipeline builds so far (>1 means a fatal was recovered from).
    pub fn pipeline_builds(&self) -> u64 {
        self.pipeline_builds
    }

    /// True iff a pipeline is currently up.
    pub fn is_up(&self) -> bool {
        self.source.is_some()
    }

    /// Feed ONE access unit off the wire at node-clock time `now_ns` and return
    /// the NEWEST JPEG the decoder has ready, if any.
    ///
    /// The access unit is UNFILTERED — the rendition filter and the SPS gate
    /// live here so a caller cannot forget them. A skipped access unit still
    /// drains the encoder: the decoder's output for an EARLIER access unit may
    /// be ready now, and holding it back until the next admitted frame would
    /// add a whole rendition period of latency for nothing.
    pub fn feed(&mut self, now_ns: u64, video_height: u32, au: &[u8]) -> Option<JpegFrame> {
        if self.source.is_none() {
            self.try_build(now_ns);
        }
        // Take the source out: the gate + latches beside it are mutated in the
        // same breath, which a live `&mut self.source` borrow would forbid. It
        // goes back before the drain unless the push tore it down.
        let Some(mut src) = self.source.take() else {
            self.aus_dropped_while_down += 1;
            return None;
        };

        let outcome = self.gate.classify(video_height, au);
        self.log_gate(outcome.verdict, outcome.log, video_height, au.len());
        if outcome.verdict == AuVerdict::Push {
            if let Err(e) = src.push_au(au) {
                // A refused push is not a retryable condition for THIS buffer:
                // the pipeline is flushing/unlinked/dead. Tear down and rebuild
                // on the backoff (`src` drops here → pipeline to NULL).
                self.tear_down(now_ns, format!("appsrc push refused: {e}"));
                return None;
            }
            // Counted only on SUCCESS — the gate's `admitted` already counted
            // the decision, so a refused push must not be double-claimed as a
            // push (the teardown summary reports both, and their difference).
            self.aus_pushed += 1;
        }
        self.source = Some(src);
        self.drain(now_ns)
    }

    /// Take the NEWEST JPEG the decoder currently has ready, WITHOUT feeding it
    /// anything. Returns `None` when the pipeline is down or has nothing ready.
    ///
    /// This is the second half of [`feed`](Self::feed) — production reaches it
    /// only through there, because the node is data-triggered and has no reason
    /// to poll a decoder it just gave nothing to. It is public because a
    /// BOUNDED stream has a tail: the loopback e2e's generator emits N access
    /// units and stops, and the last frames are still inside the decoder when
    /// the last one has been fed, so the harness must be able to keep draining.
    pub fn drain(&mut self, now_ns: u64) -> Option<JpegFrame> {
        let mut src = self.source.take()?;
        let mut newest: Option<JpegFrame> = None;
        let mut alive = true;
        for _ in 0..MAX_POLLS_PER_FEED {
            match src.poll() {
                Pull::Frame(frame) => {
                    if newest.is_some() {
                        // Latest wins — an older JPEG from this same drain is
                        // stale before it ever reaches the wire.
                        self.stale_jpegs_dropped += 1;
                    }
                    newest = Some(frame);
                    // A good frame re-arms the map-failure warn latch.
                    self.map_fail_warned = false;
                }
                Pull::NoFrame => break,
                Pull::MapFailure => {
                    self.map_failures += 1;
                    if self.map_fail_warned {
                        tracing::debug!(
                            total_map_failures = self.map_failures,
                            "appsink sample had no readable buffer (suppressed repeat)"
                        );
                    } else {
                        self.map_fail_warned = true;
                        tracing::warn!(
                            total_map_failures = self.map_failures,
                            "appsink sample had no readable buffer — frame skipped \
                             (repeats demoted to debug until the next good frame)"
                        );
                    }
                }
                Pull::Eos => {
                    if !self.eos_warned {
                        self.eos_warned = true;
                        tracing::warn!(
                            "camera transcode pipeline reached end-of-stream — no more \
                             frames from this instance"
                        );
                    }
                    break;
                }
                Pull::Fatal(msg) => {
                    self.tear_down(now_ns, msg);
                    alive = false;
                    break;
                }
            }
        }
        if alive {
            self.source = Some(src);
        }
        if newest.is_some() {
            self.jpegs_produced += 1;
        }
        newest
    }

    /// Backoff-gated (re)build attempt. Leaves `self.source` untouched on a
    /// gated or failed attempt.
    fn try_build(&mut self, now_ns: u64) {
        if let Some(retry_at) = self.retry_at_ns {
            if now_ns < retry_at {
                return;
            }
        }
        match (self.factory)() {
            Ok(src) => {
                if self.failed_builds > 0 {
                    tracing::info!(
                        failed_attempts = self.failed_builds,
                        "camera transcode pipeline recovered — decoding (re)started"
                    );
                }
                self.failed_builds = 0;
                self.retry_at_ns = None;
                self.eos_warned = false;
                // A fresh pipeline is a fresh regime on EVERY latch: a map
                // failure on the rebuilt pipeline must warn loudly again, not
                // ride the old regime's demotion. And a fresh DECODER has no
                // parameter sets, so the SPS gate closes again (h264::AuGate).
                self.map_fail_warned = false;
                self.gate.reset_for_new_pipeline();
                self.pipeline_builds += 1;
                self.source = Some(src);
                tracing::info!(
                    build = self.pipeline_builds,
                    target_height = self.gate.target_height(),
                    "camera transcode pipeline up — waiting for a keyframe (SPS) \
                     before feeding the decoder"
                );
            }
            Err(e) => {
                self.failed_builds += 1;
                if self.failed_builds == 1 {
                    tracing::warn!(
                        error = %e,
                        backoff_ms = REBUILD_BACKOFF_NS / 1_000_000,
                        "camera transcode pipeline failed to start — retrying on backoff \
                         (repeats demoted to debug until recovery)"
                    );
                } else {
                    tracing::debug!(
                        error = %e,
                        attempt = self.failed_builds,
                        "camera transcode pipeline start retry failed (suppressed repeat)"
                    );
                }
                self.retry_at_ns = Some(now_ns.saturating_add(REBUILD_BACKOFF_NS));
            }
        }
    }

    /// Log LOUDLY, drop the pipeline (the caller already took it out of
    /// `self.source`, so not re-inserting it IS the teardown), and arm the
    /// rebuild backoff. Never exits — the camera always tries again.
    fn tear_down(&mut self, now_ns: u64, reason: String) {
        tracing::error!(
            error = %reason,
            backoff_ms = REBUILD_BACKOFF_NS / 1_000_000,
            "camera transcode pipeline FATAL — tearing down and scheduling rebuild"
        );
        self.retry_at_ns = Some(now_ns.saturating_add(REBUILD_BACKOFF_NS));
    }

    /// Surface one gate decision at the level the flood latch chose.
    fn log_gate(&self, verdict: AuVerdict, log: FloodLog, video_height: u32, au_len: usize) {
        let (total, first) = match log {
            FloodLog::Quiet => return,
            FloodLog::First { total } => (total, true),
            FloodLog::Repeat { total } => (total, false),
        };
        if !first {
            tracing::debug!(
                ?verdict,
                video_height,
                au_len,
                total,
                "camera access unit skipped (suppressed repeat)"
            );
            return;
        }
        match verdict {
            AuVerdict::SkipUnknownRendition => tracing::warn!(
                video_height,
                target_height = self.gate.target_height(),
                known = ?crate::h264::KNOWN_RENDITION_HEIGHTS,
                total,
                "camera access unit has an UNKNOWN frame height — skipped. The Go2 has \
                 only ever published the known heights; a new one means the firmware \
                 changed or the sample was mis-decoded. Set CAMERA_TARGET_HEIGHT if the \
                 stream really moved. Repeats demoted to debug until the next decoded frame"
            ),
            AuVerdict::SkipWaitingForSps => tracing::warn!(
                video_height,
                au_len,
                total,
                "camera is WAITING FOR A KEYFRAME — access units are arriving but none has \
                 carried an SPS yet, and a decoder cannot start mid-GOP. This clears itself \
                 on the stream's next IDR (the Go2 sends SPS+PPS with every IDR). Repeats \
                 demoted to debug"
            ),
            AuVerdict::SkipEmpty => tracing::warn!(
                video_height,
                total,
                "camera access unit had a ZERO-LENGTH payload — nothing to decode, skipped. \
                 Repeats demoted to debug"
            ),
            // Quiet classes never reach here (they return FloodLog::Quiet).
            AuVerdict::Push | AuVerdict::SkipOtherRendition => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::h264::DEFAULT_TARGET_HEIGHT;
    use std::collections::VecDeque;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::{Arc, Mutex};

    /// A scripted transcoder: every `poll` pops the next scripted outcome; an
    /// exhausted script returns `NoFrame` forever (an idle-but-alive
    /// pipeline). Every `push_au` is RECORDED so a test can assert exactly
    /// which bytes reached the decoder — the gate's whole job.
    struct FakeTranscoder {
        polls: VecDeque<Pull>,
        pushed: Arc<Mutex<Vec<Vec<u8>>>>,
        push_fails: bool,
    }

    impl JpegTranscoder for FakeTranscoder {
        fn push_au(&mut self, au: &[u8]) -> Result<(), String> {
            if self.push_fails {
                return Err("scripted push failure".to_string());
            }
            self.pushed
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(au.to_vec());
            Ok(())
        }

        fn poll(&mut self) -> Pull {
            self.polls.pop_front().unwrap_or(Pull::NoFrame)
        }
    }

    fn jpeg(tag: u8) -> JpegFrame {
        JpegFrame {
            data: vec![tag; 4],
            pts_ns: None,
        }
    }

    /// An IDR access unit (SPS+PPS+IDR) — opens the SPS gate.
    fn idr_au() -> Vec<u8> {
        let mut v = Vec::new();
        for (ty, len) in [(7u8, 8usize), (8, 4), (5, 64)] {
            v.extend_from_slice(&[0x00, 0x00, 0x00, 0x01]);
            v.push(0x60 | ty);
            v.resize(v.len() + len, 0xAB);
        }
        v
    }

    /// A non-IDR slice access unit — needs an already-open gate.
    fn slice_au() -> Vec<u8> {
        let mut v = vec![0x00, 0x00, 0x00, 0x01, 0x61];
        v.resize(v.len() + 48, 0xCD);
        v
    }

    /// Scripted-factory rig. Returns (loop, factory-call counter, the vec every
    /// fake records its pushed access units into).
    #[allow(clippy::type_complexity)]
    fn rig(
        scripts: Vec<Result<Vec<Pull>, String>>,
    ) -> (TranscodeLoop, Arc<AtomicUsize>, Arc<Mutex<Vec<Vec<u8>>>>) {
        rig_with_target(scripts, DEFAULT_TARGET_HEIGHT, false)
    }

    #[allow(clippy::type_complexity)]
    fn rig_with_target(
        scripts: Vec<Result<Vec<Pull>, String>>,
        target_height: u32,
        push_fails: bool,
    ) -> (TranscodeLoop, Arc<AtomicUsize>, Arc<Mutex<Vec<Vec<u8>>>>) {
        let calls = Arc::new(AtomicUsize::new(0));
        let pushed: Arc<Mutex<Vec<Vec<u8>>>> = Arc::new(Mutex::new(Vec::new()));
        let calls_in = Arc::clone(&calls);
        let pushed_in = Arc::clone(&pushed);
        let mut scripts: VecDeque<Result<Vec<Pull>, String>> = scripts.into();
        let factory: TranscoderFactory = Box::new(move || {
            calls_in.fetch_add(1, Ordering::SeqCst);
            let Some(next) = scripts.pop_front() else {
                panic!("factory script exhausted — the loop rebuilt more times than scripted");
            };
            next.map(|polls| {
                Box::new(FakeTranscoder {
                    polls: polls.into(),
                    pushed: Arc::clone(&pushed_in),
                    push_fails,
                }) as Box<dyn JpegTranscoder>
            })
        });
        (TranscodeLoop::new(factory, target_height), calls, pushed)
    }

    fn pushed_bytes(pushed: &Arc<Mutex<Vec<Vec<u8>>>>) -> Vec<Vec<u8>> {
        pushed.lock().unwrap_or_else(|p| p.into_inner()).clone()
    }

    // 1 — the happy path: the first feed builds the pipeline, pushes the
    // keyframe, and hands back the decoder's JPEG.
    #[test]
    fn first_feed_builds_pushes_and_returns_the_frame() {
        let (mut cl, calls, pushed) = rig(vec![Ok(vec![Pull::Frame(jpeg(0x11))])]);
        let got = cl.feed(0, 720, &idr_au());
        assert_eq!(got, Some(jpeg(0x11)));
        assert_eq!(calls.load(Ordering::SeqCst), 1, "one build");
        assert_eq!(
            pushed_bytes(&pushed),
            vec![idr_au()],
            "exactly the keyframe reached the decoder, byte for byte"
        );
        assert_eq!(cl.jpegs_produced(), 1);
        assert!(cl.is_up());
        // On a HEALTHY push the two counters agree — the anti-tautology
        // control for the refused-push test below, which is the only thing
        // that can pull them apart.
        assert_eq!(cl.gate().admitted(), 1);
        assert_eq!(cl.aus_pushed(), 1);
    }

    // 2 — THE rendition pin over the loop: the sibling rendition never reaches
    // the decoder, but the decoder is still DRAINED on that feed (an earlier
    // access unit's JPEG must not wait a whole frame period).
    #[test]
    fn the_sibling_rendition_is_never_pushed_but_still_drains_the_encoder() {
        // The `NoFrame` terminators are what make the two feeds distinguishable:
        // each drain stops at the first `NoFrame`, so the script says "one JPEG
        // ready per feed" rather than "two ready at once".
        let (mut cl, _calls, pushed) = rig(vec![Ok(vec![
            Pull::Frame(jpeg(0x11)), // ready during the 720 keyframe feed
            Pull::NoFrame,
            Pull::Frame(jpeg(0x22)), // ready during the 360 feed
        ])]);
        assert_eq!(cl.feed(0, 720, &idr_au()), Some(jpeg(0x11)));
        assert_eq!(
            cl.feed(1_000, 360, &idr_au()),
            Some(jpeg(0x22)),
            "a skipped access unit still drains the encoder"
        );
        assert_eq!(
            pushed_bytes(&pushed),
            vec![idr_au()],
            "the 360 rendition was NOT fed to the decoder"
        );
        assert_eq!(cl.gate().skipped_other_rendition(), 1);
    }

    // 3 — the SPS gate through the loop: mid-GOP slices are dropped before the
    // decoder ever sees them; the first IDR opens the gate.
    #[test]
    fn mid_gop_slices_never_reach_the_decoder_until_the_first_keyframe() {
        let (mut cl, _calls, pushed) = rig(vec![Ok(vec![])]);
        assert_eq!(cl.feed(0, 720, &slice_au()), None);
        assert_eq!(cl.feed(1_000, 720, &slice_au()), None);
        assert!(
            pushed_bytes(&pushed).is_empty(),
            "not one pre-keyframe byte reached the decoder"
        );
        cl.feed(2_000, 720, &idr_au());
        cl.feed(3_000, 720, &slice_au());
        assert_eq!(
            pushed_bytes(&pushed),
            vec![idr_au(), slice_au()],
            "the keyframe and everything after it"
        );
        assert_eq!(cl.gate().skipped_waiting_for_sps(), 2);
    }

    // 4 — build failure: backoff-GATED retry (no rebuild storm), loud once per
    // regime, recovery, and the access units that arrive meanwhile are counted.
    #[test]
    fn build_failure_backoff_gates_retry_then_recovers() {
        let (mut cl, calls, _pushed) = rig(vec![
            Err("no such element: nvv4l2decoder".to_string()),
            Ok(vec![Pull::Frame(jpeg(0x33))]),
        ]);
        // Attempt 1 fails; the AU is dropped and counted.
        assert_eq!(cl.feed(0, 720, &idr_au()), None);
        assert_eq!(calls.load(Ordering::SeqCst), 1);
        assert_eq!(cl.aus_dropped_while_down(), 1);

        // 10 ms later, still inside the 1 s backoff: INERT — no factory call.
        assert_eq!(cl.feed(10_000_000, 720, &idr_au()), None);
        assert_eq!(
            calls.load(Ordering::SeqCst),
            1,
            "the backoff gate must suppress rebuild attempts before REBUILD_BACKOFF_NS"
        );
        assert_eq!(cl.aus_dropped_while_down(), 2);

        // Past the backoff: rebuild runs, succeeds, and the SAME feed's access
        // unit is decoded (nothing is lost to the rebuild itself).
        let got = cl.feed(REBUILD_BACKOFF_NS + 1, 720, &idr_au());
        assert_eq!(calls.load(Ordering::SeqCst), 2);
        assert_eq!(got, Some(jpeg(0x33)));
        assert_eq!(
            cl.aus_dropped_while_down(),
            2,
            "the rebuild feed was NOT dropped"
        );
        assert_eq!(cl.pipeline_builds(), 1, "only the successful build counts");
    }

    // 5 — the backoff boundary is `now >= retry_at`, pinned on BOTH sides.
    #[test]
    fn the_rebuild_backoff_is_a_threshold_pinned_on_both_sides() {
        let (mut cl, calls, _pushed) = rig(vec![
            Err("boom".to_string()),
            Err("boom".to_string()),
            Ok(vec![]),
        ]);
        cl.feed(0, 720, &idr_au()); // attempt 1 → retry_at = 1s
        assert_eq!(calls.load(Ordering::SeqCst), 1);
        cl.feed(REBUILD_BACKOFF_NS - 1, 720, &idr_au());
        assert_eq!(
            calls.load(Ordering::SeqCst),
            1,
            "one ns before the gate: inert"
        );
        cl.feed(REBUILD_BACKOFF_NS, 720, &idr_au());
        assert_eq!(
            calls.load(Ordering::SeqCst),
            2,
            "exactly at the gate: retried"
        );
    }

    // 6 — the push path: a fatal bus error tears the pipeline down, the
    // rebuild waits out the backoff, and the rebuilt pipeline needs a FRESH
    // keyframe (a mid-GOP slice must not be pushed into a decoder that has no
    // parameter sets).
    #[test]
    fn a_fatal_tears_down_and_the_rebuilt_pipeline_waits_for_a_new_keyframe() {
        let (mut cl, calls, pushed) = rig(vec![
            Ok(vec![Pull::Fatal(
                "nvv4l2decoder0: decode error (corrupt IDR)".to_string(),
            )]),
            // The rebuilt pipeline has nothing ready on the feed that builds it
            // (that feed carries a slice it must not decode anyway), then emits
            // once the fresh keyframe arrives.
            Ok(vec![Pull::NoFrame, Pull::Frame(jpeg(0x44))]),
        ]);
        // Build + push the keyframe; the poll returns the fatal.
        assert_eq!(cl.feed(0, 720, &idr_au()), None);
        assert!(!cl.is_up(), "a fatal must tear the pipeline down");
        assert_eq!(calls.load(Ordering::SeqCst), 1);

        // Inside the backoff: no rebuild.
        cl.feed(500_000_000, 720, &slice_au());
        assert_eq!(calls.load(Ordering::SeqCst), 1);

        // Past it: rebuild, but this feed is a SLICE — the fresh decoder has no
        // SPS, so nothing is pushed.
        cl.feed(REBUILD_BACKOFF_NS + 1, 720, &slice_au());
        assert_eq!(calls.load(Ordering::SeqCst), 2, "rebuilt after the backoff");
        assert_eq!(
            pushed_bytes(&pushed),
            vec![idr_au()],
            "only the PRE-fatal keyframe was ever pushed — the rebuilt decoder \
             was not fed a mid-GOP slice"
        );

        // The next keyframe re-opens it and the pipeline delivers again.
        assert_eq!(
            cl.feed(REBUILD_BACKOFF_NS + 2, 720, &idr_au()),
            Some(jpeg(0x44))
        );
        assert_eq!(pushed_bytes(&pushed), vec![idr_au(), idr_au()]);
        assert_eq!(cl.pipeline_builds(), 2);
    }

    // 7 — a REFUSED push is fatal: torn down, backoff armed, counted. It is
    // also the ONE case where the gate's ADMISSION count and the loop's PUSH
    // count legitimately diverge, which is why they are two counters.
    #[test]
    fn a_refused_push_tears_the_pipeline_down_and_is_never_counted_as_pushed() {
        let (mut cl, calls, _pushed) =
            rig_with_target(vec![Ok(vec![]), Ok(vec![])], DEFAULT_TARGET_HEIGHT, true);
        assert_eq!(cl.feed(0, 720, &idr_au()), None);
        assert!(!cl.is_up(), "a refused push means the pipeline is dead");
        assert_eq!(
            cl.gate().admitted(),
            1,
            "the gate ADMITTED the keyframe — that decision is real and stands"
        );
        assert_eq!(
            cl.aus_pushed(),
            0,
            "nothing reached the decoder, so nothing may be counted as pushed \
             (incrementing before push_au's Ok fails here)"
        );
        // Backoff-gated exactly like a bus fatal.
        cl.feed(1_000, 720, &idr_au());
        assert_eq!(calls.load(Ordering::SeqCst), 1);
        cl.feed(REBUILD_BACKOFF_NS, 720, &idr_au());
        assert_eq!(calls.load(Ordering::SeqCst), 2);
        // The rebuilt pipeline re-closed the SPS gate, this feed's keyframe
        // re-opened it and was admitted — and refused again.
        assert_eq!(cl.gate().admitted(), 2);
        assert_eq!(cl.aus_pushed(), 0);
    }

    // 8 — newest-wins drain: a burst of ready JPEGs in ONE feed publishes the
    // NEWEST and counts the rest as stale.
    #[test]
    fn a_burst_drains_to_the_newest_and_counts_the_stale_ones() {
        let (mut cl, _calls, _pushed) = rig(vec![Ok(vec![
            Pull::Frame(jpeg(0x01)),
            Pull::Frame(jpeg(0x02)),
            Pull::Frame(jpeg(0x03)),
        ])]);
        assert_eq!(
            cl.feed(0, 720, &idr_au()),
            Some(jpeg(0x03)),
            "latest wins — stale video is worthless"
        );
        assert_eq!(cl.stale_jpegs_dropped(), 2);
        assert_eq!(cl.jpegs_produced(), 1, "one frame REACHED the caller");
    }

    // 9 — the drain is BOUNDED: a pathologically productive encoder cannot
    // spin one tick forever.
    #[test]
    fn the_drain_is_bounded_per_feed() {
        let script: Vec<Pull> = (0..(MAX_POLLS_PER_FEED as u8 + 5))
            .map(|i| Pull::Frame(jpeg(i)))
            .collect();
        let (mut cl, _calls, _pushed) = rig(vec![Ok(script)]);
        let got = cl.feed(0, 720, &idr_au());
        assert_eq!(
            got,
            Some(jpeg(MAX_POLLS_PER_FEED as u8 - 1)),
            "the newest of exactly MAX_POLLS_PER_FEED polls"
        );
        assert_eq!(cl.stale_jpegs_dropped(), MAX_POLLS_PER_FEED as u64 - 1);
    }

    // 10 — map failures: counted (lifetime), warn-latched once per regime,
    // re-armed by the next good frame; the poll loop keeps going.
    #[test]
    fn map_failures_count_and_the_latch_rearms_on_a_good_frame() {
        let (mut cl, _calls, _pushed) = rig(vec![Ok(vec![
            Pull::MapFailure,
            Pull::MapFailure,
            Pull::Frame(jpeg(0x55)),
        ])]);
        assert_eq!(
            cl.feed(0, 720, &idr_au()),
            Some(jpeg(0x55)),
            "map failures do not abort the drain"
        );
        assert_eq!(cl.map_failure_count(), 2);
        assert!(
            !cl.map_fail_warned,
            "the good frame that followed re-armed the warn latch"
        );
    }

    // 11 — EOS: warned once per instance, breaks the drain, and is NOT a
    // rebuild trigger (the factory is never re-called).
    #[test]
    fn eos_warns_once_and_never_rebuilds() {
        let (mut cl, calls, _pushed) =
            rig(vec![Ok(vec![Pull::Eos, Pull::Eos, Pull::Eos, Pull::Eos])]);
        for step in 0..3u64 {
            assert_eq!(cl.feed(step, 720, &idr_au()), None);
            assert!(cl.eos_warned, "the EOS latch armed on the first one");
        }
        assert_eq!(
            calls.load(Ordering::SeqCst),
            1,
            "EOS must NOT tear down / rebuild"
        );
        assert!(cl.is_up());
    }

    // 12 — an empty payload is skipped and never pushed, at any height.
    #[test]
    fn an_empty_access_unit_is_never_pushed() {
        let (mut cl, _calls, pushed) = rig(vec![Ok(vec![])]);
        assert_eq!(cl.feed(0, 720, &[]), None);
        assert!(pushed_bytes(&pushed).is_empty());
        assert_eq!(cl.gate().skipped_empty(), 1);
    }

    // 13 — the poll-only `drain`: it yields the decoder's TAIL after the last
    // access unit has been fed (the bounded-stream case the loopback e2e hits),
    // and on a DOWN pipeline it is a clean `None` that never counts a dropped
    // access unit (nothing was offered).
    #[test]
    fn drain_yields_the_tail_and_is_inert_while_down() {
        let (mut cl, _calls, _pushed) = rig(vec![Ok(vec![
            Pull::NoFrame,           // nothing ready on the feed itself
            Pull::Frame(jpeg(0x77)), // the decoder catches up afterwards
        ])]);
        assert_eq!(cl.feed(0, 720, &idr_au()), None);
        assert_eq!(cl.drain(1_000), Some(jpeg(0x77)), "the tail comes out");
        assert_eq!(cl.drain(2_000), None, "and then it is empty");
        assert_eq!(cl.jpegs_produced(), 1);

        // A loop that has never built (its first feed has not happened) drains
        // to None without touching the AU-drop counter.
        let (mut down, calls, _p) = rig(vec![Err("boom".to_string())]);
        assert_eq!(down.drain(0), None);
        assert_eq!(down.aus_dropped_while_down(), 0);
        assert_eq!(calls.load(Ordering::SeqCst), 0, "drain never builds");
    }

    // 14 — determinism (Principle #7): the same script fed twice produces
    // byte-identical output AND matches a HAND oracle (a two-run comparison
    // alone would pass a both-wrong bug).
    #[test]
    fn a_scripted_run_is_deterministic_and_matches_a_hand_oracle() {
        let script: Vec<(u64, u32, Vec<u8>)> = vec![
            (0, 360, idr_au()),       // sibling — skipped
            (1_000, 720, slice_au()), // mid-GOP — no SPS yet
            (2_000, 720, idr_au()),   // target keyframe
            (3_000, 360, slice_au()), // sibling
            (4_000, 720, slice_au()), // decoded
        ];
        let oracle: Vec<Option<JpegFrame>> =
            vec![None, None, Some(jpeg(0xA1)), None, Some(jpeg(0xA2))];

        let run = |()| -> Vec<Option<JpegFrame>> {
            // Decoder model: nothing ready until the keyframe has been fed,
            // then one JPEG per pushed access unit. Each `NoFrame` terminates
            // one feed's drain, so the script maps 1:1 onto the 5 feeds.
            let (mut cl, _calls, _pushed) = rig(vec![Ok(vec![
                Pull::NoFrame,
                Pull::NoFrame,
                Pull::Frame(jpeg(0xA1)),
                Pull::NoFrame,
                Pull::NoFrame,
                Pull::Frame(jpeg(0xA2)),
            ])]);
            script
                .iter()
                .map(|(t, h, au)| cl.feed(*t, *h, au))
                .collect()
        };
        let a = run(());
        let b = run(());
        assert_eq!(a, oracle, "run A must match the hand oracle");
        assert_eq!(b, oracle, "run B must match the hand oracle");
        assert_eq!(a, b, "the two runs must be byte-identical");
    }
}
