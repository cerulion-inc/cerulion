// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion bag` — the bag PLAYBACK / desk-side RECORD verbs.
//!
//! # Why this exists (and why it is not `cerulion replay`)
//!
//! [`crate::replay_cmd`] is a deterministic **verifier**: it re-EXECUTES the
//! bagged graph's cdylibs and byte-diffs what they produce against the
//! recording. Only external-source topics are re-injected — every PRODUCED
//! topic is recomputed, so replaying an `ros2 attach` bag on a desk with no DDS
//! robot produces nothing at all. It is also as-fast-as-possible with no pacing
//! knob (the replay loop never reads the wall clock — deliberate, Principle #7),
//! so it cannot drive a viewer.
//!
//! `bag play` is the opposite thing: a **dumb, verbatim frame pump**. It reads the
//! recorded frames and re-publishes them BYTE-VERBATIM onto local iceoryx2
//! shared memory under their RECORDED topic names, wall-paced from the bag's own
//! log times. Nothing is re-executed, nothing is recomputed, no node is loaded.
//! To anything attached to local SHM — `cerulion topic list`/`echo`/`hz`,
//! `cerulion viz`, Studio — a played bag looks exactly like a robot that is
//! publishing. That is the point: it is the robot-substitute data source for
//! working on the desk-side shell when the robot is down.
//!
//! `bag record` is its dual, and it is a FRONT-END over `cerulion_bagd` rather
//! than a second recorder: it derives a topic set (named topics, `--all`,
//! `--regex`/`--exclude` — the `ros2 bag record` surface) and hands it to
//! bagd's existing tap + drive + finalize machinery, keeping all of that
//! recorder's loss accounting. It is local only. It taps this
//! machine's shared memory and never pulls a topic's frames across the network
//! (schema lookup alone may query peers), so recording a robot
//! means running it ON the robot and transferring the file afterwards.
//!
//! # What `bag play` guarantees, and what it does not
//!
//! | Guaranteed | NOT guaranteed |
//! |---|---|
//! | Frames are published byte-verbatim (header `sequence` + `timestamp_ns` intact) | That a viewer can DECODE them (see "schema resolution" below) |
//! | Topic names are the recorded names, verbatim | Deterministic timing — this is wall-paced, so it is explicitly NOT `Replay = Live` |
//! | File order (= the recorder's arrival order) is preserved | That inter-frame gaps are reproduced exactly (they are clamped, see [`MAX_FRAME_GAP_NS`]) |
//! | A topic whose publisher slot is taken is refused LOUDLY, by name | That every topic in the bag plays — a refused one is named and skipped |
//!
//! # Schema resolution for viewers
//!
//! Resolution keys off the wire HASH, never the recorded name. A viewer decodes
//! a frame by its `WireHeader.schema_hash`; the bag's schema NAME never reaches
//! it, and an attach-mode recording could not even record one until recently
//! (the wire carries no name).
//!
//! Now a bag carries its own schema PROVENANCE — the verbatim text of
//! the CUSTOM types its frames use, plus the `schema_hash` → name bindings the
//! recorder resolved — in the [`cerulion_bag::SCHEMA_DOCS_ATTACHMENT`]
//! attachment. So `bag info` and the `bag play` banner report each channel as
//! one of:
//!
//! - resolvable LOCALLY (this machine compiled the type) — renders;
//! - resolvable FROM THE BAG (this machine did not, and the bag brought the
//!   definition) — renders, because `bag play` hands those definitions to a
//!   running `cerulion-vizd`;
//! - NAMED but undecodable (the bag says what the type is and carries no text,
//!   and this build's copy hashes differently) — a corpus skew, reported as one;
//! - unresolvable anywhere — reported as "nothing will render", with the remedy.
//!
//! What this replaced: bags carried no schema text at all, so a vendor type on a
//! bare desk rendered NOTHING and the failure was success-shaped — vizd attaches
//! `ok: true`, `cerulion viz` prints `attached … → (resolving…) [(pending)]` and
//! exits 0, and the only signal was one `warn!` per unknown hash in `vizd.log`.
//! A bag recorded by an older build, or outside a workspace, still lands in that
//! state; the banner names it and says what to do about it.
//!
//! `cerulion topic echo` was never affected: it builds its own walker from the
//! workspace schema store, so it decodes vendor types on a played bag.
//!
//! # Not network-aware (v1)
//!
//! Playback is LOCAL only: frames land in this machine's SHM. Serving a played
//! bag to another machine (running the player on a robot-substitute box and
//! letting a desk demand its topics) composes from existing pieces but is not
//! wired here.

use std::collections::BTreeMap;
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_bag::{BagChannel, BagCompleteness, BagError, BagReader, RESERVED_PREFIX};
use cerulion_core::transport::mirror_registry;
use cerulion_core::transport::network::IngressInjector;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};

use crate::error::{CliError, CliResult};
use crate::viz_client::SchemaPushOutcome;

/// The largest inter-frame gap `bag play` will actually sleep for, whatever the
/// bag's log times say (5 s).
///
/// Two independent reasons a raw log-time delta can be pathological:
///
/// 1. **A recording pause.** A bag with a ten-minute idle stretch would
///    otherwise stall playback for ten minutes.
/// 2. **Cross-epoch stamps.** `bagd` writes each message's MCAP `log_time` from
///    the frame's own `WireHeader::timestamp_ns`, i.e. the PRODUCING process's
///    clock. Within one recorded graph run the workers advance in barrier
///    lockstep so those stamps share a number line — but a bag that mixes
///    producers from different clock epochs (a restarted worker, a mirror
///    re-injected from another machine) can carry deltas that are enormous or
///    NEGATIVE. Negative deltas play immediately; enormous ones are capped here.
///
/// 5 s is comfortably above any real sensor period (so no genuine stream is
/// distorted) while bounding both pathologies. The pre-scan reports how many
/// non-monotonic log times it saw, so a cross-epoch bag is visible rather than
/// merely odd-feeling.
pub const MAX_FRAME_GAP_NS: u64 = 5_000_000_000;

/// Longest single `sleep` the pace loop performs before re-checking the
/// shutdown flag, so Ctrl-C is honoured promptly even inside a long gap.
const SLEEP_SLICE: Duration = Duration::from_millis(50);

/// Options for [`bag_play`].
#[derive(Debug, Clone)]
pub struct PlayOptions {
    /// Playback rate multiplier. `1.0` = the recorded wall pacing; `2.0` = twice
    /// as fast; `0.5` = half speed. Must be finite and `> 0`.
    pub rate: f64,
    /// Restart at the beginning when the bag ends, until interrupted.
    pub repeat: bool,
    /// Play only these topics. Empty = every user topic in the bag.
    pub topics: Vec<String>,
    /// `--start-offset`: skip the leading `S` NANOSECONDS of
    /// each channel's OWN recorded timeline. `None` starts at the beginning.
    ///
    /// PER CHANNEL, and that is forced rather than chosen: wire stamps across
    /// channels share no number line (see [`pace_step`]'s doc), so a single
    /// global `t0 + offset` would be exactly the cross-clock arithmetic this
    /// module refuses everywhere else. The seek is a SKIP-PREFIX rather than a
    /// random access because the user-frame walk is forward-only.
    pub start_offset_ns: Option<u64>,
    /// `--duration`: stop republishing a channel once it has
    /// advanced this many NANOSECONDS through its OWN recorded timeline
    /// (measured from [`Self::start_offset_ns`]). `None` plays to the end.
    ///
    /// Per channel for the same reason as the offset above.
    pub duration_bound_ns: Option<u64>,
    /// Workspace `schemas/` directory, used to resolve each channel's wire hash
    /// so the banner can say truthfully whether a viewer will render it.
    pub schemas_dir: Option<std::path::PathBuf>,
}

impl Default for PlayOptions {
    fn default() -> Self {
        Self {
            rate: 1.0,
            repeat: false,
            topics: Vec::new(),
            start_offset_ns: None,
            duration_bound_ns: None,
            schemas_dir: None,
        }
    }
}

/// One user channel of a bag, as summarised by [`scan_bag`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChannelScan {
    /// The recorded topic name, verbatim.
    pub topic: String,
    /// The qualified schema name from the MCAP Schema record (`"unknown"` for a
    /// bag recorded in bagd's attach mode, which learns the hash from the first
    /// frame but never the name).
    pub schema_name: String,
    /// The channel's expected wire `schema_hash`, from the Cerulion schema
    /// descriptor. `None` when the channel carries no Cerulion descriptor at all
    /// (a foreign MCAP), which makes the channel unplayable — there is nothing
    /// to validate frames against.
    pub schema_hash: Option<u64>,
    /// Frames counted on this channel.
    pub frames: u64,
    /// The largest frame (full wire frame: 32-byte header + payload) in bytes.
    pub max_frame_len: usize,
    /// The first and last MCAP `log_time` seen, in ns. `None` for an empty
    /// channel.
    pub first_log_time: Option<u64>,
    /// See [`ChannelScan::first_log_time`].
    pub last_log_time: Option<u64>,
}

/// The result of one full pre-scan pass over a bag.
#[derive(Debug, Clone)]
pub struct BagScan {
    /// Every USER channel (reserved `__cerulion/*` excluded), in topic order.
    pub channels: Vec<ChannelScan>,
    /// Total user frames counted.
    pub total_frames: u64,
    /// How many times a frame's wire timestamp went BACKWARDS relative to its
    /// predecessor in recorded order. Non-zero means the bag mixes clock
    /// epochs (see [`MAX_FRAME_GAP_NS`]); playback still runs, in recorded
    /// order.
    pub backwards_log_times: u64,
    /// Frames whose 32-byte wire header would not parse. They are still
    /// republished verbatim; they contribute no timing.
    pub headerless_frames: u64,
    /// The longest span WITHIN any single channel, in ns — never a subtraction
    /// across channels, whose stamps share no number line.
    pub span_ns: Option<u64>,
    /// How the bag ENDED, rendered for the operator.
    pub completeness: String,
    /// Whether the bag carries the finalization epilogue. A non-finalized bag
    /// has no summary footer, so its frames cannot be walked — `bag info`
    /// still reports what it can, `bag play` refuses it loudly.
    pub finalized: bool,
}

/// Per-topic accounting for one [`bag_play`] run.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TopicPlayback {
    /// The topic name.
    pub topic: String,
    /// Frames validated AND published into local SHM.
    pub injected: u64,
    /// Frames refused by wire validation (`schema_hash` / `total_size` /
    /// unparseable header). Always zero for a bag this recorder wrote.
    pub rejected: u64,
    /// Frames that validated but could not be published locally (loan-pool
    /// pressure). Investigate THIS machine.
    pub failed: u64,
}

/// The outcome of a [`bag_play`] run.
#[derive(Debug, Clone)]
pub struct PlaySummary {
    /// Per-topic accounting, in topic order.
    pub topics: Vec<TopicPlayback>,
    /// Complete passes over the bag (1 without `--loop`; the count reached
    /// before interruption with it).
    pub passes: u64,
    /// Topics present in the bag (and selected) that could NOT be played, each
    /// with the reason. Never silent.
    pub refused: Vec<(String, String)>,
    /// Wall time the playback loop ran for.
    pub elapsed: Duration,
    /// The worst amount, in ns, by which a frame was published BEHIND its
    /// scheduled time. Under lag the player slips the schedule; it never drops
    /// a frame and never reorders one.
    pub max_slip_ns: u64,
    /// How many frames were published behind schedule.
    pub slipped_frames: u64,
    /// How many times a clock domain's timeline stepped BACKWARDS, re-anchoring
    /// its schedule. A property of the RECORDING (a producer restart, a
    /// `--loop` wrap), not of this machine — reported separately from
    /// [`PlaySummary::slipped_frames`] so the two are never confused.
    pub timeline_restarts: u64,
    /// How many DISTINCT channels had to FAST-FORWARD — i.e. hit the
    /// [`CATCHUP_FACTOR`] rate bound at least once during the run.
    ///
    /// This is an observation about THIS RUN, never a claim about the
    /// recording: a channel fast-forwards when its frames were reached well
    /// after their scheduled instant, which happens both when a producer handed
    /// over mid-recording and when the recorder's first flush was large. The
    /// player does not pretend to tell those apart — it bounds the rate either
    /// way and says how many channels it applied to.
    ///
    /// A SET underneath, so five `--loop` passes over a two-channel bag report
    /// two channels, not ten events. It is CUMULATIVE over the run and never
    /// decays: a channel that fast-forwarded once in pass 1 and never again
    /// still counts at the end. The summary says "at some point" for that
    /// reason.
    pub catchup_channels: u64,
    /// How long the recording SHOULD have taken at this `--rate`: the longest
    /// PLAYED channel's anchor plus its cumulative scheduled advance.
    ///
    /// This is the whole-run companion to [`PlaySummary::max_slip_ns`], which a
    /// re-anchor RESETS — on a bag with frequent regressions the worst slip
    /// measures lag since the last restart, not lag against the recording, and
    /// can read three orders of magnitude below a run's real overrun. Comparing
    /// this to [`PlaySummary::elapsed`] cannot be reset by anything.
    pub scheduled_span_ns: u64,
}

impl PlaySummary {
    /// Total frames published across every topic and pass.
    pub fn total_injected(&self) -> u64 {
        self.topics.iter().map(|t| t.injected).sum()
    }

    /// How far the run ran BEHIND the recording's own duration, or `None` when
    /// it kept up (within [`WALL_OVERRUN_REPORT_FLOOR_NS`]).
    ///
    /// Unlike [`PlaySummary::max_slip_ns`] this cannot be reset by a timeline
    /// restart, which is exactly why it exists: the slip accumulator restarts
    /// at every discontinuity, so on a bag full of them the worst slip measures
    /// lag since the LAST restart rather than lag against the recording.
    pub fn wall_overrun_ns(&self) -> Option<u64> {
        let wall = self.elapsed.as_nanos().min(u128::from(u64::MAX)) as u64;
        let over = wall.saturating_sub(self.scheduled_span_ns);
        (over >= WALL_OVERRUN_REPORT_FLOOR_NS).then_some(over)
    }
}

/// Overrun below this is not reported: playback ends when the LAST frame is
/// published, so the run legitimately exceeds the recorded span by the setup
/// cost plus one publish. Reporting that as lag would be the same false alarm
/// the first-frame anchor removes.
pub const WALL_OVERRUN_REPORT_FLOOR_NS: u64 = 250_000_000;

// ---------------------------------------------------------------------------
// Pure helpers (oracle-tested — no bag, no transport)
// ---------------------------------------------------------------------------

/// The pacing decision for one frame: how long to wait, whether the schedule
/// has already slipped past it, and whether the bag's timeline restarted here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PaceStep {
    /// Nanoseconds to sleep before publishing. `0` when the frame is already
    /// due (or overdue).
    pub sleep_ns: u64,
    /// How far BEHIND the ideal schedule this frame is, in ns. Non-zero means
    /// THE MACHINE could not keep up; the frame is published immediately and
    /// nothing is dropped or reordered. A timeline regression never reports
    /// slip — see [`PaceStep::re_anchored`].
    pub slip_ns: u64,
    /// The frame's stamp went BACKWARDS, so the schedule was re-anchored to
    /// now. Counts clock-epoch discontinuities WITHIN one clock domain of the
    /// BAG (a producer restart, a `--loop` wrap), not lag on this machine.
    pub re_anchored: bool,
    /// How far this frame advanced its domain's schedule, in ns (0 for an
    /// anchor or a re-anchor). Summed by the caller into an un-resettable
    /// measure of how long the recording SHOULD have taken — a re-anchor
    /// resets the slip accumulator, so slip alone cannot answer that.
    pub advance_ns: u64,
}

/// Advance ONE CLOCK DOMAIN's playback schedule by one frame.
///
/// The schedule is an ANCHORED cumulative target, not a chain of per-frame
/// sleeps: `target = anchor + Σ(clamped delta / rate)`. Accumulating sleeps
/// would let every scheduling overshoot compound, so a 1 kHz stream would drift
/// permanently slower; anchoring makes a late frame catch up against the
/// original anchor instead.
///
/// # `prev_log_time` MUST come from the same clock domain
///
/// Wire stamps are PER-PROCESS clock domains and must never be cross-compared —
/// the repo's standing rule (the liveness epoch-reset class). Passing the previous
/// frame of a DIFFERENT domain injects that domain's clock OFFSET into this
/// schedule instead of the topic's real inter-frame delta. Measured on a
/// transliteration of this function: a 20 Hz topic interleaved with a 3 Hz
/// topic whose clock sits 200 ms away plays at **13.02 Hz**, and at a 5 s
/// offset it plays at **1.22 Hz** — a 10 s recording taking 163 s, silently,
/// with `slipped_frames` reading 0 the whole way. `bag_play_with_manager`
/// therefore keeps one [`ChannelClock`] per CHANNEL, and never compares one
/// channel's stamps against another's.
///
/// Three clamps on the raw delta, each load-bearing:
///
/// - no predecessor (this channel's first frame) ⇒ the caller-seeded RUN ORIGIN
///   stands, so every channel shares one wall origin (see below);
/// - a NEGATIVE delta (stamps went backwards) ⇒ the timeline RESTARTED, see
///   below;
/// - a delta above [`MAX_FRAME_GAP_NS`] ⇒ exactly that cap, applied BEFORE the
///   rate divide so `--rate` still speeds a clamped gap up.
///
/// # One RUN ORIGIN, seeded by the caller
///
/// The run's origin is the wall instant its FIRST frame was reached, and every
/// channel's schedule starts there. Two things follow.
///
/// The run's first frame is never reported late: before the origin existed,
/// `schedule` sat at 0 while the wall clock had already advanced through the
/// walk-open, the footer read and the header parse, so `schedule <= elapsed`
/// held on frame 0 of EVERY pass and every run — however healthy — reported a
/// frame "published behind schedule" and advised lowering `--rate`.
///
/// And channels do not drift apart: a recorder writes each flush grouped by
/// TOPIC, so a later topic's frames are reached after the earlier topics' have
/// been paced. Anchoring each channel at its own arrival instant would bake that
/// offset in permanently (up to `(K-1)` flush windows for K topics); sharing one
/// origin makes it a transient catch-up at the start instead, which the summary
/// reports as STRUCTURAL rather than as machine lag.
///
/// # A backwards stamp RE-ANCHORS the schedule
///
/// Within one domain a backwards stamp means the PRODUCER restarted (or, under
/// `--loop`, the bag wrapped). Such a frame is due IMMEDIATELY: there is no
/// meaningful schedule relationship across a clock discontinuity.
///
/// Merely clamping its delta to zero is not enough, and the difference is not
/// theoretical. The clamp leaves the schedule where it was while the wall clock
/// keeps moving, so the schedule falls permanently behind and EVERY LATER FRAME
/// reports slip. MEASURED on a real 79 s Go2 recording (1813 frames, two
/// producers, 150 backwards stamps): the run reported "147 frames published
/// behind schedule, worst 53.6 ms late" while `cerulion topic hz` measured the
/// played topic at a faithful 19.94–20.13 Hz against its recorded 20.02 Hz.
/// The playback was fine; the metric was wrong, and it accused the operator's
/// machine of lag that did not exist.
///
/// So a regression re-anchors: `schedule = max(schedule, elapsed)`. `max`, not
/// a bare assignment — a run that is AHEAD of schedule (mid-sleep) must not
/// have its schedule dragged backwards, which would speed playback up.
///
/// Pacing changes WHEN a frame is published, never WHICH frame or in what
/// order — the repo's firewall discipline. A frame that is already overdue
/// reports its slippage and publishes at once.
pub fn pace_step(
    elapsed_ns: u64,
    schedule_ns: &mut u64,
    prev_log_time: Option<u64>,
    log_time: u64,
    rate: f64,
) -> PaceStep {
    let mut re_anchored = false;
    let mut advance_ns = 0u64;
    match prev_log_time {
        // This channel's FIRST frame: the caller has already seeded
        // `schedule_ns` with the RUN ORIGIN (the wall instant the run's first
        // frame was reached), so the frame is due at that origin. Leaving the
        // schedule alone here is what keeps every channel on ONE wall origin;
        // anchoring each channel at its own arrival instant instead would give
        // a K-topic bag up to (K-1) flush windows of PERMANENT inter-topic skew
        // (7 s on a 75-topic `ros2 attach` bag), because a recorder writes each
        // flush grouped by topic.
        None => {}
        Some(prev) if log_time < prev => {
            // The domain's timeline restarted. Clear accumulated lag rather
            // than reporting it forever against a schedule this frame has no
            // relationship to.
            *schedule_ns = (*schedule_ns).max(elapsed_ns);
            re_anchored = true;
        }
        Some(prev) => {
            let delta = (log_time - prev).min(MAX_FRAME_GAP_NS);
            // `rate` is validated finite and `> 0` by `validate_rate` at the
            // entry gate, so this cannot be NaN or negative. Saturate rather
            // than wrap on a pathologically small rate.
            let scaled = delta as f64 / rate;
            advance_ns = if scaled >= u64::MAX as f64 {
                u64::MAX
            } else {
                scaled as u64
            };
            *schedule_ns = schedule_ns.saturating_add(advance_ns);
        }
    }
    if *schedule_ns > elapsed_ns {
        PaceStep {
            sleep_ns: *schedule_ns - elapsed_ns,
            slip_ns: 0,
            re_anchored,
            advance_ns,
        }
    } else {
        // A RE-ANCHORED frame reports ZERO slip, and does so STRUCTURALLY
        // rather than by a special case: `max(schedule, elapsed) >= elapsed`,
        // and this arm is reached only when `schedule <= elapsed`, so the two
        // together force `schedule == elapsed` and `elapsed - schedule` is
        // necessarily 0. An explicit guard here would be unreachable — a branch
        // no test can distinguish, which is exactly the dead code this repo
        // refuses to carry. (Deleting such a guard changes no
        // observable behaviour and fails no test.)
        //
        // The FIRST frame of a channel is a different case: the caller seeds the
        // schedule with the run origin, which may sit BELOW `elapsed`, so that
        // arm legitimately reports the catch-up as slip. Hence the
        // `re_anchored` qualifier — asserting the equality unconditionally
        // would be false.
        debug_assert!(
            !re_anchored || *schedule_ns == elapsed_ns,
            "a re-anchored schedule can never sit behind the wall clock"
        );
        PaceStep {
            sleep_ns: 0,
            slip_ns: elapsed_ns - *schedule_ns,
            re_anchored,
            advance_ns,
        }
    }
}

/// How many times its recorded rate a channel may publish while CATCHING UP.
///
/// # Why a rate bound and not a deficit cap
///
/// A channel's frames are REACHED after the earlier topics' first batches have
/// been paced, so it starts with a deficit. Two very different situations
/// produce one:
///
/// * **Write batching.** A recorder writes each flush grouped by topic, so a
///   channel that was active from the very first instant is still reached late.
///   Its deficit must be carried in full, or the channels drift apart
///   permanently.
/// * **A genuine late start.** A producer handed over halfway through the
///   recording. Carrying that deficit dumps its whole backlog at full speed —
///   MEASURED before any bound, a handover at the halfway point of a 6 s bag
///   flushed 3 s of data in 0.6 ms (about 500 kHz).
///
/// A later version tried to separate them by the deficit's SIZE (cap it at
/// 250 ms). That was WRONG, and wrong in the common direction: the deficit is a function of the
/// FIRST per-tap chunk's span, and bagd cannot flush until its writer exists
/// (schema-wait, `cerulion_bagd/src/lib.rs:3872`), after which it writes every
/// held frame in ONE chunk of up to `held_budget` (= `max_borrowed - 1`, so up
/// to 7 frames = 6 periods, `:2296`). At 10 Hz that is 600 ms. So ANY topic
/// below roughly 24 Hz — `/tf` at 20 Hz, joint states, most cameras — blew the
/// cap, got falsely labelled a late start, and was anchored 350 ms behind its
/// siblings FOREVER, compounding with topic count (8 topics ≈ 2.45 s of skew).
/// The pre-cap code played that shape CORRECTLY.
///
/// The real discriminator is not the deficit's size but the RATE at which the
/// channel publishes while catching up. So the deficit is carried IN FULL
/// (correct pacing for every tap-grouped shape, no skew, nothing to
/// misclassify) and the catch-up rate is bounded instead: when a frame's
/// schedule has already passed, it still waits until this channel's OWN
/// previous publish plus `own recorded delta / CATCHUP_FACTOR`.
///
/// That uses ONLY the channel's own recorded cadence — no cross-channel
/// arithmetic — and turns the 500 kHz dump into a bounded fast-forward that is
/// accurate and useful for a handover. Nothing is classified as "late", so no
/// false claim about the recording is possible; what is reported is an
/// observation about THIS RUN: how many channels had to fast-forward.
///
/// # Scope of the bound
///
/// The bound is `own recorded delta / CATCHUP_FACTOR`, so it is only meaningful
/// where the recording HAS a delta. A channel whose consecutive stamps are
/// identical, or separated by less than `CATCHUP_FACTOR` nanoseconds, has no
/// spacing to divide and publishes as fast as the loop runs. That is FAITHFUL —
/// the recording itself says those frames were simultaneous — but it means the
/// "4x" figure is a bound on reproducing a RECORDED CADENCE, not a universal
/// publish-rate ceiling.
pub const CATCHUP_FACTOR: u64 = 4;

/// How far behind, IN UNITS OF ITS OWN INTER-FRAME DELTA, a channel may be and
/// still publish IMMEDIATELY. The [`CATCHUP_FACTOR`] rate bound engages only
/// STRICTLY BEYOND this — a channel exactly this far behind is still free (the
/// test is `>`, not `>=`).
///
/// # Why a free window is required, not a nicety
///
/// The player is SINGLE-THREADED, so spacing is an AGGREGATE throughput
/// ceiling, not a per-channel one. If every behind frame waits `P/4`, then a
/// round of `K` taps each contributing `F` frames costs `K·(F−1)·P/4` of wall
/// against `F·P` of recorded time — i.e. real-time playback requires
/// `Σ_k (1 − 1/F_k) ≤ 4`. Beyond roughly four fast topics EVERY channel falls
/// permanently behind, the deficit grows without bound, and the overrun branch
/// tells a healthy machine to "Lower --rate" — the exact false accusation this
/// design exists to prevent. MEASURED: K=3 → 1.08x (passes, which is why every
/// fixture missed it), K=8 → 1.945x, a 75-topic robot bag → 4.7x slow. That is
/// the DEFAULT shape for `ros2 attach`.
///
/// The deficit a channel carries from ordinary write batching is bounded by the
/// recorder's held budget (`max_borrowed - 1`, so at most 7 frames), so a window
/// comfortably above that lets every routine flush-window deficit publish
/// immediately — costing ~0 wall, which removes the aggregate ceiling entirely —
/// while a handover backlog of hundreds of deltas still rate-bounds at 4x.
///
/// Expressed in the channel's OWN deltas, so it holds at any rate and needs no
/// wall-clock constant and no cross-channel arithmetic.
pub const CATCHUP_FREE_DELTAS: u64 = 16;

/// ONE CHANNEL's playback timeline.
///
/// # Why per CHANNEL, and not per inferred "clock domain"
///
/// A Cerulion topic is SINGLE-WRITER by construction — the transport provisions
/// graph topics with `max_publishers = 1` and refuses a second — so one channel
/// is one producing process, hence exactly one clock domain. That is
/// RECORD-SIDE truth, available without inferring anything.
///
/// A first version inferred domains by comparing stamps ACROSS channels, treating a
/// frame whose stamp sat below its file-predecessor's on another channel as
/// evidence of a clock break. That premise was FALSE for every bag Cerulion's
/// own recorder writes: `cerulion_bagd` writes each flush TAP-GROUPED on both
/// paths (inline `for tap { for sample in &tap.held }`,
/// `cerulion_bagd/src/lib.rs:3211`; threaded `batch.per_tap`, `:1868`) and
/// `tap.held` accumulates until the 100 ms flush interval or the held budget.
/// So at any real rate each tap contributes several frames per batch, EVERY tap
/// boundary is a backwards step, and the inference read ordinary write batching
/// as a clock break — inventing "independent clock domains" on a single-clock
/// bag and stretching playback by roughly the tap multiplicity. MEASURED: a
/// 590 ms three-topic 100 Hz bag played in 1.255 s (2.13x slow), with every
/// boundary frame reported "behind schedule … Lower --rate" on a healthy
/// machine.
///
/// Comparing stamps across channels was itself the forbidden operation — wire
/// stamps are per-process clock domains and must never be cross-compared, the
/// very rule that version set out to enforce. So no channel's stamp is ever compared to
/// another's now: each channel anchors on its own first frame and advances only
/// by its OWN deltas.
///
/// # The cost
///
/// Inter-topic alignment is no longer derived from stamps; it comes from FILE
/// POSITION, because frames publish in recorded order and a channel anchors at
/// the wall instant its first frame is REACHED. That preserves cross-topic
/// ordering to the granularity of the recorder's flush window (~100 ms), not
/// better. For a bag driving a viewer that is the right trade: every topic's own
/// rate is exact, which is what a robot substitute has to get right.
///
/// A `multi_publisher_topics` channel genuinely carries several writers' clocks.
/// Its within-channel regressions re-anchor exactly like a producer restart —
/// bounded and documented, not silently wrong.
#[derive(Debug, Clone, Copy, Default)]
pub struct ChannelClock {
    /// Wall-time target (ns since playback start) for this channel's next frame.
    pub schedule_ns: u64,
    /// The previous frame's wire stamp ON THIS CHANNEL. Carried across a
    /// `--loop` wrap so the wrap reads as the regression it is, rather than as a
    /// fresh anchor that would publish the wrap's first two frames back to back.
    pub prev_stamp: Option<u64>,
    /// The wall instant this channel's timeline STARTS, in ns since playback
    /// start.
    ///
    /// This is the RUN ORIGIN for every channel: the deficit a channel starts
    /// with is carried in full, so channels never drift apart. The term is still
    /// carried per channel (rather than folded into a single origin) because
    /// `scheduled_span_ns` adds it, and a future design may anchor differently.
    pub anchor_ns: Option<u64>,
    /// The wall instant this channel last PUBLISHED, in ns since playback start.
    /// The catch-up rate bound spaces from here (see [`CATCHUP_FACTOR`]).
    pub last_publish_ns: Option<u64>,
    /// Cumulative scheduled advance — NEVER reset by an anchor or a re-anchor,
    /// so it stays a truthful measure of how long this channel's recording
    /// should have taken. `anchor + cumulative` is the wall time its last frame
    /// was due; the max over channels is the run's ideal duration, and comparing
    /// that to the actual wall answers "did this machine keep up?" — which
    /// `max_slip_ns` cannot, because a re-anchor resets the slip accumulator.
    pub cumulative_ns: u64,
}

/// Gate for `--rate`: finite and strictly positive.
pub fn validate_rate(rate: f64) -> CliResult<f64> {
    if !rate.is_finite() || rate <= 0.0 {
        return Err(CliError::Validation(format!(
            "--rate must be a finite number greater than 0 (got {rate}). \
             1.0 plays at the recorded pace, 2.0 twice as fast, 0.5 half speed."
        )));
    }
    Ok(rate)
}

/// Resolve the `--topics` filter against a scan.
///
/// An empty filter selects every channel. A named topic that is not in the bag
/// is a LOUD error naming it and listing what the bag does hold — never a
/// silently-empty playback.
pub fn select_channels<'a>(
    scan: &'a BagScan,
    filter: &[String],
) -> CliResult<Vec<&'a ChannelScan>> {
    if filter.is_empty() {
        return Ok(scan.channels.iter().collect());
    }
    let mut selected: Vec<&ChannelScan> = Vec::new();
    let mut missing = Vec::new();
    for want in filter {
        match scan.channels.iter().find(|c| &c.topic == want) {
            // A repeated `--topics X X` selects X once.
            Some(c) if !selected.iter().any(|s| s.topic == c.topic) => selected.push(c),
            Some(_) => {}
            None => missing.push(want.clone()),
        }
    }
    if !missing.is_empty() {
        let available: Vec<&str> = scan.channels.iter().map(|c| c.topic.as_str()).collect();
        return Err(CliError::Validation(format!(
            "--topics named {} topic(s) the bag does not carry: {}.\nThe bag holds: {}",
            missing.len(),
            missing.join(", "),
            if available.is_empty() {
                "(no user topics)".to_string()
            } else {
                available.join(", ")
            }
        )));
    }
    Ok(selected)
}

/// How a channel's schema resolves — what a viewer can actually do with its
/// frames, given this machine's corpus AND the provenance the bag itself carries.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SchemaReading {
    /// The wire hash resolves against THIS machine's own corpus. Viewers decode
    /// and render it with nothing further needed. `name` is the resolved
    /// qualified name, which may be RICHER than what the bag recorded (an
    /// earlier attach-mode recorder wrote "unknown", since the wire carries no
    /// name — while the HASH it recorded is exact).
    Local { name: String },
    /// The hash resolves ONLY because the BAG carries the type's text.
    /// This machine never compiled it, and the bag is what makes it decodable —
    /// which is exactly what `bag play` hands to the viewer.
    FromBag { name: String },
    /// The bag NAMES the type but carries no text for it, and this machine
    /// cannot resolve the hash either. Almost always a corpus SKEW: the bag was
    /// recorded against a different version of a built-in whose text this build
    /// therefore hashes differently. Worth its own arm because "we know what it
    /// is and still cannot read it" is a completely different remedy from "we
    /// have never heard of this hash".
    NamedButUndecodable { name: String },
    /// The hash resolves to nothing anywhere: not locally, and the bag says
    /// nothing about it. A viewer will render nothing.
    Unresolvable,
    /// The channel carries no Cerulion descriptor at all, so there is no hash to
    /// resolve. Such a channel is unplayable for the same reason.
    NoDescriptor,
}

impl SchemaReading {
    /// The resolved qualified name, when there is one.
    pub fn name(&self) -> Option<&str> {
        match self {
            SchemaReading::Local { name }
            | SchemaReading::FromBag { name }
            | SchemaReading::NamedButUndecodable { name } => Some(name),
            SchemaReading::Unresolvable | SchemaReading::NoDescriptor => None,
        }
    }

    /// Whether the type RESOLVES at all — locally, or from the definition the
    /// bag carries.
    ///
    /// Deliberately NOT named for what a VIEWER will render. A `FromBag` reading
    /// whose definition is workspace-YAML resolves here (this crate has a YAML
    /// schema parser) and still renders NOTHING in Studio (the daemon does not).
    /// Conflating those two is precisely the over-claim schema provenance exists to
    /// remove; renderability is `bag_doc_is_viewable`'s question.
    pub fn is_resolvable(&self) -> bool {
        matches!(
            self,
            SchemaReading::Local { .. } | SchemaReading::FromBag { .. }
        )
    }
}

/// Resolve a channel's schema against this machine's corpus and the bag's own
/// schema provenance.
///
/// # Why the HASH and not the recorded NAME
///
/// A viewer resolves a frame by its wire `schema_hash`; the bag's schema NAME
/// never reaches it. And an attach-mode recorder (which is every bag
/// `cerulion bag record` writes) records the literal
/// `cerulion_bagd::ATTACH_MODE_SCHEMA_NAME` ("unknown") because the wire carries
/// no name, while recording the hash exactly.
///
/// Keying the "a viewer will render nothing" banner off the NAME therefore fired
/// on EVERY bag this verb's own record→play workflow produced, including bags
/// full of ordinary `sensor_msgs` frames that render perfectly. Resolving the
/// hash answers the question actually being asked.
///
/// # Precedence
///
/// LOCAL first, then the bag. The local corpus is what a viewer already has
/// wired up, so when both can name a hash the local answer is the one that
/// describes what will actually happen. The bag's own text only ever ADDS types
/// this machine lacks — it never shadows one it has.
///
/// # Residual: the doc is joined by NAME, not re-hashed
///
/// [`SchemaReading::FromBag`] is reached when the catalog binds the hash to a
/// name AND carries a doc under that name. It does NOT recompute the doc's own
/// schema hash and check it against the channel's — so a catalog in which one
/// name is bound to a hash its shipped text does not define would read `FromBag`
/// where `NamedButUndecodable` is the truth.
///
/// Deliberately left as a documented residual rather than fixed, because
/// reaching it needs a catalog the recorder cannot produce: `closure_for_hashes`
/// pairs `hash → name → doc` from ONE machine's corpus, so within a name the
/// doc always defines its own hash; the pairing can only skew across a
/// hash-for-name binding inherited from a corpus the doc did not come from, i.e.
/// a hand-edited bag or a reader whose vendored corpus has drifted (the upstream-drift
/// class) — and in the drifted-reader case the LOCAL arm above answers first
/// whenever the reader knows the hash at all. Re-hashing every shipped doc at
/// scan time would also mean parsing the whole catalog on every `bag info`.
pub fn resolve_schema(
    walker: &cerulion_core::codegen::FrameWalker,
    catalog: Option<&cerulion_bag::BagSchemaCatalog>,
    channel: &ChannelScan,
) -> SchemaReading {
    let Some(hash) = channel.schema_hash else {
        return SchemaReading::NoDescriptor;
    };
    if let Some(name) = walker.schema_name_for_hash(hash) {
        return SchemaReading::Local {
            name: name.to_string(),
        };
    }
    let Some(catalog) = catalog else {
        return SchemaReading::Unresolvable;
    };
    let Some(name) = catalog.name_for_hash(hash) else {
        return SchemaReading::Unresolvable;
    };
    if catalog.docs.iter().any(|d| d.qualified == name) {
        SchemaReading::FromBag {
            name: name.to_string(),
        }
    } else {
        SchemaReading::NamedButUndecodable {
            name: name.to_string(),
        }
    }
}

/// Whether the bag's definition of `qualified` is in a form the VIEWER can
/// actually seed.
///
/// This models BOTH of `cerulion-vizd`'s structural refusals, because the banner
/// built on it promises that a `FromBag` topic "will decode and render here"
/// — and a promise the daemon refuses before the walker ever sees the doc is the
/// exact success-shaped failure schema provenance exists to remove.
///
/// 1. **ENCODING.** The daemon parses only ROS `.msg` definitions: the
///    workspace-YAML schema parser lives in this crate, which the daemon must
///    not depend on (it pulls the whole rerun SDK the other way). So a
///    YAML-encoded definition is genuinely decodable HERE — `topic echo` and
///    `bag info` read it — and still renders nothing in Studio.
/// 2. **NAME.** The daemon REFUSES a side-loaded definition whose
///    qualified name is a BUILT-IN (`Ctx::schemas` partitions on
///    `cerulion_viz::schema_registry::is_builtin_schema_name`): docs are folded
///    into its walker LAST and the layout resolver is last-insert-wins, so
///    accepting one would redefine that type DAEMON-WIDE, including for live
///    robot taps unrelated to the bag. A recording made inside a workspace that
///    SHADOWS a built-in (`schemas/sensor_msgs/msg/Image.msg` — a first-class,
///    documented configuration; see [`crate::schema_store::SchemaStore::builtin_shadows`])
///    ships exactly such a doc, so this is reachable with no corpus skew at all.
///
/// The predicate is [`crate::schema_store::builtin_has_qualified`], the
/// same-crate twin of the daemon's: `cerulion_cli_engine` deliberately does NOT
/// depend on `cerulion_viz` (that edge would pull rerun into every `cerulion`
/// build), so the two are separate functions over the ONE embedded registry,
/// `native_ros2_messages::BUILTIN_MSGS`. Each has its own oracle test against
/// that registry.
///
/// A name with no doc at all is not viewable either: only
/// [`SchemaReading::FromBag`] reaches this, and that arm exists precisely
/// because a doc was found, so the `false` is the correct answer for any caller
/// that asks about a name the bag only NAMES.
fn bag_doc_is_viewable(catalog: Option<&cerulion_bag::BagSchemaCatalog>, qualified: &str) -> bool {
    if crate::schema_store::builtin_has_qualified(qualified) {
        return false;
    }
    catalog.is_some_and(|c| {
        c.docs
            .iter()
            .any(|d| d.qualified == qualified && d.encoding == cerulion_core::SchemaEncoding::Msg)
    })
}

/// The name to DISPLAY for a channel: the resolved one when the bag's own is the
/// attach-mode placeholder, else the bag's.
pub fn display_schema_name(channel: &ChannelScan, reading: &SchemaReading) -> String {
    match reading.name() {
        Some(name) if channel.schema_name == ATTACH_SCHEMA_NAME => {
            format!("{name} (resolved by hash)")
        }
        _ => channel.schema_name.clone(),
    }
}

// ---------------------------------------------------------------------------
// Scan (zero-copy)
// ---------------------------------------------------------------------------

fn bag_err(path: &Path, what: &str, e: BagError) -> CliError {
    CliError::Validation(format!("{what} '{}': {e}", path.display()))
}

/// One zero-copy pass over the bag collecting everything both verbs need up
/// front: per-topic frame counts, the largest frame (which sizes each
/// publisher's SHM slot ceiling), the time span, and the non-monotonic-stamp
/// count.
///
/// Runs on [`BagReader::user_frames`] — the streaming, index-free,
/// reserved-channel-skipping walk that borrows each frame straight out of the
/// mmap. Nothing is copied and nothing is buffered: a 300 GB bag scans in
/// bounded memory. The per-frame timestamps come from each frame's own
/// [`WireHeader`], which is the SAME number the recorder wrote into the MCAP
/// `log_time` field (`cerulion_bagd` stamps `write_message(topic, seq, ts, ts,
/// ..)` from `WireHeader::timestamp_ns`), so reading it out of the payload
/// slice costs nothing and needs no message decode.
pub fn scan_bag(reader: &BagReader, path: &Path) -> CliResult<BagScan> {
    // Finalization is decided by whether the zero-copy walk can START, NOT by
    // `BagReader::completeness()`.
    //
    // `completeness()` streams every message through `mcap::MessageStream`,
    // which COPIES each payload to give its `Message` a `'static` lifetime —
    // one heap allocation per frame (MEASURED: exactly 1/frame). That is a fine
    // price for `bag info`, which exists to classify a damaged bag, and a bad
    // one for `bag play`, whose setup would then allocate once per recorded
    // frame before publishing anything. Opening the walk reads only the footer,
    // so this costs nothing and answers the only question playback has.
    let finalized = reader.user_frames().is_ok();
    let completeness_text = if finalized {
        "finalized".to_string()
    } else {
        "NOT FINALIZED — no readable summary footer".to_string()
    };

    // The Schema record's name + descriptor come from the channel table. On a
    // NON-finalized bag there is no summary and `channels()` falls back to
    // streaming the messages, which needs the end magic a killed recorder never
    // wrote — so it fails with a low-level "bad magic number". Do not let that
    // become the user-facing error: the right report is the COMPLETENESS
    // state, which is already known, and `bag play` refuses on `finalized`
    // below with the reason and the remedy.
    let channel_meta: BTreeMap<String, BagChannel> = match reader.channels() {
        Ok(cs) => cs.into_iter().map(|c| (c.topic.clone(), c)).collect(),
        Err(e) if !finalized => {
            tracing::debug!(
                bag = %path.display(),
                error = %e,
                "bag scan: no readable channel table (the bag is not finalized)"
            );
            BTreeMap::new()
        }
        Err(e) => return Err(bag_err(path, "cannot read the channel table of bag", e)),
    };

    // Channel-id -> slot in `order`, so the per-frame path does no string work
    // and no map-by-name lookup (a whole-bag pre-pass runs this once per frame).
    let mut channel_slot: BTreeMap<u16, usize> = BTreeMap::new();
    let mut order: Vec<ChannelScan> = Vec::new();
    let mut total_frames = 0u64;
    let mut backwards = 0u64;
    let mut headerless = 0u64;
    // Per-CHANNEL previous stamp. Never a running global one: comparing a
    // frame against a different channel's predecessor is the cross-clock
    // comparison this design forbids, and on a tap-grouped bag it reads write
    // batching as a clock break (see `ChannelClock`).
    let mut prev_of_channel: BTreeMap<u16, u64> = BTreeMap::new();
    // Per channel: (first stamp, last stamp). The bag's "span" is the widest of
    // these, NOT a subtraction across channels (see below).
    let mut per_channel_span: BTreeMap<u16, (u64, u64)> = BTreeMap::new();

    // A non-finalized bag has no footer, so the zero-copy walk cannot start.
    // Report the state (that is what `bag info` is for) instead of pretending.
    if finalized {
        let mut walk = reader
            .user_frames()
            .map_err(|e| bag_err(path, "cannot walk the frames of bag", e))?;
        while let Some((channel_id, span)) = walk
            .next_user_frame()
            .map_err(|e| bag_err(path, "cannot walk the frames of bag", e))?
        {
            // Resolve channel id -> scan slot ONCE per channel, never per
            // frame: `walk.topic()` returns a borrow, and turning it into an
            // owned key on every frame would allocate once per frame across a
            // whole-bag pre-pass.
            let slot = match channel_slot.get(&channel_id) {
                Some(s) => *s,
                None => {
                    let topic = walk.topic(channel_id).to_string();
                    let s = order.len();
                    order.push(new_channel_scan(&topic, channel_meta.get(&topic)));
                    channel_slot.insert(channel_id, s);
                    s
                }
            };
            let frame = reader.frame(&span);
            let stamp = WireHeader::read_from_buf(frame).map(|h| h.timestamp_ns);
            match stamp {
                Some(ts) => {
                    // Counted PER CHANNEL: a backwards step against this
                    // channel's own predecessor is a real producer restart. A
                    // step backwards against some OTHER channel is just write
                    // batching and means nothing.
                    if prev_of_channel.get(&channel_id).is_some_and(|p| ts < *p) {
                        backwards += 1;
                    }
                    prev_of_channel.insert(channel_id, ts);
                    per_channel_span
                        .entry(channel_id)
                        .and_modify(|(_, last)| *last = (*last).max(ts))
                        .or_insert((ts, ts));
                }
                None => headerless += 1,
            }
            total_frames += 1;
            let entry = &mut order[slot];
            entry.frames += 1;
            entry.max_frame_len = entry.max_frame_len.max(span.len);
            if let Some(ts) = stamp {
                entry.first_log_time.get_or_insert(ts);
                entry.last_log_time = Some(ts);
            }
        }
    }

    // A channel declared in the table but carrying zero frames is still worth
    // listing — an operator asking "why is /foo silent?" should see it exists.
    // Fold into a name-keyed map here (once, off the per-frame path) so the
    // rendered order is canonical rather than first-frame order.
    let mut per_topic: BTreeMap<String, ChannelScan> =
        order.into_iter().map(|c| (c.topic.clone(), c)).collect();
    for (topic, meta) in &channel_meta {
        if topic.starts_with(RESERVED_PREFIX) {
            continue;
        }
        per_topic
            .entry(topic.clone())
            .or_insert_with(|| new_channel_scan(topic, Some(meta)));
    }

    // L1: the span is the longest span WITHIN a channel, never `last - first`
    // across the whole bag. Stamps from different producers share no number
    // line, so subtracting one channel's last from another's first is the same
    // forbidden cross-clock arithmetic the pacer refuses — on a two-clock
    // fixture it reported 5.2 s for a 200 ms recording, and it silently reported
    // NO span at all whenever the last stamp happened to sit below the first.
    let span_ns = per_channel_span
        .values()
        .filter_map(|(first, last)| last.checked_sub(*first))
        .max()
        .filter(|s| *s > 0);

    Ok(BagScan {
        channels: per_topic.into_values().collect(),
        total_frames,
        backwards_log_times: backwards,
        headerless_frames: headerless,
        span_ns,
        completeness: completeness_text,
        finalized,
    })
}

fn new_channel_scan(topic: &str, meta: Option<&BagChannel>) -> ChannelScan {
    ChannelScan {
        topic: topic.to_string(),
        schema_name: meta
            .map(|m| m.schema_name.clone())
            .filter(|s| !s.is_empty())
            .unwrap_or_else(|| ATTACH_SCHEMA_NAME.to_string()),
        schema_hash: meta.and_then(|m| m.descriptor).map(|d| d.schema_hash),
        frames: 0,
        max_frame_len: 0,
        first_log_time: None,
        last_log_time: None,
    }
}

// ---------------------------------------------------------------------------
// `cerulion bag info`
// ---------------------------------------------------------------------------

/// Render a bag's contents without publishing anything — the same pre-scan
/// `bag play` runs, printed instead of played.
pub fn bag_info(path: &Path, schemas_dir: Option<&Path>) -> CliResult<String> {
    let reader = open_bag(path)?;
    let mut scan = scan_bag(&reader, path)?;
    // `bag info` — and ONLY `bag info` — pays for the precise damage
    // classification, because that is the question it exists to answer. The
    // full-stream `completeness()` walk costs one allocation per frame (see
    // `scan_bag`), which is why the playback path does not run it.
    if !scan.finalized {
        scan.completeness = match reader.completeness() {
            Ok(BagCompleteness::TruncatedAtChunkBoundary) => {
                "TRUNCATED at a chunk boundary — the recorder never finalized it".to_string()
            }
            Ok(BagCompleteness::TornTail(e)) => format!("TORN TAIL ({e})"),
            Ok(other) => format!("{other:?}"),
            // The stream itself is unreadable — say so rather than guessing.
            Err(e) => format!("UNREADABLE ({e})"),
        };
    }
    let walker = crate::topic_cmd::local_walker_from_workspace(schemas_dir);
    // The bag's own schema provenance, so a vendor type this machine
    // never compiled still reports its real name and its decodability.
    let catalog = reader.schema_catalog();
    // The coverage manifest. `bag info` is where it is READ — the
    // manifest is written into every finalized bag, and a durable artifact that
    // no shipped command ever surfaces is a claim nobody can check.
    let coverage = read_record_coverage(&reader, scan.finalized);
    // The STATE section. A second block beside the coverage
    // one rather than a field inside it, for the same reason the recorder logs a
    // second terminal line: the two answer different questions (what was on the
    // wire vs what state was captured) and escalate on different conditions.
    let state = read_state_coverage(&reader, scan.finalized);
    // The FLASHBACK section. A capture's manifest states what it
    // CLAIMS to cover and what it actually holds, and until this it was written
    // into the bag and surfaced by nothing — so the one number that says a
    // capture covers two seconds of the thirty it advertises was readable only by
    // opening the JSON by hand.
    let flashback = read_flashback_manifest(&reader, scan.finalized);
    // The record-time PRODUCER ATTRIBUTION. Its own block for the same
    // reason the state one has its own: a `__cerulion/frame_producers` channel
    // written into a bag that no shipped command surfaces is a claim nobody can
    // check (the `prefix_lost` / `mirrors_established` precedent — the PR that
    // adds a durable field renders it).
    let labels = read_producer_labels(&reader, scan.finalized);
    // The per-topic ABSORBANCE VERDICT. The
    // always-on Flashback plane trades absorbance for a bounded standing
    // footprint. We accept that trade on one condition: the
    // narrowed boundary is DECLARED and REPORTED per topic — which includes the
    // bag, since that is the only surface left once the run has ended.
    let absorbance = read_absorbance(&reader, scan.finalized);
    // The state-ring decision: what a MID-RUN ATTACH did about the run's per-rank
    // node-state rings. Its own block, and it earns one for a sharper reason
    // than the others: a bag whose attach DECLINED the state plane carries no
    // `state_coverage.json` at all, which is byte-for-byte what a recording that
    // was never configured for checkpoints looks like. Until this, the decision
    // was written into the bag and surfaced by nothing, so the two printed
    // IDENTICALLY and an operator holding an anchorless bag could not tell
    // "nobody asked for anchors" from "anchors are in the run's Flashback
    // captures instead".
    let state_rings = read_state_rings(&reader, scan.finalized);
    let ros2 = read_ros2_entries(&reader, scan.finalized);
    let mut out = render_scan(&scan, path, &walker, catalog.as_ref());
    out.push_str(&render_coverage_section(&coverage));
    out.push_str(&render_state_coverage_section(&state));
    out.push_str(&render_state_rings_section(&state_rings));
    out.push_str(&render_flashback_section(&flashback));
    out.push_str(&render_producer_labels_section(&labels));
    out.push_str(&render_absorbance_section(&absorbance));
    out.push_str(&render_ros2_entries_section(&ros2));
    Ok(out)
}

/// Decision (`ros2:` graph entries): a mixed graph's bag embeds its
/// ROS 2 entries VERBATIM in the `graph.yaml` attachment, and a durable
/// field no shipped command surfaces is a claim nobody can check — so
/// `bag info` renders them. Empty when the bag is not finalized (no
/// attachment index), carries no graph attachment, or the attachment does
/// not parse (real damage is `bag migrate`'s business, not this section's).
fn read_ros2_entries(reader: &BagReader, finalized: bool) -> Vec<(String, String)> {
    if !finalized {
        return Vec::new();
    }
    // The attachment name is `replay_cmd::GRAPH_ATTACHMENT`, spelled here
    // because that module is Unix-only and this section is not.
    let att = match reader.attachment("graph.yaml") {
        Ok(Some(att)) => att,
        _ => return Vec::new(),
    };
    let Ok(yaml) = std::str::from_utf8(&att.data) else {
        return Vec::new();
    };
    let Ok(config) = cerulion_core::graph::parse_graph_raw(yaml) else {
        return Vec::new();
    };
    config
        .ros2_nodes()
        .map(|n| {
            let argv = n
                .ros2
                .as_ref()
                .map(|r| r.argv(None).join(" "))
                .unwrap_or_default();
            (n.id.clone(), argv)
        })
        .collect()
}

/// Render the `ros2 entries` section — absent entirely for a pure-native bag.
fn render_ros2_entries_section(entries: &[(String, String)]) -> String {
    if entries.is_empty() {
        return String::new();
    }
    let mut out = String::from(
        "\nros2 entries (recorded as part of the run; a resim never respawns them — their \
         topics are recorded inputs):\n",
    );
    for (id, argv) in entries {
        out.push_str(&format!("  {id}  ros2 {argv}\n"));
    }
    out
}

// ---------------------------------------------------------------------------
// The producer labels, as `bag info` reports them
// ---------------------------------------------------------------------------

/// What `bag info` found when it went looking for a bag's producer attribution.
///
/// The same arms, and the same reasons, as [`FlashbackReading`]: "no attribution
/// was recorded", "the manifest could not be LOOKED AT", "it is there and
/// unreadable" and "here is what it says" are four DISTINCT situations, kept
/// apart in the type so a reader is never told the wrong one.
///
/// It reads `record_health.json` — where the per-topic label counters live
/// beside the frame counts they are bounded by — and, when a bag carries no such
/// manifest, falls back to COUNTING that bag's own records on the reserved
/// channel (decision R2; a FLASHBACK CAPTURE carries
/// `__cerulion/flashback.json` instead, and its labels would otherwise be
/// durable and reported by nothing).
///
/// The two sources are NOT interchangeable evidence, and this is worth knowing
/// before trusting a row: the manifest records what the RECORDER handed to its
/// writer (`TopicHealth::producer_labels` says in as many words that it "is not
/// a re-count taken from the finished file"), while the fallback records what the
/// FINISHED FILE contains. They agree on every healthy bag and can diverge on a
/// torn one. The fallback refuses to answer at all rather than report a floor as
/// a total (see [`IndexUnreadable`](Self::IndexUnreadable)); carrying the basis
/// ALONGSIDE the rows — the `TopicRateEstimate::is_floor` / `GatherCompleteness`
/// shape — is not implemented, and would let the manifest source state its
/// own limit too.
///
/// They do NOT map onto four printed remedies, and the renderer is where that is
/// decided: [`Absent`](Self::Absent) and
/// [`IndexUnreadable`](Self::IndexUnreadable) both render NOTHING — a bag with
/// no attribution story and a bag whose story could not be looked at are equally
/// silent, because this block is ADDITIVE reporting and neither state is
/// something `bag info` can ask the operator to act on. Only
/// [`Malformed`](Self::Malformed) and a NON-EMPTY [`Present`](Self::Present)
/// print. The distinctions still earn their place: they are what stop an
/// unfinalized bag's un-lookable index from being reported as "this recording
/// has no producer attribution", which is a claim, and they are asserted
/// separately by the unit arms.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProducerLabelReading {
    /// The bag is FINALIZED, carries no health manifest, AND has no
    /// `__cerulion/frame_producers` channel at all: an older bag. Claims
    /// NOTHING about labels.
    ///
    /// Decision R2 narrowed this arm. `BagWriter::create` now registers
    /// the reserved channel UNCONDITIONALLY, so no bag
    /// written by a current binary can land here: one with no health manifest
    /// but a registered channel is COUNTED instead, and an empty count is a
    /// stronger and truer statement than this arm's silence. What remains here
    /// is the genuinely older bag, which is exactly what the arm always meant.
    Absent,
    /// The report could not be ASSEMBLED — the attachment index or the channel
    /// table could not be READ AT ALL. NOT evidence that none exists.
    ///
    /// Renders NOTHING, and that is only defensible because a bag in this state
    /// is ALREADY reported damaged elsewhere in `bag info`: an unfinalized bag
    /// says so on its `state:` line. Contrast [`RecordsTorn`](Self::RecordsTorn),
    /// which is NOT covered by that line and therefore may not be silent.
    IndexUnreadable,
    /// The bag DOES carry a producer-label channel, and its record stream ended
    /// in a READ ERROR part-way through.
    ///
    /// Its own arm rather than [`IndexUnreadable`](Self::IndexUnreadable), and
    /// the difference is what `bag info` says elsewhere about the same bag. The
    /// two walks are different engines: `scan_bag` opens `user_frames()`, a
    /// framing-only walk that neither decompresses nor checks a CRC, so a bag
    /// with intact framing and one corrupt chunk BODY is reported `finalized`
    /// with no damage noted — while `recover_messages_on_topic`, which does
    /// decompress and does check, tears. Rendering nothing there would leave the
    /// operator unable to tell a torn capture from the single-writer case that
    /// legitimately has no block, on a bag every other line calls healthy.
    ///
    /// The count read before the tear is deliberately NOT carried: it is a
    /// FLOOR, this block renders exact numbers, and a floor presented as a total
    /// is the confident-wrong-number shape the whole vocabulary exists to
    /// prevent. What is reported is the CONDITION.
    RecordsTorn,
    /// An attribution source is PRESENT and this build cannot read it.
    ///
    /// Carries the ARTIFACT, because there are now two and they are different
    /// files with different remedies: `record_health.json` whose JSON did not
    /// decode, or the reserved `__cerulion/frame_producers` channel whose
    /// records this build cannot decode (a version or kind discriminant a newer
    /// binary wrote). Before the counting fallback there was only one source, so the
    /// renderer hardcoded its name — which, once the second source existed, made
    /// a skewed CAPTURE report that its `record_health.json` was "present but
    /// MALFORMED" when a capture never carries one at all.
    Malformed {
        /// Which artifact could not be read — rendered verbatim to the operator.
        artifact: String,
        /// The decoder's own message.
        detail: String,
    },
    /// Attribution was READ — `(topic, labels, catch_up)` for every topic that
    /// earned it, in topic order. EMPTY on the overwhelmingly common
    /// single-writer recording, which is what makes the block render nothing.
    ///
    /// From EITHER of two sources, and deliberately not split into two arms
    /// (decision R2): the recorder's `record_health.json`, or — when a
    /// bag carries no such manifest, which is what a FLASHBACK CAPTURE is — a
    /// count of the bag's OWN records on the reserved channel. They answer the
    /// same question about the same bag, the renderer treats them identically,
    /// and a caller that had to branch on WHICH would be branching on how the
    /// bag was produced rather than on what it says.
    Present {
        /// `(topic, labels, catch_up)` for every topic that earned attribution,
        /// in topic order.
        rows: Vec<(String, u64, bool)>,
        /// Records that were READ on the reserved channel and could not be
        /// attributed to a row — an unknown record version or kind, or a channel
        /// this bag does not declare.
        ///
        /// NON-ZERO makes every count in `rows` a FLOOR rather than a total, and
        /// the renderer says so. It is carried IN the reading rather than logged
        /// at `debug!`, because the alternative is rows
        /// printed as exact totals over a stream this build only partly
        /// understood. `decode` refuses on THREE independent conditions — length,
        /// version, and an unknown KIND discriminant — and the kind axis needs no
        /// version bump (the framing was designed so a reader need not know the
        /// kind to frame the stream), so "a skew makes every record fail" is true
        /// of the version gate and false of the format as a whole. A future third
        /// kind at version 1 yields exactly this partial state.
        unreportable: u64,
    },
}

/// Read the OPTIONAL `record_health.json` and project just its label counters,
/// falling back to counting the bag's OWN label records when a FINALIZED bag
/// carries no health manifest.
///
/// The `finalized` gate is load-bearing and easy to miss: an UNFINALIZED bag's
/// attachment index cannot be looked at, so its `Ok(None)` is not evidence that
/// no manifest exists and is not a licence to count — it becomes
/// [`IndexUnreadable`]. A reader debugging "why does my torn capture show no
/// producers block?" lands here, so the gate is stated rather than left to the
/// match arms.
///
/// [`IndexUnreadable`]: ProducerLabelReading::IndexUnreadable
///
/// WARN-NEVER-REFUSE, the `record_coverage.json` precedent exactly: this is
/// ADDITIVE reporting, so no failure here may turn a readable bag into an
/// unreadable one.
///
/// # Why the fallback exists (decision R2)
///
/// `record_health.json` is written by the CONTINUOUS recorder. A FLASHBACK
/// CAPTURE carries `__cerulion/flashback.json` instead — and since decision R2 it
/// also carries real producer records. Reading only the health manifest reported
/// a capture of a shared topic as having no attribution story at all, which is
/// the "a durable artifact no shipped command surfaces is a claim nobody can
/// check" shape this block was added to close in the first place.
///
/// An older bag has no such channel and no such records, so the fallback
/// finds nothing and renders nothing — exactly what [`ProducerLabelReading::Absent`]
/// rendered for it before.
fn read_producer_labels(reader: &BagReader, finalized: bool) -> ProducerLabelReading {
    match reader.attachment(cerulion_bagd::RECORD_HEALTH_ATTACHMENT) {
        Ok(Some(att)) => match serde_json::from_slice::<cerulion_bagd::RecordHealth>(&att.data) {
            Ok(h) => ProducerLabelReading::Present {
                rows: h
                    .topics
                    .into_iter()
                    .filter(|(_, t)| t.producer_labels > 0 || t.label_catch_up)
                    .map(|(topic, t)| (topic, t.producer_labels, t.label_catch_up))
                    .collect(),
                // The manifest states its own totals; nothing was skipped
                // reading it.
                unreportable: 0,
            },
            Err(e) => ProducerLabelReading::Malformed {
                artifact: cerulion_bagd::RECORD_HEALTH_ATTACHMENT.to_string(),
                detail: e.to_string(),
            },
        },
        Ok(None) if finalized => count_producer_labels_in_place(reader),
        Ok(None) => ProducerLabelReading::IndexUnreadable,
        Err(e) => {
            tracing::debug!(
                attachment = cerulion_bagd::RECORD_HEALTH_ATTACHMENT,
                error = %e,
                "bag info: could not read the attachment index while looking for the producer \
                 labels"
            );
            ProducerLabelReading::IndexUnreadable
        }
    }
}

/// Count a bag's producer records BY READING THEM — the fallback for a bag with
/// no `record_health.json` (today: a flashback capture).
///
/// Ground truth rather than a second manifest: the numbers come from the very
/// records a reader would resolve, so they cannot describe a bag other than this
/// one. Each record names the DATA channel it annotates, and that channel's
/// topic is resolved from the bag's own channel table — never assumed, since
/// reserved ids shift with the registration.
///
/// A record that will not decode is COUNTED AS NOTHING and logged at debug: the
/// alternative is reporting a whole capture's attribution as `Malformed` over one
/// bad record, which is a stronger claim than the evidence supports.
fn count_producer_labels_in_place(reader: &BagReader) -> ProducerLabelReading {
    let channels = match reader.channels() {
        Ok(c) => c,
        Err(e) => {
            tracing::debug!(error = %e, "bag info: could not read the channel table for labels");
            return ProducerLabelReading::IndexUnreadable;
        }
    };
    // Only a bag that REGISTERS the reserved channel can carry a record; an
    // older one does not, and short-circuiting here keeps its `bag info`
    // free of a message walk it can gain nothing from.
    if !channels
        .iter()
        .any(|c| c.topic == cerulion_bag::FRAME_PRODUCERS_TOPIC)
    {
        return ProducerLabelReading::Absent;
    }
    let by_id: std::collections::HashMap<u16, &str> =
        channels.iter().map(|c| (c.id, c.topic.as_str())).collect();
    // TOPIC-SCOPED, not `messages()`: that one converts EVERY message into an
    // owned `BagMessage`, which COPIES each payload — one heap allocation per
    // frame across a bag that can be hundreds of megabytes, for the sake of a
    // handful of 28-byte records. `recover_messages_on_topic` runs the topic
    // test against the BORROWED message, so a frame on any other channel is
    // never copied, and its peak is one decompressed chunk. (`scan_bag` avoids
    // `completeness()` for the same reason, and says so at its own call site.)
    let messages = match reader.recover_messages_on_topic(cerulion_bag::FRAME_PRODUCERS_TOPIC) {
        Ok(m) => m,
        Err(e) => {
            tracing::debug!(error = %e, "bag info: could not walk the messages for labels");
            return ProducerLabelReading::IndexUnreadable;
        }
    };
    // `(labels, catch_up)` per annotated topic, in TOPIC order — the same
    // ordering the health-manifest arm yields, so the renderer cannot tell the
    // two sources apart.
    let mut rows: std::collections::BTreeMap<String, (u64, bool)> =
        std::collections::BTreeMap::new();
    // Records that were READ but cannot be REPORTED — an unknown record version,
    // or a channel this bag does not declare. Counted rather than skipped: see
    // the check after the walk.
    let mut unreportable = 0u64;
    let mut first_unreportable: Option<String> = None;
    for item in messages {
        let Ok(msg) = item else {
            // A TORN tail. The count so far is a FLOOR, not a measurement — the
            // records past the tear are unread and unknowable — and this block
            // renders EXACT numbers, so the floor is not carried.
            //
            // Its OWN arm, not `IndexUnreadable`: that one renders nothing, which
            // is defensible only because a bag it describes is already reported
            // damaged on the `state:` line. This bag is not. `scan_bag`'s
            // `user_frames()` walk checks framing only — no decompression, no
            // CRC — so a bag with one corrupt chunk body reads `finalized` there
            // while tearing here, and a silent block would be indistinguishable
            // from the single-writer case that legitimately has none.
            tracing::debug!(
                rows_read = rows.len(),
                "bag info: the producer-label stream ended in a read error, so the counts so far \
                 are a floor rather than a total — reporting the condition instead"
            );
            return ProducerLabelReading::RecordsTorn;
        };
        let record = match cerulion_bag::ProducerRecord::decode(&msg.data) {
            Ok(r) => r,
            Err(e) => {
                // COUNTED, not merely skipped — see the `unreportable` check
                // below. `decode` refuses an unknown `PRODUCER_RECORD_VERSION`,
                // which is a DESIGNED-FOR event rather than a hypothetical.
                unreportable += 1;
                if first_unreportable.is_none() {
                    first_unreportable = Some(e.to_string());
                }
                continue;
            }
        };
        let Some(topic) = by_id.get(&record.channel_id) else {
            // A record naming a channel this bag has no table entry for. It
            // cannot be attributed to a topic, so it is not reported as one —
            // and it is counted with the undecodable ones, because it is the
            // same situation (a record was READ and cannot be reported) with the
            // same remedy, and two silences for one condition is how one of them
            // goes unnoticed.
            unreportable += 1;
            if first_unreportable.is_none() {
                first_unreportable = Some(format!(
                    "a record names channel {}, which this bag does not declare",
                    record.channel_id
                ));
            }
            continue;
        };
        let row = rows.entry((*topic).to_string()).or_insert((0, false));
        match record.attribution {
            cerulion_bag::ProducerAttribution::FrameLabel { .. } => row.0 += 1,
            cerulion_bag::ProducerAttribution::CatchUpPrefix { .. } => row.1 = true,
        }
    }
    // A bag whose records this build cannot READ must not be reported as a bag
    // that labels NOTHING.
    //
    // `ProducerRecord::decode` refuses an unknown `PRODUCER_RECORD_VERSION`, and
    // that is the event the version byte exists to anticipate: a capture written
    // by a NEWER binary, read by this one. Every record fails, `rows` stays
    // empty, and returning `Present(vec![])` there would state — in the
    // strongest arm the enum has — that a bag carrying complete per-frame
    // attribution carries none. That is a fabricated ABSENCE, and it is exactly
    // what `Malformed` already exists to say for the manifest: present, and this
    // build cannot read it. Same situation, same remedy, so the same arm rather
    // than a second, quieter one.
    //
    // Gated on `rows.is_empty()` so a bag that reports SOME topics is not
    // WITHHELD over one stray record — but a partial answer is only correct if it
    // says it is partial, which is why `unreportable` rides the `Present` arm
    // rather than being logged here and forgotten. That case is real:
    // `decode` refuses on length, version AND an unknown KIND discriminant, and
    // the kind axis needs no version bump, so a future third kind at version 1
    // yields surviving rows beside unreadable ones — printed, before this, as
    // exact totals.
    if unreportable > 0 && rows.is_empty() {
        return ProducerLabelReading::Malformed {
            artifact: cerulion_bag::FRAME_PRODUCERS_TOPIC.to_string(),
            detail: format!(
                "{unreportable} record(s) could not be read by this build ({}) — the bag may \
                 have been written by a newer version of Cerulion, in which case upgrading will \
                 read them",
                first_unreportable.unwrap_or_else(|| "no detail".to_string())
            ),
        };
    }
    ProducerLabelReading::Present {
        rows: rows
            .into_iter()
            .map(|(topic, (labels, catch_up))| (topic, labels, catch_up))
            .collect(),
        unreportable,
    }
}

/// Render a [`ProducerLabelReading`] as the PRODUCERS block of `bag info`. PURE.
///
/// ABSENT, INDEX-UNREADABLE and an EMPTY set all render NOTHING. That last one
/// is the important case rather than an edge: a single-writer recording earns no
/// labels at all, so a "producers: 0 labelled frames" line would appear on very
/// nearly every bag ever made and train an operator to skip the block on the few
/// where it says something. MALFORMED prints, because that bag DOES have an
/// attribution story somebody cannot read.
pub fn render_producer_labels_section(reading: &ProducerLabelReading) -> String {
    match reading {
        // Silent, and each for its own reason. ABSENT: the bag has no
        // producer-label channel, so there is nothing to report. INDEX-UNREADABLE:
        // the bag could not be looked at, and `bag info`'s `state:` line already
        // reports that damage — this block adding a second sentence about it
        // would be noise.
        ProducerLabelReading::Absent | ProducerLabelReading::IndexUnreadable => String::new(),
        // NOT silent, unlike its two neighbours above: nothing else in `bag info`
        // reports this damage. See `RecordsTorn`'s own doc.
        ProducerLabelReading::RecordsTorn => format!(
            "\nproducers: this bag carries per-frame publisher attribution on the `{}` channel, \
             but that channel's record stream ends in a READ ERROR, so how many frames each topic \
             labelled cannot be reported. The frames and their labels are unaffected — only this \
             summary is unavailable.\n",
            cerulion_bag::FRAME_PRODUCERS_TOPIC
        ),
        // The artifact is CARRIED, never hardcoded: this arm serves two sources
        // now (a `record_health.json` whose JSON did not decode, and a reserved
        // channel whose records this build cannot decode), and naming the wrong
        // one told an operator holding a capture that its `record_health.json`
        // was "present but MALFORMED" — a file a capture never carries.
        ProducerLabelReading::Malformed { artifact, detail } => format!(
            "\nproducers: the bag's `{}` is present and MALFORMED — this build cannot read it \
             ({}), so which publisher wrote which frame cannot be reported. The frames and their \
             labels are unaffected; only the report is unavailable for this bag.\n",
            crate::topic_cmd::sanitize_display(artifact),
            crate::topic_cmd::sanitize_display(detail)
        ),
        // An EMPTY set renders nothing — the overwhelmingly common single-writer
        // recording, which is what keeps this block off nearly every bag. (An
        // empty set beside unreadable records cannot occur: that is `Malformed`.)
        ProducerLabelReading::Present { rows, .. } if rows.is_empty() => String::new(),
        ProducerLabelReading::Present { rows, unreportable } => {
            // This may not assert the topics had two writers.
            // A DECLARED `multi_publisher` topic is labelled from its first frame
            // whether or not a second writer ever attaches, so on such a topic
            // the claim is affirmatively false. What is true of every row is the
            // ROUTE that put it here.
            let mut out = String::from(
                "\nproducers: these topics carry per-frame publisher attribution on the \
                 `__cerulion/frame_producers` channel — each is declared `multi_publisher`, or \
                 had a second writer observed on it while recording, so a wire `sequence` alone \
                 cannot say who wrote what.\n",
            );
            // With unreadable records on the channel, every count below is a
            // FLOOR — the skipped records may belong to any of these topics — so
            // the wording changes rather than the number being dressed up as a
            // total.
            let qualifier = if *unreportable > 0 { "at least " } else { "" };
            for (topic, labels, catch_up) in rows {
                // The catch-up marker is what says the plurality was OBSERVED
                // rather than declared: one record attributes the whole
                // single-writer run that preceded the second writer's arrival.
                let prefix = if *catch_up {
                    "  [+ a catch-up record attributing the run before the second writer appeared]"
                } else {
                    ""
                };
                out.push_str(&format!(
                    "  {:<40} {qualifier}{labels} labelled frame(s){prefix}\n",
                    crate::topic_cmd::sanitize_display(topic)
                ));
            }
            if *unreportable > 0 {
                out.push_str(&format!(
                    "  ({unreportable} record(s) on that channel could not be read by this build \
                     and are not counted above — if this bag was written by a newer version of \
                     Cerulion, upgrade to read them.)\n"
                ));
            }
            out
        }
    }
}

// ---------------------------------------------------------------------------
// The absorbance verdict, as `bag info` reports it
// ---------------------------------------------------------------------------

/// What `bag info` found when it went looking for a bag's per-topic ABSORBANCE
/// VERDICT.
///
/// The same four arms, and the same reasons, as [`ProducerLabelReading`] — it
/// reads the same `record_health.json`, where the verdict lives beside the loss
/// counts it prices.
#[derive(Debug, Clone, PartialEq)]
pub enum AbsorbanceReading {
    /// The bag is FINALIZED and carries no health manifest: an older bag,
    /// or one whose recorder never wrote one. Claims NOTHING.
    Absent,
    /// The attachment index could not be read. NOT evidence that none exists.
    IndexUnreadable,
    /// The manifest is present but its JSON did not decode.
    ///
    /// Carries the SCOPE as well as the error, because the two scopes live
    /// under DIFFERENT attachment names and a message naming the wrong one
    /// sends an operator to an attachment their bag provably does not
    /// contain — the exact name collision
    /// [`cerulion_bagd::CAPTURE_RECORDER_HEALTH_ATTACHMENT`] exists to prevent.
    Malformed(AbsorbanceScope, String),
    /// The manifest decoded. `rows` carries every topic that has a verdict, in
    /// topic order; `basis` is what the recording's loss numbers were able to
    /// SEE, or `None` on an older document that makes no claim about it.
    ///
    /// EMPTY `rows` on every bag that predates these verdicts, which is what makes the block
    /// render nothing there — an ABSENT verdict is UNKNOWN, and printing a
    /// heading over no rows would read as "this recording was checked".
    Present {
        /// `(topic, verdict)` for every topic carrying one.
        rows: Vec<(String, cerulion_bagd::TopicAbsorbance)>,
        /// What the loss numbers beside those verdicts can see.
        basis: Option<cerulion_bagd::LossCountingBasis>,
        /// WHOSE run the verdicts describe.
        scope: AbsorbanceScope,
        /// The drain-gap ladder THIS recording was ranked against, taken from
        /// its own `drain_gaps` histogram.
        ///
        /// One of `TopicAbsorbance::inconsistency`'s rules is not arithmetic on
        /// the row's own numbers — a tail-less `Short` row is the ladder-OVERFLOW
        /// arm, and whether it is producible depends on where that ladder's last
        /// edge sits. `DrainGapHistogram`'s wire shape carries `edges_us` beside
        /// the counts precisely so a reader does not have to assume this build's
        /// constant, so the check is asked against the bag's own vocabulary
        /// rather than against ours.
        ///
        /// `None` on a document whose histogram was vacant — it is omitted at
        /// its zero, so such a bag STATES no ladder. The check then falls back
        /// to this build's own vocabulary, which is the same posture every
        /// caller without a ladder takes and is the best available answer: a
        /// document that names no ladder is not thereby exempt from being
        /// checked against the only one its reader speaks.
        ///
        /// `Some(vec![])` is a THIRD state and a meaningful one: a
        /// self-describing histogram that recorded gaps while DECLARING no edges
        /// — every gap in the one unbounded bucket. `inconsistency_against`
        /// documents that arm ("ranked against a histogram with no edges, where
        /// the arm cannot be asked at all and is skipped rather than guessed"),
        /// and collapsing it to `None` made the check fall back to the READER's
        /// ladder and convict tail-less rows the writer's ladder never ranked.
        ladder_edges_us: Option<Vec<u64>>,
    },
}

/// Whose run an absorbance verdict describes.
///
/// A `--record` bag's health document is the recording's own; a FLASHBACK
/// CAPTURE carries the RECORDER's document under its own attachment name, and
/// every counter in it is run-cumulative — so a thirty-second capture out of a
/// six-hour run reports six hours of drain cadence. The renderer says which,
/// because "the drain stalls this recording measured" is affirmatively wrong on
/// the second and there is no way to tell from the numbers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AbsorbanceScope {
    /// The bag's own recording.
    Recording,
    /// The RECORDER that took this capture, over its whole run.
    CaptureRecorder,
}

impl AbsorbanceScope {
    /// The attachment a bag of this scope carries its health document under.
    ///
    /// ONE mapping, so a message that names an attachment and a reader that went
    /// looking for one cannot disagree about which name this bag holds.
    pub fn attachment(self) -> &'static str {
        match self {
            AbsorbanceScope::Recording => cerulion_bagd::RECORD_HEALTH_ATTACHMENT,
            AbsorbanceScope::CaptureRecorder => cerulion_bagd::CAPTURE_RECORDER_HEALTH_ATTACHMENT,
        }
    }
}

/// Read the OPTIONAL `record_health.json` and project just its absorbance
/// verdicts.
///
/// WARN-NEVER-REFUSE, the `read_producer_labels` precedent exactly: additive
/// reporting may never turn a readable bag into an unreadable one.
/// The ladder a decoded document STATES, told apart from the one
/// it never named. PURE.
///
/// THREE states, not two. `RecordHealth::drain_gaps` is
/// `#[serde(default, skip_serializing_if = "DrainGapHistogram::is_vacant")]`, so
/// an ABSENT field, a SKIPPED vacant one and an EXPLICITLY edge-less one all
/// decode with `edges_us` empty — but `counts` separates them: the serde default
/// carries BOTH vectors empty, while a self-describing histogram that genuinely
/// declares no edges still carries its one unbounded overflow bucket.
///
/// Collapsing the third onto `None` made `inconsistency_against` fall back to
/// the READER's own ladder and convict tail-less rows the writer's ladder never
/// ranked — the arm that function documents as "skipped rather than guessed" was
/// unreachable from the production reader.
///
/// An edge-less ladder is accepted only in the ONE SHAPE a
/// histogram can hold. `DrainGapHistogram`'s documented invariant is
/// `counts.len() == edges_us.len() + 1`, so a genuinely edge-less histogram
/// carries exactly its unbounded overflow bucket and nothing else. Deserializing
/// enforces no such thing — a hand-edited attachment can declare no edges beside
/// any number of counts — and a decode that reads `counts` for PRESENCE
/// alone lets `edges = []` with `counts = [1, 2]` buy the row a `Some(&[])`
/// ladder and SKIP the tail-less-`Short` arms outright.
fn decoded_ladder_edges(gaps: &cerulion_bagd::DrainGapHistogram) -> Option<Vec<u64>> {
    if !gaps.edges_us.is_empty() {
        return Some(gaps.edges_us.clone());
    }
    // Edge-less, so the only question left is which of the three states this is.
    //
    // * exactly ONE count is the producible edge-less histogram (its overflow
    //   bucket): the document STATES an empty ladder, and the arms that need
    //   edges are skipped rather than guessed.
    // * BOTH empty is the serde default — absent, or skipped while vacant.
    // * anything else is a shape no `DrainGapHistogram` can hold.
    //
    // The last two both fall back to the reader's own ladder, and for a
    // MALFORMED shape that is the safe direction rather than the convenient one:
    // `None` means the arms are still ASKED (against the only vocabulary this
    // reader speaks, which is what it does for a document that names no ladder
    // at all), while honouring the forged emptiness would skip them and hand a
    // forged row an unchecked verdict. A row wrongly convicted renders LOUDLY
    // with its numbers; a row wrongly waved through is counted as healthy.
    (gaps.counts.len() == 1).then(Vec::new)
}

fn read_absorbance(reader: &BagReader, finalized: bool) -> AbsorbanceReading {
    // A recording carries its own document; a CAPTURE carries the recorder's
    // under a different name (see `CAPTURE_RECORDER_HEALTH_ATTACHMENT` for why
    // the names differ). A bag is one or the other, so the first hit wins and
    // the scope travels with it — on the MALFORMED arm too, which is what lets
    // that message name the attachment this bag actually holds.
    for (name, scope) in [
        (
            cerulion_bagd::RECORD_HEALTH_ATTACHMENT,
            AbsorbanceScope::Recording,
        ),
        (
            cerulion_bagd::CAPTURE_RECORDER_HEALTH_ATTACHMENT,
            AbsorbanceScope::CaptureRecorder,
        ),
    ] {
        match reader.attachment(name) {
            Ok(Some(att)) => {
                return match serde_json::from_slice::<cerulion_bagd::RecordHealth>(&att.data) {
                    Ok(h) => AbsorbanceReading::Present {
                        rows: h
                            .topics
                            .iter()
                            .filter_map(|(topic, t)| t.absorbance.map(|a| (topic.clone(), a)))
                            .collect(),
                        basis: h.loss_counting_basis,
                        scope,
                        ladder_edges_us: decoded_ladder_edges(&h.drain_gaps),
                    },
                    Err(e) => AbsorbanceReading::Malformed(scope, e.to_string()),
                };
            }
            Ok(None) => {}
            Err(e) => {
                tracing::debug!(
                    attachment = name,
                    error = %e,
                    "bag info: could not read the attachment index while looking for the \
                     absorbance verdicts"
                );
                return AbsorbanceReading::IndexUnreadable;
            }
        }
    }
    if finalized {
        AbsorbanceReading::Absent
    } else {
        AbsorbanceReading::IndexUnreadable
    }
}

/// Render an [`AbsorbanceReading`] as the ABSORBANCE block of `bag info`. PURE.
///
/// # Why the header states TOTALS and the rows are only the actionable ones
///
/// A machine-wide recorder taps up to `DISCOVERY_MAX_TAPS` topics, so a row per
/// topic would be 256 lines of which almost all say "fine". The header carries
/// the whole set — nothing is hidden in aggregate — and the rows are the ones an
/// operator can act on: a tap that fell SHORT, one whose stall was worse than
/// the ladder can describe, one that makes no claim, and one whose queue is over
/// the budget it was priced against. A topic that absorbed everything this run
/// measured is counted, not printed.
///
/// # Why the caveats are printed rather than left to the docs
///
/// Both are ways the number can be read as better than it is, and a reader who
/// has the bag but not the source has no other route to them: the rate is a
/// WINDOW AVERAGE (a bursty topic's absorbance against its burst rate is
/// worse), and loss before a tap's first drained frame is invisible to
/// `frames_lost` — and, unless the recorder was armed before its producers, to
/// `prefix_lost` as well.
pub fn render_absorbance_section(reading: &AbsorbanceReading) -> String {
    use cerulion_bagd::AbsorbanceVerdict;
    match reading {
        AbsorbanceReading::Absent | AbsorbanceReading::IndexUnreadable => String::new(),
        AbsorbanceReading::Malformed(scope, e) => format!(
            "\nabsorbance: the bag's `{}` is present but MALFORMED ({}), so what its taps could \
             and could not absorb cannot be read. The frames are unaffected — only the report is \
             unavailable for this bag.\n",
            scope.attachment(),
            crate::topic_cmd::sanitize_display(e)
        ),
        AbsorbanceReading::Present { rows, .. } if rows.is_empty() => String::new(),
        AbsorbanceReading::Present {
            rows,
            basis,
            scope,
            ladder_edges_us,
        } => {
            // PRINCIPLE #2, at the READER. These rows are DECODED from a bag —
            // written by an unknown robot, possibly hand-edited, possibly by a
            // writer from a version nobody here has — so `verdict` and the
            // numbers beside it are two independent claims that a reader may not
            // assume agree. Where they disagree, neither surface may pick the
            // `verdict` side silently: an `Absorbs` row carrying a shortfall would be
            // COUNTED AS HEALTHY and `render_line` would drop its shortfall on the
            // floor, so a bag that says a tap fell short could be summarised as
            // one where nothing did.
            //
            // A contradicting row is therefore neither believed nor discarded.
            // It is EXCLUDED from every verdict count and reported in its own
            // bucket saying WHAT disagrees, which is the only correct answer: the
            // reader cannot know which half is true.
            // The REASON travels with the row, so the render arm cannot reach a
            // state where it has classified a row as contradictory and has no
            // reason to print for it (a degradation with no message).
            let mut sound: Vec<&(String, cerulion_bagd::TopicAbsorbance)> = Vec::new();
            let mut inconsistent: Vec<(&String, &cerulion_bagd::TopicAbsorbance, &'static str)> =
                Vec::new();
            for row in rows {
                match row.1.inconsistency_against(ladder_edges_us.as_deref()) {
                    Some(why) => inconsistent.push((&row.0, &row.1, why)),
                    None => sound.push(row),
                }
            }
            let count = |v: AbsorbanceVerdict| sound.iter().filter(|(_, a)| a.verdict == v).count();
            let (absorbs, short, beyond, no_claim) = (
                count(AbsorbanceVerdict::Absorbs),
                count(AbsorbanceVerdict::Short),
                count(AbsorbanceVerdict::Unrankable),
                count(AbsorbanceVerdict::NoClaim),
            );
            // An `Absorbs` on a FLOOR rate is an optimistic pass — the rate is a
            // lower bound, so the absorbance derived from it is a ceiling — and
            // folding it into one total lets a `multi_publisher` topic (`/tf` on
            // any real robot, whose per-publisher sequence counters cannot be
            // differenced) be counted beside an exactly-measured one.
            let optimistic = sound
                .iter()
                .filter(|(_, a)| a.verdict == AbsorbanceVerdict::Absorbs && a.rate_is_floor)
                .count();
            // A row that RECOVERED still happened. Nothing on the row is sticky
            // except this count, so without it a run that could not absorb for
            // most of its life finalizes reading entirely clean.
            // RECOVERED means the tap now ABSORBS, never merely "is not short".
            // `NoClaim` and `Unrankable` are absences of evidence — a tap short
            // all run whose topic went quiet past `ABSORBANCE_MAX_IDLE_GAP`
            // before the final evaluation finalizes `NoClaim` with a nonzero
            // `short_evaluations`, and counting that as recovered mints a
            // positive health claim out of the one state the module defines as
            // "nothing to say". The terminal roll-up already scopes its claim
            // this way ("a row whose verdict now reads `absorbs`").
            let recovered = sound
                .iter()
                .filter(|(_, a)| a.verdict == AbsorbanceVerdict::Absorbs && a.short_evaluations > 0)
                .count();
            // …and a row that fell short earlier and can no longer be ranked is
            // not silently folded away either: counted APART, because "was
            // short, now unknown" is worse news than either half alone.
            let short_then_unknown = sound
                .iter()
                .filter(|(_, a)| {
                    matches!(
                        a.verdict,
                        AbsorbanceVerdict::NoClaim | AbsorbanceVerdict::Unrankable
                    ) && a.short_evaluations > 0
                })
                .count();
            let whose = match scope {
                AbsorbanceScope::Recording => "this recording",
                AbsorbanceScope::CaptureRecorder => {
                    "the RECORDER that took this capture, over its whole run (not this \
                     capture's window — see drive_span_us)"
                }
            };
            // Named only when there ARE any: an every-bag ", and 0 disagree with
            // themselves" would train a reader to skip the clause that matters.
            let inconsistent_clause = if inconsistent.is_empty() {
                String::new()
            } else {
                format!(
                    ", and {} DISAGREE WITH THEMSELVES and are counted in none of the above",
                    inconsistent.len()
                )
            };
            let mut out = format!(
                "\nabsorbance: {} topic(s) carry a verdict — {absorbs} absorb the drain stalls \
                 {whose} measured ({optimistic} of them on an OPTIMISTIC floor rate), {short} \
                 fall SHORT of them, {beyond} met a stall the histogram cannot measure, \
                 {no_claim} make no claim, {recovered} fell short earlier in the run and now \
                 absorb, and {short_then_unknown} fell short earlier and can no longer be \
                 ranked{inconsistent_clause}. A verdict is the tap's queue depth divided by \
                 that topic's rate, \
                 against that run's own drain-gap tail; the rate is a WINDOW AVERAGE, so a \
                 bursty topic is worse than its figure, and the gaps are DRIVE-LOOP gaps, so a \
                 topic whose own drain was skipped (staging_full_passes) saw a longer stall than \
                 the one shown.\n",
                rows.len()
            );
            // FIRST, and unconditionally: a row nobody can believe is the one an
            // operator most needs to see, and it is never filtered by the
            // "actionable" test below (which reads the very fields in dispute).
            for (topic, a, why) in &inconsistent {
                // The DISPUTED NUMBERS, printed raw beside the label.
                //
                // `render_line` formats a row the way a SOUND row reads, which
                // means it prints only the fields that belong to the verdict it
                // is given — so on an `absorbs`-plus-shortfall row every number
                // it shows is self-consistent and looks healthy, and the value
                // that disagrees is invisible. Labelling the row without showing
                // that value would report a contradiction while withholding the
                // half of it a reader needs. This is the one row where the
                // uniform format is the wrong tool.
                out.push_str(&format!(
                    "  {:<40} INCONSISTENT: {why} — reported as {}, {}\n\
                     {:<42} disputed fields: absorbance_us={:?} measured_tail_us={:?} \
                     required_depth={:?} shortfall_at_least_us={:?}\n",
                    crate::topic_cmd::sanitize_display(topic),
                    a.verdict.as_str(),
                    a.render_line(),
                    "",
                    a.absorbance_us,
                    a.measured_tail_us,
                    a.required_depth,
                    a.shortfall_at_least_us,
                ));
            }
            for (topic, a) in sound {
                // Printed: every row an operator can act on — short, unrankable,
                // no-claim, over budget, an optimistic pass, and one that fell
                // short earlier. A topic that absorbed everything on an exact
                // rate throughout is counted, not printed.
                if a.verdict == AbsorbanceVerdict::Absorbs
                    && a.over_budget.is_none()
                    && !a.rate_is_floor
                    && a.short_evaluations == 0
                {
                    continue;
                }
                let history = if a.short_evaluations > 0 {
                    format!("  [fell short {} time(s) this run]", a.short_evaluations)
                } else {
                    String::new()
                };
                out.push_str(&format!(
                    "  {:<40} {}{history}\n",
                    crate::topic_cmd::sanitize_display(topic),
                    a.render_line()
                ));
            }
            // The counting caveat, stated where it applies. `PrefixProven` is the
            // ONE case where a head loss IS separately accounted, so it
            // is the one case this does not print.
            //
            // The token is the recorder's FLOOR over its taps, so
            // it is `PrefixProven` only when EVERY row's head was covered — and
            // the text names BOTH ways that fails, because an armed run
            // reaches here too, on a tap live discovery attached to a producer
            // that was already running.
            if *basis != Some(cerulion_bagd::LossCountingBasis::PrefixProven) {
                out.push_str(
                    "  loss BEFORE a tap's first drained frame is invisible: a contiguous \
                     prefix dropped before the recorder's first drain is taken AS the \
                     baseline, and this recording cannot claim head coverage for every tap \
                     (either it was not armed before its producers, or some taps were \
                     attached by live discovery to producers already running), so \
                     `prefix_lost` cannot see it either. A zero on either \
                     means nothing was COUNTED, not that nothing was lost. The per-topic \
                     `loss_counting_basis` in record_health.json says WHICH rows.\n",
                );
            }
            out
        }
    }
}

// ---------------------------------------------------------------------------
// The state-ring decision, as `bag info` reports it
// ---------------------------------------------------------------------------

/// The key a mid-run bag's [`RUN_JSON_ATTACHMENT`] carries its state-ring
/// decision under.
///
/// Spelled ONCE, and read back through this constant by `read_state_rings`, so
/// the writer and the reader cannot drift. That matters more than the usual
/// tidiness argument: this key is the ONLY durable record that an attach
/// DECLINED the state plane, and a declined bag carries no `state_coverage.json`
/// either — so a reader looking for the wrong key sees exactly what a recording
/// that never checkpointed looks like.
pub const STATE_RINGS_KEY: &str = "state_rings";

/// What `bag info` found when it went looking for a bag's state-ring decision.
///
/// The same four-arm shape as [`FlashbackReading`], for the same reasons, plus a
/// fifth distinction INSIDE the present arm that this block exists for.
///
/// # Why `Present { verdict: None }` is not `NotAnAttach`
///
/// [`NotAnAttach`](Self::NotAnAttach) is a fact about the BAG: it carries no
/// `run.json`, so it is an ordinary recording or a capture and there is no
/// mid-run attach decision to report. That renders NOTHING, on the flashback
/// precedent — most bags are not attaches.
///
/// `Present { verdict: None }` is a fact about a bag that IS an attach and did
/// not say. Its bag is byte-indistinguishable from one whose attach declined the
/// plane: both carry a `run.json`, neither carries a `state_coverage.json`.
/// Rendering that as silence would state the recording simply had no anchors,
/// which is the positive-claim-from-an-absence class this crate splits
/// everywhere else. It gets an explicit UNKNOWN row instead.
///
/// # FOUR runs reach it, and the row says so rather than picking one
///
/// This row is reached from four different situations, not one:
///
/// 1. a bag recorded before the key existed;
/// 2. a VIRTUAL-CLOCK run — `graph_run` writes the run descriptor BEFORE the
///    deployment dispatch, while the monolith declare site sits in the
///    `Real | External` arm, so a virtual run publishes an attachable
///    `run.json` and never takes the decision at all;
/// 3. an attach that read the manifest during BRING-UP, in the window between
///    the descriptor write and the declaration;
/// 4. a run whose best-effort declaration FAILED — it warned at the time, and
///    left nothing durable behind.
///
/// # Why there is no `pending` fourth state (the alternative, rejected)
///
/// Writing `"pending"` into the descriptor would make an absent key mean exactly
/// (1) again, and it is the obvious repair for (3). It is rejected because it
/// MOVES the falsehood rather than removing it: the descriptor is written before
/// the dispatch, so a virtual-clock run would carry `"pending"` for its whole
/// life while never deciding — and "had not yet decided" implies it will.
/// Making that true needs the writer to predict which control-flow path the
/// run will take from a spec field, which is the exact prediction
/// `run_dir::declare_state_ring_consumer`'s own docs reject, for the same
/// reason.
///
/// It also buys no BEHAVIOUR. An attach cannot decline on "not decided yet" —
/// no recorder is known to exist — so it must sweep either way, which is what
/// an absent key already makes it do. (3) therefore stays an open, documented
/// residual: a bring-up attach sweeps, and says it could not tell.
#[derive(Debug, Clone, PartialEq)]
pub enum StateRingsReading {
    /// The bag is FINALIZED and carries no `run.json` — not a mid-run attach.
    NotAnAttach,
    /// The attachment index could not be read. NOT evidence that none exists.
    IndexUnreadable,
    /// `run.json` is present but its JSON did not decode.
    Malformed(String),
    /// The bag is a mid-run attach. `verdict` is its own words, or `None` when
    /// the bag predates the key.
    Present {
        /// The attach's `state_rings` sentence, verbatim.
        verdict: Option<String>,
    },
    /// The key is PRESENT but holds a shape this build cannot read.
    ///
    /// The fifth arm, and it exists for the reason its sibling reader
    /// `run_dir::run_manifest_state_ring_consumer` grew the same one earlier in
    /// this arc: `as_str()` answers `None` for an object, an array, a number and
    /// a null alike, so folding it onto [`Present`](Self::Present) with no
    /// verdict renders "this bag carries no state-ring decision" about a bag
    /// that carries one. That row then enumerates two causes, neither of which
    /// is the real one, and tells the operator to discard the very inference
    /// they should be drawing.
    ///
    /// No shipped writer produces a non-string today — `StateRingVerdict::render`
    /// returns a `String` on every arm — so this is reachable only from a NEWER
    /// build or a hand-edited bag. An object form (`{"state": …, "reason": …}`)
    /// is the single most likely way the key gets extended, which is exactly
    /// when a reader must say "I cannot read this" rather than "there is
    /// nothing here".
    UnreadableValue {
        /// The value as JSON, bounded at render time.
        raw: String,
    },
}

/// Read the state-ring decision out of a bag's `run.json`.
///
/// WARN-NEVER-REFUSE, the `record_coverage.json` precedent exactly: this is
/// ADDITIVE reporting, so no failure here may turn a readable bag into an
/// unreadable one.
///
/// The value is taken VERBATIM rather than re-derived. The decision was made
/// once, at attach time, from the RUN's manifest — a document this bag does not
/// carry and which no longer exists by the time anyone runs `bag info` (a run
/// directory is removed on a clean shutdown). Re-deriving anything here would be
/// a second opinion formed from strictly less evidence, and the two surfaces
/// could then disagree about one decision.
fn read_state_rings(reader: &BagReader, finalized: bool) -> StateRingsReading {
    match reader.attachment(RUN_JSON_ATTACHMENT) {
        Ok(Some(att)) => classify_state_rings_bytes(&att.data),
        Ok(None) if finalized => StateRingsReading::NotAnAttach,
        Ok(None) => StateRingsReading::IndexUnreadable,
        Err(e) => {
            tracing::debug!(
                attachment = RUN_JSON_ATTACHMENT,
                error = %e,
                "bag info: could not read the attachment index while looking for the run manifest"
            );
            StateRingsReading::IndexUnreadable
        }
    }
}

/// PURE: re-point a verdict's document-relative references at the bag.
///
/// Two of the `STATE_RINGS_*` sentences end by naming a key "in this document",
/// which is correct INSIDE the bag's `run.json` — where those keys really live —
/// and wrong everywhere the sentence is re-used, because neither `bag info`'s
/// report nor a terminal line contains them.
///
/// The `bag record --run` warn path solved this inline first. It is a function
/// now because there are two surfaces and a third would have to remember.
fn repoint_verdict(text: &str) -> String {
    text.replace(
        "in this document",
        &format!("in this bag's `{RUN_JSON_ATTACHMENT}`"),
    )
}

/// PURE: bound a foreign string echoed into the report.
///
/// The verdict is written by another process into a file this reader does not
/// control and is printed verbatim, so a corrupt or hand-edited bag could
/// otherwise put an arbitrary-length line — or a terminal escape — in front of
/// an operator. This bounds the length AND neuters the controls, so its output
/// needs no further treatment.
///
/// It is NOT [`quote_unknown_state`]: that one wraps its input in backticks
/// because it quotes a TOKEN inside a sentence, while this echoes a whole
/// sentence and backticks would be wrong. What is shared is the ceiling and the
/// char-boundary cut, so a multi-byte value cannot panic.
fn bound_verdict(raw: &str) -> String {
    // The ONE place this helper's output is neutered, and the reason it is here
    // rather than at the call sites: wrapping at both call sites would make
    // this function's own contract "bounded, but you must still sanitize me" —
    // a contract a third caller would have to know. It returns a string that
    // is safe to print, and the call sites just print it.
    //
    // The warn path does not reach here. It renders through
    // `quote_unknown_state`, which sanitizes for itself.
    let owned = crate::topic_cmd::sanitize_display(raw.trim());
    let raw = owned.as_str();
    if raw.chars().count() > STATE_RINGS_MAX_ECHO {
        let cut: String = raw.chars().take(STATE_RINGS_MAX_ECHO).collect();
        format!("{cut}… (truncated)")
    } else {
        raw.to_string()
    }
}

/// How much of a foreign `state_rings` value [`bound_verdict`] will echo.
///
/// Generous rather than tight: every sentence this crate's own vocabulary
/// produces fits well inside it (the longest, `STATE_RINGS_REFUSED_STANDING`, is
/// ~430 characters), so the ceiling only ever bites on a value this build did
/// not write.
const STATE_RINGS_MAX_ECHO: usize = 600;

/// PURE: classify a bag's `run.json` bytes into a [`StateRingsReading`].
///
/// Split out of the reader so the CLASSIFICATION can be driven without building
/// a bag — which is not a convenience. The four interesting states are all
/// shapes of the DOCUMENT, and every one of them was previously reachable only
/// through a real `BagReader`, so the arms that matter were pinned by hand-built
/// readings that could not see a mis-classification at all. (Measured: folding a
/// present non-string back onto the absence row left all six unit arms green.)
///
/// # The two filters, and why neither is optional
///
/// The document must be an OBJECT. The three sibling readers deserialize into a
/// typed struct and get that for free; reading a `serde_json::Value` here
/// removes the filter, and `Value::get` answers `None` for a bare `7` or `[]` —
/// which would fold a manifest nobody can use onto the "carries no decision"
/// row.
///
/// The value must be a STRING, and a present non-string is
/// [`UnreadableValue`](StateRingsReading::UnreadableValue) rather than an
/// absence. `as_str()` answers `None` for an object, an array, a number and a
/// null alike, and an object form is the single most likely way a newer
/// `cerulion` extends this key.
fn classify_state_rings_bytes(bytes: &[u8]) -> StateRingsReading {
    let Some(doc) = serde_json::from_slice::<serde_json::Value>(bytes)
        .ok()
        .filter(serde_json::Value::is_object)
    else {
        return StateRingsReading::Malformed(
            "the run manifest is not a readable JSON object".to_string(),
        );
    };
    match doc.get(STATE_RINGS_KEY) {
        // Absent — the bag predates the key.
        None => StateRingsReading::Present { verdict: None },
        Some(v) => match v.as_str() {
            Some(text) => StateRingsReading::Present {
                verdict: Some(text.to_string()),
            },
            None => StateRingsReading::UnreadableValue { raw: v.to_string() },
        },
    }
}

/// Render a [`StateRingsReading`] as the STATE RINGS block of `bag info`. PURE.
///
/// NOT-AN-ATTACH and INDEX-UNREADABLE render NOTHING; MALFORMED prints, because
/// that bag DOES have a decision somebody cannot read. BOTH present arms print —
/// including the UNKNOWN one, which is the whole point of the block.
pub fn render_state_rings_section(reading: &StateRingsReading) -> String {
    match reading {
        StateRingsReading::NotAnAttach | StateRingsReading::IndexUnreadable => String::new(),
        StateRingsReading::Malformed(e) => format!(
            "\nstate rings: the bag's `{RUN_JSON_ATTACHMENT}` is present but MALFORMED ({}), so \
             what this attach did about the run's node-state rings cannot be read. The frames \
             themselves are unaffected — only the report is unavailable for this bag.\n",
            crate::topic_cmd::sanitize_display(e)
        ),
        // The attach's own words. Sanitized because they reach here from a file
        // another process wrote — and BOUNDED for the same reason: a corrupt or
        // hand-edited bag would otherwise print an arbitrary-length string into
        // the terminal. `sanitize_display` neuters control characters and does
        // not truncate.
        //
        // An EMPTY value gets its own sentence rather than rendering a bare
        // `state rings: ` label, which reads as a renderer bug rather than as a
        // bag whose writer wrote nothing into a key it did create.
        StateRingsReading::Present {
            verdict: Some(text),
        } if text.trim().is_empty() => format!(
            "\nstate rings: this bag's `{RUN_JSON_ATTACHMENT}` carries an EMPTY state-ring \
             decision, so what its attach did about the run's node-state rings cannot be read \
             from it.\n"
        ),
        StateRingsReading::Present {
            verdict: Some(text),
        } => format!("\nstate rings: {}\n", repoint_verdict(&bound_verdict(text))),
        // Present and unreadable — NOT an absence. See `UnreadableValue`.
        StateRingsReading::UnreadableValue { raw } => format!(
            "\nstate rings: this bag's `{RUN_JSON_ATTACHMENT}` carries a state-ring decision in \
             a shape this build cannot read ({}), so what its attach did about the run's \
             node-state rings is unknown. A newer `cerulion` most likely wrote it, or it was \
             hand-edited. This is NOT the same as a bag that carries no decision.\n",
            bound_verdict(raw)
        ),
        // The bag IS an attach and said nothing. An explicit row, never silence:
        // a declined attach carries no `state_coverage.json` either, so silence
        // here is exactly what a recording that never checkpointed looks like.
        StateRingsReading::Present { verdict: None } => format!(
            "\nstate rings: unknown — this bag is a mid-run attach whose `{RUN_JSON_ATTACHMENT}` \
             carries no state-ring decision. FOUR runs reach this and it cannot tell them \
             apart: one recorded before this key existed; a virtual-clock run, which never \
             takes the decision at all; an attach that read the manifest during bring-up, \
             before the run had decided; and a run whose declaration failed, which said so at \
             the time. Whether its recorder swept the run's per-rank node-state rings, or \
             declined them to a recorder already draining them, was never written down — so an \
             absent `state_coverage.json` beside this row is no evidence either way.\n"
        ),
    }
}

// ---------------------------------------------------------------------------
// The capture manifest, as `bag info` reports it
// ---------------------------------------------------------------------------

/// What `bag info` found when it went looking for a bag's
/// [`cerulion_bagd::FLASHBACK_ATTACHMENT`].
///
/// The same four arms, and the same reasons, as [`StateCoverageReading`]:
/// "this is not a capture", "the manifest could not be LOOKED AT", "it is there
/// and unreadable" and "here is what it says" are four situations with four
/// remedies.
///
/// ABSENT claims nothing about the RECORDER, on the state-manifest precedent: an
/// ordinary recording carries no capture manifest, and neither does a capture
/// written before these fields existed.
#[derive(Debug, Clone, PartialEq)]
pub enum FlashbackReading {
    /// The bag is FINALIZED and carries no capture manifest — an ordinary
    /// recording, not a flashback.
    Absent,
    /// The attachment index could not be read, so the manifest could not be
    /// looked at. NOT evidence that none exists.
    IndexUnreadable,
    /// The manifest is present but its JSON did not decode.
    Malformed(String),
    /// The manifest decoded.
    Present(Box<cerulion_bagd::FlashbackManifest>),
}

/// Read the OPTIONAL capture manifest out of a bag.
///
/// WARN-NEVER-REFUSE, the `record_coverage.json` precedent exactly: this is
/// ADDITIVE reporting, so no failure here may turn a readable bag into an
/// unreadable one.
fn read_flashback_manifest(reader: &BagReader, finalized: bool) -> FlashbackReading {
    match reader.attachment(cerulion_bagd::FLASHBACK_ATTACHMENT) {
        Ok(Some(att)) => {
            match serde_json::from_slice::<cerulion_bagd::FlashbackManifest>(&att.data) {
                Ok(m) => FlashbackReading::Present(Box::new(m)),
                Err(e) => FlashbackReading::Malformed(e.to_string()),
            }
        }
        Ok(None) if finalized => FlashbackReading::Absent,
        Ok(None) => FlashbackReading::IndexUnreadable,
        Err(e) => {
            tracing::debug!(
                attachment = cerulion_bagd::FLASHBACK_ATTACHMENT,
                error = %e,
                "bag info: could not read the attachment index while looking for the capture \
                 manifest"
            );
            FlashbackReading::IndexUnreadable
        }
    }
}

/// Render a [`FlashbackReading`] as the FLASHBACK block of `bag info`. PURE.
///
/// ABSENT and INDEX-UNREADABLE render NOTHING — most bags are not captures, and
/// a paragraph saying so on every recording is noise that trains an operator to
/// skip the block. MALFORMED prints, because that bag DOES have a capture story
/// somebody cannot read.
pub fn render_flashback_section(reading: &FlashbackReading) -> String {
    match reading {
        FlashbackReading::Absent | FlashbackReading::IndexUnreadable => String::new(),
        FlashbackReading::Malformed(e) => format!(
            "\nflashback: the bag's `{}` is present but MALFORMED ({}), so what this capture \
             claims to cover cannot be read. The frames themselves are unaffected — only the \
             report is unavailable for this bag.\n",
            cerulion_bagd::FLASHBACK_ATTACHMENT,
            crate::topic_cmd::sanitize_display(e)
        ),
        FlashbackReading::Present(m) => render_flashback_manifest(m),
    }
}

/// Render a decoded capture manifest. PURE.
///
/// # Why the shortfall gets its own sentence
///
/// The CLAIM and the ACHIEVEMENT are two numbers an operator has to subtract,
/// and a report that offers only the claim hides the shortfall. So the
/// shortfall is stated in words when there is one, and the block stays a single
/// quiet line when there is not — a capture that covered everything it promised
/// should not read like a report about coverage.
fn render_flashback_manifest(m: &cerulion_bagd::FlashbackManifest) -> String {
    let mut out = String::from("\nflashback capture");
    if let Some(seq) = m.seq {
        out.push_str(&format!(" #{seq}"));
    }
    if m.pinned == Some(true) {
        out.push_str(" (PINNED — excluded from retention eviction)");
    }
    out.push_str(":\n");

    // The two spans, and the difference. `None` on either side means the bag
    // predates the field, so nothing is asserted about it rather than a zero
    // being printed.
    match (m.span_ms, m.achieved_span_ms) {
        (Some(claimed), Some(achieved)) => {
            let shortfall = m
                .coverage_shortfall_ms
                .unwrap_or_else(|| claimed.saturating_sub(achieved));
            out.push_str(&format!(
                "  covers {:.1}s of the {:.1}s it claims\n",
                achieved as f64 / 1000.0,
                claimed as f64 / 1000.0
            ));
            if shortfall > 0 {
                // The MEASUREMENT, stated without a cause. A shortfall does NOT
                // imply eviction: a capture triggered inside its first span (its
                // floor saturates, so it claims less but still reaches less than
                // it claims), a robot whose topics are sparse, or an interval
                // nobody published in each produce one with NOTHING evicted —
                // and the byte-ceiling remediation is then wrong twice over,
                // sending an operator to raise a cap that never bound.
                out.push_str(&format!("  SHORT BY {:.1}s\n", shortfall as f64 / 1000.0));
                // …and the CAUSE, only on the manifest's own evidence. `None`
                // here is an older bag, which makes no claim either way,
                // so it gets no causal sentence rather than a guessed one.
                if m.truncated_frames.is_some_and(|t| t > 0) {
                    out.push_str(
                        "  the window's byte ceiling evicted frames during this capture — raise \
                         CERULION_FLASHBACK_WINDOW_MAX_MB, or reduce or downsample the heaviest \
                         topics",
                    );
                    if let Some(cap) = m.window_cap_bytes {
                        out.push_str(&format!(
                            " (ceiling {:.0} MiB)",
                            cap as f64 / (1024.0 * 1024.0)
                        ));
                    }
                    out.push('\n');
                }
            }
        }
        (Some(claimed), None) => out.push_str(&format!(
            "  claims {:.1}s; this bag predates achieved-span reporting, so what it actually \
             carries is not stated\n",
            claimed as f64 / 1000.0
        )),
        _ => {}
    }
    if let Some(span) = m.window_span_ms {
        out.push_str(&format!(
            "  window span: {:.1}s (the recorder's standing promise)\n",
            span as f64 / 1000.0
        ));
    }
    if let (Some(frames), Some(truncated)) = (m.frames, m.truncated_frames) {
        out.push_str(&format!(
            "  frames: {frames} carried, {truncated} taken by the byte ceiling during the capture\n"
        ));
    }
    // A range implies coverage, and for a topic slower than it
    // there is none.
    if let Some(missing) = m.topics_with_no_frames {
        if missing > 0 {
            out.push_str(&format!(
                "  {missing} tapped topic(s) contribute NO frame to that range — a range is not \
                 coverage for a topic slower than it\n"
            ));
        }
    }
    out
}

// ---------------------------------------------------------------------------
// The node-state checkpoint manifest, as `bag info` reports it
// ---------------------------------------------------------------------------

/// What `bag info` found when it went looking for the bag's
/// [`cerulion_bagd::STATE_COVERAGE_ATTACHMENT`].
///
/// Four arms, kept APART for the same reason [`CoverageReading`]'s are: "this
/// recording carried no checkpoints", "the manifest could not be LOOKED AT",
/// "it is there and unreadable", and "here is what it says" are four situations
/// with four remedies, and collapsing any two states something false.
///
/// The ABSENT arm differs from its coverage sibling in one way: it cannot
/// distinguish a recorder that predates checkpoints from one that was simply not
/// asked to checkpoint, because neither writes the attachment. It therefore
/// claims neither.
///
/// That reticence covers a THIRD cause a reader would otherwise be told nothing
/// about: an EARLIER file of a ROTATED recording. `WriterCore::finalize_bag`
/// writes this manifest — and `record_coverage.json` and `record_health.json`
/// with it — into the file that is open when the recording ends, while
/// `maybe_rotate` closes each outgoing file through the raw
/// `BagWriter::finalize`, carrying forward only the ring manifests, the caller's
/// `--attach` files and the schema catalog. So `bag info` on a rotated
/// recording's BASE file finds all three absent, and this block saying NOTHING
/// is the only correct reading of that — the manifest is in the final file, and
/// nothing in a single file's bytes says whether it was the last one.
/// `state_ring_e2e_test::a_rotated_recording_carries_its_state_manifest_exactly_
/// where_its_siblings_go` pins that all three move together, which is what
/// makes "the house shape" a fact rather than an assumption.
///
/// Do NOT turn this arm into a positive claim about the RECORDER (the way the
/// coverage sibling's `AbsentPre942` does — that line is wrong on a rotated file
/// today): telling these three causes apart needs evidence a single bag does not
/// carry. Evidence for the ROTATION cause is not recorded
/// (it would belong to a recording size-cap
/// feature), so the three stay indistinguishable and saying nothing stays the
/// only correct reading.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StateCoverageReading {
    /// The bag is FINALIZED (so its attachment index was readable) and carries
    /// no state manifest: this recording was configured for no checkpoints, or
    /// it predates them.
    Absent,
    /// The attachment index itself could not be read — the bag is not finalized,
    /// so the manifest could not be looked at. NOT evidence that none exists.
    IndexUnreadable,
    /// The manifest is present but its JSON did not decode.
    Malformed(String),
    /// The manifest decoded.
    Present(cerulion_bagd::StateCoverage),
}

/// Read the OPTIONAL state-coverage manifest out of a bag.
///
/// WARN-NEVER-REFUSE, the `record_coverage.json` precedent exactly: this is
/// ADDITIVE reporting, so no failure here may turn a readable bag into an
/// unreadable one.
fn read_state_coverage(reader: &BagReader, finalized: bool) -> StateCoverageReading {
    match reader.attachment(cerulion_bagd::STATE_COVERAGE_ATTACHMENT) {
        Ok(Some(att)) => match serde_json::from_slice::<cerulion_bagd::StateCoverage>(&att.data) {
            Ok(state) => StateCoverageReading::Present(state),
            Err(e) => StateCoverageReading::Malformed(e.to_string()),
        },
        Ok(None) if finalized => StateCoverageReading::Absent,
        Ok(None) => StateCoverageReading::IndexUnreadable,
        Err(e) => {
            tracing::debug!(
                attachment = cerulion_bagd::STATE_COVERAGE_ATTACHMENT,
                error = %e,
                "bag info: could not read the attachment index while looking for the state \
                 manifest"
            );
            StateCoverageReading::IndexUnreadable
        }
    }
}

/// Render a [`StateCoverageReading`] as the STATE block of `bag info`. PURE.
///
/// The ABSENT and INDEX-UNREADABLE arms render NOTHING, and that is the one
/// place this differs from the coverage section: every bag has a coverage story,
/// so a missing coverage manifest is a fact worth a line, while MOST bags carry
/// no checkpoints at all and a paragraph saying so on every one of them is noise
/// that would train an operator to skip the block. MALFORMED prints, because
/// that bag DOES have a checkpoint story somebody cannot read.
pub fn render_state_coverage_section(reading: &StateCoverageReading) -> String {
    match reading {
        StateCoverageReading::Absent | StateCoverageReading::IndexUnreadable => String::new(),
        // The decoder's own message, sanitized on the same rule as everything
        // else here: serde interpolates what it was reading, and what it was
        // reading is bag bytes.
        StateCoverageReading::Malformed(e) => format!(
            "\nnode state: the bag's `{}` is present but MALFORMED ({}), so what this recording \
             checkpointed cannot be read. The anchors themselves are unaffected — only the \
             report is unavailable for this bag.\n",
            cerulion_bagd::STATE_COVERAGE_ATTACHMENT,
            crate::topic_cmd::sanitize_display(e)
        ),
        StateCoverageReading::Present(state) => render_state_coverage(state),
    }
}

/// The `, N TORN, N skipped (cause xK, …)` suffix a node row carries. PURE.
///
/// ONE renderer for BOTH the declared-node rows and the unattributed-index rows.
/// Two renderers drift: an unattributed path that prints a bare `, N skipped` and
/// drops the CAUSE NAMES the manifest already stores leaves the rows an operator
/// understands least the ones told least, with `contended` vs `low_memory`
/// vs `capture_failed` (three different problems with three different next
/// steps) collapsed into a number. Sharing the code is what stops the two
/// from drifting.
///
/// Cause names are sanitized: an unrecognised wire code renders as
/// `unrecognized_{n}`, but a RECOGNISED name is still a string that came out of
/// a bag written elsewhere.
fn anchor_outcome_suffix(n: &cerulion_bagd::StateNodeCoverage) -> String {
    let mut extra = String::new();
    if n.anchors_torn > 0 {
        extra.push_str(&format!(", {} TORN", n.anchors_torn));
    }
    if n.anchors_skipped > 0 {
        let causes: Vec<String> = n
            .skip_causes
            .iter()
            .map(|(c, k)| format!("{} x{k}", crate::topic_cmd::sanitize_display(c)))
            .collect();
        extra.push_str(&format!(
            ", {} skipped ({})",
            n.anchors_skipped,
            causes.join(", ")
        ));
    }
    extra
}

/// Render a decoded state manifest. PURE.
///
/// EVERY string this prints comes out of the bag — the ring names and their
/// failure reasons off the manifest, the node ids off a ring's node table, the
/// skip-cause names off a wire discriminant — and the bag was written by another
/// process, possibly on another machine, possibly by another version. So they
/// all go through `topic_cmd::sanitize_display` before they reach a terminal,
/// exactly as the sibling coverage section's topic names and ring reasons do: a
/// bag carrying an ANSI CSI sequence in a node id must not be able to repaint the
/// screen of whoever runs `bag info` on it.
pub fn render_state_coverage(state: &cerulion_bagd::StateCoverage) -> String {
    let mut out = String::new();
    out.push_str(&format!(
        "\nnode state: {} record(s) across {} node(s) from {} ring(s){}.\n",
        state.records,
        state.nodes.len(),
        state.rings_declared,
        if state.attached_mid_run {
            " (attached MID-RUN)"
        } else {
            ""
        }
    ));
    // The recorder ARMS nothing, by design: the graph owns the capture
    // plane, so this states a fact about the PLANE this bag drained. Saying "this
    // recorder armed" would now be simply false, and it is the sentence an
    // operator reads when deciding whether a gap in the anchors is a recorder
    // problem or a run that was never anchoring.
    match &state.armed {
        Some(arm) => out.push_str(&format!(
            "  the capture plane was ARMED: every {} step(s) from step {}.\n",
            arm.cadence_steps, arm.first_anchor_step
        )),
        None => out.push_str(
            "  no armed capture plane was seen, so this bag makes no claim about the cadence \
             anchors were due on (the run's Flashback plane may have been switched off, \
             refused by the arm-time memory gate, or written by an older build).\n",
        ),
    }
    if state.attached_mid_run && state.head_records_discarded > 0 {
        out.push_str(&format!(
            "  {} leading record(s) belong to an anchor whose head was committed before this \
             recorder attached; they ARE in the bag and a reader discards them (they are not \
             loss).\n",
            state.head_records_discarded
        ));
    }
    for (ring, why) in &state.rings_unavailable {
        out.push_str(&format!(
            "  ring {} UNAVAILABLE: {} — its anchors are NOT in this bag.\n",
            crate::topic_cmd::sanitize_display(ring),
            crate::topic_cmd::sanitize_display(why)
        ));
    }
    for (node, n) in &state.nodes {
        let last = match n.last_complete_step {
            Some(s) => format!("last complete anchor at step {s}"),
            None => "NO complete anchor".to_string(),
        };
        out.push_str(&format!(
            "  {}: {} complete{} — {last}.\n",
            crate::topic_cmd::sanitize_display(node),
            n.anchors_complete,
            anchor_outcome_suffix(n)
        ));
    }
    // The UNATTRIBUTED indices, rendered after the declared nodes and clearly
    // marked as not being nodes at all. They live in their own `u32`-keyed map
    // (see `StateCoverage::unattributed_indices`) precisely so a real node whose
    // id looks like one of these labels cannot be shadowed by it — the label is
    // built HERE, at render, where it is text and nothing keys off it.
    for (ring, by_index) in &state.unattributed_indices {
        for (idx, n) in by_index {
            out.push_str(&format!(
                "  [no manifest entry] node_idx {idx}: {} complete{} — record(s) arrived for an \
                 index ring {} does not declare.\n",
                n.anchors_complete,
                anchor_outcome_suffix(n),
                crate::topic_cmd::sanitize_display(ring)
            ));
        }
    }
    if state.malformed_records > 0 {
        out.push_str(&format!(
            "  {} record(s) could not be keyed at all.\n",
            state.malformed_records
        ));
    }
    if state.is_incomplete() {
        // Deliberately its OWN verdict, and deliberately NOT folded into the
        // coverage block's COMPLETE/INCOMPLETE line.
        //
        // That line answers "is every live producer in this bag?" — a question
        // about the WIRE. This one answers "can this bag resume every node it
        // names?" — a question about STATE. A recording can be perfect on one
        // and useless on the other, and a single word covering both would tell
        // an operator which of two unrelated things to look at only by accident.
        out.push_str(
            "  CHECKPOINT COVERAGE INCOMPLETE — this bag cannot be used to resume every node \
             it names. See the per-node lines above.\n",
        );
    }
    out
}

// ---------------------------------------------------------------------------
// The coverage manifest, as `bag info` reports it
// ---------------------------------------------------------------------------

/// What `bag info` found when it went looking for the bag's
/// [`cerulion_bagd::RECORD_COVERAGE_ATTACHMENT`].
///
/// The four arms are kept APART because they answer the operator's question
/// differently, and collapsing any two of them would state something false:
/// "this recorder never measured coverage" (an older bag), "coverage could
/// not be looked at" (the attachment index is unreadable), "the manifest is
/// there and unreadable", and "here is what it says" are four different
/// situations with four different remedies.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CoverageReading {
    /// The bag is FINALIZED (so its attachment index was readable) and carries
    /// no coverage manifest — it was recorded before the coverage manifest existed.
    AbsentPre942,
    /// The attachment index itself could not be read, so the manifest could not
    /// be LOOKED AT. This is the non-finalized case: `BagReader::attachment`
    /// walks the summary's attachment index, and a bag whose recorder never
    /// finalized has no summary. It is NOT evidence the bag carries no manifest
    /// (attachment bytes are written long before the summary) — the same
    /// distinction [`render_scan`] draws for schema provenance.
    IndexUnreadable,
    /// The manifest is present but its JSON did not decode. Carries the decode
    /// error verbatim.
    Malformed(String),
    /// The manifest decoded.
    Present(cerulion_bagd::RecordCoverage),
}

/// Read the OPTIONAL coverage manifest out of a bag.
///
/// WARN-NEVER-REFUSE, following the `record_health.json` / `recorder.json`
/// precedent: coverage is ADDITIVE reporting, so no failure mode here may turn
/// a readable bag into an unreadable one. Every arm answers a
/// [`CoverageReading`] and `bag info` prints what it found.
fn read_record_coverage(reader: &BagReader, finalized: bool) -> CoverageReading {
    match reader.attachment(cerulion_bagd::RECORD_COVERAGE_ATTACHMENT) {
        Ok(Some(att)) => match serde_json::from_slice::<cerulion_bagd::RecordCoverage>(&att.data) {
            Ok(coverage) => CoverageReading::Present(coverage),
            Err(e) => CoverageReading::Malformed(e.to_string()),
        },
        // A finalized bag's attachment index WAS readable, so an absent
        // manifest is a real absence: the recorder predates the coverage manifest.
        //
        // The code once branched HERE on a second attachment, to tell that absence
        // apart from an EARLIER FILE of a rotated recording — whose manifest is
        // in the final file, making the `AbsentPre942` sentence affirmatively false
        // rather than merely unhelpful. The single-artifact decision removed
        // size-cap rotation from the product before launch, so that branch (and
        // the marker it read) moved to the post-launch issue. With no shipping
        // way to produce a rotated set, the one absence is the only one a new
        // bag can have; the residual is our own earlier internal rotated bags,
        // which read the `AbsentPre942` sentence.
        Ok(None) if finalized => CoverageReading::AbsentPre942,
        Ok(None) => CoverageReading::IndexUnreadable,
        Err(e) => {
            tracing::debug!(
                attachment = cerulion_bagd::RECORD_COVERAGE_ATTACHMENT,
                error = %e,
                "bag info: could not read the attachment index while looking for the coverage \
                 manifest"
            );
            CoverageReading::IndexUnreadable
        }
    }
}

/// Render a [`CoverageReading`] as the coverage block of `bag info`. PURE.
///
/// The three non-`Present` arms each say what they DO know and, crucially, what
/// they do not: none of them may read as "coverage was clean". A recorder that
/// finalized while live producers went unrecorded is the original defect; a
/// reader that renders "no manifest" the same as "nothing was missed" would
/// re-create it one layer up.
pub fn render_coverage_section(reading: &CoverageReading) -> String {
    match reading {
        CoverageReading::Present(coverage) => render_coverage(coverage),
        // RESIDUAL, stated here rather than hidden: an EARLIER file of a ROTATED
        // recording lands on this arm and reads this sentence, which is
        // affirmatively FALSE of it — that recorder DID measure coverage, into
        // the set's final file. The evidence to separate the two was built once
        // (a marker written at each rollover). The single-artifact decision
        // moved it to the post-launch issue with the size-cap feature
        // itself, so no SHIPPING path can produce a rotated set any more and the
        // exposure is bounded to rotated bags recorded before that decision — our
        // own earlier internal ones. The alternative that needs no marker,
        // inferring rotation from the `<name>.N.mcap` naming convention, was
        // rejected then and still is: a file can be renamed, moved or copied out
        // of its set, and guessing from a name is how a reader starts making
        // claims the bytes do not support.
        CoverageReading::AbsentPre942 => format!(
            "\nrecord coverage: this bag carries no `{}`, so it was recorded by a build that \
             never measured what else was live on that machine. That is an ABSENCE of \
             information, NOT a clean-coverage claim — re-record to get one.\n",
            cerulion_bagd::RECORD_COVERAGE_ATTACHMENT
        ),
        CoverageReading::IndexUnreadable => format!(
            "\nrecord coverage: could NOT BE READ — this bag is not finalized, so its attachment \
             index is missing and a `{}` it may well carry could not be looked at. Nothing here \
             says this bag's coverage is complete OR incomplete.\n",
            cerulion_bagd::RECORD_COVERAGE_ATTACHMENT
        ),
        CoverageReading::Malformed(e) => format!(
            "\nrecord coverage: the bag's `{}` is present but MALFORMED ({e}), so what this \
             recording covers cannot be read. The frames are unaffected — only the coverage \
             report is unavailable for this bag.\n",
            cerulion_bagd::RECORD_COVERAGE_ATTACHMENT
        ),
    }
}

/// The environment variable that widens bagd's discovery SETTLE window —
/// re-exported from the crate that OWNS it (and that reads it), never respelled
/// here, so the remedy this renderer prints cannot name a variable bagd stopped
/// honouring.
const RECORD_DISCOVERY_SETTLE_ENV_NAME: &str = cerulion_bagd::RECORD_DISCOVERY_SETTLE_ENV;

/// Render one [`cerulion_bagd::UntappedReason`] as its greppable tag PLUS the
/// payload that variant carries.
///
/// `tag()` is deliberately payload-free — it is the stable token an operator
/// greps for, and the recorder logs it — but printing ONLY the tag withheld the
/// two facts a reader can actually act on:
///
/// - `remote_mirror` without the origin ROBOT does not say WHOSE stream was
///   folded out. On a desk holding mirrors of two robots, the tag alone cannot
///   tell them apart, and the whole reason the variant exists is attribution.
/// - `attach_failed` without the transport's own message does not say WHY the
///   tap could not be opened. Slot exhaustion, a borrow budget below the
///   recorder's floor, and a service that vanished between the scan and the open
///   are three different problems with three different fixes — and the reason's
///   own doc lists exactly those three.
///
/// `budget_exhausted` carries its ceiling too; that one is also in the remedy
/// line, but a row that names its own number reads standalone.
///
/// Both string payloads can carry a remote machine's bytes — a mirror's robot
/// identity comes off the LAN, and a transport error interpolates the topic
/// name, which for a mirror is likewise remote-supplied — so both go through
/// `topic_cmd::sanitize_display` before they reach a terminal. The match is
/// EXHAUSTIVE by variant: a sixth reason must decide what it shows here rather
/// than silently inheriting a bare tag.
fn untapped_detail(reason: &cerulion_bagd::UntappedReason) -> String {
    use cerulion_bagd::UntappedReason as R;
    match reason {
        R::RemoteMirror { robot } => format!(
            "{} (robot {})",
            reason.tag(),
            crate::topic_cmd::sanitize_display(robot)
        ),
        R::BudgetExhausted { budget } => format!("{} (ceiling {budget})", reason.tag()),
        R::AttachFailed { error } => format!(
            "{}: {}",
            reason.tag(),
            crate::topic_cmd::sanitize_display(error)
        ),
        // The pattern, on the same reasoning as the two
        // above — with several `--exclude` patterns in play the tag alone
        // withholds the one fact an operator can act on, namely WHICH of their
        // patterns caught this topic. Sanitized like the others: the pattern is
        // the operator's own text, but it reaches a terminal and this renderer
        // makes no exceptions.
        R::ExcludedByRequest { pattern } => format!(
            "{} (matched `{}`)",
            reason.tag(),
            crate::topic_cmd::sanitize_display(pattern)
        ),
        // `declared_not_live` is already complete as a tag — the
        // topic NAME is the row's key and the reason has no payload, because
        // "nothing was producing it" admits no further detail the recorder
        // holds. (Which node declared it is a question for the bag's embedded
        // `graph.yaml`, not for this line.)
        R::ExcludedInternal | R::AppearedAfterBagCreation | R::DeclaredNotLive => {
            reason.tag().to_string()
        }
    }
}

/// Render a decoded coverage manifest. PURE — the oracle for the block above.
///
/// Two halves, and both are load-bearing: what the bag CONTAINS (every tap,
/// with how it got there and whether it attached late) and what it does NOT
/// (every live producer the recorder saw and did not record, with the reason).
/// The second half is the whole point of the manifest — `record_health.json`'s
/// `frames_lost = 0` was TRUTHFUL on the recording that motivated the manifest and
/// told the operator nothing, because the producers it missed were outside the
/// tapped set entirely.
///
/// The PRODUCER half of every qualified verdict sentence, in the
/// one place all four arms read it from.
///
/// Three arms opened with "every live producer the recorder enumerated is in this
/// bag, but …", and that is a sentence about a RECORDING: a capture holds a
/// rolling WINDOW of each of those producers. Printed under the capture caveat it
/// contradicts it — the same defect the COMPLETE gate was corrected for, arriving
/// through the arms nobody guarded. Both reachable on a shipping capture: the
/// always-on window recorder is run-bound (so the run-binding arm fires on a
/// failed or never-heard watcher) and `mirrors_established: Some(false)` latches
/// for the recorder's whole life.
///
/// A function rather than three edits so a FIFTH arm cannot reintroduce the
/// contradiction by copying a neighbour's literal.
fn producer_claim(coverage: &cerulion_bagd::RecordCoverage) -> &'static str {
    if coverage.window_capture {
        "every live producer the recorder enumerated has a channel in this bag (a WINDOW of it, \
         see above), but"
    } else {
        "every live producer the recorder enumerated is in this bag, but"
    }
}

/// What still holds a verdict back once the producer picture is
/// clean: the pointer the qualified arms send a reader after.
///
/// A capture's own qualifier (the WINDOW) is stated ABOVE the table, while the
/// schema and trace qualifiers print BELOW it, and a capture can carry either:
/// `build_capture_coverage` grades `replay_grade` from the capture's own
/// descriptors and deliberately carries `rings_declared` / `rings_unavailable`.
/// So the sentence names whichever apply rather than assuming one.
fn held_back_by(coverage: &cerulion_bagd::RecordCoverage) -> &'static str {
    let below = schemas_unresolved(coverage) || coverage.trace_degraded();
    match (coverage.window_capture, below) {
        (true, true) => {
            "held back from COMPLETE by the WINDOW — this is a capture, so what it holds of each \
             producer is a span, not the recording — AND by the line(s) below"
        }
        (true, false) => {
            "held back from COMPLETE by the WINDOW: this is a capture, so what it holds of each \
             producer is a span, not the recording"
        }
        _ => {
            "held back from COMPLETE only by the line(s) below, which describe this recording's \
             own artifacts rather than a missing producer"
        }
    }
}

pub fn render_coverage(coverage: &cerulion_bagd::RecordCoverage) -> String {
    let mut out = String::new();
    let declared = coverage
        .tapped
        .values()
        .filter(|t| matches!(t.source, cerulion_bagd::TapSource::Declared))
        .count();
    let discovered = coverage.tapped.len() - declared;
    out.push_str(&format!(
        "\nrecord coverage: {} topic(s) tapped ({} declared, {} discovered); live-service \
         enumeration {}.\n",
        coverage.tapped.len(),
        declared,
        discovered,
        if coverage.enumerated {
            "RAN"
        } else {
            "did NOT run"
        }
    ));
    // Read before every count below, which it qualifies. A
    // capture is a rolling WINDOW dumped into one bag, so the FRAMES column is
    // the window's and the bag begins mid-stream on every topic BY DESIGN —
    // without this line the same rows read as a recorder's lifetime account, and
    // the absent head-loss markers read as a proof of no head loss rather than
    // as the no-claim they are.
    if coverage.window_capture {
        out.push_str(&format!(
            "  this is a FLASHBACK CAPTURE, not a continuous recording: the FRAMES column below \
             is what this WINDOW holds (its recorder kept draining before and after it), and the \
             bag begins no earlier than the window's floor on any topic — so it makes no claim \
             about the START of any stream. What the capture was ABOUT — its span, its causes \
             and whether it can be resumed — is in its own `{}`.\n",
            cerulion_bagd::FLASHBACK_ATTACHMENT
        ));
    }
    if !coverage.enumerated {
        // The flag exists precisely so an empty untapped list is never mistaken
        // for a coverage guarantee nobody checked — and `discovery_requested`
        // splits the two ways of getting here, which carry OPPOSITE remedies.
        if coverage.discovery_requested {
            out.push_str(
                "  discovery was REQUESTED but every attempt to read the live service directory \
                 FAILED, so this bag makes NO claim about what else was live on that machine.\n",
            );
        } else {
            // This arm must NOT say "explicit selection".
            // It is reached by three different callers and only one of them
            // named its topics: `cerulion bag record --topic a b c` did, but
            // `--all` / `--regex` DERIVED the set from a live enumeration this
            // verb ran itself (`derive_record_topics` over `list_topics`, with
            // the same excluded-prefix list and the same netd-mirror fold), and
            // `graph run --record` under the `CERULION_RECORD_DISCOVERY=off`
            // kill-switch inherited the graph's declared outputs. What is true
            // of all three is that the tap set was FIXED BEFORE the recording
            // started, so the RECORDER never enumerated and its empty untapped
            // list asserts nothing. (Why `--all`'s own enumeration is not
            // reported here: this manifest is the RECORDER's account of what it
            // observed, and `bag record`'s scan is a one-shot pre-arm probe, not
            // the recorder's rescan-and-hold-creation-open loop. Carrying it
            // would need a `BagdConfig` seam for a caller-supplied manifest.)
            out.push_str(
                "  the recorder was not asked to enumerate live services — the tap set was \
                 FIXED by the caller before the recording started (`cerulion bag record` \
                 derives its own, whether from an explicit `--topic` list or from its own \
                 one-shot live scan under `--all` / `--regex`), so this bag makes NO claim \
                 about what else was live on that machine.\n",
            );
        }
    } else if coverage.enumeration_failures > 0 {
        // Enumeration succeeded at least once AND failed at least once: the
        // untapped list below is real but incomplete by an unknown amount.
        out.push_str(&format!(
            "  {} enumeration(s) FAILED during this recording, so the untapped list below is \
             incomplete by an unknown amount.\n",
            coverage.enumeration_failures
        ));
    }
    // The doubt this one raises is about what is IN the bag, not what
    // is missing from it, so it is reported independently of the arms above. A
    // manifest that carries no verdict (an older bag, or discovery never requested)
    // says nothing here — silence is the correct rendering of `None`.
    if coverage.mirrors_established == Some(false) {
        out.push_str(
            "  this recording could NOT establish which local topics are mirrors of another \
             robot, so a topic listed below as tapped may really be another robot's re-injected \
             stream rather than this machine's data (mirrors_established: false).\n",
        );
    }
    // A head loss is about what is MISSING from a topic the bag DOES
    // contain, so it is reported independently of the arms above, exactly like
    // the mirror doubt. A manifest that proves no head loss says nothing here.
    if coverage.prefix_lost_topics() > 0 {
        out.push_str(&format!(
            "  {} frame(s) across {} topic(s) were committed BEFORE the recorder's tap drained \
             anything, so this bag begins mid-stream on them (marked below). This loss is \
             invisible to record_health.json's frames_lost, which needs a first recorded frame to \
             compare against.\n",
            coverage.prefix_lost_total(),
            coverage.prefix_lost_topics(),
        ));
    }
    // The same shape at the OTHER end of a capture's window. The close
    // drains the taps once more before reading the window; when that drain did
    // not complete, the tail is what the pass reached, and the manifest says so.
    if coverage.pre_close_drain_incomplete == Some(true) {
        let reasons: Vec<String> = coverage
            .pre_close_drain_stops
            .iter()
            .map(|stop| match stop {
                cerulion_bagd::PreCloseDrainStop::ReceiveError { error } => {
                    format!(
                        "a receive error ({})",
                        crate::topic_cmd::sanitize_display(error)
                    )
                }
                cerulion_bagd::PreCloseDrainStop::StagingFull { topic } => format!(
                    "{} was left at the recorder's staging bound with frames still queued",
                    crate::topic_cmd::sanitize_display(topic)
                ),
                cerulion_bagd::PreCloseDrainStop::NotDrained { topic } => format!(
                    "{} was not drained",
                    crate::topic_cmd::sanitize_display(topic)
                ),
            })
            .collect();
        let reasons = if reasons.is_empty() {
            "no reason was recorded".to_string()
        } else {
            reasons.join("; ")
        };
        out.push_str(&format!(
            "  the close's final drain did not complete: {reasons}. The window holds what had \
             been drained up to that pass and its trace may reach one step past it, so the END \
             of this capture may be missing frames on those topics \
             (pre_close_drain_incomplete: true).\n"
        ));
    }
    // The RUN BINDING, reported whenever there is one — an operator
    // opening a bag wants to know which run it is named for and how that run
    // ended, not only when something went wrong. The AMBIGUITY it may carry gets
    // its own verdict below.
    if let Some(binding) = &coverage.run_binding {
        let ended = match binding.ended {
            Some(cerulion_bagd::RunEndedTag::Graceful) => {
                "ended GRACEFULLY (it announced its own end)"
            }
            Some(cerulion_bagd::RunEndedTag::Vanished) => {
                "STOPPED without announcing (a crash, or an exit whose last word was not heard)"
            }
            None if binding.watch_failed => {
                "could not be WATCHED, so this bag's boundary says nothing about it"
            }
            // `ended: None` means "no end was observed", and WHY
            // depends on the artifact. On a recording the recorder outlives
            // nothing — it stopped first. A CAPTURE is cut from a live window
            // while the run keeps going, so "the RECORDER was stopped" is
            // affirmatively false there, and it is the ORDINARY case: the
            // always-on window recorder is run-bound, so every trigger-fired
            // capture of a healthy run lands on this arm.
            None if coverage.window_capture => {
                "was still executing when this capture was cut from the live window"
            }
            None => "was still executing as far as this recorder knew — the RECORDER was stopped",
        };
        let heard = if binding.never_heard {
            " That run was NEVER HEARD by this recorder, so the window below is the whole \
             recording."
        } else {
            ""
        };
        out.push_str(&format!(
            "  bound to run {:#034x}, which {ended}.{heard} {} frame(s) were committed in the {} \
             ms after that run was last heard alive; a wire frame carries no run id, so those are \
             the frames this bag cannot attribute.\n",
            binding.run_id, binding.unattributed_frames, binding.unheard_for_ms,
        ));
    }
    out.push_str(&format!(
        "{:<40} {:>9}  {}\n",
        "TOPIC", "FRAMES", "TAPPED VIA"
    ));
    for (topic, tap) in &coverage.tapped {
        let source = match tap.source {
            cerulion_bagd::TapSource::Declared => "declared",
            cerulion_bagd::TapSource::Discovered => "discovered",
        };
        // A late tap covers its topic only from the moment it attached: a
        // data-only tap requests no late-joiner history, so whatever that
        // producer emitted earlier is simply not in the bag.
        //
        // NOT on a capture. `attached_late` means "the tap attached
        // after this RECORDER armed", and the always-on window recorder is
        // spawned before the graph it watches — so on a capture essentially
        // EVERY row carries it, while saying nothing about whether the WINDOW
        // begins at that topic's first frame (it never does, on any row). A
        // marker that is universally true teaches an operator to skim rows,
        // which is the same argument `is_incomplete`'s own docs make for not
        // escalating on a condition every real recording meets. The general fact
        // is stated ONCE, in the capture sentence above.
        let late = if tap.attached_late && !coverage.window_capture {
            "  [attached after the recording started — no back-fill]"
        } else {
            ""
        };
        // The QUANTITATIVE sibling of the marker above — the tap was
        // there from the start and the frames were lost anyway.
        let prefix = match tap.prefix_lost {
            Some(n) if n > 0 => format!("  [missing the first {n} frame(s) of this stream]"),
            _ => String::new(),
        };
        out.push_str(&format!(
            "{:<40} {:>9}  {}{}{}\n",
            crate::topic_cmd::sanitize_display(topic),
            tap.frames_recorded,
            source,
            late,
            prefix
        ));
    }

    // Partition on the reason's OWN verdict rather than re-deriving one here:
    // an exclusion by RULE (the recorder's status channel, a remote robot's
    // netd mirror) is deliberately not this recording's data, and counting it
    // as a gap would train an operator to skim the number that matters.
    //
    // The not-a-gap rows are two classes, split on
    // the reason's own class — the same classification the recorder's terminal
    // line splits its counts on, so the two surfaces cannot disagree. Folding
    // `declared_not_live` under "by rule" said a rule had excluded a topic that
    // nothing excluded: it was ASKED FOR and had no producer, which is a
    // different fact with a different next step.
    //
    // An exhaustive `match` on `UntappedClass`, not an `else`
    // catch-all. A catch-all makes the "cannot disagree" claim true only of
    // today's variant list: an eighth not-a-gap, not-by-rule reason would
    // fall into `declared_not_live` and render under a heading — "declared
    // by the run but never live" — that would be affirmatively FALSE about it,
    // while the terminal's `matches!` counted it in no bucket at all. A new
    // variant is a compile error at `UntappedReason::class`, and a new
    // BUCKET a compile error here.
    let mut gaps: Vec<(&String, &cerulion_bagd::UntappedReason)> = Vec::new();
    let mut by_rule: Vec<(&String, &cerulion_bagd::UntappedReason)> = Vec::new();
    let mut declared_not_live: Vec<(&String, &cerulion_bagd::UntappedReason)> = Vec::new();
    for (topic, reason) in &coverage.untapped {
        match reason.class() {
            cerulion_bagd::UntappedClass::Gap => gaps.push((topic, reason)),
            cerulion_bagd::UntappedClass::ExcludedByRule => by_rule.push((topic, reason)),
            cerulion_bagd::UntappedClass::DeclaredNotLive => {
                declared_not_live.push((topic, reason))
            }
        }
    }

    // The VERDICT word tracks the recorder's own `RecordCoverage::is_incomplete`
    // — the same predicate that escalates the run's terminal line to a WARN — so
    // `bag info` and the recording that produced the bag cannot disagree about
    // whether its coverage held. Note that `!is_incomplete()` is NOT the same as
    // COMPLETE: a caller who opted out of discovery triggers no warning AND can
    // claim nothing, so that case gets no verdict at all — and with the
    // qualified arm below, that is the only thing an absent `coverage:`
    // line means.
    if gaps.is_empty() {
        // The mirror doubt gets its OWN verdict sentence.
        // `is_incomplete()` now has two very different causes, and the generic
        // sentence below describes only one of them: with enumeration having RUN
        // and succeeded, "the recorder could not establish what was live" is not
        // a hedge, it is FALSE — and it contradicts the "enumeration RAN" line
        // printed two rows above. The doubt here is not about what is MISSING
        // from the bag; it is about whether something IN it belongs to this
        // machine at all. Ordered first because it is the more specific claim:
        // a run that is incomplete for BOTH reasons should say the one an
        // operator cannot infer from the untapped list.
        let mirror_doubt = coverage.mirrors_established == Some(false);
        if coverage.prefix_lost_topics() > 0 {
            // Head loss, ordered FIRST among the no-gap verdicts because it is the
            // only PROVEN loss among them — the mirror arm below states a doubt,
            // and the generic arm states an unknown. Both of those keep their own
            // detail lines above regardless of which verdict prints, so nothing
            // an operator needs disappears; what would be intolerable is the
            // generic arm's sentence ("no producer is KNOWN to be missing, but
            // the recorder could not establish what was live") printing over a
            // run whose enumeration RAN and whose loss is measured — the
            // contradicting-verdict class this block already carries two fixes
            // for.
            //
            // The ENUMERATION half is stated PER PATH for the same reason the
            // mirror arm's is: `BagdConfig::armed_before_producers`
            // is independent of `discover_live`, so this arm is reachable with
            // enumeration having run, having failed, and never having been
            // asked — and claiming "every live producer is in this bag" on the
            // last two would print directly under this block's own "makes NO
            // claim about what else was live" text.
            let enumeration_clause = if coverage.enumerated {
                producer_claim(coverage)
            } else if coverage.discovery_requested {
                "the recorder could not establish what was live (see above), and"
            } else {
                "this bag makes no claim about what else was live (see above), and"
            };
            out.push_str(&format!(
                "coverage: INCOMPLETE — {enumeration_clause} {} topic(s) it DID record are \
                 missing the START of their stream ({} frame(s) total, see above).\n",
                coverage.prefix_lost_topics(),
                coverage.prefix_lost_total(),
            ));
        } else if coverage.pre_close_drain_incomplete == Some(true) {
            // The second PROVEN loss among the no-gap verdicts, at the
            // other end of the window from the prefix arm above: the capture's
            // close drained the taps once more before reading the window and
            // that drain did not complete, so the tail is what the pass reached.
            let enumeration_clause = if coverage.enumerated {
                producer_claim(coverage)
            } else if coverage.discovery_requested {
                "the recorder could not establish what was live (see above), and"
            } else {
                "this bag makes no claim about what else was live (see above), and"
            };
            out.push_str(&format!(
                "coverage: INCOMPLETE, {enumeration_clause} the close's final drain did not \
                 complete (see above), so the END of this capture may be missing frames on the \
                 topics it left queued.\n"
            ));
        } else if coverage
            .run_binding
            .as_ref()
            .is_some_and(cerulion_bagd::RunBindingCoverage::is_ambiguous)
        {
            // The run-binding arm, ordered directly after the prefix arm and before the
            // mirror one because it is the same KIND of claim as prefix loss —
            // a MEASURED quantity with a specific cause — while the mirror arm
            // states a doubt and the generic arm an unknown. What it doubts,
            // though, is the mirror arm's question rather than the prefix arm's:
            // not what is missing FROM this bag, but whether what is IN it
            // belongs to the run the bag is named for.
            //
            // The enumeration half is stated PER PATH for the reason the two
            // arms below already are: a run binding is independent
            // of `discover_live` — `graph run --record` sets both while a
            // hand-run `cerulion bagd --run-id` need not — so this arm is
            // reachable with enumeration having run, having failed, and never
            // having been asked.
            let enumeration_clause = if coverage.enumerated {
                producer_claim(coverage)
            } else if coverage.discovery_requested {
                "the recorder could not establish what was live (see above), and"
            } else {
                "this bag makes no claim about what else was live (see above), and"
            };
            let binding = coverage.run_binding.as_ref().expect("guarded above");
            if binding.watch_failed {
                out.push_str(&format!(
                    "coverage: INCOMPLETE — {enumeration_clause} it was asked to record ONE run \
                     and could not watch that run at all, so this bag's boundary is simply where \
                     the recorder was stopped: if the graph restarted underneath it, both runs \
                     are in here.\n"
                ));
            } else if binding.never_heard {
                out.push_str(&format!(
                    "coverage: INCOMPLETE — {enumeration_clause} it was asked to record ONE run \
                     and NEVER HEARD that run, so none of its {} frame(s) can be dated to it and \
                     a successor taking over its topics could not have been noticed.\n",
                    binding.unattributed_frames,
                ));
            } else {
                out.push_str(&format!(
                    "coverage: INCOMPLETE — {enumeration_clause} a SUCCESSOR run of the same \
                     graph was announcing while the bound run was silent, so the last {} frame(s) \
                     in this bag (the {} ms after that run was last heard) may belong to the next \
                     run rather than this one.\n",
                    binding.unattributed_frames, binding.unheard_for_ms,
                ));
            }
        } else if mirror_doubt {
            // The ENUMERATION half of this sentence is stated
            // PER PATH, because the same arm is now reached by paths where it
            // is true, false, and not-even-asked.
            //
            // The first version opened with "every live producer the recorder
            // enumerated is in this bag" unconditionally — and the first review's
            // own fix made `cerulion bag record` reach here, where discovery is OFF
            // and that claim is FALSE. Worse, it printed three lines under this
            // block's own "live-service enumeration did NOT run" / "makes NO
            // claim about what else was live" text: exactly the
            // contradicting-verdict class F2 was raised to fix, mirrored onto
            // the discovery-OFF path by the fix for it. The COMPLETE arm below
            // has been guarded on `enumerated` for precisely this reason since
            // the manifest was introduced; this arm now is too.
            // NB the SUBJECT the tail needs ("the recorder could not…") is
            // supplied HERE, per path: the two doubt clauses below carry their
            // own ("and it also" / "and the recorder"), while the shared
            // `producer_claim` clause ends at "but" — used bare, the tail's
            // subject became "every live producer" (ent round: the verdict read
            // as every PRODUCER failing to establish mirror state).
            let enumeration_clause = if coverage.enumerated {
                std::borrow::Cow::from(format!("{} the recorder", producer_claim(coverage)))
            } else if coverage.discovery_requested {
                // Asked and could not be answered — the enumeration doubt is
                // real AND stands alongside the mirror one.
                std::borrow::Cow::from(
                    "the recorder could not establish what was live (see above), and it also",
                )
            } else {
                // Never asked (`cerulion bag record`): no enumeration claim to
                // make in either direction.
                std::borrow::Cow::from(
                    "this bag makes no claim about what else was live (see above), and the \
                     recorder",
                )
            };
            // FINALLY: "could not READ the registry" is true on every path
            // that ASKED, and false on the one that did not — a run cancelled
            // before its first attempt read nothing because it was never asked
            // to. `RecordCoverage` carries no attempts/stopped-early field, so
            // the renderer cannot tell the two apart; rather than document the
            // imprecision, the tail takes the path-NEUTRAL wording already
            // established for this same doubt on the two sibling surfaces (the
            // detail line above, and replay's warn). "Could not establish" is
            // true whether the answer timed out or was never sought, so all
            // three surfaces now say one thing.
            out.push_str(&format!(
                "coverage: UNVERIFIED — {enumeration_clause} could not establish which local \
                 topics are mirrors of another robot, so it cannot rule out that a topic it \
                 recorded as local is really another robot's mirrored stream.\n"
            ));
        } else if coverage_is_incomplete(coverage) {
            out.push_str(
                "coverage: INCOMPLETE — no producer is KNOWN to be missing, but the recorder \
                 could not establish what was live (see above), so this bag cannot claim to hold \
                 everything that was.\n",
            );
        } else if coverage.enumerated
            && !schemas_unresolved(coverage)
            && !coverage.trace_degraded()
            && !coverage.window_capture
        {
            // COMPLETE is WITHHELD while the schema or trace verdict below is
            // unhappy (the head-loss rule, for the same reason): an unqualified
            // COMPLETE is the line an operator stops reading at.
            //
            // The WINDOW term is added on that identical reasoning, and it
            // is the sharpest instance of it: "every live producer the recorder
            // enumerated is in this bag" is a sentence about a RECORDING, and a
            // capture holds a rolling window of each of those producers rather
            // than all of it. Printed three lines under the capture caveat, that
            // COMPLETE would contradict the very paragraph this block just
            // emitted — and a bag that carried NO manifest at all (every capture
            // before the co-tenancy fix) was told "that is an ABSENCE of information, NOT
            // a clean-coverage claim", so acquiring a manifest must not upgrade
            // it to one. `is_incomplete()` is UNCHANGED — being a WINDOW is not
            // a term of it (a capture reaches INCOMPLETE by the ordinary routes,
            // and the tests below drive one) — so the recorder's terminal
            // verdict and replay's reading of this manifest are both unaffected
            // by the window.
            //
            // The trace term is added on that identical
            // reasoning, and only there: `is_incomplete()` is UNCHANGED, so the
            // recorder's terminal verdict and replay's reading of this manifest
            // both still treat a missing trace as what it is — not a coverage
            // gap.
            out.push_str(
                "coverage: COMPLETE — every live producer the recorder enumerated is in this \
                 bag.\n",
            );
        } else if coverage.enumerated {
            // The qualified verdict, which exists so
            // that an ABSENT `coverage:` line keeps exactly ONE meaning.
            //
            // Withholding COMPLETE above is right; printing NOTHING in its place
            // was not. The arm above is the only one that can fire once the gap
            // / mirror / prefix arms have not, so a run with a clean producer
            // picture and an unhappy schema-or-trace verdict rendered no
            // coverage line AT ALL — while ~30 lines up an absent line already
            // means something else entirely ("a caller who opted out of
            // discovery can claim nothing, so that case gets no verdict"). One
            // blank meaning two different things is precisely what the block's
            // own contradicting-verdict rule forbids, and the comment that used
            // to sit in the arm above claimed the two surfaces "still agree line
            // for line" — which they could not, since one of them printed
            // nothing.
            //
            // The verdict WORD is deliberately not a softened COMPLETE
            // ("COMPLETE WITH CAVEATS" and friends still start with the token a
            // reader stops at, and would satisfy a `contains("coverage:
            // COMPLETE")` grep). NO GAPS says the producer half plainly and
            // sends the reader to the lines that qualify it.
            //
            // `enumerated`-GUARDED, on the precedent that governs
            // every other arm here: without enumeration this recording is
            // entitled to no producer claim in either direction, and that is the
            // one case the absent line now means.
            // WHICH qualifier applies is `held_back_by`'s business —
            // a capture's own is stated ABOVE the table, the schema and trace
            // ones BELOW it, and a capture can carry either.
            out.push_str(&format!(
                "coverage: NO GAPS — {} and the verdict is {}.\n",
                if coverage.window_capture {
                    "every live producer the recorder enumerated has a channel in this bag,"
                } else {
                    "every live producer the recorder enumerated is in this bag,"
                },
                held_back_by(coverage),
            ));
        }
    } else {
        out.push_str(&format!(
            "coverage: INCOMPLETE — {} live producer(s) existed that this bag does NOT contain:\n",
            gaps.len()
        ));
        for (topic, reason) in &gaps {
            out.push_str(&format!(
                "  {:<40} {}\n",
                crate::topic_cmd::sanitize_display(topic),
                untapped_detail(reason)
            ));
        }
        // One remedy line per reason CLASS present — an operator reading
        // `appeared_after_bag_creation` needs a different action from one
        // reading `budget_exhausted`, and neither is derivable from the tag.
        if gaps
            .iter()
            .any(|(_, r)| matches!(r, cerulion_bagd::UntappedReason::AppearedAfterBagCreation))
        {
            // The knob is the discovery SETTLE window, NOT the schema-wait
            // grace: the schema grace is a force-create DEADLINE, and on the
            // `graph run --record` path every declared tap is exact-mode so
            // every schema is known at construction and the bag would be
            // created on pass 1 regardless of it.
            //
            // The ENV var is named FIRST because it is the
            // only form that reaches the paths this gap is reachable from by
            // default. The `--run` attach made that plural: discovery is ON by default
            // for `--topics-json` (i.e. `graph run --record`, whose
            // `spawn_bagd_recorder` builds bagd's whole argv internally) AND for
            // `cerulion bag record --run`, which assembles a `BagdConfig`
            // in-process and accepts no settle flag of its own. Neither gives a
            // reader any way to type `cerulion bagd --discovery-settle-ms`. The
            // flag is still named, because a hand-run `cerulion bagd` is the
            // other way to reach this gap and there the flag is the direct knob.
            out.push_str(&format!(
                "  appeared_after_bag_creation: MCAP channels are registered when the bag is \
                 created and are immutable afterwards, so a producer that appears later can be \
                 given no channel. Hold bag creation open longer by setting \
                 `{RECORD_DISCOVERY_SETTLE_ENV_NAME}=<ms>` in the recording's environment — that \
                 reaches every path, including `cerulion graph run --record`, which builds the \
                 recorder's command line itself. `cerulion bagd --discovery-settle-ms <ms>` is \
                 the same knob for a recorder you invoke by hand.\n"
            ));
        }
        if gaps
            .iter()
            .any(|(_, r)| matches!(r, cerulion_bagd::UntappedReason::BudgetExhausted { .. }))
        {
            // The old remedy ("name the topics you need
            // explicitly") is unactionable on the one path that reaches this
            // by default. `graph run --record` GENERATES its tap list from the
            // graph's declared outputs, so there is nothing for the operator to
            // name; and the ceiling itself is a compile-time constant with no
            // flag on any path. What IS true and actionable: declared taps are
            // exempt (`plan_discovery` counts only DISCOVERED taps and skips
            // anything already `known`), and the discovered set is filled by a
            // SORTED walk, so which topics won is reproducible rather than an
            // artifact of service-directory order.
            out.push_str(&format!(
                "  budget_exhausted: the discovered-tap ceiling ({}) was already full. It is a \
                 build-time constant with no flag, and the discovered set is filled in sorted \
                 name order, so which topics won is deterministic. DECLARED taps are exempt from \
                 it — under `graph run --record` those are the graph's own outputs, so anything \
                 the graph produces is recorded regardless; to capture a topic outside that set, \
                 name it to a hand-run `cerulion bagd --topic <topic>` (a separate, \
                 observability-grade bag), or reduce the number of live producers on the \
                 machine.\n",
                cerulion_bagd::DISCOVERY_MAX_TAPS
            ));
        }
        if gaps
            .iter()
            .any(|(_, r)| matches!(r, cerulion_bagd::UntappedReason::AttachFailed { .. }))
        {
            out.push_str(
                "  attach_failed: the recorder found the producer but could not open a tap on it \
                 (a subscriber slot, a borrow budget below the recorder's floor, or a service \
                 that vanished). The transport's own message is printed with the topic above.\n",
            );
        }
    }
    let render_row = |(t, r): &(&String, &cerulion_bagd::UntappedReason)| {
        format!(
            "{} [{}]",
            crate::topic_cmd::sanitize_display(t),
            untapped_detail(r)
        )
    };
    if !by_rule.is_empty() {
        out.push_str(&format!(
            "also untapped, by rule (not a coverage gap): {}\n",
            by_rule
                .iter()
                .map(render_row)
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }
    // Its own line, because it is not an
    // exclusion. Nothing took these topics out — the run DECLARED them and
    // nothing was producing them, which is the ordinary shape of a graph
    // holding an output that has not fired (and, under `--peer-loss continue`,
    // of a group whose worker is gone). Also not a coverage gap.
    if !declared_not_live.is_empty() {
        out.push_str(&format!(
            "also untapped, declared by the run but never live (not a coverage gap): {}\n",
            declared_not_live
                .iter()
                .map(render_row)
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }
    // A SEPARATE line, never an arm of the coverage chain above — the
    // head-loss precedent, for the same reason. Every arm above answers "what is
    // MISSING from this bag"; this answers "can what IS in it be read anywhere
    // else", which is a different question with a different remedy. Folding it
    // in printed "the recorder could not establish what was live" on a run whose
    // enumeration RAN and found everything, which is the mirror-provenance
    // contradicting-verdict class on a third path.
    if schemas_unresolved(coverage) {
        out.push_str(
            "schemas: UNRESOLVED — this recording asked to resolve its channels' schema names \
             and not one could be described, so every channel carries a bare wire hash and the \
             bag renders nothing anywhere. This is not a coverage gap; see each topic's \
             schema_source in the attachment below for which rung was tried.\n",
        );
    }
    // The authoritative trace verdict.
    //
    // A SEPARATE line for the reason the two above it are separate — a missing
    // scheduler trace is a different artifact from a missing producer and from
    // an unreadable channel — and load-bearing beyond that: `run.json`'s
    // `trace` field is chosen when the bag is CREATED, strictly before
    // `Recorder::setup` opens a single ring, so a bag whose every declared ring
    // had already been unlinked still carries `trace: "…from the attach
    // point…"`. That attachment is frozen and cannot be corrected in place, so
    // THIS is the line that says what happened, and it says so explicitly
    // rather than leaving a reader to reconcile two artifacts.
    if coverage.trace_degraded() {
        let opened = coverage.rings_opened();
        out.push_str(&format!(
            "trace: {} — this recording declared {} scheduler-trace ring(s) and could open {}. \
             `__cerulion/run.json`'s `trace` field is chosen when the bag is created, BEFORE the \
             rings are opened, so it describes what was INTENDED; this line is the outcome. The \
             run was most likely exiting: a ring's shared-memory name is unlinked the moment its \
             owner drops.\n",
            if opened == 0 { "NONE" } else { "PARTIAL" },
            coverage.rings_declared,
            opened,
        ));
        for (ring, err) in &coverage.rings_unavailable {
            out.push_str(&format!(
                "  {:<40} {}\n",
                crate::topic_cmd::sanitize_display(ring),
                crate::topic_cmd::sanitize_display(err)
            ));
        }
    }
    // ALWAYS name the artifact — including on a clean run, for the same reason
    // `render_record_summary` always names `record_health.json`: the reader who
    // needs the detail is often the one whose summary looks fine.
    out.push_str(&format!(
        "per-topic coverage detail (incl. each untapped topic's full reason) is in the bag's \
         `{}` attachment.\n",
        cerulion_bagd::RECORD_COVERAGE_ATTACHMENT
    ));
    out
}

/// The DESCRIPTOR half of
/// [`cerulion_bagd::RecordCoverage::is_incomplete`] — the run
/// asked to resolve its channels' schemas and could describe none of them.
///
/// Split out because the renderer must answer two questions separately: a bag
/// can hold every live producer (coverage COMPLETE) and still render nowhere,
/// and a reader told only one of those learns the wrong thing.
fn schemas_unresolved(coverage: &cerulion_bagd::RecordCoverage) -> bool {
    coverage.schema_demand_requested
        && coverage.replay_grade == Some(cerulion_bagd::ReplayGrade::Observability)
}

/// The COVERAGE half of [`cerulion_bagd::RecordCoverage::is_incomplete`],
/// stated as its OWN
/// UNION rather than as a subtraction.
///
/// `is_incomplete()` is the union of every term; this is the union of the
/// COVERAGE terms only. Writing it as `is_incomplete() && !schemas_unresolved()`
/// looked equivalent and was not: it MASKS rather than subtracts, so a run
/// carrying BOTH a coverage cause and unresolved schemas rendered NO coverage
/// line at all — while the recorder's own terminal still fired one, which is
/// exactly the "bag info and the recording that produced it cannot disagree"
/// rule this block exists to keep.
///
/// The MIRROR cause is deliberately absent: it has its own arm above with its
/// own wording, and folding it in here would let the generic sentence pre-empt
/// the specific one. The PREFIX term is listed for completeness and is dead at
/// this reach point for the same reason — the prefix arm fires earlier — but it
/// is kept so this reads as the coverage union it claims to be, rather than as a
/// list whose omissions a reader has to reconstruct from the arm order above.
fn coverage_is_incomplete(coverage: &cerulion_bagd::RecordCoverage) -> bool {
    coverage.gap_count() > 0
        || coverage.enumeration_failures > 0
        || (coverage.discovery_requested && !coverage.enumerated)
        || coverage.prefix_lost_topics() > 0
        // A bag whose tail may be another RUN's must not reach the
        // COMPLETE arm. The dedicated verdict above normally renders first; this
        // term is what stops COMPLETE printing if that arm is ever reordered
        // away, and keeps this renderer's predicate in step with the recorder's
        // own `RecordCoverage::is_incomplete`.
        || coverage
            .run_binding
            .as_ref()
            .is_some_and(cerulion_bagd::RunBindingCoverage::is_ambiguous)
        // A capture whose close's final drain did not complete. Its
        // dedicated verdict above renders first; listed here for the same reason
        // the prefix term is, so this reads as the coverage union it claims to be.
        || coverage.pre_close_drain_incomplete == Some(true)
}

/// The shared human-readable rendering of a [`BagScan`] — the `bag info` output
/// and the `bag play` startup banner are the same text, so they cannot drift.
pub fn render_scan(
    scan: &BagScan,
    path: &Path,
    walker: &cerulion_core::codegen::FrameWalker,
    catalog: Option<&cerulion_bag::BagSchemaCatalog>,
) -> String {
    let mut out = String::new();
    out.push_str(&format!("bag: {}\n", path.display()));
    out.push_str(&format!("state: {}\n", scan.completeness));
    out.push_str(&format!(
        "frames: {} across {} topic(s)",
        scan.total_frames,
        scan.channels.len()
    ));
    match scan.span_ns {
        Some(ns) => out.push_str(&format!("; span {:.3}s\n", ns as f64 / 1e9)),
        None => out.push('\n'),
    }
    if !scan.finalized {
        out.push_str(
            "warning: this bag was never finalized, so it carries no summary footer and its \
             frames cannot be walked. `cerulion bag play` will refuse it. A bag is finalized \
             when its recorder shuts down cleanly (Ctrl-C / SIGTERM), not when the process is \
             killed.\n",
        );
    }
    if scan.backwards_log_times > 0 {
        out.push_str(&format!(
            "note: {} frame(s) carry a wire timestamp EARLIER than their predecessor. Those \
             stamps come from the PRODUCING process's clock, so a bag mixing clock epochs (a \
             restarted producer, several producers) looks like this. Playback still runs in \
             recorded order; a backwards step publishes immediately.\n",
            scan.backwards_log_times
        ));
    }
    if scan.headerless_frames > 0 {
        out.push_str(&format!(
            "note: {} frame(s) carry no parseable wire header — they are recorded and will be \
             republished verbatim, but they contribute no timing and pace as instant.\n",
            scan.headerless_frames
        ));
    }
    out.push('\n');
    out.push_str(&format!(
        "{:<40} {:>9}  {:>10}  {}\n",
        "TOPIC", "FRAMES", "MAX BYTES", "SCHEMA"
    ));
    let mut unresolvable: Vec<String> = Vec::new();
    let mut no_descriptor: Vec<String> = Vec::new();
    let mut from_bag: Vec<String> = Vec::new();
    let mut from_bag_unviewable: Vec<String> = Vec::new();
    let mut from_bag_builtin_named: Vec<String> = Vec::new();
    let mut skewed: Vec<String> = Vec::new();
    for c in &scan.channels {
        let reading = resolve_schema(walker, catalog, c);
        let shown = display_schema_name(c, &reading);
        let mut marker = match &reading {
            SchemaReading::Local { .. } => String::new(),
            SchemaReading::FromBag { name } => {
                // The viewer refuses a bag definition on TWO structural grounds,
                // and they carry OPPOSITE remedies, so they get separate rows and
                // separate notes (see `bag_doc_is_viewable`). Claiming either one
                // renders would re-create the exact success-shaped failure this
                // work exists to remove.
                if bag_doc_is_viewable(catalog, name) {
                    from_bag.push(c.topic.clone());
                    "  [from the bag's own schema records]".to_string()
                } else if crate::schema_store::builtin_has_qualified(name) {
                    from_bag_builtin_named.push(c.topic.clone());
                    "  [defined by the bag under a BUILT-IN name — the viewer refuses it]"
                        .to_string()
                } else {
                    from_bag_unviewable.push(c.topic.clone());
                    "  [defined by the bag, but the viewer cannot use that form]".to_string()
                }
            }
            SchemaReading::NamedButUndecodable { .. } => {
                skewed.push(c.topic.clone());
                "  [named, but this build cannot decode it]".to_string()
            }
            SchemaReading::Unresolvable => {
                unresolvable.push(c.topic.clone());
                "  [hash resolves to nothing here]".to_string()
            }
            SchemaReading::NoDescriptor => {
                // NOT folded in with `Unresolvable`: this channel
                // carries no Cerulion descriptor AT ALL, so it has no wire schema
                // hash to resolve and `open_route` refuses it outright. The
                // unresolvable note's remedy — re-record so the bag carries the
                // definitions, or point `DDS_BRIDGE_CONFIG` at a `msg_dirs` — is
                // about resolving a hash, and cannot help a channel that has none.
                no_descriptor.push(c.topic.clone());
                "  [no Cerulion descriptor — unplayable]".to_string()
            }
        };
        // A recorded name that DISAGREES with what the hash resolves to was
        // previously invisible: the row printed the bag's name with no marker at
        // all. It means the bag's writer and this reader are looking at two
        // different definitions of the same type name, which is precisely the
        // condition that renders nothing while looking fine.
        if let Some(resolved) = reading.name() {
            if c.schema_name != ATTACH_SCHEMA_NAME && c.schema_name != resolved {
                marker.push_str(&format!(
                    "  [recorded as this name, but the hash is {resolved} here]"
                ));
            }
        }
        out.push_str(&format!(
            "{:<40} {:>9}  {:>10}  {}{}\n",
            c.topic, c.frames, c.max_frame_len, shown, marker
        ));
    }
    if !from_bag.is_empty() {
        out.push_str(&format!(
            "\nnote: {} topic(s) carry a type this machine never compiled, and the BAG carries \
             its definition: {}.\n  \
             `cerulion bag play` hands those definitions to a running `cerulion-vizd`, so they \
             decode and render here without anyone installing a schema.\n",
            from_bag.len(),
            from_bag.join(", ")
        ));
    }
    if !from_bag_unviewable.is_empty() {
        out.push_str(&format!(
            "\nnote: {} topic(s) are defined by the bag in a form the VIEWER cannot read: {}.\n  \
             `cerulion topic echo` and `cerulion bag info` decode them from the bag's own \
             records, but `cerulion-vizd` seeds only ROS `.msg` definitions (it carries no \
             workspace-YAML parser), so Studio will render NOTHING for them. Express those \
             types as `schemas/<pkg>/msg/<Type>.msg` and re-record to make them renderable.\n",
            from_bag_unviewable.len(),
            from_bag_unviewable.join(", ")
        ));
    }
    if !from_bag_builtin_named.is_empty() {
        out.push_str(&format!(
            "\nnote: {} topic(s) are defined by the bag under the name of a BUILT-IN type: {}.\n  \
             `cerulion topic echo` and `cerulion bag info` decode them from the bag's own \
             records, but `cerulion-vizd` REFUSES a definition named after a built-in — a \
             side-load applies daemon-wide, so accepting one would change how every topic of \
             that type is decoded, including live robot taps. Studio will render NOTHING for \
             them. This is what a recording made inside a workspace that SHADOWS a built-in \
             (`schemas/<pkg>/msg/<Type>.msg` over a type Cerulion already compiles in) looks \
             like: rename the shadow to a package of your own and re-record.\n",
            from_bag_builtin_named.len(),
            from_bag_builtin_named.join(", ")
        ));
    }
    if !skewed.is_empty() {
        out.push_str(&format!(
            "\nnote: {} topic(s) name a type this build cannot decode: {}.\n  \
             The bag records what the type is CALLED but carries no text for it — which is what \
             happens when a recorder omits a definition it assumed every reader has, and this \
             build's copy of that type hashes differently. The frames still publish verbatim; \
             re-record on a machine whose message corpus matches, or install the matching \
             definition into the workspace `schemas/` store.\n",
            skewed.len(),
            skewed.join(", ")
        ));
    }
    if !unresolvable.is_empty() {
        // A NON-finalized bag has no readable attachment index, so its schema
        // provenance could not be LOOKED AT — which is not the same as the bag
        // not carrying any (the attachment bytes are written right after the
        // header, long before the summary). Saying "the bag says nothing about
        // it" there would send an operator to re-record a bag that already holds
        // the answer.
        let provenance_clause = if scan.finalized {
            "and that the bag says nothing about"
        } else {
            "and whose provenance could NOT BE READ (this bag is not finalized, so its \
             attachment index is missing — it may well carry the definitions)"
        };
        out.push_str(&format!(
            "\nnote: {} topic(s) carry a wire schema hash that resolves to NOTHING here {}: \
             {}.\n  \
             Their frames still publish byte-verbatim, so `cerulion topic echo` decodes them if \
             the type happens to be in the workspace `schemas/` store — but `cerulion viz` / \
             Studio will render NOTHING.\n  \
             The fix is to re-record from inside the workspace holding those `.msg` files — \
             `cerulion bag record`, or `cerulion graph run --record` if a graph produced them \
             (both verbs carry the definitions now) — so the bag describes itself and \
             plays anywhere. For a bag you cannot re-record, point DDS_BRIDGE_CONFIG at a \
             config whose `msg_dirs` includes that directory BEFORE vizd starts.\n",
            unresolvable.len(),
            provenance_clause,
            unresolvable.join(", ")
        ));
    }
    if !no_descriptor.is_empty() {
        out.push_str(&format!(
            "\nnote: {} topic(s) carry NO Cerulion descriptor at all: {}.\n  \
             They have no wire schema hash, so there is nothing to resolve and no provenance \
             the bag could carry for them — `cerulion bag play` refuses these topics by name \
             and plays the rest. They were recorded by a writer that does not stamp Cerulion \
             wire headers (a foreign MCAP, or a channel written outside `cerulion bag record`); \
             re-record through `cerulion bag record` to make them playable.\n",
            no_descriptor.len(),
            no_descriptor.join(", ")
        ));
    }
    out
}

fn open_bag(path: &Path) -> CliResult<BagReader> {
    if !path.exists() {
        return Err(CliError::Validation(format!(
            "bag '{}' does not exist",
            path.display()
        )));
    }
    BagReader::open(path).map_err(|e| bag_err(path, "cannot open bag", e))
}

// ---------------------------------------------------------------------------
// `cerulion bag play`
// ---------------------------------------------------------------------------

/// A topic wired up for playback.
struct PlayRoute {
    topic: String,
    injector: IngressInjector,
}

/// Play `path` onto local iceoryx2 SHM, wall-paced.
///
/// Brings the process transport up and delegates to
/// [`bag_play_with_manager`], which is the whole implementation.
pub fn bag_play(
    path: &Path,
    opts: PlayOptions,
    running: Arc<AtomicBool>,
    banner: &mut dyn std::io::Write,
) -> CliResult<PlaySummary> {
    // PREFLIGHT BEFORE THE TRANSPORT. Bringing the manager up registers an
    // iceoryx2 node and prints its lifecycle breadcrumbs, so a typo'd bag path
    // or a bad `--rate` would otherwise leave the operator reading two screens
    // of transport chatter before the one line that matters — and would create
    // process state for a run that was never going to happen. Both checks are
    // pure: an argument compare and a file open.
    validate_rate(opts.rate)?;
    drop(open_bag(path)?);

    let manager = TransportManager::init(TransportConfig {
        node_name: "cerulion_bag_play".to_string(),
        ..Default::default()
    })?;
    bag_play_with_manager(&manager, path, opts, running, banner)
}

/// [`bag_play`] over a caller-supplied transport — the seam tests drive so each
/// can own an isolated SHM root (`TransportManager::init_for_test`) instead of
/// the process singleton.
///
/// # The frame path
///
/// `mmap slice → publish_raw → loaned SHM slot`. The bag reader hands out a
/// borrowed slice straight out of the memory map ([`BagReader::frame`]) and
/// `publish_raw` copies it into the loaned shared-memory slot. That is **ONE
/// memcpy per frame** — exactly the count the production network-ingress
/// re-inject path pays, and the true number: it is not zero, because the
/// bytes must cross from a file mapping into the SHM segment a subscriber
/// reads. What it is NOT is a per-frame heap allocation: the hot loop allocates
/// nothing at steady state (pinned by an allocation probe).
///
/// # The determinism contract
///
/// Two plays of one bag publish byte-identical frames in an identical order.
/// The order is the bag's FILE order — which is the recorder's arrival order —
/// walked by [`BagReader::user_frames`]. That is deliberately not a sort on
/// `(log_time, channel, sequence)`: file order is already a total order fixed
/// by immutable bytes on disk (so it needs no comparator and cannot tie), it
/// reproduces what the recorder actually observed rather than a re-derived
/// ideal, and it costs no index (a sort would materialise a span per frame,
/// which a 300 GB bag turns into gigabytes). Pacing changes WHEN a frame is
/// published, never WHICH or in what order.
///
/// Under pacing lag nothing is dropped and nothing is reordered: the schedule
/// SLIPS, the frame publishes immediately, and the slippage is accounted and
/// reported at exit ([`PlaySummary::max_slip_ns`]).
pub fn bag_play_with_manager(
    manager: &Arc<TransportManager>,
    path: &Path,
    opts: PlayOptions,
    running: Arc<AtomicBool>,
    banner: &mut dyn std::io::Write,
) -> CliResult<PlaySummary> {
    let rate = validate_rate(opts.rate)?;
    let reader = open_bag(path)?;
    let scan = scan_bag(&reader, path)?;
    let selected = select_channels(&scan, &opts.topics)?;

    let walker = crate::topic_cmd::local_walker_from_workspace(opts.schemas_dir.as_deref());
    // The bag's own schema provenance. It changes what the banner can
    // truthfully say (a vendor type the bag defines is decodable here even
    // though this machine never compiled it) and, below, it is what `bag play`
    // hands to a running viewer.
    let catalog = reader.schema_catalog();
    write!(
        banner,
        "{}",
        render_scan(&scan, path, &walker, catalog.as_ref())
    )?;

    // Keep any desk viewer supplied with THIS bag's definitions for as
    // long as it plays. Spawns nothing when the bag carries none, and reaps
    // itself on EVERY exit path below — including the `?` on a banner write and
    // the two hard refusals — because it is an RAII guard rather than a reap
    // call somebody has to remember to keep reachable.
    //
    // Scoped to what is actually playing. Offering the whole
    // catalog under `--topics /one` hands the viewer definitions for types this
    // playback never publishes — and a side-load is DAEMON-WIDE, so a parseable
    // doc for an unrelated type can legitimately REPLACE a definition the daemon
    // holds, taking whatever was decoding the previous one dark. The player has
    // no business changing how a type it is not publishing gets decoded.
    // `closure_for_hashes` is exactly this operation (it is what the RECORDER
    // prunes with), so the selected channels' wire hashes resolve through the
    // catalog's own bindings to their docs plus the transitive `deps` closure.
    let offer_docs = catalog
        .as_ref()
        .map(|c| {
            c.closure_for_hashes(selected.iter().filter_map(|ch| ch.schema_hash))
                .docs
        })
        .unwrap_or_default();
    let _offer = SchemaOffer::start(offer_docs);

    if !scan.finalized {
        return Err(CliError::Validation(format!(
            "bag '{}' is not finalized ({}), so its frames cannot be walked — playback needs \
             the summary footer a clean recorder shutdown writes. `cerulion bag info` reports \
             what is readable. Re-record, stopping the recorder with Ctrl-C / SIGTERM rather \
             than killing it.",
            path.display(),
            scan.completeness
        )));
    }

    let mut routes: Vec<PlayRoute> = Vec::new();
    let mut refused: Vec<(String, String)> = Vec::new();
    for c in &selected {
        match open_route(manager, c) {
            Ok(Some(route)) => routes.push(route),
            Ok(None) => {}
            Err(reason) => {
                tracing::error!(topic = %c.topic, reason = %reason, "bag play: topic refused");
                refused.push((c.topic.clone(), reason));
            }
        }
    }

    if routes.is_empty() {
        let detail = if refused.is_empty() {
            "no selected topic carries any frame".to_string()
        } else {
            refused
                .iter()
                .map(|(t, r)| format!("  {t}: {r}"))
                .collect::<Vec<_>>()
                .join("\n")
        };
        return Err(CliError::Validation(format!(
            "bag play has nothing to publish from '{}':\n{detail}",
            path.display()
        )));
    }

    if !refused.is_empty() {
        writeln!(banner, "\nREFUSED (not played):")?;
        for (topic, reason) in &refused {
            writeln!(banner, "  {topic}: {reason}")?;
        }
    }

    writeln!(
        banner,
        "\nplaying {} topic(s) at rate {rate}{}. Attach a viewer now — `cerulion topic list` / \
         `cerulion topic echo <topic>` / `cerulion viz`; the topics stay open for the whole run. \
         A viewer that attaches mid-run sees only what is published AFTER it attaches (the \
         player retains no late-joiner history{}).\nCtrl-C to stop.\n",
        routes.len(),
        if opts.repeat { " (looping)" } else { "" },
        if opts.repeat {
            ", so --loop gives it the next pass"
        } else {
            ""
        }
    )?;
    tracing::info!(
        bag = %path.display(),
        topics = routes.len(),
        rate,
        looping = opts.repeat,
        "bag play: starting"
    );

    let started = std::time::Instant::now();
    let mut passes = 0u64;
    let mut max_slip_ns = 0u64;
    let mut slipped_frames = 0u64;
    let mut timeline_restarts = 0u64;
    // ONE anchored schedule PER CHANNEL (see `ChannelClock`). Declared OUTSIDE
    // the pass loop so a `--loop` wrap carries each channel's last stamp: the
    // wrap then reads as the regression it is (re-anchor, no sleep, no phantom
    // slip) rather than as a fresh anchor, which would publish the wrap's first
    // two frames back to back.
    let mut clocks: BTreeMap<u16, ChannelClock> = BTreeMap::new();
    // Per channel, the FIRST wire stamp of the CURRENT pass — the
    // origin the `--start-offset` / `--duration` bag-time bounds are measured
    // from. Cleared at a `--loop` wrap beside `clocks`, so each pass re-applies
    // its window to its own frames.
    let mut bag_time_origin: BTreeMap<u16, u64> = BTreeMap::new();
    // Channels whose `--duration` window has CLOSED in the
    // CURRENT pass. Cleared at a `--loop` wrap beside `bag_time_origin`, so the
    // bound re-opens per pass exactly as the origin does.
    //
    // The bound is a comparison against `ts - first`, which a stamp BELOW
    // `first` collapses to 0 — and a stamp below `first` is not exotic, it is
    // the epoch reset (a restarted producer's gating clock begins again
    // near zero) recorded on the topic it happened to. Without the latch a
    // channel that had already run past its window was ADMITTED AGAIN by the
    // first frame of the new epoch and every frame after it, so `--duration`
    // silently republished past the end the operator asked for.
    //
    // A latch is sound because it can never close a channel early: within one
    // producer epoch a channel's stamps are non-decreasing, so once one frame
    // is past the bound every LATER frame of that epoch is too, and the only
    // way a later frame reads as inside the window is the regression this
    // refuses.
    let mut bag_window_closed: std::collections::BTreeSet<u16> = std::collections::BTreeSet::new();
    // The wall instant the run's FIRST frame was reached. Every channel's
    // schedule starts here, so channels share one origin (see `pace_step`).
    let mut run_origin_ns: Option<u64> = None;
    // Set at a `--loop` wrap, consumed by the first frame the new pass paces.
    let mut wrap_pending = false;
    // Channels that had to FAST-FORWARD — i.e. that hit the catch-up rate
    // bound at least once. A SET, so it is a channel count rather than an event
    // count even across `--loop` passes.
    let mut catchup_channels: std::collections::BTreeSet<u16> = std::collections::BTreeSet::new();
    // Channels that actually PUBLISHED — the only ones whose schedule describes
    // the run's ideal duration (M1).
    let mut played_channels: std::collections::BTreeSet<u16> = std::collections::BTreeSet::new();

    'outer: loop {
        if !running.load(Ordering::Relaxed) {
            break;
        }
        if passes > 0 {
            // B3: a `--loop` wrap RESTARTS the pass, so every channel's
            // schedule must be rebuilt from a fresh origin exactly as pass 1
            // built it.
            //
            // Carrying the clocks across the wrap looked right (it kept the
            // wrap from reading as a fresh anchor) but was catastrophic: at the
            // wrap every channel regresses, each re-anchors to `elapsed`, and
            // that COLLAPSES the per-channel deficits that made pass 1 finish on
            // time. From pass 2 the first-batch tap groups then pace
            // SEQUENTIALLY. MEASURED on an 8-tap 100 Hz bag: 590 ms for pass 1,
            // 1220 ms for every later pass (2.07x), with the overrun
            // accumulating without bound until a healthy machine is told to
            // lower --rate.
            //
            // Clearing them makes pass 2 structurally identical to pass 1. The
            // wrap is still counted as the timeline restart it is — the bag's
            // clock really does go backwards — it is just counted HERE, once,
            // rather than once per channel with the deficits destroyed.
            clocks.clear();
            // Each pass re-measures its bag-time window from its
            // own first frame, so the window applies to every pass rather than
            // silently truncating only the first.
            bag_time_origin.clear();
            bag_window_closed.clear();
            run_origin_ns = None;
            // Counted when the new pass reaches its first STAMPED frame, not
            // here: a wrap interrupted before that point is a restart nobody
            // observed. (A frame with no parseable header does not count — it
            // carries no timing, so it cannot evidence a timeline restart.)
            wrap_pending = true;
        }
        // Resolved lazily per pass: channel ids are stable within a bag, but a
        // fresh walk re-reads the table, so this stays local to the pass.
        let mut route_of_channel: Vec<Option<usize>> = Vec::new();
        let mut walk = reader
            .user_frames()
            .map_err(|e| bag_err(path, "cannot walk the frames of bag", e))?;
        loop {
            if !running.load(Ordering::Relaxed) {
                break 'outer;
            }
            let next = match walk.next_user_frame() {
                Ok(Some(v)) => v,
                Ok(None) => break,
                Err(e) => {
                    tracing::warn!(
                        error = %e,
                        "bag play: the frame walk ended early — playing stopped at the last \
                         readable frame of this pass"
                    );
                    break;
                }
            };
            let (channel_id, span) = next;
            let idx = usize::from(channel_id);
            if route_of_channel.len() <= idx {
                route_of_channel.resize(idx + 1, None);
            }
            // `Vec<Option<usize>>` indexed by channel id: one branch, no hashing
            // and no allocation in the steady state (the resize happens once per
            // channel, on its first frame).
            let route_idx = match route_of_channel[idx] {
                Some(r) => Some(r),
                None => {
                    let topic = walk.topic(channel_id);
                    let found = routes.iter().position(|r| r.topic == topic);
                    // `None` is cached too — a filtered-out or refused channel
                    // is resolved once, not on every frame.
                    route_of_channel[idx] = found;
                    found
                }
            };
            let frame = reader.frame(&span);
            let stamp = WireHeader::read_from_buf(frame).map(|h| h.timestamp_ns);
            // The BAG-TIME window (`--start-offset` / `--duration`).
            //
            // Measured against THIS CHANNEL's own first stamp, never against a
            // shared origin: wire stamps in different channels are different
            // producers' clocks and comparing them is the class this module
            // refuses everywhere else. A skipped frame touches NO pacing state,
            // so the first frame actually played anchors the channel — which is
            // what makes a seek a seek rather than a silent sleep through the
            // prefix. A headerless frame carries no bag time and is therefore
            // never skipped by a bound (it is played verbatim, in order, exactly
            // as an unbounded run plays it).
            if let Some(ts) = stamp {
                if opts.start_offset_ns.is_some() || opts.duration_bound_ns.is_some() {
                    // Asked BEFORE the arithmetic, because the arithmetic is
                    // exactly what a regressing stamp defeats (see
                    // `bag_window_closed`).
                    if bag_window_closed.contains(&channel_id) {
                        continue;
                    }
                    // A SEPARATE map, deliberately not a `ChannelClock` field:
                    // the pacing code below creates that entry with
                    // `or_insert(ChannelClock { schedule_ns: origin, .. })`, so
                    // an entry this check created first would silently give the
                    // channel a zero origin and no anchor.
                    let first = *bag_time_origin.entry(channel_id).or_insert(ts);
                    // A `--loop` wrap regresses the stamp; the pass restart
                    // clears `clocks`, so each pass re-measures from its own
                    // first frame and the bounds re-apply per pass.
                    let bag_elapsed = ts.saturating_sub(first);
                    let from = opts.start_offset_ns.unwrap_or(0);
                    if bag_elapsed < from {
                        continue;
                    }
                    if let Some(d) = opts.duration_bound_ns {
                        // HALF-OPEN `[from, from + D)`, from the SAME rule the
                        // resim half applies (`replay_rank::
                        // beyond_duration_bound`). A `> d` here
                        // against a `>= d` in the resim loop, with `docs/bag.md`
                        // documenting half-open for BOTH, would make `--duration 0`
                        // republish each channel's first frame on this half
                        // and execute nothing on the other.
                        if crate::replay_rank::beyond_duration_bound(
                            bag_elapsed.saturating_sub(from),
                            d,
                        ) {
                            // This channel is past its window. Skipping rather
                            // than breaking the walk keeps the OTHER channels'
                            // windows correct — they may still have frames due,
                            // and their timelines are independent.
                            //
                            // LATCHED for the rest of the pass: the verdict is
                            // about the CHANNEL, not about this frame, and a
                            // later frame can only re-enter the window by
                            // regressing its stamp.
                            bag_window_closed.insert(channel_id);
                            continue;
                        }
                    }
                }
            }
            let elapsed_ns = started.elapsed().as_nanos().min(u128::from(u64::MAX)) as u64;
            let Some(route_idx) = route_idx else {
                // Filtered out (`--topics`) or REFUSED (an occupied publisher
                // slot). Its stamp still advances ITS OWN domain's timeline, so
                // the domain's other topics keep their relative pacing — and it
                // is paced against the REAL elapsed clock, not a hardcoded 0,
                // which would make the re-anchor a no-op (`max(schedule, 0)`)
                // and leave the phantom-lag defect live whenever the regressing
                // channel is the skipped one. Its restarts are COUNTED, so the
                // operator does not lose the counter that explains the timing.
                if let Some(ts) = stamp {
                    let origin = *run_origin_ns.get_or_insert(elapsed_ns);
                    // The deficit is carried IN FULL — see `CATCHUP_FACTOR` for
                    // why capping it was wrong. What is bounded is the RATE at
                    // which a behind channel may publish, applied below.
                    let clock = clocks.entry(channel_id).or_insert(ChannelClock {
                        schedule_ns: origin,
                        anchor_ns: Some(origin),
                        ..ChannelClock::default()
                    });
                    let step = pace_step(
                        elapsed_ns,
                        &mut clock.schedule_ns,
                        clock.prev_stamp,
                        ts,
                        rate,
                    );
                    clock.prev_stamp = Some(ts);
                    clock.cumulative_ns = clock.cumulative_ns.saturating_add(step.advance_ns);
                    if step.re_anchored {
                        timeline_restarts += 1;
                    }
                }
                continue;
            };
            // A headerless frame contributes no timing (it cannot); it is still
            // published verbatim, in order.
            let step = match stamp {
                Some(ts) => {
                    if wrap_pending {
                        wrap_pending = false;
                        timeline_restarts += 1;
                    }
                    let origin = *run_origin_ns.get_or_insert(elapsed_ns);
                    // The deficit is carried IN FULL — see `CATCHUP_FACTOR` for
                    // why capping it was wrong. What is bounded is the RATE at
                    // which a behind channel may publish, applied below.
                    let clock = clocks.entry(channel_id).or_insert(ChannelClock {
                        schedule_ns: origin,
                        anchor_ns: Some(origin),
                        ..ChannelClock::default()
                    });
                    let s = pace_step(
                        elapsed_ns,
                        &mut clock.schedule_ns,
                        clock.prev_stamp,
                        ts,
                        rate,
                    );
                    clock.prev_stamp = Some(ts);
                    clock.cumulative_ns = clock.cumulative_ns.saturating_add(s.advance_ns);
                    s
                }
                None => PaceStep {
                    sleep_ns: 0,
                    slip_ns: 0,
                    re_anchored: false,
                    advance_ns: 0,
                },
            };
            if step.slip_ns > 0 {
                slipped_frames += 1;
                max_slip_ns = max_slip_ns.max(step.slip_ns);
            }
            if step.re_anchored {
                timeline_restarts += 1;
            }
            // The catch-up RATE bound. A frame whose schedule has already
            // passed still waits until this channel's OWN previous publish plus
            // its own recorded delta divided by `CATCHUP_FACTOR`, so a channel
            // that is far behind fast-forwards at a bounded multiple of its
            // recorded rate instead of dumping. Uses only this channel's own
            // cadence — no cross-channel arithmetic.
            let mut sleep_ns = step.sleep_ns;
            if sleep_ns == 0 && step.advance_ns > 0 {
                // Only a DEEP backlog is rate-bounded. A channel within
                // `CATCHUP_FREE_DELTAS` of its own schedule publishes at once —
                // that is ordinary write-batching catch-up, and spacing it would
                // impose an aggregate throughput ceiling on the single-threaded
                // player (see `CATCHUP_FREE_DELTAS`).
                let behind_deltas = step.slip_ns / step.advance_ns;
                if behind_deltas > CATCHUP_FREE_DELTAS {
                    if let Some(last_pub) = clocks.get(&channel_id).and_then(|c| c.last_publish_ns)
                    {
                        let earliest = last_pub.saturating_add(step.advance_ns / CATCHUP_FACTOR);
                        if earliest > elapsed_ns {
                            sleep_ns = earliest - elapsed_ns;
                            catchup_channels.insert(channel_id);
                        }
                    }
                }
            }
            if let Some(c) = clocks.get_mut(&channel_id) {
                c.last_publish_ns = Some(elapsed_ns.saturating_add(sleep_ns));
            }
            if !sleep_interruptible(sleep_ns, &running) {
                break 'outer;
            }
            // The ONE memcpy: `frame` borrows the mmap; `publish_raw` copies it
            // into the loaned SHM slot. Outcomes are counted by the injector
            // (and its own flood latch keeps a failure regime from flooding).
            played_channels.insert(channel_id);
            let _ = routes[route_idx].injector.reinject_raw(frame);
        }
        passes += 1;
        if !opts.repeat {
            break;
        }
    }

    let topics: Vec<TopicPlayback> = routes
        .iter()
        .map(|r| {
            let s = r.injector.stats();
            TopicPlayback {
                topic: r.topic.clone(),
                injected: s.frames,
                rejected: s.schema_mismatch_drops + s.decode_errors,
                failed: s.reinject_failure_drops,
            }
        })
        .collect();

    // The run's IDEAL duration is the LATEST wall instant any channel's final
    // frame was due — `anchor + cumulative`, per channel, maxed. The anchor term
    // is load-bearing: a channel that only starts publishing partway through the
    // recording has a small cumulative span but a late anchor, and ignoring the
    // offset would under-state the ideal duration and report a false overrun on
    // any bag with staggered topic activity.
    // M1: only PLAYED channels may contribute. A filtered (`--topics`) or
    // refused channel's clock still advances — it has to, so the domain's other
    // topics keep their relative pacing — but counting it here would let
    // `play --topics /small` out of a 60 s bag report a 60 s ideal duration, so
    // a genuinely 20 s-lagging run would show no overrun and be told "no action
    // needed".
    let scheduled_span_ns = clocks
        .iter()
        .filter(|(ch, _)| played_channels.contains(*ch))
        .map(|(_, c)| c.anchor_ns.unwrap_or(0).saturating_add(c.cumulative_ns))
        .max()
        .unwrap_or(0);
    let summary = PlaySummary {
        topics,
        passes,
        refused,
        elapsed: started.elapsed(),
        max_slip_ns,
        slipped_frames,
        timeline_restarts,
        scheduled_span_ns,
        catchup_channels: catchup_channels.len() as u64,
    };
    tracing::info!(
        passes = summary.passes,
        published = summary.total_injected(),
        slipped_frames = summary.slipped_frames,
        max_slip_ms = summary.max_slip_ns / 1_000_000,
        timeline_restarts = summary.timeline_restarts,
        "bag play: stopped"
    );
    Ok(summary)
}

/// How often the player re-offers the bag's schema definitions to a viewer.
///
/// It must REPEAT, not fire once, because of the order the two commands run in:
/// `cerulion viz <topic>` can only attach to a topic that already exists, so the
/// player necessarily starts FIRST — often before any `cerulion-vizd` is
/// running, since `cerulion viz` is what spawns one. A single offer at startup
/// would therefore miss the daemon that ends up doing the rendering.
///
/// The re-offer is free at the daemon: seeding definitions it already holds is a
/// no-op that rebuilds nothing (`SeedState::absorb`).
const SCHEMA_OFFER_INTERVAL: Duration = Duration::from_millis(2_000);

/// The running schema-offer thread, reaped by `Drop`.
///
/// An RAII guard rather than a reap call at the end of `bag_play_with_manager`,
/// because that function has SIX fallible exits between the spawn point and the
/// end — two `return Err` (the not-finalized refusal, the no-playable-channel
/// refusal), THREE `writeln!(banner, …)?`, and the `?` on the frame walk's
/// `bag_err` — and a reap that any of them skips leaves a thread re-connecting
/// to the viewer socket every two seconds for the life of the process. That is
/// bounded for the CLI binary (it exits) and unbounded for
/// `bag_play_with_manager` called as a LIBRARY, which is how the tests and any
/// embedder drive it. A guard also cannot be defeated by the next early return
/// somebody adds — which is the point, and is why the count above is a
/// description of today rather than the argument.
struct SchemaOffer {
    offering: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl SchemaOffer {
    fn start(docs: Vec<cerulion_core::SchemaDoc>) -> Self {
        let offering = Arc::new(AtomicBool::new(true));
        let thread = spawn_schema_offer(docs, Arc::clone(&offering));
        SchemaOffer { offering, thread }
    }
}

impl Drop for SchemaOffer {
    fn drop(&mut self) {
        // Clearing the flag it watches is what makes it exit. It sleeps in short
        // slices against that same flag, so a thread that is SLEEPING joins
        // immediately.
        //
        // BOUND: the flag is not the only thing it waits on. A thread
        // inside `VizdConn::connect` or `push_schemas` is bounded by
        // `CONN_IO_TIMEOUT` instead, so against a daemon that accepts and never
        // writes — exactly the case those timeouts exist for — teardown can take
        // up to that long. Bounded and rare, and shortening it would mean either
        // a detached thread (which is what the guard exists to prevent) or
        // non-blocking socket I/O for a 2 s-cadence background offer.
        self.offering.store(false, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            // A PANICKED offer thread is reported, not swallowed: playback then
            // ran un-offered, and `announced` means no other line was ever
            // emitted about it. `Drop` cannot fail, so this is all that can be
            // done — but silence here would leave a viewer rendering nothing
            // with no trace of why.
            if let Err(panic) = t.join() {
                let what = panic
                    .downcast_ref::<&str>()
                    .map(|s| (*s).to_string())
                    .or_else(|| panic.downcast_ref::<String>().cloned())
                    .unwrap_or_else(|| "<non-string panic payload>".to_string());
                tracing::error!(
                    panic = %what,
                    "bag play: the schema-offer thread PANICKED — the bag played, but any \
                     cerulion-vizd was left without the definitions it carries, so those \
                     topics rendered NOTHING"
                );
            }
        }
    }
}

/// Keep a desk's viewer supplied with the definitions THIS BAG carries, for as
/// long as it plays.
///
/// The daemon builds its schema walker at boot from the built-in corpus plus the
/// bridge config's `msg_dirs`, so a vendor type from a recorded bag resolves to
/// nothing and renders nothing: the original defect, whose failure is
/// success-shaped (the attach reports `ok: true`). The bag carries those
/// definitions, and this hands them over.
///
/// Deliberately CONNECT-ONLY: it never spawns a daemon. `bag play` is also how
/// frames are fed to `topic echo`, to tests, and to anything else on this
/// machine; a viewer daemon appearing as a side effect of playing a bag would be
/// a surprise, and one that never gets used is pure cost. If no viewer is
/// running there is nothing to teach, and the next sweep will find one when
/// there is.
///
/// Returns `None` — spawning no thread at all — when the bag carries no
/// definitions, so an all-built-in recording pays exactly nothing.
fn spawn_schema_offer(
    docs: Vec<cerulion_core::SchemaDoc>,
    offering: Arc<AtomicBool>,
) -> Option<std::thread::JoinHandle<()>> {
    // Offer only what the daemon can actually seed. It parses ROS `.msg`
    // definitions and nothing else — the workspace-YAML parser lives in this
    // crate, which the daemon must not depend on — so a YAML-encoded definition
    // would be accepted-then-skipped, inflating the daemon's `accepted` count
    // while nothing became renderable. Say it here instead; the banner says the
    // same thing to the operator.
    let (docs, unviewable): (Vec<_>, Vec<_>) = docs
        .into_iter()
        .partition(|d| d.encoding == cerulion_core::SchemaEncoding::Msg);
    if !unviewable.is_empty() {
        tracing::warn!(
            count = unviewable.len(),
            types = ?unviewable.iter().map(|d| &d.qualified).collect::<Vec<_>>(),
            "bag play: the bag defines these types in workspace-YAML form, which cerulion-vizd \
             cannot seed — `cerulion topic echo` still decodes them, but Studio will render \
             NOTHING for them"
        );
    }
    if docs.is_empty() {
        return None;
    }
    std::thread::Builder::new()
        .name("cer-bag-schema-offer".to_string())
        .spawn(move || {
            let socket = crate::viz_client::vizd_socket_path();
            let mut id = 1u64;
            let mut announced = false;
            let mut refusal_announced = false;
            let mut rejection_announced = false;
            // The LAST connect-failure kind reported, so an ANOMALOUS failure is
            // logged once per regime rather than every 2 s for a whole playback —
            // the same one-shot-per-condition shape the three latches above use,
            // keyed on `ErrorKind` so a daemon that dies mid-playback (NotFound →
            // ConnectionRefused) is still reported when the condition CHANGES.
            let mut connect_failure_announced: Option<std::io::ErrorKind> = None;
            while offering.load(Ordering::Relaxed) {
                // No daemon running is the ORDINARY case for most of a playback
                // (the viewer usually starts later), so it is SILENT. A daemon
                // that answers and REFUSES is different in kind — it will refuse
                // every retry too — so it is reported once, loudly.
                match crate::viz_client::VizdConn::connect(&socket) {
                    // ENOENT: no socket, i.e. no viewer. The ordinary case, and
                    // the one this thread exists to keep re-trying through.
                    Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                        connect_failure_announced = None;
                    }
                    // Anything else means a socket IS there and we could not talk
                    // to it: a dead daemon's stale socket (ECONNREFUSED), a
                    // permissions problem, a foreign listener whose banner will
                    // not parse (InvalidData), or one that accepts and never
                    // writes (TimedOut — the case `VizdConn::connect` arms its
                    // I/O timeouts for). None of those self-heal by retrying, and
                    // before this arm they produced no output at ANY level.
                    Err(e) => {
                        if connect_failure_announced != Some(e.kind()) {
                            connect_failure_announced = Some(e.kind());
                            tracing::debug!(
                                socket = %socket.display(),
                                kind = ?e.kind(),
                                error = %e,
                                "bag play: could not reach cerulion-vizd at its socket — the bag \
                                 still plays, but a viewer there will not learn the types it \
                                 carries. This is NOT the ordinary 'no viewer running' case \
                                 (that is silent); something is listening and would not talk"
                            );
                        }
                    }
                    Ok(mut conn) => {
                        connect_failure_announced = None;
                        match conn.push_schemas(id, &docs) {
                            Ok(SchemaPushOutcome::Applied { accepted, rejected }) => {
                                if accepted > 0 && !announced {
                                    announced = true;
                                    tracing::info!(
                                        accepted,
                                        offered = docs.len(),
                                        "bag play: handed the bag's schema definitions to \
                                     cerulion-vizd — its topics decode and render here"
                                    );
                                }
                                if !rejected.is_empty() && !rejection_announced {
                                    rejection_announced = true;
                                    tracing::warn!(
                                        types = ?rejected,
                                        "bag play: cerulion-vizd could not use some of the bag's \
                                         definitions — those topics will render NOTHING (its log \
                                         names the reason for each)"
                                    );
                                }
                            }
                            Ok(SchemaPushOutcome::Refused { reason }) => {
                                if !refusal_announced {
                                    refusal_announced = true;
                                    tracing::warn!(
                                        reason = %reason,
                                        "bag play: cerulion-vizd REFUSED the bag's schema \
                                         definitions — most often a daemon older than this CLI, \
                                         which does not know the `schemas` verb. Its topics will \
                                         render NOTHING until it is restarted from this build"
                                    );
                                }
                            }
                            Err(e) => tracing::debug!(
                                error = %e,
                                "bag play: could not offer the bag's schemas to cerulion-vizd — \
                                 retrying while the bag plays"
                            ),
                        }
                        id += 1;
                    }
                }
                // Sleeps in short slices against the SAME flag, so the thread
                // exits promptly when playback ends rather than up to one whole
                // interval later.
                if !sleep_interruptible(SCHEMA_OFFER_INTERVAL.as_nanos() as u64, &offering) {
                    break;
                }
            }
        })
        .map_err(|e| {
            tracing::warn!(
                error = %e,
                "bag play: could not start the schema-offer thread — the bag still plays, but a \
                 viewer will not learn the types it carries"
            );
        })
        .ok()
}

/// Wire one channel up for playback.
///
/// `Ok(None)` = nothing to play (an empty channel), `Err(reason)` = a LOUD
/// per-topic refusal the caller reports by name and continues past.
fn open_route(
    manager: &Arc<TransportManager>,
    c: &ChannelScan,
) -> Result<Option<PlayRoute>, String> {
    if c.frames == 0 {
        return Ok(None);
    }
    let Some(schema_hash) = c.schema_hash else {
        return Err(
            "the channel carries no Cerulion schema descriptor, so its frames cannot be \
             validated against a schema hash (is this a bag Cerulion wrote?)"
                .to_string(),
        );
    };

    // Refuse a taken slot BEFORE taking one, so the message names the real
    // cause instead of an iceoryx2 port-creation error. `None` = the probe
    // itself failed (UNKNOWN, never "dead"), so we proceed and let the port cap
    // be the backstop — which it also is for the TOCTOU race where a publisher
    // attaches between this probe and our create.
    if let Some(n) = manager.topic_publisher_count_checked(&c.topic) {
        if n > 0 {
            return Err(format!(
                "{n} publisher(s) are already publishing this topic on this machine — a bag \
                 player must not compete with a live producer for the same topic. Stop the live \
                 producer, or select other topics with --topics."
            ));
        }
    }

    // Size the SHM slot from the largest frame the scan actually saw. Frames are
    // validated (`total_size == len`) before publishing, so nothing can exceed
    // this at run time.
    let slot = c.max_frame_len.max(WireHeader::SIZE);
    let max_slice_len = u32::try_from(slot)
        .ok()
        .and_then(MaxSliceLen::try_new)
        .ok_or_else(|| format!("the largest recorded frame ({slot} bytes) exceeds u32"))?;

    manager
        .create_ingress_injector(&c.topic, schema_hash, max_slice_len)
        .map(|injector| {
            Some(PlayRoute {
                topic: c.topic.clone(),
                injector,
            })
        })
        .map_err(|e| e.to_string())
}

/// Sleep `ns`, waking every [`SLEEP_SLICE`] to re-check `running`. Returns
/// `false` if the shutdown flag cleared (the caller must stop).
fn sleep_interruptible(ns: u64, running: &AtomicBool) -> bool {
    if ns == 0 {
        return running.load(Ordering::Relaxed);
    }
    let mut remaining = Duration::from_nanos(ns);
    while !remaining.is_zero() {
        if !running.load(Ordering::Relaxed) {
            return false;
        }
        let slice = remaining.min(SLEEP_SLICE);
        std::thread::sleep(slice);
        remaining -= slice;
    }
    running.load(Ordering::Relaxed)
}

/// Human-readable run summary for [`bag_play`].
pub fn render_play_summary(summary: &PlaySummary) -> String {
    let mut out = String::new();
    out.push_str(&format!(
        "\nbag play stopped after {} pass(es) in {:.3}s.\n",
        summary.passes,
        summary.elapsed.as_secs_f64()
    ));
    out.push_str(&format!(
        "{:<40} {:>9}  {:>9}  {:>9}\n",
        "TOPIC", "PLAYED", "REJECTED", "FAILED"
    ));
    for t in &summary.topics {
        out.push_str(&format!(
            "{:<40} {:>9}  {:>9}  {:>9}\n",
            t.topic, t.injected, t.rejected, t.failed
        ));
    }
    out.push_str(&format!("total published: {}\n", summary.total_injected()));
    if summary.slipped_frames > 0 {
        // Slip alone does NOT mean the machine fell behind, and telling the
        // operator to lower --rate when it did not is the same false alarm the
        // first-frame anchor removed.
        //
        // A recorder writes each flush TAP-GROUPED, so a bag's frames are not in
        // global arrival order: within a flush window every topic's frames sit
        // together. Publishing in recorded order therefore reaches a later
        // topic's frames after the earlier topics' have been paced, and those
        // frames are legitimately "past their own target" by up to one flush
        // window. That is STRUCTURAL — a property of how the bag was written —
        // and it costs nothing: the run still finishes on time.
        //
        // The discriminator is the WALL OVERRUN. Slip WITH an overrun is the
        // machine failing to keep up; slip WITHOUT one is batch phase.
        if summary.wall_overrun_ns().is_some() {
            out.push_str(&format!(
                "pacing: {} frame(s) published behind schedule, worst {:.3}ms late, AND the run \
                 overran (below). Nothing was dropped or reordered — the schedule slipped. Lower \
                 --rate, or play fewer topics.\n",
                summary.slipped_frames,
                summary.max_slip_ns as f64 / 1e6
            ));
        } else {
            out.push_str(&format!(
                "pacing: {} frame(s) published behind their own topic's target, worst {:.3}ms — \
                 but the run did NOT overrun, so this is STRUCTURAL, not lag on this machine. A \
                 recorder writes each flush grouped by topic, so publishing in recorded order \
                 reaches a later topic's frames after the earlier ones are paced (bounded by the \
                 recorder's flush window). Every topic still plays at its recorded rate. No \
                 action needed.\n",
                summary.slipped_frames,
                summary.max_slip_ns as f64 / 1e6
            ));
        }
    }
    if summary.catchup_channels > 0 {
        out.push_str(&format!(
            "fast-forward: {} channel(s) had frames reached well after their scheduled \
             instant at some point in this run, and caught up at a BOUNDED \
             {CATCHUP_FACTOR}x their own recorded rate rather than dumping a backlog. \
             This describes THIS RUN, not the recording — it happens both when a \
             producer handed over mid-recording and when the recorder's first flush \
             was large.\n",
            summary.catchup_channels
        ));
    }
    if summary.timeline_restarts > 0 {
        out.push_str(&format!(
            "timeline: a channel's stamps stepped BACKWARDS {} time(s), so its schedule was \
             re-anchored there. That is a property of the RECORDING (a producer restart, a \
             --loop wrap) — not lag on this machine.\n",
            summary.timeline_restarts
        ));
    }
    // The overrun is the measure a re-anchor CANNOT reset. `max_slip_ns`
    // restarts at every timeline discontinuity, so on a bag full of them it
    // reports lag since the last restart and can read far below the run's real
    // overrun; this compares the wall against the recording's own duration.
    if let Some(overrun) = summary.wall_overrun_ns() {
        out.push_str(&format!(
            "overrun: the run took {:.3}s for a {:.3}s recording — {:.3}s BEHIND. This is the \
             measure a timeline restart cannot reset (unlike the worst-slip figure above). Lower \
             --rate, or play fewer topics.\n",
            summary.elapsed.as_secs_f64(),
            summary.scheduled_span_ns as f64 / 1e9,
            overrun as f64 / 1e9,
        ));
    }
    for (topic, reason) in &summary.refused {
        out.push_str(&format!("refused {topic}: {reason}\n"));
    }
    out
}

// ---------------------------------------------------------------------------
// `cerulion bag record`
// ---------------------------------------------------------------------------
//
// This verb is a FRONT-END over `cerulion_bagd`, not a second recorder.
//
// `cerulion_bagd` already IS the production capture path: listener-less
// data-only taps, the zero-copy `writev` MCAP writer, a backlog-aware drive
// loop, per-topic loss accounting (`dropped_unwritten` / `frames_lost` /
// `gap_events` / `headerless`), byte-deterministic chunk-indexed output, the
// `__cerulion/record_health.json` attachment, size-cap rotation, and a
// finalize-on-shutdown contract. Everything below derives a TOPIC SET and hands
// it to that machinery through `BagdConfig` + `run_bagd`. There is no drain loop
// and no writer here.
//
// The verb's own surface is deliberately shaped after `ros2 bag record` (our
// parity bar) — see `derive_record_topics` for the selection rules
// and `docs/bag.md` for the flag-by-flag map, including the ros2 flags that are
// deliberately NOT mirrored and why.
//
// Local only. `bag record` taps this machine's shared memory
// and never pulls a topic's frames across the network. Recording a robot's topics means running this
// verb ON the robot and transferring the file afterwards. A named topic that is
// not locally visible is a loud refusal that says exactly that — never a silent
// network fallback that would pull every frame of every topic across the wire.

/// The literal schema name an attach-mode recorder writes when the wire header
/// gave it a hash but nothing gave it a name (`cerulion_bagd`'s
/// `ATTACH_MODE_SCHEMA_NAME`). Re-exported here because `bag info` / `bag play`
/// render it and must agree on the token.
pub const ATTACH_SCHEMA_NAME: &str = "unknown";

/// Topic-name prefixes never auto-selected by `--all` / `--regex`.
///
/// `__cerulion/` is the bag format's own reserved namespace: `BagWriter`
/// rejects it as a user topic outright. `/bagd/` is the recorder's OWN status
/// channel: auto-recording it means a recorder recording itself, which grows
/// without bound and tells the operator nothing.
///
/// These filter the AUTOMATIC selections only. Naming one explicitly still
/// reaches the writer's own refusal, which is the right place to say no.
///
/// This is an ALIAS of `topic_cmd::INTERNAL_TOPIC_PREFIXES`, the ONE prefix
/// list of "the framework's own channels" in this crate. The auto-selection
/// itself (`auto_selectable`) does not walk this slice: it calls
/// `topic_cmd::is_internal_topic`, the predicate `topic list` hides rows with,
/// which is this list PLUS the bare `/__cerulion` namespace token (a token no
/// prefix can spell). One function serves both verbs, so the set `topic list`
/// hides by default IS the set `bag record --all` / `--regex` never selects.
///
/// `cerulion_bagd::EXCLUDED_TOPIC_PREFIXES` is the SAME prefix list,
/// hand-copied into that crate for bagd's live-discovery planner, and the docs
/// on both sides say so. That planner walks the prefixes alone, so the bare
/// `/__cerulion` token is the one name it would not skip; no producer creates a
/// data service under that exact name today (the gateway refuses it as robot
/// egress), so the difference is not observable yet. What actually holds the two
/// crates together is `the_two_excluded_prefix_lists_are_the_same_list` below,
/// an equality test, not the `const _` beneath, which only pins the
/// `__cerulion/` element against `cerulion_bag`'s reserved prefix and would
/// stay green if either list grew a fourth entry the other lacked.
pub const AUTO_SELECT_EXCLUDED_PREFIXES: &[&str] = crate::topic_cmd::INTERNAL_TOPIC_PREFIXES;

// L5: the reserved prefix is owned by `cerulion_bag`; pin the shared list against
// it so a change there cannot silently leave the list stale. NOTE the narrow
// scope: this asserts ONE element's spelling, never that the two crates' lists
// agree (see the doc above).
const _: () = assert!(
    matches!(RESERVED_PREFIX.as_bytes(), b"__cerulion/"),
    "topic_cmd::INTERNAL_TOPIC_PREFIXES hardcodes the reserved prefix; update both together"
);

/// How the topic set for a recording was chosen.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TopicSelection {
    /// An explicit positional list.
    Explicit,
    /// `--all`: every live local topic.
    All,
    /// `--regex`: live local topics matching a pattern.
    Regex,
    /// `--run` — the topics the attached run DECLARES.
    ///
    /// It is its own variant rather than [`Explicit`](Self::Explicit) because
    /// the two differ on exactly one rule that matters: a named topic that
    /// cannot be tapped is a REFUSAL (the operator asked for it), while a
    /// declared one that has no live producer costs that topic and nothing
    /// else. A graph legitimately holds outputs that have not published yet, or
    /// nodes that never fire in this configuration, and refusing to record a
    /// live run because one of its declared topics is quiet would make `--run`
    /// unusable on precisely the graphs it exists for.
    Run,
}

/// Options for [`bag_record`].
#[derive(Debug, Clone)]
pub struct RecordOptions {
    /// Explicit topics (empty when `--all` / `--regex` chose the set).
    pub topics: Vec<String>,
    /// Select every genuinely-local live topic. A `cerulion-netd` MIRROR of
    /// a remote robot's topic is NOT one, and is never auto-selected.
    pub all: bool,
    /// Select genuinely-local live topics matching this pattern. Mirrors are
    /// excluded from the candidate set, exactly as under `--all`.
    pub regex: Option<String>,
    /// Drop any topic matching one of these patterns — from BOTH halves of the
    /// recording, not only the selected set.
    ///
    /// The patterns travel down to the recorder as
    /// `BagdConfig::discovery_excludes`, so they also bound what live-service
    /// DISCOVERY may attach — including a co-tenant topic this selection never
    /// named. That is the whole point: with discovery ON for `--run`, filtering
    /// only the selection let the rescan re-find an excluded topic as an
    /// "undeclared live producer" and give it a channel. An excluded topic is
    /// neither recorded nor reported as missing; when discovery caught it, the
    /// manifest says so with
    /// [`UntappedReason::ExcludedByRequest`](cerulion_bagd::UntappedReason::ExcludedByRequest)
    /// naming the pattern that matched.
    pub exclude: Vec<String>,
    /// Where to write the bag.
    pub out: std::path::PathBuf,
    /// Stop after this long. `None` = until interrupted.
    pub duration: Option<Duration>,
    /// Roll to the next file once one exceeds this many bytes.
    ///
    /// **DORMANT: no CLI flag reaches this** (one
    /// recording = one artifact before launch). `cerulion bag record`'s
    /// `-b/--max-bag-size` is GONE, so every CLI invocation leaves this `None`
    /// and a library caller is the only way to set it. See
    /// [`cerulion_bagd::BagdConfig::size_cap_bytes`] for the reasoning and for
    /// what the post-launch re-ship has to change first.
    pub max_bag_size: Option<u64>,
    /// Grace period for learning each topic's schema hash from its first frame
    /// before the bag is created.
    pub schema_wait: Duration,
    /// The workspace `schemas/` directory, when this command runs
    /// inside a workspace. It is what makes a recording SELF-DESCRIBING: the
    /// recorder resolves each tapped topic's wire `schema_hash` against the
    /// store + workspace YAML + the built-in corpus, stamps the real qualified
    /// name into the channel (the wire carries none), and writes the custom
    /// types' verbatim text into the bag's `__cerulion/schemas.json` attachment
    /// so a desk that never compiled them can still decode and render.
    ///
    /// `None` (no workspace) still resolves BUILT-IN names from the compiled-in
    /// corpus — a wire frame carries no name, and `"unknown"` is what every
    /// standard MCAP reader would otherwise display. What a workspace-less
    /// machine cannot do is resolve a CUSTOM type: those channels stay
    /// `"unknown"` and carry no definition, and the recorder says so.
    pub schemas_dir: Option<std::path::PathBuf>,
    /// ATTACH to a live `cerulion graph run` rather than recording
    /// a bare topic list.
    ///
    /// An attached recording carries what the run is — its effective graph, its
    /// env snapshot, its host identity and its run identity — so the bag
    /// describes a RUN instead of describing a set of topics that happened to be
    /// live. It also selects the topic set from the run's own declaration, which
    /// is what keeps a co-tenant's stream out of it.
    ///
    /// `None` is the verb as it was before `--run` existed, byte-for-byte.
    pub run: Option<RunTarget>,
}

impl Default for RecordOptions {
    fn default() -> Self {
        Self {
            topics: Vec::new(),
            all: false,
            regex: None,
            exclude: Vec::new(),
            out: std::path::PathBuf::from("recording.mcap"),
            duration: None,
            max_bag_size: None,
            schema_wait: Duration::from_millis(2_000),
            schemas_dir: None,
            run: None,
        }
    }
}

/// This machine's schema knowledge, as a [`cerulion_bag::BagSchemaCatalog`]
/// the recorder resolves wire hashes through.
///
/// **Both record paths build it here.** `cerulion bag record` runs
/// bagd in-process and sets `BagdConfig::schema_catalog` directly;
/// `cerulion graph run --record` spawns bagd as a subprocess, so
/// `crate::graph_cmd`'s recording prep calls this and writes the encoded catalog
/// to a scratch file bagd reads via `--schema-catalog`. ONE builder, so the two
/// verbs apply the SAME resolution rules to the corpus they are given — which is
/// a narrower claim than "they cannot disagree": they are given
/// the same `schemas/` dir (both derive it as `<workspace root>/schemas`) but
/// DIFFERENT bridge inputs, deliberately — see the `bridge_config` handling
/// below for what that can and cannot change.
///
/// TWO tiers, and the split is deliberate:
///
/// - **Names, always.** Every BUILT-IN type contributes a `schema_hash` → name
///   binding whether or not there is a workspace. A wire frame carries no name,
///   so without this every channel `bag record` writes is literally `"unknown"`
///   — including in the MCAP channel record that MCAP viewers, `mcap info` and every
///   other standard reader displays. A binding costs tens of bytes and turns
///   that row into `sensor_msgs/Image`.
/// - **Text, only for CUSTOM types, only from a workspace.** Built-in text is
///   dead weight (every Cerulion binary compiles it in); custom text is the
///   whole point, and it can only come from the workspace `.msg` store /
///   `schemas/*.yaml` this machine holds.
///
/// The custom half runs through [`crate::schema_serve::build_schema_docs`], which
/// reads the workspace `schemas/` store + `schemas/*.yaml` — the corpus
/// `topic echo` and `bag info` resolve against (`crate::topic_cmd::local_walker_from_workspace`)
/// — PLUS, when `DDS_BRIDGE_CONFIG` is set, that bridge's own `msg_dirs`. On a
/// `cerulion ros2 attach` robot the acquired vendor types live wherever the bridge
/// points, and that is exactly the machine most likely to be recording types no
/// desk has, so folding them in is the point of the feature.
///
/// **The two are therefore NOT the same corpus**, and an earlier
/// version of this comment claimed they were while describing the bridge fold two
/// paragraphs later. On a bridge-configured robot the recorder's corpus is a
/// strict SUPERSET of the reader's: such a type lands in the bag and `bag info`
/// on that same machine reports it `[from the bag's own schema records]` rather
/// than locally resolvable. That is the accurate reading — the bag really is where
/// the definition came from — but it is not "a recording can never claim
/// provenance this machine could not itself read back", which is what the old
/// sentence asserted.
///
/// **Built-in-NAMED text is deliberately NOT filtered out here (a schema-provenance
/// decision).** `cerulion-vizd` refuses such a doc by name, so filtering it would
/// buy no rendering — and it would COST: a workspace that shadows a built-in
/// publishes frames under the SHADOW's hash, and the shadow's text is the only
/// definition that explains that hash, so dropping it turns a channel `topic echo`
/// and `bag info` can decode from the bag's own records into one they cannot
/// (`NamedButUndecodable`). A bag is a record of what this machine knew; the
/// place to state what a VIEWER will do with it is the reader-side
/// report, which is [`bag_doc_is_viewable`]'s job and covers bags this recorder
/// never wrote.
///
/// Never fails — an unreadable schema file is skipped with a `warn!` inside the
/// builder and the rest still resolve. What lands in the BAG is the pruned
/// closure of the hashes actually recorded, not this whole corpus (see
/// `BagSchemaCatalog::closure_for_hashes`), so a recording whose hashes resolve
/// to nothing still writes no attachment at all.
pub(crate) fn build_record_schema_catalog(
    schemas_dir: Option<&Path>,
    bridge_config: Option<&Path>,
) -> Option<cerulion_bag::BagSchemaCatalog> {
    let mut hashes = crate::schema_serve::builtin_hash_bindings();
    let mut docs = Vec::new();
    if let Some(dir) = schemas_dir {
        // An explicitly RESOLVED bridge config wins; otherwise the env alone.
        // `bag record` has no graph to derive a sibling `graphs/<name>.bridge.yaml`
        // from, so the env is all it can consult; `graph run --record` resolves
        // through `resolve_bridge_config_path` (env OR sibling) and passes the
        // answer here.
        //
        // SCOPE: this is the ONE input on which the two
        // record verbs can differ, and what it buys is NARROWER than it looks.
        // `bridge_cfg` has a single consumer inside `build_schema_docs` — folding
        // the bridge's own `msg_dirs` into the `.msg` store — and a `ros2 attach`
        // config names `../schemas`, which resolves to the store already loaded.
        // So on the ordinary attach robot this changes NOTHING; it matters only
        // for a HAND-EDITED `msg_dirs` pointing outside `schemas/`, where a
        // `graph run --record` from a shell with no env set would otherwise miss
        // types the same run's gateway serves.
        let env_fallback = bridge_config
            .is_none()
            .then(crate::schema_serve::bridge_config_from_env)
            .flatten();
        let bridge = bridge_config.or(env_fallback.as_deref());
        let (custom_docs, custom_hashes) = crate::schema_serve::build_schema_docs(dir, bridge);
        docs = custom_docs;
        // Customs LAST. The ORDER is inert and that is fine: an
        // earlier comment justified it as "so a workspace type shadows a
        // colliding built-in binding — the same last-wins shadow ladder", but
        // that ladder is NAME-keyed while `BagSchemaCatalog::normalize` dedups
        // `hashes` by `schema_hash`, and `MessageSchema::schema_hash` folds the
        // qualified name into the digest. So a "colliding" binding is one with an
        // identical hash, which (barring an FNV collision) implies an identical
        // NAME — last-wins can only ever replace a binding with an identical one.
        // A workspace copy that genuinely differs from a built-in hashes
        // DIFFERENTLY, so both bindings survive side by side and nothing is
        // shadowed. Extending is simply how the two lists are joined.
        hashes.extend(custom_hashes);
    }
    let catalog = cerulion_bag::BagSchemaCatalog::new(docs, hashes);
    if catalog.is_empty() {
        return None;
    }
    tracing::debug!(
        docs = catalog.docs.len(),
        hashes = catalog.hashes.len(),
        schemas_dir = ?schemas_dir,
        "recording: resolved this machine's schema corpus for the recording's provenance"
    );
    Some(catalog)
}

/// The half of the `bag record` gate that needs NO transport: argument shape
/// only.
///
/// Split out so [`bag_record`] can run it BEFORE bringing the manager up — a
/// missing `--all`/topics, a combined selection or a zero `--duration` must not
/// first register an iceoryx2 node and print its lifecycle breadcrumbs at the
/// operator. [`derive_record_topics`] calls it too, so a caller that skips
/// straight there is gated identically.
pub fn validate_record_options(opts: &RecordOptions) -> CliResult<()> {
    if let Some(d) = opts.duration {
        if d.is_zero() {
            return Err(CliError::Validation(
                "--duration must be greater than 0 (a zero window records nothing)".to_string(),
            ));
        }
    }
    let sources = usize::from(!opts.topics.is_empty())
        + usize::from(opts.all)
        + usize::from(opts.regex.is_some());
    if sources == 0 && opts.run.is_none() {
        return Err(CliError::Validation(
            "bag record needs a topic set: name topics positionally, or pass --all for every \
             live local topic, or --regex <PATTERN> to match them. `cerulion topic list` shows \
             what is live here. (`--run` picks the set for you — it records the topics the run \
             it attaches to declares.)"
                .to_string(),
        ));
    }
    if sources > 1 {
        return Err(CliError::Validation(
            "bag record takes exactly ONE topic source: positional topics, --all, or --regex \
             (they were combined). Pick one, then narrow it with --exclude."
                .to_string(),
        ));
    }
    Ok(())
}

/// Derive the exact topic set to record. PURE — the selection oracle.
///
/// `live` is the raw local service enumeration (`TransportManager::list_topics`)
/// and `mirrors` the `/__cerulion/mirrors` provenance map. `list_topics` is a
/// RAW `{topic}/data` scan with no provenance filter, so on a desk holding a
/// `cerulion-netd` mirror it reports another robot's stream as a local topic —
/// the fold below is what keeps `--all` from silently recording it.
///
/// Rules, in order:
///
/// 1. EXACTLY ONE source of truth: explicit names, `--all`, or `--regex`.
///    Combining them is a loud error, because the merge semantics an operator
///    would assume ("union? intersection?") are not knowable.
/// 2. An explicit name that is not live is a REFUSAL that names the reality —
///    a topic is recorded where it is produced, and this verb never reaches the
///    network to find it elsewhere.
/// 3. `--all` / `--regex` consider only GENUINELY-LOCAL topics (netd mirrors
///    are folded out by `mirror_registry::partition_local_topics`, the same
///    predicate `topic list` and vizd use) and skip the framework's own
///    channels (`topic_cmd::is_internal_topic`: the
///    [`AUTO_SELECT_EXCLUDED_PREFIXES`] plus the bare `/__cerulion` token, the
///    same rows `topic list` hides by default). An EXPLICITLY named mirror IS
///    recorded — the operator asked for it — with a loud note naming its
///    origin robot.
/// 4. `--exclude` then removes matches from whatever was selected, including
///    from an explicit list (so `-a` minus a few, or a hand list minus one,
///    both work).
/// 5. An empty final set is a loud error, never a recording of nothing.
///
/// The returned list is SORTED and deduplicated, so a given `(live, flags)`
/// always yields the same bag channel order.
pub fn derive_record_topics(
    live: &[String],
    mirrors: &BTreeMap<String, String>,
    opts: &RecordOptions,
) -> CliResult<(Vec<String>, TopicSelection)> {
    validate_record_options(opts)?;
    let excludes = compile_patterns(&opts.exclude, "--exclude")?;
    // THE fold, from `cerulion_core::transport::mirror_registry` — the same one
    // `cerulion topic list` and vizd use, never a second copy.
    let (genuine_local, streaming) =
        mirror_registry::partition_local_topics(live.iter().cloned(), mirrors);
    // L4: an auto-selection that silently skipped a topic the operator can SEE
    // in `topic list` is a surprise. Name them once, with their origin.
    if (opts.all || opts.regex.is_some()) && !streaming.is_empty() {
        tracing::info!(
            count = streaming.len(),
            topics = %streaming
                .iter()
                .map(|r| format!("{} (from {})", r.topic, r.robot))
                .collect::<Vec<_>>()
                .join(", "),
            "bag record: these locally-visible topics are MIRRORS of other robots and are not \
             auto-selected — name one explicitly to record it as this desk sees it"
        );
    }
    let (mut selected, selection) = if opts.all {
        (auto_selectable(&genuine_local), TopicSelection::All)
    } else if let Some(pattern) = &opts.regex {
        let re = compile_patterns(std::slice::from_ref(pattern), "--regex")?
            .into_iter()
            .next()
            .expect("one pattern in, one pattern out");
        (
            auto_selectable(&genuine_local)
                .into_iter()
                .filter(|t| re.is_match(t))
                .collect(),
            TopicSelection::Regex,
        )
    } else {
        let missing: Vec<&String> = opts
            .topics
            .iter()
            .filter(|t| !live.iter().any(|l| l == *t))
            .collect();
        if !missing.is_empty() {
            // "What is live here" must render the PARTITIONED-LOCAL set: listing
            // a netd mirror as local would be the exact claim this message
            // exists to correct.
            return Err(CliError::Validation(format!(
                "not a local topic: {}.\n`cerulion bag record` taps THIS machine's shared \
                 memory and never pulls a topic's frames across the network: a robot's topics \
                 are recorded by running this verb ON the robot and transferring the file \
                 afterwards. `cerulion topic list` shows what is live here{}.{}",
                missing
                    .iter()
                    .map(|t| t.as_str())
                    .collect::<Vec<_>>()
                    .join(", "),
                if genuine_local.is_empty() {
                    " (nothing is)".to_string()
                } else {
                    format!(": {}", genuine_local.join(", "))
                },
                if streaming.is_empty() {
                    String::new()
                } else {
                    format!(
                        "\nAlso visible here, but STREAMED FROM ANOTHER ROBOT (a cerulion-netd \
                         mirror): {}. Naming one explicitly records the mirror as this desk \
                         sees it.",
                        streaming
                            .iter()
                            .map(|r| format!("{} (from {})", r.topic, r.robot))
                            .collect::<Vec<_>>()
                            .join(", ")
                    )
                }
            )));
        }
        // An EXPLICIT mirror is recorded — the operator named it — but never
        // silently: the bag would otherwise claim a local capture of another
        // robot's stream.
        for named in &opts.topics {
            if let Some(row) = streaming.iter().find(|r| &r.topic == named) {
                tracing::warn!(
                    topic = %row.topic,
                    origin_robot = %row.robot,
                    "bag record: this topic is a cerulion-netd MIRROR of another robot's \
                     stream, not a local producer. Recording it captures the mirror as this \
                     desk received it (subject to network loss); recording at the source means \
                     running `cerulion bag record` ON that robot."
                );
            }
        }
        (opts.topics.clone(), TopicSelection::Explicit)
    };

    if !excludes.is_empty() {
        selected.retain(|t| !excludes.iter().any(|re| re.is_match(t)));
    }
    selected.sort();
    selected.dedup();

    if selected.is_empty() {
        return Err(CliError::Validation(format!(
            "the topic selection is empty — nothing would be recorded. {} selectable local \
             topic(s) were considered (mirrors of other robots and Cerulion's internal \
             channels are not selectable){}.",
            auto_selectable(&genuine_local).len(),
            if opts.exclude.is_empty() {
                String::new()
            } else {
                format!(
                    "; --exclude removed every match ({})",
                    opts.exclude.join(", ")
                )
            }
        )));
    }
    Ok((selected, selection))
}

/// A RUN's declared topic set, split by whether anything is producing it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RunTopicSet {
    /// Declared topics backed by a live local service — what gets tapped.
    pub selected: Vec<String>,
    /// Declared topics with NO live local service. Costed individually: warned,
    /// and carried into the bag's coverage manifest as
    /// [`UntappedReason::DeclaredNotLive`](cerulion_bagd::discovery::UntappedReason::DeclaredNotLive).
    pub not_live: Vec<String>,
}

/// Resolve a RUN's DECLARED topics against what is live. PURE.
///
/// # Why this is not [`derive_record_topics`]
///
/// The two selections differ on exactly one rule, and it is the rule that
/// matters: a NAMED topic that is not live is a REFUSAL (the operator asked for
/// it by name, so silently dropping it would record a bag quietly missing what
/// was asked for), while a DECLARED one that is merely quiet costs THAT TOPIC
/// AND NOTHING ELSE. [`TopicSelection::Run`] has said so since C2 shipped; what
/// was missing was a code path that could act on it, because the run-derived
/// names were substituted into `opts.topics` and therefore reached
/// `derive_record_topics`' EXPLICIT branch, which hard-errors on the first
/// not-live name.
///
/// That is not a corner. It is the WHOLE of multi-process run start-up — the
/// supervisor's planning build runs on a throwaway namespace, so each group's
/// services appear only as its worker builds — and it is PERMANENT on a degraded
/// roster (`--peer-loss continue` with a dead worker leaves that group's topics
/// unproduced for the rest of the run). Worse, the refusal it produced named the
/// wrong remedy: "a robot's topics are recorded by running this verb ON the
/// robot" is exactly backwards when the verb is already on the robot, attached
/// to the run that declares them.
///
/// # Rules
///
/// 1. A declared topic with a live `{topic}/data` service is SELECTED. Liveness
///    is checked against the RAW enumeration, the same predicate the explicit
///    branch uses — a run's own output is never folded out as a netd mirror,
///    because the run is what produces it.
/// 2. A declared topic with no live service is NOT LIVE: reported, never fatal.
/// 3. `--exclude` removes matches from BOTH halves. An excluded topic is neither
///    recorded nor reported as missing — the operator took it out.
/// 4. ZERO live declared topics is a loud refusal. Recording a bag with no
///    channels while claiming to describe a run would be worse than saying so,
///    and at that point nothing about the run is being captured anyway.
///
/// Both halves come back SORTED and deduplicated, so one `(live, declared)` pair
/// always yields the same bag channel order.
pub fn derive_run_topics(
    live: &[String],
    declared: &[String],
    opts: &RecordOptions,
    run_label: &str,
) -> CliResult<RunTopicSet> {
    let excludes = compile_patterns(&opts.exclude, "--exclude")?;
    let kept = |t: &String| !excludes.iter().any(|re| re.is_match(t));
    let (mut selected, mut not_live): (Vec<String>, Vec<String>) = declared
        .iter()
        .filter(|t| kept(t))
        .cloned()
        .partition(|t| live.iter().any(|l| l == t));
    for v in [&mut selected, &mut not_live] {
        v.sort();
        v.dedup();
    }
    if selected.is_empty() {
        return Err(CliError::Validation(format!(
            "run {run_label} declares {} topic(s) and NONE of them is live on this machine, so \
             there is nothing to record.{}\nThe run may not have reached the point where its \
             nodes publish, or `--exclude` may have removed everything. `cerulion topic list` \
             shows what is live here.",
            declared.len(),
            if not_live.is_empty() {
                String::new()
            } else {
                format!(" Not live: {}.", not_live.join(", "))
            }
        )));
    }
    Ok(RunTopicSet { selected, not_live })
}

/// The `/__cerulion/mirrors` provenance map, read from LOCAL shared memory,
/// plus whether that reading SETTLED the question.
///
/// `--all` / `--regex` DERIVE the recorded topic set from this map, so a gather
/// that times out having heard nothing auto-selects another robot's mirrored
/// stream as this machine's data — the exact defect the checked gather fixes for `bagd`,
/// in a bag that is just as durable. It therefore runs the SAME shared policy
/// (`cerulion_bagd::discovery::resolve_mirror_snapshot` over the checked
/// gather), not a second copy of it: retry while the answer is a timeout, and
/// report the verdict either way.
///
/// Still best-effort at the end of the budget — a `bag record` must not fail
/// because a registry read did — but the degradation is now RECORDED
/// (`RecordCoverage::mirrors_established`) instead of being indistinguishable
/// from a desk with no mirrors.
fn gather_mirror_provenance(
    manager: &Arc<TransportManager>,
    running: &AtomicBool,
) -> (BTreeMap<String, String>, bool) {
    let (snapshot, established) = cerulion_bagd::discovery::resolve_mirror_snapshot(
        cerulion_bagd::discovery::MIRROR_GATHER_ATTEMPTS,
        || manager.gather_mirror_provenance_checked(MIRROR_GATHER_WINDOW),
        |e| {
            tracing::warn!(
                error = %e,
                "bag record: could not read the mirror-provenance registry — a topic that is \
                 really another robot's mirror may be auto-selected as local this run"
            );
        },
        // The CALLER's own run flag, threaded rather than
        // hardcoded. The first version passed `|| false` justified by "there
        // is no shutdown flag in scope yet" — which the compiler refutes:
        // `running` is a parameter of `bag_record_with_manager`, live two
        // statements above the gather. Note the INVERTED polarity against
        // bagd's (`shutdown.load(..)`): here `true` means KEEP GOING, which is
        // the same convention the duration bridge below reads.
        || !running.load(Ordering::Relaxed),
    );
    if !established {
        // `established == false` also covers a run asked to
        // stop (cancellation), where "every attempt
        // expired" would be a fabricated mechanism.
        let stopped_early = !running.load(Ordering::Relaxed);
        tracing::warn!(
            attempts = cerulion_bagd::discovery::MIRROR_GATHER_ATTEMPTS,
            mirrors_found = snapshot.len(),
            stopped_early,
            "bag record: could NOT establish which local topics are netd mirrors — either every \
             attempt expired with a live writer unheard, or the command was asked to stop first \
             (see stopped_early). An auto-selected topic may really be another robot's \
             re-injected stream; the bag's record_coverage.json carries mirrors_established: false"
        );
    }
    (snapshot, established)
}

/// How long the provenance gather listens.
///
/// L3: this is the CANONICAL constant from `mirror_registry`, not a private
/// copy. The private 250 ms one it replaces silently gave `bag record` less than
/// half the time `cerulion topic list` gets (600 ms) to see a mirror, so the two
/// verbs could disagree about whether a topic is local — exactly the
/// disagreement the shared fold exists to prevent.
const MIRROR_GATHER_WINDOW: Duration = mirror_registry::MIRROR_GATHER_WINDOW;

/// The live topics `--all` / `--regex` may select: everything that is not one
/// of the framework's own channels.
///
/// The filter is `topic_cmd::is_internal_topic`, the SAME predicate
/// `topic list` hides rows with, called rather than re-spelled: a topic the
/// default listing hides is a topic this verb never auto-selects, and the two
/// cannot disagree because there is one function. That predicate is
/// [`AUTO_SELECT_EXCLUDED_PREFIXES`] plus the bare `/__cerulion` namespace
/// token, which no prefix can spell.
fn auto_selectable(live: &[String]) -> Vec<String> {
    live.iter()
        .filter(|t| !crate::topic_cmd::is_internal_topic(t))
        .cloned()
        .collect()
}

/// The outcome of probing one topic for tappability.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TapProbe {
    /// Topics that attached.
    pub attachable: Vec<String>,
    /// Topics that did NOT, each with the reason.
    pub refused: Vec<(String, String)>,
}

/// The refusal for an EXPLICIT topic list containing un-tappable topics, or
/// `None` when every named topic attached.
///
/// Names EVERY offender with its own reason — naming only the first makes an
/// operator fix them one run at a time.
pub fn explicit_untappable_error(probe: &TapProbe) -> Option<String> {
    if probe.refused.is_empty() {
        return None;
    }
    Some(format!(
        "cannot tap {} named topic(s):\n{}\nThey were named explicitly, so this is a hard \
         failure rather than a silent omission. Drop them from the list, or use --all / --regex, \
         which record what CAN be tapped and report the rest.",
        probe.refused.len(),
        probe
            .refused
            .iter()
            .map(|(t, r)| format!("  {t}: {r}"))
            .collect::<Vec<_>>()
            .join("\n")
    ))
}

/// Probe each topic by opening — and immediately dropping — a data-only tap.
///
/// # Why this exists
///
/// `run_bagd` attaches every tap with `?`, so ONE un-attachable topic fails the
/// ENTIRE run: an operator running `-a` on a busy robot gets NO BAG AT ALL
/// rather than a bag missing one topic. `ros2 bag record -a` degrades per-topic,
/// and the failure is not hypothetical — subscriber slots are finite
/// (`INTROSPECTION_SUBSCRIBER_HEADROOM` is 5, shared with the liveness observer,
/// vizd taps and the topic verbs), and a producer that EXITS between the
/// `list_topics` snapshot and the attach turns into a fatal `DoesNotExist`
/// because `create_data_only_subscriber` is open-only.
///
/// # Scope
///
/// This NARROWS the window; it does not close it. The probe is itself a TOCTOU
/// — a producer can still exit between the probe and bagd's own attach — and a
/// slot freed by the probe's drop can be taken by another process. Closing it
/// properly means per-tap tolerance inside bagd's `Recorder::new`, which is a
/// change to the shipping recorder (see `docs/bag.md`). What this buys is that
/// the COMMON causes — a topic that was already gone, or a topic whose slots are
/// already exhausted — cost that topic instead of the whole recording.
pub fn probe_tappable(manager: &Arc<TransportManager>, topics: &[String]) -> TapProbe {
    let mut attachable = Vec::new();
    let mut refused = Vec::new();
    for topic in topics {
        match manager.create_data_only_subscriber(topic) {
            Ok(sub) => {
                // Drop immediately — bagd opens its own tap. Holding it would
                // consume the slot bagd needs.
                drop(sub);
                attachable.push(topic.clone());
            }
            Err(e) => refused.push((topic.clone(), e.to_string())),
        }
    }
    TapProbe {
        attachable,
        refused,
    }
}

fn compile_patterns(patterns: &[String], flag: &str) -> CliResult<Vec<regex::Regex>> {
    patterns
        .iter()
        .map(|p| {
            regex::Regex::new(p).map_err(|e| {
                CliError::Validation(format!("{flag} pattern '{p}' is not a valid regex: {e}"))
            })
        })
        .collect()
}

/// Record `opts`' topics into an MCAP bag by driving `cerulion_bagd`.
///
/// # What this verb owns, and what bagd owns
///
/// This function owns exactly three things: deriving the topic set
/// ([`derive_record_topics`]), translating the CLI's `running` flag and
/// `--duration` into bagd's shutdown flag, and rendering the summary. Tapping,
/// draining, writing, loss accounting, rotation and finalization are all
/// `cerulion_bagd`'s, unchanged.
///
/// # Grade
///
/// ATTACH mode (observability-grade), which is the only accurate grade for a
/// capture of already-running publishers: the schema HASH is learned from each
/// topic's first frame and recorded exactly, but the schema NAME is recorded as
/// [`ATTACH_SCHEMA_NAME`] because the wire carries no name. That is enough for
/// `bag play` (which validates frames against the hash) and for `topic echo`
/// (which resolves names from the workspace schema store). It is NOT the
/// replay-grade descriptor `graph run --record` writes, and this verb does not
/// claim `cerulion bag play --resim` compatibility.
pub fn bag_record(
    opts: RecordOptions,
    running: Arc<AtomicBool>,
    out: &mut dyn std::io::Write,
) -> CliResult<cerulion_bagd::BagdSummary> {
    // Gate the arguments BEFORE the transport comes up — see
    // `validate_record_options`.
    validate_record_options(&opts)?;
    let manager = TransportManager::get_or_init()?;
    bag_record_with_manager(manager, opts, running, out)
}

/// [`bag_record`] over a caller-supplied transport — the seam tests drive so
/// each can own an isolated SHM root instead of the process singleton.
pub fn bag_record_with_manager(
    manager: Arc<TransportManager>,
    opts: RecordOptions,
    running: Arc<AtomicBool>,
    out: &mut dyn std::io::Write,
) -> CliResult<cerulion_bagd::BagdSummary> {
    use cerulion_bagd::{BagdConfig, TapSpec};

    validate_record_options(&opts)?;

    // Resolve the run FIRST. It decides the topic set, so it must
    // settle before the selection runs — and a refusal (ambiguous / no such run)
    // must happen before any tap is opened, so a rejected invocation costs
    // nothing and leaves no bag.
    let attach = match &opts.run {
        Some(target) => resolve_run_attach(&manager, target, out)?,
        None => None,
    };
    // Hold a lock inside the run's directory for as long as
    // this recorder lives.
    //
    // An attacher OUTLIVES its run by design — it finalizes a bag after the run
    // ends — so without this the next `graph run`'s sweeper meets a directory
    // whose every lock is free while a process is still reading it, and
    // `remove_dir_all` runs under a live `read_run_artifacts`. The lock makes
    // that window impossible; a held attach lock on a DEAD run is reported and
    // swept by a later run once the attacher is gone.
    //
    // Keyed by PID because attachers are independent processes that may crash:
    // the name needs no coordination with any other attacher, and a stale FILE
    // left by a crash is exactly what the flock probe distinguishes from a live
    // one. DEGRADES — a recorder that cannot lock still records; the cost is
    // that the sweeper may reclaim the directory out from under it, and the warn
    // says so.
    #[cfg(unix)]
    let _attach_lock = attach.as_ref().and_then(|a| {
        let path = std::path::Path::new(&a.run.run_dir)
            .join(crate::run_lock::attach_lock_file(std::process::id()));
        match crate::run_lock::RunLock::acquire(&path) {
            Ok(lock) => Some(lock),
            Err(e) => {
                tracing::warn!(
                    run_dir = %a.run.run_dir,
                    lock = %path.display(),
                    error = %e,
                    "could not lock this run's directory for the attach — recording \
                     continues, but a later `graph run` may reclaim the run directory while \
                     this recorder is still reading it"
                );
                None
            }
        }
    });
    // A `--run` with no explicit selection records the topics the RUN declares
    // — not every topic on the machine. The two differ on any co-tenanted box,
    // and `--all` would sweep in a second graph, an external publisher or
    // another bridge (the co-tenancy residual). An explicit list or
    // `--all`/`--regex` still wins: the operator asked for something specific.
    //
    // The declared names are kept in their OWN binding rather than substituted
    // into `opts.topics`. That substitution was the C2 defect: it routed a
    // DERIVED set into `derive_record_topics`' EXPLICIT branch, whose rule is
    // "a name that is not live is a refusal" — so one quiet declared output
    // refused the entire recording and `TopicSelection::Run`'s documented
    // per-topic degrade was unreachable. See [`derive_run_topics`].
    let mut run_declared: Option<Vec<String>> = None;
    if let Some(a) = &attach {
        if opts.topics.is_empty() && !opts.all && opts.regex.is_none() {
            let declared = match &a.artifacts.graph_yaml {
                Some(yaml) => run_declared_topics(yaml)?,
                None => {
                    return Err(CliError::Validation(format!(
                        "attached to run 0x{:032x} ({}) but its run directory carries no \
                         readable graph.yaml, so the topics it declares are unknown and there \
                         is nothing to select. Name topics explicitly, or pass --all to record \
                         every live local topic.",
                        a.run.run_id, a.run.graph_name
                    )));
                }
            };
            if declared.is_empty() {
                return Err(CliError::Validation(format!(
                    "run 0x{:032x} ({}) declares no output topics, so `--run` selects nothing. \
                     Pass --all to record every live local topic instead.",
                    a.run.run_id, a.run.graph_name
                )));
            }
            writeln!(
                out,
                "attached to run 0x{:032x} ({}); recording the {} topic(s) it declares",
                a.run.run_id,
                a.run.graph_name,
                declared.len()
            )?;
            run_declared = Some(declared);
        } else {
            // L2: `--run` combined with a topic source BINDS the recording to
            // that run (its graph, env, identity and trace rings all ride along)
            // while the topic set comes from somewhere else entirely. Silence
            // there reads as if the flag had been ignored.
            writeln!(
                out,
                "attached to run 0x{:032x} ({}) for its graph, env and identity; the topic set \
                 comes from {} instead of the run's declaration",
                a.run.run_id,
                a.run.graph_name,
                if opts.all {
                    "--all"
                } else if opts.regex.is_some() {
                    "--regex"
                } else {
                    "the topics you named"
                }
            )?;
        }
    }

    let live = manager.list_topics()?;
    // Network-free: provenance is read from LOCAL shared memory. The
    // verdict travels with the snapshot, because `--all`/`--regex` DERIVE the
    // recorded set from it — a timed-out gather auto-selects another robot's
    // mirrored stream as local, and the bag must say so rather than look clean.
    let (mirrors, mirrors_established) = gather_mirror_provenance(&manager, &running);
    // The two selections are DIFFERENT RULES, not one rule with a label — see
    // `derive_run_topics`. A declared topic that is quiet costs itself; a named
    // one that is not live refuses.
    let (topics, selection, declared_untapped) = match &run_declared {
        Some(declared) => {
            let a = attach
                .as_ref()
                .expect("run_declared is only ever set inside the `attach` arm above");
            let set = derive_run_topics(
                &live,
                declared,
                &opts,
                &format!("0x{:032x} ({})", a.run.run_id, a.run.graph_name),
            )?;
            for topic in &set.not_live {
                tracing::warn!(
                    topic = %topic,
                    "bag record --run: the run DECLARES this topic but nothing is producing it \
                     here, so it is not in the recording — the other declared topics still \
                     record (the node may not have fired yet, or its worker may be gone). Named \
                     in the bag's coverage manifest as declared_not_live"
                );
                writeln!(out, "  NOT LIVE {topic} (declared by the run, no producer)").ok();
            }
            let untapped = set
                .not_live
                .into_iter()
                .map(|t| (t, cerulion_bagd::discovery::UntappedReason::DeclaredNotLive))
                .collect();
            (set.selected, TopicSelection::Run, untapped)
        }
        None => {
            let (topics, selection) = derive_record_topics(&live, &mirrors, &opts)?;
            (topics, selection, Vec::new())
        }
    };

    // Per-topic tappability preflight. An AUTO-derived set degrades: a topic
    // that cannot be tapped costs THAT topic, not the whole recording. An
    // EXPLICIT list does not — the operator named those topics, so silently
    // dropping one would record a bag that is quietly missing what was asked
    // for.
    let probe = probe_tappable(&manager, &topics);
    let topics = if selection == TopicSelection::Explicit {
        if let Some(msg) = explicit_untappable_error(&probe) {
            return Err(CliError::Validation(msg));
        }
        topics
    } else {
        for (topic, reason) in &probe.refused {
            tracing::warn!(
                topic = %topic,
                reason = %reason,
                "bag record: this topic cannot be tapped and is EXCLUDED from the recording — \
                 the rest still record (its producer may have exited since the topic scan, or \
                 its subscriber slots may be exhausted)"
            );
            writeln!(out, "  EXCLUDED {topic}: {reason}").ok();
        }
        probe.attachable
    };
    if topics.is_empty() {
        return Err(CliError::Validation(format!(
            "no selected topic could be tapped — nothing would be recorded. {} topic(s) were \
             refused: {}",
            probe.refused.len(),
            probe
                .refused
                .iter()
                .map(|(t, r)| format!("{t} ({r})"))
                .collect::<Vec<_>>()
                .join("; ")
        )));
    }

    writeln!(
        out,
        "recording {} topic(s) ({}) to {}{}:",
        topics.len(),
        match selection {
            TopicSelection::Explicit => "named",
            TopicSelection::All => "--all",
            TopicSelection::Regex => "--regex",
            TopicSelection::Run => "--run",
        },
        opts.out.display(),
        match opts.duration {
            Some(d) => format!(" for {:.1}s", d.as_secs_f64()),
            None => String::new(),
        }
    )?;
    for t in &topics {
        writeln!(out, "  {t}")?;
    }
    writeln!(out, "Ctrl-C to stop and finalize.")?;

    let mut config = BagdConfig::new(
        opts.out.clone(),
        topics.iter().map(TapSpec::attach).collect(),
    );
    config.size_cap_bytes = opts.max_bag_size;
    config.schema_wait = opts.schema_wait;
    // Hand the recorder this machine's schema knowledge. bagd resolves
    // each learned wire hash through it to NAME the channel (the wire carries no
    // name, so an attach tap can only write "unknown") and prunes it to the
    // closure these topics need for the bag's `__cerulion/schemas.json`.
    // Network-free and workspace-local — the same corpus `topic echo` decodes
    // against, so what a recording claims and what this machine can read agree
    // by construction.
    //
    // The BRIDGE-CONFIG argument is `None` on every path here, and the reason
    // is per-path rather than "there is no graph in hand" (which is false under
    // `--run`, where `a.run.graph_name` is right there): a sibling
    // `graphs/<name>.bridge.yaml` is a WORKSPACE file, and this verb resolves no
    // workspace — `graph run --record` reads it because it was invoked inside
    // one and already holds `graphs_dir`, while `bag record` may be run from
    // anywhere, and the run directory carries the run's EFFECTIVE graph, not its
    // sibling files. Guessing a workspace from the run's `graph_file` would be a
    // path inference on another process's cwd. The env remains the only bridge
    // source (`None` selects that fallback inside the builder).
    config.schema_catalog = build_record_schema_catalog(opts.schemas_dir.as_deref(), None);
    // The NETWORKED schema-resolution rungs, threaded exactly as
    // `cerulion bagd`'s own CLI assembly threads them (same default, same env
    // override, one helper — a second copy here is how the two would drift).
    //
    // `BagdConfig::new` leaves this ZERO on the `discover_live` rule, so without
    // this call every `bag record` bag resolved schemas from LOCAL knowledge
    // only and the coverage report's unresolved-schemas line was suppressed
    // (`schema_demand_requested: false`). An ATTACH recording is precisely the
    // shape that needs the rungs: its channels are attach-mode by construction
    // (the wire carries no name), and the run it attaches to may be a
    // `ros2 attach` bridge whose types this desk never compiled.
    cerulion_bagd::apply_schema_demand_settings(
        &mut config,
        cerulion_bagd::DEFAULT_SCHEMA_DEMAND_MS,
        cerulion_bagd::schema_demand_from_env(),
    );
    // WHOSE universe is this? — resolved through bagd's OWN shared
    // policy, never a boolean re-derived here.
    //
    // An explicit `--topic` list or a `--all`/`--regex` auto-select is an
    // operator's CHOSEN set, and letting bagd auto-add every live topic
    // underneath it would silently overrule someone who named three topics — so
    // discovery stays OFF and the manifest's `enumerated: false` says the empty
    // untapped list is not a coverage claim.
    //
    // `--run` is the OTHER kind, and the live-discovery rule decides it: the
    // run-declared set is `config.nodes[].outputs[]`, literally the static
    // declaration that was proved insufficient (on the flagship `ros2 attach`
    // run it names FOUR topics while ~71 per-DDS-topic bridge routes stream
    // unrecorded). A bag that claims to describe a run while silently omitting
    // most of what the run put on the wire is precisely the defect discovery
    // exists to close, so `--run` is an INFERRED universe and discovery is ON,
    // with the standard settle machinery. `CERULION_RECORD_DISCOVERY=off` and
    // `CERULION_RECORD_DISCOVERY_SETTLE_MS` remain the switches (the flags are
    // bagd's own argv, which this verb never builds).
    //
    // `--exclude` rides this same call, because turning discovery on
    // is exactly what DEFEATED it. `derive_run_topics` filters the DECLARED
    // half; discovery re-finds an excluded topic as an "undeclared live
    // producer" (its only name filter is `discovery_known`, seeded from the tap
    // set the exclusion had already emptied) and records it — MEASURED at 291
    // frames of a topic stdout said was not being recorded. The patterns are
    // passed VERBATIM: bagd compiles them with the same `regex::Regex::new` this
    // verb already used for the same strings, so one flag cannot mean two things
    // across the two halves.
    cerulion_bagd::apply_discovery_settings(
        &mut config,
        /* explicit_topic_selection */ selection != TopicSelection::Run,
        /* no_live_discovery */ false,
        cerulion_bagd::DEFAULT_DISCOVERY_SETTLE_MS,
        cerulion_bagd::discovery_disabled_by_env(),
        cerulion_bagd::discovery_settle_from_env(),
        opts.exclude.clone(),
    );
    // The caller's own coverage verdicts — declared topics with no
    // producer. Seeded whether or not discovery runs, because they are a fact
    // about the SELECTION rather than about any enumeration.
    config.declared_untapped = declared_untapped;
    // Discovery being OFF means the recorder never gathers, so WITHOUT
    // this the manifest would stamp `None` — no claim — over a topic selection
    // that was in fact made on mirror evidence, possibly timed out. This verb
    // did its own gather; the bag records what that gather actually settled.
    config.mirrors_established = Some(mirrors_established);
    // The recorder's OWN status topic is off: this verb reports through its
    // summary and `tracing`, and a status publisher would put a `/bagd/status`
    // topic into the very SHM directory `--all` enumerates.
    config.status_period = None;

    // An ATTACHED recording carries what the run IS.
    //
    // The three run-dir artifacts are byte-identical to a `graph run --record`
    // bag's, and not because two writers agree: C0 renders them with the SAME
    // pure functions that build the `--record` attachments
    // (`render_effective_graph_yaml` / `render_env_json` /
    // `render_recorder_json`), and this path copies the bytes.
    //
    // That copy is why the recorder.json `trace_format` stamp is
    // derived from a STATIC property of the binary rather than from the trace
    // stream a bag turns out to contain. These bytes were rendered at the RUN's
    // start; this recorder attaches later and drains whatever the run has
    // emitted since, so a stamp derived from the observed stream would be
    // frozen against a stream it never saw and could UNDER-claim. See
    // `replay_engine::recorder_stream_needs_v4` (risk R1).
    if let Some(a) = &attach {
        let mut push = |name: &str, bytes: Option<&Vec<u8>>| {
            if let Some(b) = bytes {
                config.attachments.push((name.to_string(), b.clone()));
            }
        };
        // Through the SAME constants the two `graph run` recorders
        // resolve, for exactly the reason the recorder-identity name already
        // was — three writers of one attachment name is how a reader ends up
        // holding a bag whose graph it cannot find.
        push(
            cerulion_bagd::GRAPH_YAML_ATTACHMENT,
            a.artifacts.graph_yaml.as_ref(),
        );
        push(
            cerulion_bagd::ENV_JSON_ATTACHMENT,
            a.artifacts.env_json.as_ref(),
        );
        // The NAME comes from `cerulion_bagd`, the same constant the
        // `graph run --record` handoff resolves to — one spelling, so the two
        // record paths cannot drift apart on it.
        push(
            cerulion_bagd::RECORDER_JSON_ATTACHMENT,
            a.artifacts.recorder_json.as_ref(),
        );
        // The run identity attachment is minted HERE, not copied: it is the
        // run's manifest PLUS the attach facts only the recorder knows.
        //
        // `first_step_recorded` is `None` at this point and stays `None` in the
        // attachment. That is a real limit, stated rather than papered over: the
        // attachments are frozen at bag creation, and the first complete step is
        // only known once a ring has been drained. What the bag DOES carry
        // unconditionally is `attached_mid_run: true` — the fact a reader must
        // never have to infer — and the per-channel `attached_late` markers.
        // Rendered BEFORE the push because the attachment takes `&str` and two
        // of the seven arms carry data (a run's stated reason; the ranks whose
        // rings were never created), so the verdict is a `String` rather than a
        // `&'static str`.
        // The state-ring decision: decided HERE, beside the trace verdict and from the
        // same three facts, because the bag's `run.json` is rendered a few lines
        // below while the decision it describes is ACTED on further down. ONE
        // value, read twice, so the manifest cannot claim one thing while the
        // recorder does another.
        let state_rings = StateRingVerdict::choose(
            a.artifacts.run_json.is_some(),
            a.artifacts.manifest_parsed(),
            &a.artifacts.state_ring_consumer,
        );
        let trace_verdict = choose_trace_verdict(TraceVerdictFacts {
            manifest_read: a.artifacts.run_json.is_some(),
            manifest_parsed: a.artifacts.manifest_parsed(),
            trace_rings: &a.artifacts.trace_rings,
            declared_unavailable: &a.artifacts.declared_unavailable,
            rings_declared: a.artifacts.rings.len(),
        })
        .render();
        config.attachments.push((
            RUN_JSON_ATTACHMENT.to_string(),
            render_attach_run_json(
                &a.run,
                a.artifacts.run_json.as_deref(),
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos() as u64,
                None,
                // SEVEN states, decided by a pure function whose
                // arms are named and individually testable — see
                // `choose_trace_verdict`. An unnamed inline chain with four
                // arms is how an unparseable
                // manifest can render "the run declared no trace rings".
                &trace_verdict,
                &state_rings.render(),
                &a.artifacts.unreadable,
            ),
        ));
        // Rings open at the LIVE cursor, every declared tap is `attached_late`,
        // and the partial head step is discarded — see the field's doc for why
        // those are one flag.
        config.attached_mid_run = true;
        // The run's OWN declared rings, RESOLVED from tag to POSIX SHM name.
        //
        // The two are different strings and the boundary is not cosmetic:
        // `run.json` carries the TAG (`cer_rec_<graph>_<pid>_r<N>` — what the
        // supervisor stamps, what a `/dev/shm` breadcrumb correlates against),
        // while `TraceRingConsumer::open*` takes the resolved
        // `/cer_rg_<fnv1a64>` NAME. Handing it the tag reaches `shm_open`
        // verbatim and fails — ENOENT, or ENAMETOOLONG on macOS, where a name
        // over 31 bytes is refused rather than truncated. `graph run --record`
        // resolves at the same boundary before building bagd's argv.
        //
        // `attached_mid_run` is what makes these openable at all: `open` starts
        // at record 0 and a ring that has already lapped is refused outright,
        // which is every ring of any run old enough to be worth attaching to.
        config.rings = a
            .artifacts
            .rings
            .iter()
            .map(|tag| cerulion_core::shm_ring::ring_shm_name(tag))
            .collect();
        // Gated on the manifest having been READ **and PARSED**, for the same
        // reason the attachment's verdict is: with no manifest — or with bytes
        // that carry no meaning — "declares no trace rings" is a claim about a
        // document nobody could open. Reading it is not enough, because
        // `run_manifest_ring_tags` yields the same empty vector for unparseable
        // bytes as for a manifest that declared none.
        //
        // Both silent states are recorded where it lasts: the unread one is
        // additionally loud (`resolve_run_attach` warns per unreadable artifact
        // naming the file and the OS error), and BOTH are stamped into the bag's
        // own `trace` verdict, so nothing is lost by not saying it here.
        // The LATE ATTACH: this recorder arms NOTHING (the
        // graph opened its arm word long before this process existed), but it
        // can still DRAIN that run's per-rank state rings, because their names
        // derive from the run's own `run_id` — which is exactly what this attach
        // just resolved. That is the whole point of the derivation: a mid-run
        // recorder needs no launch-time coordination to find the checkpoint
        // plane. A run nobody is checkpointing simply has no rings to find, and
        // the sweep costs a bounded run of `shm_open`s per scan.
        //
        // **The state-ring decision:** …unless the run says somebody is ALREADY draining
        // them. Those rings are `OverrunPolicy::Backpressure` — one shared
        // cursor slot, last publisher wins — so a second consumer laps the
        // first, and since decisions 75 + 89 the ordinary `graph run` starts a
        // standing window recorder that holds exactly that tag (decisions 75 + 89;
        // 123 is the separate `--no-rings` coupling). Attaching here
        // would not merely cost THIS bag its anchors; it can cost the run's
        // black box its own.
        //
        // Only a claim this build UNDERSTANDS declines. Every unknown proceeds
        // as before — see `StateRingVerdict::declines_state_plane`. The verdict
        // was decided above, beside the trace one, because the bag's manifest is
        // rendered before this point and the two must not disagree.
        if state_rings.declines_state_plane() {
            // ONE loud line, naming the CAUSE and where anchors DO come from
            // (never another verb's flag, by design; nothing this
            // verb accepts changes what the run decided at launch).
            //
            // It does NOT say "and the trace is unaffected", which an earlier
            // draft did: `standing` is reachable on the MONOLITH shapes, which
            // by design declare no trace rings at all, so that clause
            // would have contradicted the `info!` eight lines below on exactly
            // those runs. The trace situation is stated once, from a fact this
            // function actually read.
            //
            // The liveness claim is hedged for the same reason: `Standing` is
            // what the run REPORTED at launch, and nothing un-declares it if
            // the recorder later dies, so promising captures exist would be a
            // claim about a process nobody observed.
            tracing::warn!(
                run_id = format!("0x{:032x}", a.run.run_id),
                "bag record --run: this run reported a standing Flashback recorder draining its \
                 per-rank node-state rings, and those rings admit exactly ONE consumer — so \
                 this recording is NOT sweeping for them and will carry no anchors of its own. \
                 Frames are unaffected. Anchors for this run live in that recorder's captures \
                 (`cerulion flashback`); if it is no longer running, this run has none."
            );
        } else {
            config.state_ring_discovery_tag = Some(
                cerulion_core::state_arm::state_arm_tag_for_run(a.run.run_id),
            );
            // …and an UNKNOWN is loud too. Proceeding on absent evidence is the
            // deliberate choice (declining would cost this bag its anchors for a
            // consumer nobody observed), but it is still an INFERENCE, and the
            // hazard it accepts is real: if a recorder IS standing, both readers
            // may lap and the loser is retired with an `Overrun` — which on the
            // default run shape can be the run's own black box.
            //
            // Before this, the only record was the `state_rings` string inside a
            // bag the operator opens later, if ever. The verdict's own text is
            // reused rather than paraphrased, so the terminal and the bag cannot
            // drift — with ONE substitution: those sentences point at keys "in
            // this document", which at a terminal is no document. The bag they
            // are about is named instead.
            //
            // The verdict rides a STRUCTURED FIELD, not the message. It is a
            // runtime value, and the message is the one part of a line that must
            // stay constant so it can be grepped and aggregated (the AGENTS.md
            // logging rule, enforced by `tracing_field_discipline_test`, which is
            // what caught the first version of this line — it spliced the whole
            // verdict into the message as `"bag record --run: {}"`). The default
            // fmt subscriber renders the field on the same line, so the operator
            // still reads the whole sentence; what changed is that a JSON
            // subscriber now gets a key and an operator can `grep state_rings=`.
            //
            // The message is a CONSTANT true of every arm that reaches here, and
            // those are exactly the four UNKNOWNs: this branch is the `else` of
            // `declines_state_plane`, so the refusal cannot arrive, and the guard
            // below excludes the one arm that read a real answer. All four swept,
            // and none of them could tell — which is what the sentence says.
            //
            // It is deliberately NOT merged with the refusal warn above. That one
            // carries a remediation this one has no business claiming — WHERE the
            // anchors are (`cerulion flashback`) — and the verdict text it would
            // have to render instead does not name the verb. One constant message
            // per condition; the shared part is the `bag record --run:` prefix.
            if !matches!(state_rings, StateRingVerdict::FromAttach) {
                tracing::warn!(
                    run_id = format!("0x{:032x}", a.run.run_id),
                    state_rings = %repoint_verdict(&state_rings.render()),
                    "bag record --run: this attach could not tell whether this run's per-rank \
                     node-state rings already have a consumer, and swept for them anyway"
                );
            }
        }
        if config.rings.is_empty() && a.artifacts.manifest_parsed() {
            tracing::info!(
                run_id = format!("0x{:032x}", a.run.run_id),
                "bag record --run: this run's manifest declares no trace rings, so the bag will \
                 carry frames and attachments but NO scheduler trace. A multi-process run \
                 provisions them by default, so this run either declined them, could not have \
                 them, or is a shape that mints none — and a run already under way cannot be \
                 given them. `__cerulion/run.json` records which."
            );
        }
    }

    // bagd's flag is INVERTED relative to the CLI's (`true` = stop, vs `true` =
    // keep going), and `--duration` is a bagd capability that does not exist
    // (it caps by SIZE, never by time). One bridge thread owns both: it flips
    // bagd's flag when the CLI's clears OR the deadline passes, and exits as
    // soon as either happens, so no thread outlives the run.
    let shutdown = Arc::new(AtomicBool::new(false));
    let bridge_shutdown = Arc::clone(&shutdown);
    let bridge_running = Arc::clone(&running);
    let deadline = opts.duration.map(|d| std::time::Instant::now() + d);
    let bridge = std::thread::Builder::new()
        .name("cer-bag-record-stop".to_string())
        .spawn(move || loop {
            if bridge_shutdown.load(Ordering::Relaxed) {
                break;
            }
            if !bridge_running.load(Ordering::Relaxed) {
                bridge_shutdown.store(true, Ordering::Relaxed);
                break;
            }
            if deadline.is_some_and(|d| std::time::Instant::now() >= d) {
                tracing::info!("bag record: duration elapsed — finalizing");
                bridge_shutdown.store(true, Ordering::Relaxed);
                break;
            }
            std::thread::sleep(RECORD_STOP_POLL);
        })
        .map_err(|e| CliError::Validation(format!("cannot spawn the record stop thread: {e}")))?;

    let result = cerulion_bagd::run_bagd(manager, config, Arc::clone(&shutdown));

    // Reap the bridge on EVERY exit path (including an Err) before returning:
    // setting the flag it watches is what makes it exit.
    shutdown.store(true, Ordering::Relaxed);
    let _ = bridge.join();

    let summary = result.map_err(|e| CliError::Validation(format!("bag record failed: {e}")))?;
    tracing::info!(
        files = summary.bag_paths.len(),
        messages = summary.messages,
        frames_lost = summary.frames_lost,
        "bag record: finalized"
    );
    Ok(summary)
}

/// How often the stop-bridge thread re-checks the shutdown conditions.
const RECORD_STOP_POLL: Duration = Duration::from_millis(20);

// =========================================================================
// `bag record --run` — attaching a recorder to a LIVE run.
// =========================================================================

/// Which live run `--run` should attach to.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RunTarget {
    /// `--run` with no value: attach to the one live run. TWO or more is a loud
    /// refusal rather than a pick — see [`select_run`].
    Sole,
    /// `--run <ID>`: a `run_id` (hex, `0x`-prefixed or bare) or a graph NAME.
    Named(String),
}

/// What [`select_run`] concluded. Each arm is a distinct operator outcome with
/// its own exit behaviour.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RunSelection {
    /// Attach to this run.
    Attach(Box<cerulion_core::transport::run_registry::RunRecord>),
    /// Nothing is announcing itself. NOT an error: an older `graph run`
    /// publishes no record, and a machine with no run at all is the ordinary
    /// case for the standalone verb. The caller says so and records locally.
    NoLiveRun,
    /// Several runs are live and the operator did not say which. A refusal,
    /// never a pick: recording the wrong run silently is worse than not
    /// recording, and the message names every candidate so the retry is a
    /// copy-paste.
    Ambiguous(Vec<String>),
    /// `--run <ID>` named something that is not live. The message lists what IS.
    NoSuchRun {
        /// What the operator asked for, echoed back.
        requested: String,
        /// The live runs, rendered — empty when nothing is announcing.
        live: Vec<String>,
    },
}

/// Render one run for an operator-facing list: the id first (it is what `--run`
/// takes) then the human label.
fn render_run_choice(rec: &cerulion_core::transport::run_registry::RunRecord) -> String {
    format!(
        "0x{:032x}  {}  (pid {})",
        rec.run_id, rec.graph_name, rec.supervisor_pid
    )
}

/// PURE: pick the run to attach to.
///
/// Matching a `Named` target is deliberately generous about SPELLING and strict
/// about AMBIGUITY. An operator reads a run id out of a log or a `run.json` and
/// may or may not carry the `0x`; a graph NAME is what they actually remember.
/// So a target matches on the id (either spelling, case-insensitive) or on an
/// exact graph name — and if it matches more than one run, that is
/// [`RunSelection::Ambiguous`], not a pick. Two runs of the SAME graph is the
/// normal way to reach that, which is exactly when guessing is most harmful.
///
/// `Ending` runs are candidates like any other. A run that has announced its
/// exit is still live enough to record the tail of, and refusing it would make
/// the verb's behaviour depend on a race with the run's own `Drop`.
pub fn select_run(
    records: &[cerulion_core::transport::run_registry::RunRecord],
    target: &RunTarget,
) -> RunSelection {
    let rendered = || records.iter().map(render_run_choice).collect::<Vec<_>>();
    match target {
        RunTarget::Sole => match records.len() {
            0 => RunSelection::NoLiveRun,
            1 => RunSelection::Attach(Box::new(records[0].clone())),
            _ => RunSelection::Ambiguous(rendered()),
        },
        RunTarget::Named(want) => {
            let want_trimmed = want.trim();
            let want_id = want_trimmed
                .strip_prefix("0x")
                .or_else(|| want_trimmed.strip_prefix("0X"))
                .unwrap_or(want_trimmed);
            let matched: Vec<&cerulion_core::transport::run_registry::RunRecord> = records
                .iter()
                .filter(|r| {
                    format!("{:032x}", r.run_id).eq_ignore_ascii_case(want_id)
                        || r.graph_name == want_trimmed
                })
                .collect();
            match matched.len() {
                0 => RunSelection::NoSuchRun {
                    requested: want_trimmed.to_string(),
                    live: rendered(),
                },
                1 => RunSelection::Attach(Box::new(matched[0].clone())),
                _ => RunSelection::Ambiguous(matched.into_iter().map(render_run_choice).collect()),
            }
        }
    }
}

/// The operator-facing refusal for an ambiguous or absent run.
///
/// One function so the two arms cannot drift in tone or in the remedy they name
/// — both end at the same place (`--run <ID>`), and the id column of the list is
/// the value to paste.
fn run_selection_error(selection: &RunSelection) -> Option<String> {
    match selection {
        RunSelection::Attach(_) | RunSelection::NoLiveRun => None,
        RunSelection::Ambiguous(live) => Some(format!(
            "several runs are live and `--run` did not say which. Refusing to guess, because \
             recording the wrong run looks exactly like recording the right one.\n  {}\nRe-run \
             with `--run=<ID>` (the first column; the `=` is required).",
            live.join("\n  ")
        )),
        RunSelection::NoSuchRun { requested, live } => Some(if live.is_empty() {
            format!(
                "`--run={requested}` names no live run, and NOTHING is announcing itself on \
                 this machine. Either the run has ended, or it is a `cerulion graph run` from a \
                 build too old to publish a run record: record without `--run`, or restart \
                 the graph with a current build."
            )
        } else {
            format!(
                "`--run={requested}` names no live run. These are live:\n  {}",
                live.join("\n  ")
            )
        }),
    }
}

/// A live run's on-disk description, as the recorder needs it.
///
/// Every field is `Option`/defaulted rather than required: a run directory the
/// recorder cannot fully read still yields a recording of the frames, which is
/// the part that cannot be re-obtained later. What it must never do is claim an
/// attachment it did not carry, so a missing artifact is DROPPED and named, not
/// substituted.
#[derive(Debug, Clone, Default)]
pub struct RunArtifacts {
    /// The effective `graph.yaml` the run executes.
    pub graph_yaml: Option<Vec<u8>>,
    /// The run's env snapshot.
    pub env_json: Option<Vec<u8>>,
    /// The recording host's identity.
    pub recorder_json: Option<Vec<u8>>,
    /// The run manifest, verbatim.
    pub run_json: Option<Vec<u8>>,
    /// The trace-ring TAGS the run declares in its manifest, in
    /// declaration order.
    ///
    /// EMPTY is a fact about the RUN rather than a failure to read one, and
    /// the always-on rings moved WHICH runs it is a fact about: a multi-process
    /// `cerulion graph run` provisions rings whether or not it records, so an
    /// empty list now means a run that declined them, one that could not have
    /// them, a monolith shape that mints none, or a declaration that never
    /// landed — and the `trace_rings` state beside it says which. A recorder
    /// that finds none records frames and attachments and says so, rather than
    /// inventing a tag from a naming convention — deriving
    /// `cer_rec_{graph}_{pid}_r{N}` here would couple this crate to a private
    /// spelling in the supervisor and guess the rank count.
    pub rings: Vec<String>,
    /// The run's own `trace_rings` statement, as this reader
    /// was able to read it.
    ///
    /// THREE shapes, not two, and the third is the one that bites: `Absent` is
    /// a fact about the RUN (it said nothing — an older build, or a declaration
    /// that was never written), while `Unrecognised` is a fact about THIS BUILD
    /// (the run said something a newer `cerulion` understands and this one does
    /// not). Only `Known` licenses a claim about what the run decided. See
    /// [`crate::run_dir::TraceRingsReport`].
    pub trace_rings: crate::run_dir::TraceRingsReport,
    /// Declared ranks whose scheduler-trace ring was never
    /// created.
    ///
    /// Non-empty means [`rings`](Self::rings) OVER-declares: the tags are
    /// stamped before creation, so a rank that failed is still named there. A
    /// reader must consult this before rendering a from-attach verdict, or it
    /// claims a trace for a ring nothing could open.
    pub declared_unavailable: Vec<crate::run_dir::RankUnavailable>,
    /// Whether a standing Flashback recorder is already
    /// draining this run's per-rank node-STATE rings.
    ///
    /// THREE shapes for the same reason [`trace_rings`](Self::trace_rings) has
    /// three, and the asymmetry matters more here: those rings are
    /// `OverrunPolicy::Backpressure`, so a second consumer LAPS the first, and
    /// an `Absent`/`Unrecognised` answer read as "nobody is draining them" is
    /// the confident-false claim that causes the lap. Only `Known` licenses
    /// either decision. See [`crate::run_dir::StateRingConsumerReport`].
    pub state_ring_consumer: crate::run_dir::StateRingConsumerReport,
    /// Artifacts that could not be read, each with its reason — surfaced so a
    /// thin bag is explained rather than merely thin.
    pub unreadable: Vec<(String, String)>,
}

impl RunArtifacts {
    /// Whether `run.json` was read AND parses as a JSON object — i.e. whether
    /// [`rings`](Self::rings) is a statement about the RUN rather than about
    /// this reader's failure.
    ///
    /// [`rings`](Self::rings) is derived by `run_manifest_ring_tags`, which
    /// returns an EMPTY vector for unparseable bytes exactly as it does for a
    /// manifest that declared none, so the vector alone cannot tell the two
    /// apart. Any caller turning `rings` into a CLAIM must ask this first.
    ///
    /// Deliberately the SAME predicate [`render_attach_run_json`] uses to choose
    /// between `run` and `run_manifest_unparsed` — read, parses, and IS AN
    /// OBJECT — so a `false` here guarantees that key is present and a verdict
    /// pointing at it cannot dangle. The object check is part of the predicate,
    /// not an extra: a bare `7` or `[]` parses as valid JSON while carrying no
    /// `rings` key it could ever declare one in.
    pub fn manifest_parsed(&self) -> bool {
        self.run_json
            .as_deref()
            .and_then(|b| serde_json::from_slice::<serde_json::Value>(b).ok())
            .is_some_and(|v| v.is_object())
    }
}

/// Read a run directory's four artifacts.
///
/// Best-effort BY DESIGN: this runs against a directory another process owns and
/// may be removing (its `Drop` deletes it), so an unreadable file is a fact to
/// report, never a reason to abandon a recording that is already the only copy
/// of what is on the wire.
pub fn read_run_artifacts(dir: &std::path::Path) -> RunArtifacts {
    let mut out = RunArtifacts::default();
    let slot = |name: &str, into: &mut Option<Vec<u8>>, unreadable: &mut Vec<(String, String)>| {
        match std::fs::read(dir.join(name)) {
            Ok(bytes) => *into = Some(bytes),
            Err(e) => unreadable.push((name.to_string(), e.to_string())),
        }
    };
    slot(
        crate::run_dir::RUN_GRAPH_FILE,
        &mut out.graph_yaml,
        &mut out.unreadable,
    );
    slot(
        crate::run_dir::RUN_ENV_FILE,
        &mut out.env_json,
        &mut out.unreadable,
    );
    slot(
        crate::run_dir::RUN_RECORDER_FILE,
        &mut out.recorder_json,
        &mut out.unreadable,
    );
    slot(
        crate::run_dir::RUN_MANIFEST_FILE,
        &mut out.run_json,
        &mut out.unreadable,
    );
    out.rings = out
        .run_json
        .as_deref()
        .map(run_manifest_ring_tags)
        .unwrap_or_default();
    // The run's own statement, and the per-rank failures the ring
    // list cannot carry. Both are read through `run_dir`'s parsers — beside the
    // writers that produce them — so the key names and their grammar are spelled
    // exactly once in the system.
    out.trace_rings = out
        .run_json
        .as_deref()
        .map(crate::run_dir::run_manifest_trace_rings)
        .unwrap_or_default();
    out.declared_unavailable = out
        .run_json
        .as_deref()
        .map(crate::run_dir::run_manifest_declared_unavailable)
        .unwrap_or_default();
    out.state_ring_consumer = out
        .run_json
        .as_deref()
        .map(crate::run_dir::run_manifest_state_ring_consumer)
        .unwrap_or_default();
    out
}

/// PURE: the ring tags a run manifest declares.
///
/// Tolerant BY DESIGN, in one direction only: an absent `rings` key, a
/// non-array, or an entry without a usable `tag` yields NO tag rather than an
/// error, because the manifest is written by a different process and possibly a
/// different version, and a recording must not be refused over a field the
/// recorder can simply not use. What it will not do is accept a MALFORMED tag —
/// an empty string would reach `shm_open` as a name the recorder never meant to
/// open.
///
/// Both shapes the manifest may carry are read: the object form
/// (`{"tag": …, "rank": …, "generation": …}`, whose sibling fields belong to the
/// one-bag-one-run check) and a bare string, so a producer that writes the
/// simpler form is not silently ignored.
fn run_manifest_ring_tags(run_json: &[u8]) -> Vec<String> {
    let Ok(doc) = serde_json::from_slice::<serde_json::Value>(run_json) else {
        return Vec::new();
    };
    let Some(entries) = doc.get("rings").and_then(|r| r.as_array()) else {
        return Vec::new();
    };
    entries
        .iter()
        .filter_map(|e| match e {
            serde_json::Value::String(s) => Some(s.as_str()),
            other => other.get("tag").and_then(|t| t.as_str()),
        })
        .filter(|t| !t.trim().is_empty())
        .map(str::to_string)
        .collect()
}

/// PURE: the topics a run DECLARES, from its own effective graph.
///
/// This is what makes `--run` mean "record this run" rather than "record
/// whatever is live on this machine". The two differ on any co-tenanted box —
/// a second graph, an external publisher, another `ros2 attach` bridge — and
/// `--all` would sweep all of it in (the co-tenancy residual).
///
/// It resolves NAMES only, which is why it needs no loaded node metadata: a
/// topic name comes from `prefix` + node id + the output's own `topic:`
/// override, all of which are in the YAML. The authoritative schema HASH is not
/// available here (it lives in each node's compiled `OutputMeta`), so these are
/// attach-mode taps whose descriptors are resolved from the wire hash by the
/// recorder's own schema ladder.
pub fn run_declared_topics(graph_yaml: &[u8]) -> CliResult<Vec<String>> {
    let text = std::str::from_utf8(graph_yaml).map_err(|e| {
        CliError::Validation(format!("the run's graph.yaml is not valid UTF-8: {e}"))
    })?;
    let config = cerulion_core::graph::parse_graph(text)
        .map_err(|e| CliError::Validation(format!("the run's graph.yaml did not parse: {e}")))?;
    let mut seen = std::collections::BTreeSet::new();
    for node in &config.nodes {
        for output in &node.outputs {
            seen.insert(cerulion_core::graph::resolve_output_topic(
                &config.prefix,
                &node.id,
                output,
            ));
        }
    }
    Ok(seen.into_iter().collect())
}

/// PURE: the `__cerulion/run.json` a MID-RUN bag carries.
///
/// It is the run's OWN manifest plus the facts only the recorder knows, and the
/// added keys are exactly those facts. The run's own object is preserved
/// verbatim under `run` rather than merged, so a reader can always tell what the
/// RUN said from what the RECORDER said — merging them would make a future
/// key collision silently rewrite the run's own claim.
///
/// A run manifest that will not parse is carried as opaque TEXT under
/// `run_manifest_unparsed`. Dropping it would lose the run id, which is the
/// whole point of the attachment; substituting a parsed shape we invented would
/// be worse.
///
/// `unreadable` is what the recorder TRIED to copy out of the run directory and
/// could not — the [`RunArtifacts::unreadable`] list. It is carried because
/// otherwise the failure is a `warn!` that scrolls away while the bag, the
/// durable artifact, is merely THIN: absent `graph.yaml` is then ambiguous
/// between "the run had none" and "the recorder could not read it", and only
/// one of those is the reader's problem. Emitted ONLY when non-empty, so a
/// healthy attach's bytes are unchanged.
pub fn render_attach_run_json(
    run: &cerulion_core::transport::run_registry::RunRecord,
    run_json: Option<&[u8]>,
    attached_at_ns: u64,
    first_step_recorded: Option<u64>,
    trace: &str,
    state_rings: &str,
    unreadable: &[(String, String)],
) -> Vec<u8> {
    let parsed: Option<serde_json::Value> = run_json
        .and_then(|b| serde_json::from_slice(b).ok())
        .filter(serde_json::Value::is_object);
    let unparsed = match (&parsed, run_json) {
        (None, Some(b)) => Some(String::from_utf8_lossy(b).into_owned()),
        _ => None,
    };
    let mut doc = serde_json::json!({
        "version": crate::run_dir::RUN_MANIFEST_VERSION,
        "run_id": format!("0x{:032x}", run.run_id),
        "graph_name": run.graph_name,
        "supervisor_pid": run.supervisor_pid,
        "run_started_at_ns": run.run_started_at_ns,
        "attached_at_ns": attached_at_ns,
        // The whole point of the attachment: this bag does NOT start at step 0,
        // and every reader must be able to see that without inference.
        "attached_mid_run": true,
        "first_step_recorded": first_step_recorded,
        "trace": trace,
        // A SIBLING key, never a clause of `trace`. The two
        // planes are different rings with different consumer rules — a run can
        // declare trace rings while starting no state-ring consumer — so one
        // string answering both would have to hedge on every arm.
        //
        // Spelled through `STATE_RINGS_KEY`, which `read_state_rings` reads back
        // through, so `bag info` cannot go looking for a key this writer stopped
        // producing.
        STATE_RINGS_KEY: state_rings,
    });
    if let Some(obj) = doc.as_object_mut() {
        if let Some(run_manifest) = parsed {
            obj.insert("run".to_string(), run_manifest);
        }
        if let Some(text) = unparsed {
            obj.insert(
                "run_manifest_unparsed".to_string(),
                serde_json::Value::String(text),
            );
        }
        // ABSENT on a healthy attach — a normal bag carries no such key
        // at all — and present only when something failed. That asymmetry is
        // deliberate: an always-present empty object would be one more thing a
        // reader has to interpret on every bag, whereas the key's presence is
        // itself the signal that this bag is thin ON PURPOSE.
        if !unreadable.is_empty() {
            obj.insert(
                "artifacts_unreadable".to_string(),
                serde_json::Value::Object(
                    unreadable
                        .iter()
                        .map(|(name, reason)| {
                            (name.clone(), serde_json::Value::String(reason.clone()))
                        })
                        .collect(),
                ),
            );
        }
    }
    let mut bytes = serde_json::to_vec_pretty(&doc)
        .expect("the attach run.json is a static-shape object; serialization cannot fail");
    bytes.push(b'\n');
    bytes
}

/// A resolved attach: the run and everything its directory could tell us.
#[derive(Debug, Clone)]
pub struct RunAttach {
    /// The registry record that named it.
    pub run: cerulion_core::transport::run_registry::RunRecord,
    /// What its directory held.
    pub artifacts: RunArtifacts,
}

/// Resolve `--run` against the live registry.
///
/// `Ok(None)` is the STANDALONE fallback — nothing is announcing itself, so the
/// verb records exactly as it did before run announcements and says so. That is a
/// back-compat path by construction (residual 4): an older `graph run`
/// publishes no record at all, so it is indistinguishable from no run, and the
/// correct response is to record what IS here rather than refuse.
///
/// `Err` is a refusal: ambiguity, or a named run that is not live. Both happen
/// BEFORE any tap opens, so a rejected invocation leaves no bag behind.
fn resolve_run_attach(
    manager: &TransportManager,
    target: &RunTarget,
    out: &mut dyn std::io::Write,
) -> CliResult<Option<RunAttach>> {
    use cerulion_core::transport::mirror_registry::GatherCompleteness;
    use cerulion_core::transport::run_registry;

    // The manager's OWN namespace, never the process-global one: on an isolated
    // SHM root (every test, every multi-process deployment) those are different
    // directories, and asking the wrong one answers about the wrong machine.
    let gather = manager
        .gather_live_runs(run_registry::RUN_GATHER_WINDOW)
        .map_err(|e| CliError::Validation(format!("could not gather live runs: {e}")))?;

    // The mirror-gather lesson, applied: a windowed LISTEN that heard nothing from a
    // live writer is NOT an absence. Falling straight through to standalone on
    // an INCOMPLETE gather would record a bare topic list while a describable
    // run was sitting right there, and the operator asked for the run — so the
    // uncertainty is stated rather than resolved by assumption.
    if let GatherCompleteness::Incomplete {
        live_writers,
        writers_heard,
    } = gather.completeness
    {
        tracing::warn!(
            live_writers,
            writers_heard,
            "bag record --run: the run registry did not settle — {live_writers} writer(s) are \
             live and {writers_heard} answered, so a run may exist that this gather did not \
             hear. Proceeding with what DID answer."
        );
    }

    let selection = select_run(&gather.records, target);
    if let Some(msg) = run_selection_error(&selection) {
        return Err(CliError::Validation(msg));
    }
    let run = match selection {
        RunSelection::Attach(run) => *run,
        RunSelection::NoLiveRun => {
            // Loud, and it names what it chose — the notice IS the contract that
            // a silent standalone fallback would break.
            tracing::warn!(
                "bag record --run: no live run is announcing itself on this machine — recording \
                 as a plain local capture instead. The bag will carry no graph, no env snapshot \
                 and no run identity. (A `cerulion graph run` from an older build publishes no \
                 run record.)"
            );
            writeln!(
                out,
                "no live run found — recording as a plain local capture (no graph, no run identity)"
            )?;
            return Ok(None);
        }
        RunSelection::Ambiguous(_) | RunSelection::NoSuchRun { .. } => {
            unreachable!("both refusal arms are returned as Err above")
        }
    };

    let artifacts = read_run_artifacts(std::path::Path::new(&run.run_dir));
    for (name, reason) in &artifacts.unreadable {
        tracing::warn!(
            run_dir = %run.run_dir,
            artifact = %name,
            reason = %reason,
            "bag record --run: this run artifact could not be read, so the bag will NOT carry \
             it (the run may be exiting — its directory is removed on a clean shutdown)"
        );
    }
    Ok(Some(RunAttach { run, artifacts }))
}

/// The attachment name a mid-run bag carries its run identity under.
pub const RUN_JSON_ATTACHMENT: &str = "__cerulion/run.json";

/// How a mid-run bag describes its scheduler trace — a CLOSED vocabulary, because
/// "there is no trace" has several causes and they are not interchangeable.
///
/// **The always-on rings moved which cause is ORDINARY.** A multi-process `graph run` now
/// provisions per-rank trace rings whether or not it records, so a manifest
/// declaring none is no longer the everyday shape — it is a run that DECLINED
/// them (which says so, in its own words), a run that could not HAVE them
/// (likewise), a monolith shape that mints none, or a binary that predates the
/// key. [`TRACE_NONE_NO_RINGS`] is what remains once the run has made no
/// statement at all, and it still says WHY rather than leaving a reader to
/// conclude the recorder lost something.
///
/// It names NO FLAG, deliberately, and that is the per-verb remedy rule rather
/// than terseness: the reader of this string is holding a bag from
/// `cerulion bag record --run`, and nothing that verb accepts can put a
/// scheduler trace into it — the rings are the RUN's to create, and the run has
/// already started. Naming `--record` would name a flag on a different verb for
/// a run that is already going; naming a `bag record` flag would name one that
/// does not exist. What is true is the cause, so that is what it says.
pub const TRACE_NONE_NO_RINGS: &str =
    "none: the run's manifest declared no trace rings and gave no reason, so there was none \
     to attach to. A run that declined them, or could not have them, records that in its own \
     `trace_rings` state; a bare empty list means a run whose declaration never landed, a \
     single-process run (which mints none), or a build older than the key. Either way a run \
     already under way cannot be given rings, and no `bag record` flag can add one.";

/// The run's manifest could not be READ, so what it declared is UNKNOWN.
///
/// Distinct from [`TRACE_NONE_NO_RINGS`], and the distinction is the whole
/// point: `RunArtifacts::rings` reads exclusively from the manifest, so an
/// unreadable manifest yields an EMPTY ring list — byte-identical to a manifest
/// that genuinely declared none. Rendering the `NONE_NO_RINGS` text for both
/// states a fact about the RUN ("its manifest declared no trace rings") on
/// evidence nobody has, which is the positive-claim-from-an-absence class this
/// crate splits everywhere else — `RecordCoverage::mirrors_established`'s
/// three-state `Option<bool>` and `enumerated`-vs-`discovery_requested`
/// are the same rule.
///
/// It matters more here than the symmetry suggests, because NOTHING ELSE
/// CORRECTS IT. The reconciliation for a frozen `trace` verdict is
/// `record_coverage.json`'s `rings_unavailable` plus `bag info`'s `trace:
/// NONE|PARTIAL` line — and both are gated on `RecordCoverage::trace_degraded()`,
/// i.e. `!rings_unavailable.is_empty()`. With an unreadable manifest there are
/// zero DECLARED rings, so nothing is ever recorded as unavailable, the
/// corrective line never fires, and this string is the bag's only statement
/// about its trace. That is why the caveat has to live in the string itself.
///
/// Names no flag, for the same per-verb reason as its sibling: the reader is
/// holding a bag from `cerulion bag record --run`, and nothing that verb accepts
/// can put a trace into it. What is actionable is `artifacts_unreadable` in the
/// same document, so that is what it points at.
pub const TRACE_UNKNOWN_NO_MANIFEST: &str =
    "unknown: this run's manifest could not be read at attach time, so whether it declared any \
     trace ring is unknown and none could be opened. The run was most likely exiting — a run \
     directory is removed on a clean shutdown. See `artifacts_unreadable` in this document for \
     what failed and why.";

/// The manifest was READ but could not be PARSED, so what it declared is
/// UNKNOWN.
///
/// The third degraded state, and the one with NO `artifacts_unreadable` entry:
/// `std::fs::read` SUCCEEDED, so [`RunArtifacts::unreadable`] is empty and
/// `resolve_run_attach`'s per-artifact `warn!` never fires. That is precisely
/// why it cannot borrow [`TRACE_UNKNOWN_NO_MANIFEST`]'s text, which points a
/// reader at `artifacts_unreadable` — a key this state does not put in the
/// document. The evidence here is `run_manifest_unparsed`, carried in the same
/// `run.json` by [`render_attach_run_json`], so that is what this string names.
///
/// Without this arm the state fell through to [`TRACE_NONE_NO_RINGS`] and the
/// bag CONTRADICTED ITSELF inside one document: `trace` claiming the run's
/// manifest declared no rings, beside `run_manifest_unparsed` in the same object
/// recording that the manifest could not be parsed at all. One artifact, two
/// answers — and the false one is the one a reader acts on.
///
/// **Reachable on the flagship path, not only on a corrupt disk or a version
/// skew.** `run_dir::declare_run_rings` rewrites `run.json` IN PLACE — it reads
/// the manifest, inserts the `rings` array, and writes the result back through
/// `run_dir::write_artifact`, whose `OpenOptions` carries `.truncate(true)`. The
/// truncation lands at `open`, the bytes at a subsequent `write_all`, so between
/// those two calls the manifest is ZERO BYTES on disk. It is not a window the
/// recorder can be kept out of: C0 publishes the run to the registry BEFORE the
/// deployment dispatch, while the ring declaration happens inside the record
/// bring-up (between all-workers-READY and the GO sentinel) — so the run is
/// discoverable, and therefore attachable, for the whole of it. An empty file
/// reads back `Ok(vec![])`: a SUCCESSFUL read of bytes that parse as nothing.
///
/// Names no flag, for the same per-verb reason as its two siblings: the reader
/// is holding a bag from `cerulion bag record --run`, and nothing that verb
/// accepts can put a trace into it. What is actionable is re-attaching, since
/// unlike its siblings this state is usually TRANSIENT.
pub const TRACE_UNKNOWN_UNPARSEABLE_MANIFEST: &str =
    "unknown: this run's manifest was read but could not be parsed, so whether it declared any \
     trace ring is unknown and none could be opened. A run replaces `run.json` atomically, so \
     this is not a half-written file caught mid-rewrite — the bytes on disk do not parse, which \
     is persistent. See `run_manifest_unparsed` in this document for what was read.";

/// What a mid-run attach found when it went looking for the
/// run's scheduler trace — the CLOSED cause vocabulary, as a type.
///
/// # Why this is a type and a pure function, not an `if`-chain at the call site
///
/// It was an `if`-chain, with four arms, and the arms are the interesting part:
/// every one of them is a different CLAIM about a different subject, and three
/// of the seven are claims a reader ACTS on ("this run declined", "this rank's
/// ring was never created"). An inline chain has no name for any state, cannot
/// be exercised without building a whole attach, and grows a fifth arm by
/// somebody appending an `else if` in the right place — which is how such a
/// chain can ship `TRACE_NONE_NO_RINGS` for an unparseable manifest and
/// contradict itself inside one document.
///
/// So the decision is a pure function over the four facts it actually reads, and
/// each state has a name a test can ask for.
///
/// # The order, and why it is the order
///
/// The arms are ordered BY HOW LITTLE EACH KNOWS, and every arm below a given
/// one is a strictly stronger claim:
///
/// 1. no manifest — no bytes at all;
/// 2. bytes, no meaning — read, did not parse;
/// 3. the run said it DECLINED — a claim, and the run's own;
/// 4. the run said it was REFUSED — likewise;
/// 5. the run declared rings and some rank's never existed — a claim about
///    specific ranks, which must outrank the from-attach arm so a bag never
///    renders "from the attach point" for a ring nothing could open;
/// 6. the run declared none and said nothing about why — the LEGACY arm (a
///    binary predating the key, or a declaration that was never written);
/// 7. rings, openable — from the attach point.
///
/// Arms 3-5 sit between the parse check and the empty-list check exactly because
/// they are the states an empty (or partial) list would otherwise be silently collapsed
/// into.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TraceVerdict {
    /// `run.json` could not be READ, so what the run declared is unknown.
    UnknownNoManifest,
    /// `run.json` was read but did not parse, so what the run declared is
    /// unknown.
    UnknownUnparseableManifest,
    /// The manifest declares a trace state this BUILD does not know — almost
    /// always a manifest written by a newer `cerulion`.
    ///
    /// Distinct from every arm below it, and from [`NoneNoRings`] above all:
    /// the run DID say something, so "the run declared no trace rings" is a
    /// confident FALSE claim, made on evidence that says the opposite. What is
    /// true is that this reader cannot read it, and that has a remedy the
    /// legacy text does not name.
    ///
    /// [`NoneNoRings`]: Self::NoneNoRings
    UnknownTraceState {
        /// The token verbatim, so the operator can act on it.
        raw: String,
    },
    /// The run DECLINED scheduler-trace rings at launch — BY CHOICE.
    DeclinedAtLaunch { reason: String },
    /// The run wanted rings and could not have them — BY REFUSAL.
    UnavailableAtLaunch { reason: String },
    /// The run declared rings, and at least one DECLARED RANK's ring was never
    /// created. The bag carries the trace of the ranks that exist and says which
    /// ones it could not open.
    ///
    /// `rings_declared` rides along because the sentence is not the same in both
    /// halves of this state: with rings left over, some trace IS attached; with
    /// every declared rank failed, none is. Claiming the first while the second
    /// is true would be the from-attach lie in a different sentence.
    DeclaredAbsent {
        ranks: Vec<crate::run_dir::RankUnavailable>,
        rings_declared: usize,
    },
    /// The run declared NO rings and said nothing about why — the legacy state
    /// (a binary predating the key, or a declaration that was never written).
    NoneNoRings,
    /// Rings were found and opened: the trace begins at the attach point.
    FromAttach,
}

/// The four facts [`choose_trace_verdict`] reads, gathered by the caller.
///
/// A borrowed input struct rather than four positional arguments because two of
/// them are `Option`/slice shaped and a positional `bool, bool, Option<&_>,
/// &[_], usize` call is unreadable at the site and trivially transposable in a
/// test.
#[derive(Debug, Clone, Copy)]
pub struct TraceVerdictFacts<'a> {
    /// Whether `run.json` was READ at all (bytes obtained, meaning not implied).
    pub manifest_read: bool,
    /// Whether those bytes parse as a JSON OBJECT — i.e. whether anything else
    /// in this struct is a statement about the RUN rather than about a failure
    /// to read one. Deliberately the same predicate `RunArtifacts::manifest_parsed`
    /// applies, so a verdict pointing at `run_manifest_unparsed` cannot dangle.
    pub manifest_parsed: bool,
    /// The run's own `trace_rings` statement, as this reader could read it —
    /// absent, known, or present-but-unrecognised. The last two are different
    /// facts about different parties; see
    /// [`crate::run_dir::TraceRingsReport`].
    pub trace_rings: &'a crate::run_dir::TraceRingsReport,
    /// Declared ranks whose ring was never created.
    pub declared_unavailable: &'a [crate::run_dir::RankUnavailable],
    /// How many trace rings the manifest declared that this reader can name.
    pub rings_declared: usize,
}

/// PURE — choose the cause a mid-run attach reports for its
/// scheduler trace.
///
/// See [`TraceVerdict`] for the arms and why they are in this order.
#[must_use]
pub fn choose_trace_verdict(facts: TraceVerdictFacts<'_>) -> TraceVerdict {
    use crate::run_dir::{TraceRingsDecl, TraceRingsReport};
    if !facts.manifest_read {
        return TraceVerdict::UnknownNoManifest;
    }
    if !facts.manifest_parsed {
        return TraceVerdict::UnknownUnparseableManifest;
    }
    // The run's own statement outranks anything inferred from the ring list,
    // because it is the only evidence that distinguishes the four things an
    // empty list can mean. `Declared` falls THROUGH deliberately: it says rings
    // were created, which is a claim about the whole run and not about any
    // particular rank, so the per-rank and the empty-list tests below still
    // have to run.
    match facts.trace_rings {
        // A state this build cannot read outranks everything below, and the
        // reason is the same one that orders the whole chain: `trace_rings` is
        // the run's PRIMARY statement about its trace, so a reader that cannot
        // read it cannot safely narrow to a more specific claim — the state it
        // could not parse may be exactly the one that contradicts the ring
        // list. Saying so, and naming the token, is the minimum correct claim.
        TraceRingsReport::Unrecognised { raw } => {
            return TraceVerdict::UnknownTraceState { raw: raw.clone() }
        }
        TraceRingsReport::Known(TraceRingsDecl::Declined { reason }) => {
            return TraceVerdict::DeclinedAtLaunch {
                reason: reason.clone(),
            }
        }
        TraceRingsReport::Known(TraceRingsDecl::Unavailable { reason }) => {
            return TraceVerdict::UnavailableAtLaunch {
                reason: reason.clone(),
            }
        }
        // `Declared` FALLS THROUGH deliberately: it is a claim about the whole
        // run and not about any particular rank, so the per-rank and
        // empty-list tests below still have to run. `Absent` falls through to
        // the LEGACY by-omission arm — and only because the key is genuinely
        // missing, which is a fact about the run rather than about this reader.
        TraceRingsReport::Known(TraceRingsDecl::Declared) | TraceRingsReport::Absent => {}
    }
    if !facts.declared_unavailable.is_empty() {
        return TraceVerdict::DeclaredAbsent {
            ranks: facts.declared_unavailable.to_vec(),
            rings_declared: facts.rings_declared,
        };
    }
    if facts.rings_declared == 0 {
        return TraceVerdict::NoneNoRings;
    }
    TraceVerdict::FromAttach
}

impl TraceVerdict {
    /// The operator-facing sentence, as the bag's `trace` value.
    ///
    /// **Names no flag, on any arm** (the project's rule, and the per-verb
    /// remedy rule the four original constants already followed): the reader of
    /// this string is holding a bag from `cerulion bag record --run`, and
    /// nothing that verb accepts can put a scheduler trace into it. Naming
    /// `--no-rings` would name a flag on a DIFFERENT verb, for a run that has
    /// already started and cannot be given rings now. What is true is the CAUSE,
    /// and where the run said it — so that is what each arm says.
    #[must_use]
    pub fn render(&self) -> String {
        match self {
            TraceVerdict::UnknownNoManifest => TRACE_UNKNOWN_NO_MANIFEST.to_string(),
            TraceVerdict::UnknownUnparseableManifest => {
                TRACE_UNKNOWN_UNPARSEABLE_MANIFEST.to_string()
            }
            TraceVerdict::UnknownTraceState { raw } => render_unknown_trace_state(raw),
            TraceVerdict::DeclinedAtLaunch { reason } => {
                format!("{TRACE_NONE_DECLINED_PREFIX}{}", render_reason(reason))
            }
            TraceVerdict::UnavailableAtLaunch { reason } => {
                format!("{TRACE_NONE_UNAVAILABLE_PREFIX}{}", render_reason(reason))
            }
            TraceVerdict::DeclaredAbsent {
                ranks,
                rings_declared,
            } => render_declared_absent(ranks, *rings_declared),
            TraceVerdict::NoneNoRings => TRACE_NONE_NO_RINGS.to_string(),
            TraceVerdict::FromAttach => TRACE_FROM_ATTACH.to_string(),
        }
    }
}

/// How much of an unreadable `trace_rings` token is quoted back.
///
/// The token comes from a file another process wrote and reaches an MCAP
/// attachment verbatim, so it is BOUNDED rather than trusted to be short. 120
/// characters is far more than any state this vocabulary will ever spell and far
/// less than a manifest can carry.
const UNKNOWN_TRACE_STATE_MAX_RAW: usize = 120;

/// PURE: render an unrecognised manifest state for a sentence that quotes it.
///
/// Shared by the two vocabularies that carry one — `trace_rings` and
/// `state_ring_consumer` — because both face the same two edges and only
/// one of them had got them right. An EMPTY value is reachable (a present key
/// holding `""` is present-but-unrecognised) and a bare `` `` `` reads as a
/// reader bug rather than a run that wrote nothing into a key it did create; an
/// over-long token must say it was CUT, or a 200-character value renders as if
/// it were the whole thing. Truncation is on a CHAR boundary, so a multi-byte
/// token cannot panic.
fn quote_unknown_state(raw: &str) -> String {
    // SANITIZED HERE, not at the call sites. The token comes from a `run.json`
    // another process wrote, and the sentence built from it reaches THREE
    // destinations: a `bag info` row, an MCAP attachment, and a `tracing::warn!`
    // on the operator's terminal. Only the first was sanitizing, so a crafted or
    // newer manifest could put a CSI sequence straight into the terminal via the
    // warn — the class `topic_cmd::sanitize_display` exists for, and the same
    // rule `topic list` already applies to LAN-supplied robot names.
    //
    // `sanitize_display` maps every C0/C1 control to U+FFFD, so it is
    // IDEMPOTENT: the call sites that already wrap this are unaffected.
    let raw = crate::topic_cmd::sanitize_display(raw.trim());
    if raw.is_empty() {
        "an empty value".to_string()
    } else if raw.chars().count() > UNKNOWN_TRACE_STATE_MAX_RAW {
        let cut: String = raw.chars().take(UNKNOWN_TRACE_STATE_MAX_RAW).collect();
        format!("`{cut}…` (truncated)")
    } else {
        format!("`{raw}`")
    }
}

/// The UNKNOWN-STATE sentence: this build cannot read what the run declared.
///
/// It names the TOKEN, because that is the whole difference between this arm and
/// the legacy one — an operator told "no trace rings were declared" goes looking
/// for a run that never happened, while one told which value could not be read
/// knows to upgrade. The quoting and the char-boundary bound are
/// [`quote_unknown_state`]'s.
fn render_unknown_trace_state(raw: &str) -> String {
    let quoted = quote_unknown_state(raw);
    format!(
        "unknown: this run's manifest declares a scheduler-trace state this build does not know \
         ({quoted}), so whether it carries a trace is unknown and none could be opened. The run \
         was most likely written by a NEWER cerulion — upgrade this build to read it. See \
         `trace_rings` in `run.json` (carried as `run` in this document) for the run's own \
         statement."
    )
}

/// A run's stated reason, rendered as a parenthetical — or NOTHING when the run
/// gave none.
///
/// An empty reason is a real state (`trace_rings: "declined:"` is a recognised
/// value: the STATE is the load-bearing half and the reason is whatever the run
/// managed to say), and rendering it as a bare ` ()` would look like a reader
/// bug rather than a run that was terse.
fn render_reason(reason: &str) -> String {
    let reason = reason.trim();
    if reason.is_empty() {
        String::new()
    } else {
        format!(" ({reason})")
    }
}

/// The DECLARED-ABSENT sentence: which ranks declared a ring that was never
/// created.
///
/// The RANKS are always named in full — they are the actionable part and there
/// is one per worker, so the list is bounded by the partition. The REASONS are
/// rendered inline only for a single rank; past that they are left to
/// `run.declared_unavailable` in this same document, because a per-rank reason
/// list on a 25-rank graph would bury the one sentence a reader needs.
///
/// The TAIL is chosen by whether any declared ring survives. "The ranks whose
/// rings DO exist are attached" is the ordinary case and is FALSE when every
/// declared rank failed — a bag with no trace at all, described as a partial
/// one, is the from-attach lie wearing a different sentence.
fn render_declared_absent(
    ranks: &[crate::run_dir::RankUnavailable],
    rings_declared: usize,
) -> String {
    let numbers = ranks
        .iter()
        .map(|r| r.rank.to_string())
        .collect::<Vec<_>>()
        .join(", ");
    let detail = match ranks {
        [one] => format!(" ({})", one.reason.trim()),
        _ => String::new(),
    };
    // `>` never holds on a well-formed manifest (tags are stamped for every
    // declared rank, so a failed rank is always ALSO in `rings`), but a reader
    // does not get to assume a well-formed manifest: `>=` keeps the weaker,
    // always-true sentence for anything that is not strictly "some survived".
    let tail = if ranks.len() >= rings_declared {
        "no declared rank's ring could be opened, so this bag carries NO scheduler trace at all"
    } else {
        "the ranks whose rings DO exist are attached from the attach point"
    };
    format!(
        "partial: rank(s) {numbers} declared a scheduler-trace ring that was never created{detail}, \
         so nothing could be opened for them — {tail}. See `run.declared_unavailable` in this \
         document for each rank's reason."
    )
}

/// The run DECLINED scheduler-trace rings at launch — BY CHOICE, not by
/// failure, and that difference is the whole reason this arm exists.
///
/// It cannot borrow [`TRACE_NONE_NO_RINGS`]'s text, which describes a run that
/// gave NO REASON — a declaration that was never written, a shape that mints
/// none, a build older than the key. On a run that declined, every one of those is
/// FALSE: it could have had rings and said no, and telling an operator to go
/// looking at their build sends them away from their own launch line.
///
/// Names no flag, by design: it names the cause and where the run recorded
/// it, and `cerulion graph run --help` is where the switch is discoverable.
pub const TRACE_NONE_DECLINED_PREFIX: &str =
    "none: this run DECLINED scheduler-trace rings at launch, so there was none to attach to. \
     This is a choice the run made, not a failure — see `trace_rings` in `run.json` (carried as \
     `run` in this document) for the run's own statement";

/// The run wanted rings and was REFUSED one — by a resource gate, or by a run
/// shape whose clock cannot produce a resimmable trace.
///
/// Distinct from [`TRACE_NONE_DECLINED_PREFIX`] for the reason the whole
/// vocabulary is split: an operator who chose this gets a different next step
/// from one whose machine denied it. Collapsing the two would tell somebody
/// staring at a full `/dev/shm` that they had asked for this.
pub const TRACE_NONE_UNAVAILABLE_PREFIX: &str =
    "none: this run wanted scheduler-trace rings and could not have them, so there was none to \
     attach to. This is a refusal, not a choice — see `trace_rings` in `run.json` (carried as \
     `run` in this document) for the reason the run recorded";

#[cfg(test)]
mod trace_verdict_tests {
    use super::*;
    use crate::run_dir::{RankUnavailable, TraceRingsDecl, TraceRingsReport};

    /// The facts of a HEALTHY attach — rings declared, nothing failed. Every
    /// test below perturbs exactly ONE field of it, so what each arm is keyed on
    /// is visible in the diff rather than in prose.
    fn healthy<'a>(
        trace_rings: &'a TraceRingsReport,
        declared_unavailable: &'a [RankUnavailable],
    ) -> TraceVerdictFacts<'a> {
        TraceVerdictFacts {
            manifest_read: true,
            manifest_parsed: true,
            trace_rings,
            declared_unavailable,
            rings_declared: 2,
        }
    }

    /// A `trace_rings` value this build knows.
    fn known(decl: TraceRingsDecl) -> TraceRingsReport {
        TraceRingsReport::Known(decl)
    }

    /// ONE oracle per state, eight states, each keyed on a
    /// single perturbation of the healthy fixture.
    ///
    /// This is a set-equality assertion in disguise: the eight verdicts must
    /// also be seven DISTINCT sentences, checked at the bottom. Without that, an
    /// arm that renders a neighbour's text passes every individual assertion —
    /// which is precisely how an unnamed chain renders `NONE_NO_RINGS` for
    /// an unparseable manifest.
    #[test]
    fn every_trace_state_has_its_own_verdict_and_its_own_sentence() {
        let declined = known(TraceRingsDecl::Declined {
            reason: "the run declined them at launch".to_string(),
        });
        let unavailable = known(TraceRingsDecl::Unavailable {
            reason: "/dev/shm free 12 MiB < 40 MiB".to_string(),
        });
        let declared = known(TraceRingsDecl::Declared);
        let unrecognised = TraceRingsReport::Unrecognised {
            raw: "partial: 3 of 4 ranks".to_string(),
        };
        let one_failed = [RankUnavailable {
            rank: 1,
            reason: "shm_open: No space left on device".to_string(),
        }];
        let none: [RankUnavailable; 0] = [];

        // 1. No bytes at all. Every other fact is irrelevant and is deliberately
        //    left at its healthy value: an unread manifest must not be rescued
        //    by a ring count nobody read out of it.
        assert_eq!(
            choose_trace_verdict(TraceVerdictFacts {
                manifest_read: false,
                ..healthy(&declared, &none)
            }),
            TraceVerdict::UnknownNoManifest
        );

        // 2. Bytes, no meaning.
        assert_eq!(
            choose_trace_verdict(TraceVerdictFacts {
                manifest_parsed: false,
                ..healthy(&declared, &none)
            }),
            TraceVerdict::UnknownUnparseableManifest
        );

        // 3. A state this BUILD cannot read. Note the ring count is left at
        //    the HEALTHY 2: an unreadable primary statement outranks a ring
        //    list, because the state it could not parse may be the one that
        //    contradicts it.
        assert_eq!(
            choose_trace_verdict(healthy(&unrecognised, &none)),
            TraceVerdict::UnknownTraceState {
                raw: "partial: 3 of 4 ranks".to_string()
            }
        );

        // 4. The run DECLINED — a claim, and the run's own.
        assert_eq!(
            choose_trace_verdict(TraceVerdictFacts {
                rings_declared: 0,
                ..healthy(&declined, &none)
            }),
            TraceVerdict::DeclinedAtLaunch {
                reason: "the run declined them at launch".to_string()
            }
        );

        // 5. The run was REFUSED.
        assert_eq!(
            choose_trace_verdict(TraceVerdictFacts {
                rings_declared: 0,
                ..healthy(&unavailable, &none)
            }),
            TraceVerdict::UnavailableAtLaunch {
                reason: "/dev/shm free 12 MiB < 40 MiB".to_string()
            }
        );

        // 6. Rings declared, one rank's never created.
        assert_eq!(
            choose_trace_verdict(healthy(&declared, &one_failed)),
            TraceVerdict::DeclaredAbsent {
                ranks: one_failed.to_vec(),
                rings_declared: 2,
            }
        );

        // 7. LEGACY: no rings, and the run said nothing about why.
        assert_eq!(
            choose_trace_verdict(TraceVerdictFacts {
                trace_rings: &TraceRingsReport::Absent,
                rings_declared: 0,
                ..healthy(&TraceRingsReport::Absent, &none)
            }),
            TraceVerdict::NoneNoRings
        );

        // 8. Healthy.
        assert_eq!(
            choose_trace_verdict(healthy(&declared, &none)),
            TraceVerdict::FromAttach
        );

        // …and the seven render seven DISTINCT sentences. A set equality, so an
        // arm that borrows a neighbour's text fails here even though its own
        // assertion above passed.
        let rendered = [
            TraceVerdict::UnknownNoManifest,
            TraceVerdict::UnknownUnparseableManifest,
            TraceVerdict::UnknownTraceState {
                raw: "r".to_string(),
            },
            TraceVerdict::DeclinedAtLaunch {
                reason: "r".to_string(),
            },
            TraceVerdict::UnavailableAtLaunch {
                reason: "r".to_string(),
            },
            TraceVerdict::DeclaredAbsent {
                ranks: one_failed.to_vec(),
                rings_declared: 2,
            },
            TraceVerdict::NoneNoRings,
            TraceVerdict::FromAttach,
        ]
        .iter()
        .map(TraceVerdict::render)
        .collect::<std::collections::BTreeSet<_>>();
        assert_eq!(rendered.len(), 8, "eight states, eight sentences");
    }

    /// **The mutation target:** collapsing a present-but-unrecognised
    /// `trace_rings` value onto an absent key.
    ///
    /// The two shapes yield an identical-looking "this reader has no known
    /// state" and they are facts about DIFFERENT PARTIES. Absent is about the
    /// RUN — it said nothing, so the legacy by-omission cause is accurate.
    /// Unrecognised is about THIS BUILD — the run said something a newer
    /// `cerulion` understands and this binary does not. Serving the legacy text
    /// there states "the run's manifest declared no trace rings" on evidence
    /// that says the opposite, which is the confident-false-claim class the
    /// whole vocabulary exists to prevent (`absent ⇒ unknown`, never
    /// `absent ⇒ a wrong answer`).
    ///
    /// The PAIR is the kill: unrecognised must reach the new verdict AND absent
    /// must still reach the legacy one. Either assertion alone is satisfied by a
    /// variant that routes both to the same place.
    #[test]
    fn a_trace_state_this_build_cannot_read_is_never_the_legacy_by_omission_cause() {
        let none: [RankUnavailable; 0] = [];
        let unrecognised = TraceRingsReport::Unrecognised {
            raw: "declined-pending: 2 ranks".to_string(),
        };

        // HALF ONE — present but unreadable ⇒ the new verdict, naming the token.
        let verdict = choose_trace_verdict(TraceVerdictFacts {
            rings_declared: 0,
            ..healthy(&unrecognised, &none)
        });
        assert_eq!(
            verdict,
            TraceVerdict::UnknownTraceState {
                raw: "declined-pending: 2 ranks".to_string()
            }
        );
        let text = verdict.render();
        assert_ne!(
            text, TRACE_NONE_NO_RINGS,
            "the legacy text claims the RUN declared none — here the run declared something"
        );
        assert!(
            text.contains("`declined-pending: 2 ranks`"),
            "the unreadable TOKEN is named — it is the whole difference between \
             this arm and the legacy one: {text}"
        );
        assert!(
            text.contains("NEWER cerulion") && text.contains("upgrade"),
            "…and the remedy is stated, which the legacy text cannot offer: {text}"
        );
        assert!(
            !text.contains("--record") && !text.contains("--no-rings"),
            "still no flag from another verb (project rule): {text}"
        );

        // HALF ONE-AND-A-HALF — the SAME verdict, reached through the
        // PRODUCTION PARSER rather than a hand-built report. Without this the
        // arm is only reachable from a fact struct a test wrote, so a collapse
        // inside `run_manifest_trace_rings` — the likelier place for one — leaves
        // this test GREEN while every real bag takes the wrong arm.
        let from_manifest = crate::run_dir::run_manifest_trace_rings(
            br#"{"version":1,"trace_rings":"declined-pending: 2 ranks","rings":[]}"#,
        );
        assert_eq!(
            choose_trace_verdict(TraceVerdictFacts {
                rings_declared: 0,
                ..healthy(&from_manifest, &none)
            }),
            TraceVerdict::UnknownTraceState {
                raw: "declined-pending: 2 ranks".to_string()
            },
            "the arm must be reachable from a real manifest, not only from a hand-built fact"
        );

        // HALF TWO — the ANTI-TAUTOLOGY and the other side of the kill: a key
        // that is genuinely ABSENT still reaches the legacy arm. Without this,
        // half one is satisfied by a variant that sends BOTH shapes to the new
        // verdict, which would be just as wrong in the other direction. Driven
        // through the parser too, for the same reason.
        let absent = crate::run_dir::run_manifest_trace_rings(br#"{"version":1,"rings":[]}"#);
        assert_eq!(absent, TraceRingsReport::Absent);
        assert_eq!(
            choose_trace_verdict(TraceVerdictFacts {
                rings_declared: 0,
                ..healthy(&absent, &none)
            })
            .render(),
            TRACE_NONE_NO_RINGS,
            "an absent key IS a fact about the run, and the legacy cause is accurate for it"
        );
    }

    /// The three shapes come out of the PARSER, not just out of hand-built
    /// facts — the production reader has to make the distinction or the arm
    /// above is unreachable from a real bag.
    ///
    /// Driven through `read_run_artifacts`' own parser over hand-written
    /// manifests, so this is the seam a real attach crosses.
    #[test]
    fn the_manifest_reader_separates_an_absent_key_from_a_value_it_cannot_read() {
        use crate::run_dir::run_manifest_trace_rings;

        // Absent: an older manifest.
        assert_eq!(
            run_manifest_trace_rings(br#"{"version":1,"rings":[]}"#),
            TraceRingsReport::Absent
        );
        // Known: the ordinary `--record` run.
        assert_eq!(
            run_manifest_trace_rings(br#"{"trace_rings":"declared"}"#),
            TraceRingsReport::Known(TraceRingsDecl::Declared)
        );
        // Unrecognised: a state a NEWER build wrote.
        assert_eq!(
            run_manifest_trace_rings(br#"{"trace_rings":"deferred: awaiting rank 3"}"#),
            TraceRingsReport::Unrecognised {
                raw: "deferred: awaiting rank 3".to_string()
            }
        );
        // An EMPTY value is present-but-unrecognised too — the run created the
        // key and put nothing in it, which is not the same as never writing it.
        assert_eq!(
            run_manifest_trace_rings(br#"{"trace_rings":""}"#),
            TraceRingsReport::Unrecognised { raw: String::new() }
        );
        assert!(
            TraceVerdict::UnknownTraceState { raw: String::new() }
                .render()
                .contains("an empty value"),
            "…and it renders as a described absence, never a bare pair of backticks"
        );
        // A NON-STRING is `Absent`, not `Unrecognised`: this arm promises a
        // token a human can act on, and a JSON array is not one.
        assert_eq!(
            run_manifest_trace_rings(br#"{"trace_rings":["declared"]}"#),
            TraceRingsReport::Absent
        );

        // The quoted token is BOUNDED — it reaches an MCAP attachment verbatim
        // from a file another process wrote. Multi-byte on purpose: truncating
        // mid-codepoint would panic rather than truncate.
        let long = "é".repeat(UNKNOWN_TRACE_STATE_MAX_RAW + 40);
        let text = TraceVerdict::UnknownTraceState { raw: long.clone() }.render();
        assert!(
            text.contains("(truncated)"),
            "over-long tokens are cut: {text}"
        );
        assert!(
            !text.contains(&long),
            "the WHOLE token is never quoted back — that is what bounding means"
        );
        assert!(
            text.contains(&"é".repeat(UNKNOWN_TRACE_STATE_MAX_RAW)),
            "…and exactly the cap's worth IS quoted, so the cut is at the cap \
             rather than somewhere arbitrary: {text}"
        );
        // The BOUNDARY is pinned on the other side too: exactly at the cap is
        // quoted whole, so the truncation arm is a real threshold.
        let at_cap = "é".repeat(UNKNOWN_TRACE_STATE_MAX_RAW);
        assert!(
            !TraceVerdict::UnknownTraceState { raw: at_cap }
                .render()
                .contains("(truncated)"),
            "a token exactly at the cap is not truncated"
        );
    }

    /// **The headline mutation target:** collapsing `declined` into
    /// the legacy `NO_RINGS` arm.
    ///
    /// The two states are byte-identical in `rings` — both empty — so nothing
    /// but the run's own statement separates them, and the sentences are not
    /// interchangeable: `TRACE_NONE_NO_RINGS` describes a run that said NOTHING
    /// (a declaration that was never written, a shape that mints none, a build older
    /// than the key), so an operator reading it on a run that DECLINED goes
    /// looking at their build or their run directory instead of at their own
    /// launch line.
    ///
    /// **The discriminator keys on what the legacy text still says.** Keying on a
    /// phrase like "only under --record" or on an issue reference would go
    /// VACUOUS as soon as the text drops them (rings are not recording-only), in
    /// exactly the direction that
    /// hides the collapse. It keys on the legacy text's surviving claim instead:
    /// that the run gave NO REASON, which is the one thing a declined run always
    /// does give.
    #[test]
    fn a_declined_run_is_never_rendered_as_the_legacy_no_rings_cause() {
        let declined = known(TraceRingsDecl::Declined {
            reason: "the run declined them at launch".to_string(),
        });
        let none: [RankUnavailable; 0] = [];
        let verdict = choose_trace_verdict(TraceVerdictFacts {
            rings_declared: 0,
            ..healthy(&declined, &none)
        });
        let text = verdict.render();

        assert_ne!(
            text, TRACE_NONE_NO_RINGS,
            "the legacy text is a DIFFERENT claim"
        );
        assert!(
            !text.contains("gave no reason"),
            "a declined run GAVE its reason — rendering the no-statement text over it sends an \
             operator looking at their build instead of at their own launch line: {text}"
        );
        // Belt and braces: the legacy text's own distinguishing phrases must not
        // reach a declined run either, whichever of them a future edit keeps.
        assert!(
            !text.contains("older than the key") && !text.contains("never landed"),
            "…and it is not an older build, nor a lost declaration: {text}"
        );
        assert!(
            text.contains("DECLINED"),
            "it says what happened, in the run's own vocabulary: {text}"
        );
        assert!(
            text.contains("the run declined them at launch"),
            "and carries the run's stated reason: {text}"
        );
        // The rule: the CAUSE, never another verb's flag.
        assert!(
            !text.contains("--no-rings") && !text.contains("--record"),
            "an absence explanation names no flag from another verb: {text}"
        );
        assert!(
            text.contains("trace_rings"),
            "it points at the key where the run recorded it: {text}"
        );

        // ANTI-TAUTOLOGY: the LEGACY state still renders the legacy text. Without
        // this, "declined is not the legacy text" is satisfied by deleting the
        // legacy arm outright.
        assert_eq!(
            choose_trace_verdict(TraceVerdictFacts {
                trace_rings: &TraceRingsReport::Absent,
                rings_declared: 0,
                ..healthy(&TraceRingsReport::Absent, &none)
            })
            .render(),
            TRACE_NONE_NO_RINGS
        );
    }

    /// **The second mutation target:** ignoring `declared_unavailable`
    /// and rendering `TRACE_FROM_ATTACH` for a rank whose ring could not be
    /// opened.
    ///
    /// This is the arm with the most to lose. Ring tags are stamped BEFORE
    /// creation, so a rank that failed is still in `rings` — the list is
    /// non-empty and the healthy arm is one `else` away. A bag that then says
    /// "from the attach point" claims a trace it does not carry for a rank
    /// nothing ever opened, and nothing downstream corrects it.
    #[test]
    fn a_declared_rank_whose_ring_was_never_created_is_never_from_the_attach_point() {
        let declared = known(TraceRingsDecl::Declared);
        let one = [RankUnavailable {
            rank: 1,
            reason: "shm_open: No space left on device".to_string(),
        }];
        // rings_declared is 2 — the healthy value — precisely because the ring
        // list still names the failed rank.
        let text = choose_trace_verdict(healthy(&declared, &one)).render();
        assert_ne!(
            text, TRACE_FROM_ATTACH,
            "a rank nothing opened is not attached"
        );
        assert!(text.contains("rank(s) 1"), "the RANK is named: {text}");
        assert!(
            text.contains("No space left on device"),
            "a single failed rank carries its reason inline: {text}"
        );
        assert!(
            text.contains("declared_unavailable"),
            "and points at the per-rank key: {text}"
        );

        // MANY ranks: every number is named, the reasons are left to the key —
        // a per-rank reason list on a 25-rank graph buries the sentence.
        let many = [
            RankUnavailable {
                rank: 3,
                reason: "first".to_string(),
            },
            RankUnavailable {
                rank: 7,
                reason: "second".to_string(),
            },
        ];
        let text = choose_trace_verdict(healthy(&declared, &many)).render();
        assert!(text.contains("rank(s) 3, 7"), "every rank is named: {text}");
        assert!(
            !text.contains("first") && !text.contains("second"),
            "past one rank the reasons live in `declared_unavailable`: {text}"
        );

        // The TAIL tracks whether anything survived. With 2 declared and 1
        // failed, some trace IS attached; with every declared rank failed, the
        // bag carries none — and saying "the ranks whose rings DO exist are
        // attached" there would be the from-attach lie in a different sentence.
        let both = [
            RankUnavailable {
                rank: 0,
                reason: "a".to_string(),
            },
            RankUnavailable {
                rank: 1,
                reason: "b".to_string(),
            },
        ];
        let survivors = choose_trace_verdict(healthy(&declared, &one)).render();
        assert!(
            survivors.contains("DO exist are attached"),
            "one of two failed: some trace is attached: {survivors}"
        );
        let none_survived = choose_trace_verdict(healthy(&declared, &both)).render();
        assert!(
            none_survived.contains("NO scheduler trace at all"),
            "both of two failed: the bag carries nothing: {none_survived}"
        );
        assert!(
            !none_survived.contains("DO exist are attached"),
            "…and must not ALSO claim some ranks are attached: {none_survived}"
        );

        // ANTI-TAUTOLOGY: with NO failed rank the same facts render the healthy
        // sentence. Without it, "not from-attach" is satisfied by an arm that
        // never renders from-attach at all.
        let none: [RankUnavailable; 0] = [];
        assert_eq!(
            choose_trace_verdict(healthy(&declared, &none)).render(),
            TRACE_FROM_ATTACH
        );
    }

    /// The order is the CONTRACT, and each precedence is asserted where a
    /// re-ordering would silently invert it.
    ///
    /// Every case here holds TWO facts at once, and in each the arm that knows
    /// LESS must win — because the stronger claim would be made on evidence the
    /// weaker arm has just said cannot be trusted.
    #[test]
    fn the_arms_are_ordered_by_how_little_each_knows() {
        let declined = known(TraceRingsDecl::Declined {
            reason: "r".to_string(),
        });
        let declared = known(TraceRingsDecl::Declared);
        let failed = [RankUnavailable {
            rank: 0,
            reason: "boom".to_string(),
        }];
        let none: [RankUnavailable; 0] = [];

        // An UNREAD manifest outranks everything — including a `trace_rings`
        // value nobody could have read out of it. (The facts are inconsistent by
        // construction; the point is which one the chooser trusts.)
        assert_eq!(
            choose_trace_verdict(TraceVerdictFacts {
                manifest_read: false,
                manifest_parsed: false,
                ..healthy(&declined, &failed)
            }),
            TraceVerdict::UnknownNoManifest
        );
        // UNPARSEABLE outranks every claim below it, for the same reason.
        assert_eq!(
            choose_trace_verdict(TraceVerdictFacts {
                manifest_parsed: false,
                ..healthy(&declined, &failed)
            }),
            TraceVerdict::UnknownUnparseableManifest
        );
        // A DECLINED run outranks the per-rank arm: it declared nothing, so no
        // rank can have failed to create what was never planned.
        assert_eq!(
            choose_trace_verdict(TraceVerdictFacts {
                rings_declared: 0,
                ..healthy(&declined, &failed)
            }),
            TraceVerdict::DeclinedAtLaunch {
                reason: "r".to_string()
            }
        );
        // A per-rank failure outranks BOTH the empty-list arm and the healthy
        // one — it is a claim about specific ranks either of them would erase.
        assert_eq!(
            choose_trace_verdict(TraceVerdictFacts {
                rings_declared: 0,
                ..healthy(&declared, &failed)
            }),
            TraceVerdict::DeclaredAbsent {
                ranks: failed.to_vec(),
                rings_declared: 0,
            }
        );
        // `Declared` FALLS THROUGH: it is a whole-run claim, so the per-rank and
        // empty-list tests still run under it. With rings and no failures that
        // means the healthy arm, not a fourth "the run said declared" sentence.
        assert_eq!(
            choose_trace_verdict(healthy(&declared, &none)),
            TraceVerdict::FromAttach
        );
    }

    /// A run that stated a STATE but no reason renders a clean sentence, not a
    /// dangling ` ()`.
    ///
    /// `trace_rings: "declined:"` is a recognised value — the state is the
    /// load-bearing half — so this is reachable from a terse writer, and an
    /// empty parenthetical reads as a reader bug rather than a terse run.
    #[test]
    fn a_state_with_no_reason_renders_without_an_empty_parenthetical() {
        for (decl, prefix) in [
            (
                known(TraceRingsDecl::Declined {
                    reason: "   ".to_string(),
                }),
                TRACE_NONE_DECLINED_PREFIX,
            ),
            (
                known(TraceRingsDecl::Unavailable {
                    reason: String::new(),
                }),
                TRACE_NONE_UNAVAILABLE_PREFIX,
            ),
        ] {
            let none: [RankUnavailable; 0] = [];
            let text = choose_trace_verdict(TraceVerdictFacts {
                rings_declared: 0,
                ..healthy(&decl, &none)
            })
            .render();
            assert_eq!(text, prefix, "a reasonless state renders the bare prefix");
            assert!(!text.contains("()"), "no dangling parenthetical: {text}");
        }
    }
}

/// How a mid-run bag describes what it did about the run's
/// per-rank node-STATE rings — a CLOSED vocabulary, because "this bag carries no
/// anchors" has several causes and they are not interchangeable.
///
/// # Why an attach may have to REFUSE state-ring discovery
///
/// A state ring is `OverrunPolicy::Backpressure`: every consumer stores its
/// cursor into ONE shared header slot and the producer trusts whoever published
/// last, so two consumers lap each other and the slower one is retired with an
/// `Overrun` it never asked for. Since decisions 75 + 89 the ordinary
/// `graph run` starts a standing Flashback window recorder holding the run's
/// capture-plane tag, so on the DEFAULT run shape a consumer already exists.
///
/// Attaching a second one does not merely cost this bag its anchors — it can
/// cost the STANDING recorder's, which is the run's black box. So when the run's
/// manifest says a recorder is standing, this attach declines the state plane
/// and says where anchors for this run do come from.
///
/// It names NO FLAG, on the same per-verb rule its trace sibling follows: the
/// reader is holding a bag from `cerulion bag record --run`, and nothing that
/// verb accepts changes what the RUN decided at launch.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StateRingVerdict {
    /// The run declares a standing recorder, so this attach declined the plane.
    RefusedStandingRecorder,
    /// The run declares no standing consumer, so this attach swept for rings.
    FromAttach,
    /// `run.json` could not be READ — nothing is known either way.
    UnknownNoManifest,
    /// `run.json` was READ but did not PARSE — nothing is known either way.
    ///
    /// Apart from [`UnknownNoManifest`](Self::UnknownNoManifest) for the reason
    /// its trace sibling keeps `TRACE_UNKNOWN_UNPARSEABLE_MANIFEST` apart: in
    /// this state `std::fs::read` SUCCEEDED, so `artifacts_unreadable` is EMPTY
    /// and pointing a reader there sends them to a key this state does not put
    /// in the document. The evidence here is `run_manifest_unparsed`, carried in
    /// the same object.
    ///
    /// A `run.json` rewritten
    /// in place would be zero bytes between `open` and `write_all` while the
    /// run was already attachable. `edit_run_manifest` replaces it
    /// atomically, so there is no such window and what remains is PERSISTENT: real
    /// corruption, a hand edit, or a crash that left unparseable bytes. The
    /// remedy is therefore inspecting them, not re-attaching.
    UnknownUnparseableManifest,
    /// The manifest carries no `state_ring_consumer` key — a build older than
    /// it, or a declaration that was never written.
    UnknownLegacyManifest,
    /// The manifest declares a state this BUILD does not know.
    UnknownState {
        /// The value verbatim, trimmed and bounded.
        raw: String,
    },
}

/// The attach REFUSED state-ring discovery: this run already has a consumer.
pub const STATE_RINGS_REFUSED_STANDING: &str =
    "refused: this run reported a standing Flashback recorder, which was already draining its \
     per-rank node-state rings. Those rings admit exactly ONE consumer — a second reader laps \
     the first — so this recording declined the state plane and carries no anchors of its own. \
     Anchors for this run live in that recorder's captures; nothing un-declares that word if \
     the recorder later stopped, so this says what the run REPORTED, not that a capture exists.";

/// The attach SWEPT for state rings from the attach point.
pub const STATE_RINGS_FROM_ATTACH: &str =
    "from the attach point: this run reports no standing consumer of its per-rank node-state \
     rings, so this recording swept for them. Nothing checkpointed before the attach is \
     recoverable.";

/// `run.json` could not be read or parsed.
pub const STATE_RINGS_UNKNOWN_NO_MANIFEST: &str =
    "unknown: this run's manifest could not be read at attach time, so whether a recorder is \
     already draining its per-rank node-state rings is unknown. This recording swept for them \
     anyway — declining on no evidence would cost it anchors for a consumer that may not \
     exist — so if one was standing, both readers may have lapped. See `artifacts_unreadable` \
     in this document.";

/// `run.json` was read but did not parse.
///
/// Its own text, never [`STATE_RINGS_UNKNOWN_NO_MANIFEST`]'s: that one points at
/// `artifacts_unreadable`, and a manifest this branch READ successfully puts no
/// entry there. Borrowing it would make one bag contradict itself about one
/// artifact — the class the trace vocabulary split for the same reason.
pub const STATE_RINGS_UNKNOWN_UNPARSEABLE: &str =
    "unknown: this run's manifest was read but could not be parsed, so whether a recorder is \
     already draining its per-rank node-state rings is unknown. A run replaces `run.json` \
     atomically, so this is not a half-written file caught mid-rewrite — the bytes on disk do \
     not parse, which is persistent: re-attaching reads the same ones. See \
     `run_manifest_unparsed` in this document for what was read. This recording swept for the \
     rings anyway — declining on no evidence would cost it anchors for a consumer that may not \
     exist.";

/// The manifest predates the key.
pub const STATE_RINGS_UNKNOWN_LEGACY: &str =
    "unknown: this run's manifest carries no state-ring-consumer statement, so it predates the \
     key, it is a run shape that declares none (a virtual-clock run arms no capture plane), or \
     its declaration never landed. This recording swept for the rings anyway — declining on no \
     evidence would cost it anchors for a consumer that may not exist — so if one was \
     standing, both readers may have lapped.";

/// The manifest declares a state this build cannot read.
const STATE_RINGS_UNKNOWN_STATE_PREFIX: &str =
    "unknown: this run declared a state-ring-consumer state this build does not understand";

impl StateRingVerdict {
    /// PURE: what an attach should do about the state plane, from the three
    /// facts it has.
    ///
    /// The arms are ordered BY HOW LITTLE EACH KNOWS, exactly as
    /// [`choose_trace_verdict`]'s are: no bytes, then bytes with no meaning,
    /// then the run's own claims. Only a claim this build UNDERSTANDS can
    /// decide, in either direction.
    #[must_use]
    pub fn choose(
        manifest_read: bool,
        manifest_parsed: bool,
        report: &crate::run_dir::StateRingConsumerReport,
    ) -> Self {
        use crate::run_dir::{StateRingConsumerDecl as D, StateRingConsumerReport as R};
        if !manifest_read {
            return Self::UnknownNoManifest;
        }
        if !manifest_parsed {
            return Self::UnknownUnparseableManifest;
        }
        match report {
            R::Known(D::Standing) => Self::RefusedStandingRecorder,
            R::Known(D::None { .. }) => Self::FromAttach,
            // The WHOLE value is carried; `render` is where it is bounded, by
            // the same `quote_unknown_state` its trace sibling uses. Truncating
            // twice would silently drop the marker the renderer adds, so a cut
            // value would print as a complete one.
            R::Unrecognised { raw } => Self::UnknownState { raw: raw.clone() },
            R::Absent => Self::UnknownLegacyManifest,
        }
    }

    /// Whether this verdict means the attach must NOT open the state rings.
    ///
    /// Exactly ONE arm refuses. Every UNKNOWN proceeds, deliberately: declining
    /// on absent evidence costs a bag its anchors to avoid contending with a
    /// consumer nobody observed, and a manifest that predates the key is the
    /// ORDINARY shape for every run started by a build older than this one.
    #[must_use]
    pub fn declines_state_plane(&self) -> bool {
        matches!(self, Self::RefusedStandingRecorder)
    }

    /// The `state_rings` string this bag's `run.json` carries.
    #[must_use]
    pub fn render(&self) -> String {
        match self {
            Self::RefusedStandingRecorder => STATE_RINGS_REFUSED_STANDING.to_string(),
            Self::FromAttach => STATE_RINGS_FROM_ATTACH.to_string(),
            Self::UnknownNoManifest => STATE_RINGS_UNKNOWN_NO_MANIFEST.to_string(),
            Self::UnknownUnparseableManifest => STATE_RINGS_UNKNOWN_UNPARSEABLE.to_string(),
            Self::UnknownLegacyManifest => STATE_RINGS_UNKNOWN_LEGACY.to_string(),
            Self::UnknownState { raw } => {
                let quoted = quote_unknown_state(raw);
                format!(
                    "{STATE_RINGS_UNKNOWN_STATE_PREFIX} ({quoted}), so whether one is already \
                     draining its per-rank node-state rings is unknown. This recording swept \
                     for them anyway. Most likely a newer `cerulion` wrote this run's manifest, \
                     or it was hand-edited."
                )
            }
        }
    }
}

/// The state-ring decision as `bag info` prints it.
#[cfg(test)]
mod state_rings_section {
    use super::*;

    /// A DECLINED attach and an unconfigured recording must not print
    /// identically — which is what they did before this block existed.
    ///
    /// The defect is structural rather than cosmetic. A declined attach carries
    /// no `state_coverage.json`, so `render_state_coverage_section` takes its
    /// `Absent` arm and prints NOTHING; a bag that was never configured for
    /// checkpoints carries none either, and prints nothing for the same reason.
    /// The `state_rings` key was written into the bag and read by no shipped
    /// command, so the ONLY durable record that a decision had been taken was
    /// invisible, and an operator holding an anchorless bag could not tell
    /// "nobody asked for anchors" from "the anchors are in this run's Flashback
    /// captures instead" — which is the actionable half.
    ///
    /// All three present shapes in one body, against each other: the pairwise
    /// distinctness is the property, and asserting any one of them alone is
    /// satisfied by a renderer that prints the same sentence every time.
    #[test]
    fn a_declined_attach_does_not_print_like_a_recording_that_never_checkpointed() {
        let declined = render_state_rings_section(&StateRingsReading::Present {
            verdict: Some(STATE_RINGS_REFUSED_STANDING.to_string()),
        });
        let swept = render_state_rings_section(&StateRingsReading::Present {
            verdict: Some(STATE_RINGS_FROM_ATTACH.to_string()),
        });
        let unknown = render_state_rings_section(&StateRingsReading::Present { verdict: None });
        let not_an_attach = render_state_rings_section(&StateRingsReading::NotAnAttach);

        // The block exists at all.
        assert!(
            declined.contains("state rings:"),
            "the decision must be PRINTED, not merely stored: {declined:?}"
        );
        // …and says which decision, in the attach's own words.
        assert!(
            declined.contains("refused") && declined.contains("standing Flashback recorder"),
            "a declined attach must say it declined, and why: {declined:?}"
        );
        // The no-flag rule reaches the rendered surface too.
        for text in [&declined, &swept, &unknown] {
            assert!(
                !text.contains("--record") && !text.contains("--no-rings"),
                "an absence names the CAUSE, never another verb's flag: {text:?}"
            );
        }

        // THE PIN: three distinct printed answers. Before this block, the first
        // two were identical to the third and to a bag with no story at all.
        let distinct: std::collections::BTreeSet<&str> =
            [declined.as_str(), swept.as_str(), unknown.as_str()]
                .into_iter()
                .collect();
        assert_eq!(
            distinct.len(),
            3,
            "declined / swept / unknown are three different facts and must read as three \
             different rows: {distinct:?}"
        );

        // An ordinary recording is SILENT — most bags are not attaches, and a
        // paragraph on every one of them trains an operator to skip the block.
        // This is also the anti-tautology half: without it, "the block prints"
        // is satisfied by a renderer that prints unconditionally.
        assert_eq!(
            not_an_attach, "",
            "a bag that is not a mid-run attach has no decision to report"
        );
    }

    /// An attach that said NOTHING gets an explicit UNKNOWN row, never silence
    /// and never a fabricated "none".
    ///
    /// This is the arm an older `bag record --run` bag lands on, and it is
    /// byte-indistinguishable from a declined one on the evidence `bag info`
    /// otherwise has: both carry a `run.json`, neither carries a
    /// `state_coverage.json`. Rendering it as silence would state the recording
    /// simply had no anchors — a positive claim from an absence.
    #[test]
    fn an_attach_that_said_nothing_is_rendered_as_unknown_not_as_none() {
        let unknown = render_state_rings_section(&StateRingsReading::Present { verdict: None });

        assert!(
            unknown.contains("state rings: unknown"),
            "the row must exist and be labelled UNKNOWN: {unknown:?}"
        );
        assert!(
            unknown.contains("was never written down"),
            "…and say that nobody recorded the answer, rather than answering it: {unknown:?}"
        );
        // The trap this row is for: an absent `state_coverage.json` beside an
        // unknown decision proves nothing, and the row has to say so or a reader
        // draws exactly the wrong conclusion from the block above it.
        assert!(
            unknown.contains("state_coverage.json"),
            "…and disarm the absent sibling manifest, which is the evidence a reader would \
             otherwise reason from: {unknown:?}"
        );
        // …and it must NOT claim to know WHICH of the four runs it is holding.
        //
        // A draft of this row said the bag "was written before this record
        // existed" and closed "there is no other way to reach this". Three other
        // runs reach it — a virtual-clock run never takes the decision, a
        // bring-up attach reads the manifest before it is taken, and a failed
        // declaration leaves nothing behind — so the row asserted a falsehood on
        // every one of them. The exclusivity claim is what made it false, so
        // that is what is forbidden here; the two causes an operator is most
        // likely to actually be holding are required by name.
        assert!(
            unknown.contains("bring-up") && unknown.contains("virtual-clock"),
            "the row must name the causes it cannot rule out, not pick one: {unknown:?}"
        );
        assert!(
            !unknown.contains("no other way"),
            "…and must never claim to have excluded the others: {unknown:?}"
        );
        // It must NOT borrow either decided sentence.
        assert!(
            !unknown.contains("refused") && !unknown.contains("from the attach point"),
            "an unknown must not read as a decision: {unknown:?}"
        );
    }

    /// A PRESENT value this build cannot read is not an absence — and neither
    /// is an EMPTY one.
    ///
    /// Both are easy to get wrong. `as_str()` answers `None`
    /// for an object, an array, a number and a null alike, so a naive reader
    /// folds all of them onto the "carries no state-ring decision" row — a
    /// confident false claim about a bag that carries one, which then tells the
    /// operator to discard the correct inference. An empty string would render a
    /// bare `state rings: ` label.
    ///
    /// The object form is the case that matters: it is the single most likely
    /// way a newer `cerulion` extends this key, which is exactly when a reader
    /// must say "I cannot read this" rather than "there is nothing here".
    #[test]
    fn a_value_this_build_cannot_read_is_not_reported_as_no_decision() {
        // Driven through the CLASSIFIER, not hand-built: the defect this arm
        // exists for lives in the classification, and a hand-built reading
        // cannot see it. (Measured — with the reading constructed by hand,
        // folding a non-string back onto the absence row left every arm green.)
        let reading = classify_state_rings_bytes(br#"{"state_rings":{"state":"standing"}}"#);
        assert_eq!(
            reading,
            StateRingsReading::UnreadableValue {
                raw: r#"{"state":"standing"}"#.to_string()
            },
            "a present NON-STRING is a fact about this build, not an absence"
        );
        // …and the other document shapes, from the same seam.
        assert_eq!(
            classify_state_rings_bytes(br#"{"state_rings":"standing"}"#),
            StateRingsReading::Present {
                verdict: Some("standing".to_string())
            }
        );
        assert_eq!(
            classify_state_rings_bytes(br#"{"version":1}"#),
            StateRingsReading::Present { verdict: None },
            "an ABSENT key is the legacy shape"
        );
        // A document that is valid JSON but not an OBJECT carries no key it
        // could ever have declared one in — the filter the typed siblings get
        // for free.
        for bogus in [&b"7"[..], &b"[]"[..], &b"null"[..], &b"{ not json"[..]] {
            assert!(
                matches!(
                    classify_state_rings_bytes(bogus),
                    StateRingsReading::Malformed(_)
                ),
                "a non-object manifest is MALFORMED, never an absence: {bogus:?}"
            );
        }

        let unreadable = render_state_rings_section(&reading);
        assert!(
            unreadable.contains("cannot read") && unreadable.contains("standing"),
            "the row must say it could not read the value, and NAME it: {unreadable:?}"
        );
        assert!(
            unreadable.contains("NOT the same as a bag that carries no decision"),
            "…and separate itself from the absence row, which is the confusion it exists for: \
             {unreadable:?}"
        );
        // THE PIN: it must not be the absence row wearing different words.
        let absent = render_state_rings_section(&StateRingsReading::Present { verdict: None });
        assert_ne!(unreadable, absent);
        assert!(
            !unreadable.contains("carries no state-ring decision"),
            "a present-but-unreadable value must never be reported as an absent one: \
             {unreadable:?}"
        );

        // An EMPTY value: its own sentence, never a bare label.
        let empty = render_state_rings_section(&StateRingsReading::Present {
            verdict: Some("   ".to_string()),
        });
        assert!(
            empty.contains("EMPTY state-ring decision"),
            "an empty value is NAMED: {empty:?}"
        );
        assert!(
            !empty.trim_end().ends_with("state rings:"),
            "…never rendered as a bare label, which reads as a renderer bug: {empty:?}"
        );
    }

    /// A foreign verdict is BOUNDED before it reaches the terminal.
    ///
    /// The value is written by another process into a file this reader does not
    /// control and is echoed verbatim; `sanitize_display` neuters control
    /// characters but does not truncate. Both sides of the ceiling are pinned so
    /// a widened bound cannot pass unnoticed, and the cut is on a CHAR boundary
    /// so a multi-byte value cannot panic.
    #[test]
    fn a_foreign_verdict_is_bounded_before_it_reaches_the_terminal() {
        let long = "é".repeat(STATE_RINGS_MAX_ECHO + 50);
        let out = render_state_rings_section(&StateRingsReading::Present {
            verdict: Some(long.clone()),
        });
        assert!(out.contains("(truncated)"), "{out:?}");
        assert!(
            !out.contains(&long),
            "…and must not carry the whole thing anyway"
        );

        // AT the ceiling: rendered whole, no marker.
        let at_cap = "x".repeat(STATE_RINGS_MAX_ECHO);
        let out = render_state_rings_section(&StateRingsReading::Present {
            verdict: Some(at_cap.clone()),
        });
        assert!(
            out.contains(&at_cap) && !out.contains("(truncated)"),
            "{out:?}"
        );

        // …and the ceiling really does clear this crate's own longest sentence,
        // or the block would truncate its own vocabulary.
        assert!(
            STATE_RINGS_REFUSED_STANDING.chars().count() < STATE_RINGS_MAX_ECHO,
            "the bound must never bite on a verdict this build wrote"
        );
    }

    /// A hostile `state_ring_consumer` value cannot drive the operator's
    /// terminal.
    ///
    /// The value is copied out of a `run.json` another process wrote and reaches
    /// THREE destinations: this row, the MCAP attachment, and — through
    /// `StateRingVerdict::render` — a `tracing::warn!` on the terminal. Only the
    /// row was sanitizing, so a crafted or newer manifest could put a CSI
    /// sequence straight into a terminal, the class `sanitize_display` exists
    /// for and the same rule `topic list` applies to LAN-supplied robot names.
    ///
    /// Both helpers sanitize now, and each owns its own path: the verbatim echo
    /// (`bound_verdict`, feeding the two `bag info` rows) and the quoted token
    /// (`quote_unknown_state`, feeding the verdict renderer that the warn and
    /// the MCAP attachment both print).
    ///
    /// Each case therefore pins ITS OWN helper and nothing else — which is only
    /// true because the call sites no longer double-wrap. While they did, the
    /// `row` and `unreadable` cases passed with `bound_verdict`'s sanitization
    /// deleted, and this doc claimed a coverage they did not have.
    #[test]
    fn a_hostile_manifest_value_cannot_drive_the_terminal() {
        // ESC + CSI erase-display, a CR overwrite, and a newline that would
        // forge a second log line.
        const HOSTILE: &str = "\u{1b}[2J\u{1b}[1;1Hrefused: all clear\r\nWARN everything is fine";

        // (1) the verbatim echo, as the `bag info` row prints it.
        let row = render_state_rings_section(&StateRingsReading::Present {
            verdict: Some(HOSTILE.to_string()),
        });
        // (2) the quoted token, as the verdict renderer builds it — this is the
        //     string the `bag record --run` warn interpolates.
        let warned = StateRingVerdict::UnknownState {
            raw: HOSTILE.to_string(),
        }
        .render();
        // (3) the unreadable-value row, the other foreign-echo path.
        let unreadable = render_state_rings_section(&StateRingsReading::UnreadableValue {
            raw: HOSTILE.to_string(),
        });

        for (what, text) in [
            ("row", &row),
            ("warn", &warned),
            ("unreadable", &unreadable),
        ] {
            for (name, ch) in [("ESC", '\u{1b}'), ("CR", '\r'), ("LF", '\n')] {
                // The renderers add their OWN framing newlines, so the check is
                // on the interpolated value: strip the frame first.
                let body = text.trim_matches('\n');
                assert!(
                    !body.contains(ch),
                    "{what} still carries a raw {name} from the manifest — a crafted value can \
                     drive the terminal: {body:?}"
                );
            }
            assert!(
                text.contains('\u{fffd}'),
                "{what} must REPLACE the controls rather than drop them, so a reader can see \
                 something was removed: {text:?}"
            );
            // …and the harmless text survives, or the sanitizer is just deleting.
            assert!(text.contains("all clear"), "{what}: {text:?}");
        }
    }

    /// The degraded arms behave like every other `bag info` manifest block.
    ///
    /// MALFORMED prints — that bag DOES have a decision somebody cannot read —
    /// while an unreadable attachment INDEX is silent, because it is not
    /// evidence that no decision exists.
    #[test]
    fn a_malformed_manifest_prints_and_an_unreadable_index_stays_quiet() {
        let malformed =
            render_state_rings_section(&StateRingsReading::Malformed("expected value".to_string()));
        assert!(
            malformed.contains("MALFORMED") && malformed.contains("expected value"),
            "{malformed:?}"
        );
        assert!(
            malformed.contains("frames themselves are unaffected"),
            "…and scope the damage to the REPORT, not the recording: {malformed:?}"
        );
        assert_eq!(
            render_state_rings_section(&StateRingsReading::IndexUnreadable),
            "",
            "an index nobody could read is not evidence that no decision exists"
        );
    }

    /// The writer and the reader spell the key ONCE.
    ///
    /// Scope, because the obvious reading is wrong: this does NOT catch
    /// writer/reader drift. Both sides spell the key through the same `const`,
    /// so drift is a compile-time impossibility rather than something a test
    /// could see — and this test re-implements the lookup inline rather than
    /// calling `read_state_rings`, which has exactly one caller (`bag_info`) and
    /// no unit caller at all.
    ///
    /// What it DOES pin is that the writer puts the decision under the key at
    /// all, and that a value found there reaches a RENDERED row rather than
    /// being parsed and dropped. The wiring from `read_state_rings` into
    /// `bag_info` is covered by the three e2e arms in
    /// `crates/cerulion_cli_engine/tests/bag_record_run_attach_test.rs`, which is where
    /// a wrong literal key would actually fail.
    #[test]
    fn the_state_rings_key_round_trips_between_the_writer_and_the_reader() {
        let run = cerulion_core::transport::run_registry::RunRecord {
            run_id: 0x1234,
            supervisor_pid: 7,
            run_started_at_ns: 1,
            state: cerulion_core::transport::run_registry::RunState::Live,
            graph_name: "g".to_string(),
            run_dir: "/tmp/g".to_string(),
        };
        let bytes = render_attach_run_json(
            &run,
            None,
            1,
            None,
            TRACE_FROM_ATTACH,
            STATE_RINGS_REFUSED_STANDING,
            &[],
        );
        let doc: serde_json::Value = serde_json::from_slice(&bytes).expect("valid JSON");
        assert_eq!(
            doc[STATE_RINGS_KEY],
            serde_json::json!(STATE_RINGS_REFUSED_STANDING),
            "the writer must put the decision under the key the reader looks for"
        );
        // …and the reader's own lookup, over the bytes the writer produced.
        let reading = StateRingsReading::Present {
            verdict: doc
                .get(STATE_RINGS_KEY)
                .and_then(|v| v.as_str())
                .map(str::to_string),
        };
        assert!(
            render_state_rings_section(&reading).contains("refused"),
            "the round trip must reach the RENDERED row, not merely the key"
        );
    }
}

/// The state-ring verdict — every arm, and the two edges its
/// trace sibling earned the hard way.
#[cfg(test)]
mod state_ring_verdict_tests {
    use super::*;
    use crate::run_dir::{StateRingConsumerDecl as D, StateRingConsumerReport as R};

    /// FIVE states, five sentences, and exactly ONE of them declines.
    ///
    /// The arms are ordered by how little each knows, so the interesting
    /// property is not that each renders — it is that the four UNKNOWNs all
    /// PROCEED. Declining on absent evidence would cost a bag its anchors to
    /// avoid contending with a consumer nobody observed, and an absent key is
    /// the ORDINARY shape for every run started by an older build.
    #[test]
    fn every_state_ring_state_has_its_own_verdict_and_only_one_declines() {
        let cases: &[(bool, bool, R, StateRingVerdict)] = &[
            (false, false, R::Absent, StateRingVerdict::UnknownNoManifest),
            (
                true,
                false,
                R::Absent,
                StateRingVerdict::UnknownUnparseableManifest,
            ),
            (
                true,
                true,
                R::Absent,
                StateRingVerdict::UnknownLegacyManifest,
            ),
            (
                true,
                true,
                R::Known(D::Standing),
                StateRingVerdict::RefusedStandingRecorder,
            ),
            (
                true,
                true,
                R::Known(D::None {
                    reason: "switched off".to_string(),
                }),
                StateRingVerdict::FromAttach,
            ),
            // The arm the reader's non-string branch feeds, and the one line
            // `choose` actually rewrote. Without a case here, folding
            // `Unrecognised` onto the legacy arm keeps every other assertion in
            // this file green.
            (
                true,
                true,
                R::Unrecognised {
                    raw: "pending".to_string(),
                },
                StateRingVerdict::UnknownState {
                    raw: "pending".to_string(),
                },
            ),
        ];
        for (read, parsed, report, expected) in cases {
            let got = StateRingVerdict::choose(*read, *parsed, report);
            assert_eq!(&got, expected, "read={read} parsed={parsed} {report:?}");
        }

        // The UNPARSEABLE arm must NOT borrow its sibling's sentence: that one
        // points at `artifacts_unreadable`, and a manifest this branch READ
        // successfully puts no entry there. One bag, two answers about one
        // artifact, and the false one is the one a reader acts on.
        let unparseable = StateRingVerdict::UnknownUnparseableManifest.render();
        assert!(
            unparseable.contains("run_manifest_unparsed")
                && !unparseable.contains("artifacts_unreadable"),
            "the unparseable arm points at the evidence it actually produces: {unparseable}"
        );
        assert_ne!(
            unparseable,
            StateRingVerdict::UnknownNoManifest.render(),
            "the two UNKNOWN-manifest arms are different facts about different subjects"
        );

        // FIVE states, FIVE distinct sentences — the backstop its trace sibling
        // carries. Without it two arms could render the same text and every
        // per-arm assertion above would still pass.
        let rendered: std::collections::BTreeSet<String> = [
            StateRingVerdict::RefusedStandingRecorder,
            StateRingVerdict::FromAttach,
            StateRingVerdict::UnknownNoManifest,
            StateRingVerdict::UnknownUnparseableManifest,
            StateRingVerdict::UnknownLegacyManifest,
        ]
        .iter()
        .map(StateRingVerdict::render)
        .collect();
        assert_eq!(
            rendered.len(),
            5,
            "five states, five sentences: {rendered:?}"
        );

        // Exactly one arm declines, and every other PROCEEDS.
        assert!(StateRingVerdict::RefusedStandingRecorder.declines_state_plane());
        for proceeds in [
            StateRingVerdict::FromAttach,
            StateRingVerdict::UnknownNoManifest,
            StateRingVerdict::UnknownUnparseableManifest,
            StateRingVerdict::UnknownLegacyManifest,
            StateRingVerdict::UnknownState {
                raw: "pending".to_string(),
            },
        ] {
            assert!(
                !proceeds.declines_state_plane(),
                "an UNKNOWN proceeds — declining on no evidence costs a bag its anchors for a \
                 consumer nobody observed: {proceeds:?}"
            );
        }

        // The no-flag rule across the whole vocabulary.
        for v in [
            StateRingVerdict::RefusedStandingRecorder,
            StateRingVerdict::FromAttach,
            StateRingVerdict::UnknownNoManifest,
            StateRingVerdict::UnknownUnparseableManifest,
            StateRingVerdict::UnknownLegacyManifest,
        ] {
            let text = v.render();
            assert!(
                !text.contains("--record") && !text.contains("--no-rings"),
                "an absence names the CAUSE, never another verb's flag: {text}"
            );
        }
    }

    /// The UNRECOGNISED arm renders like its sibling — including the two edges
    /// the sibling has dedicated arms for.
    ///
    /// Both were live defects when this verdict was first written: it truncated
    /// with no marker (a 300-character token rendering as a complete 120 one)
    /// and formatted an EMPTY value as bare backticks, which reads as a reader
    /// bug rather than as a run that wrote nothing into a key it did create.
    /// Both are now the SAME code as `trace_rings`', not a second copy of it.
    #[test]
    fn an_unrecognised_state_ring_value_is_rendered_like_its_trace_sibling() {
        // Over the cap: the reader must be told it was CUT.
        let long = "é".repeat(UNKNOWN_TRACE_STATE_MAX_RAW + 40);
        let text = StateRingVerdict::UnknownState { raw: long.clone() }.render();
        assert!(
            text.contains("(truncated)") && text.contains('…'),
            "an over-long value must say it was cut: {text}"
        );
        assert!(
            !text.contains(&long),
            "…and must not carry the whole thing anyway: {text}"
        );

        // AT the cap: rendered whole, no marker. The boundary is pinned on both
        // sides so a widened divisor cannot pass.
        let at_cap = "x".repeat(UNKNOWN_TRACE_STATE_MAX_RAW);
        let text = StateRingVerdict::UnknownState {
            raw: at_cap.clone(),
        }
        .render();
        assert!(
            text.contains(&at_cap) && !text.contains("(truncated)"),
            "{text}"
        );

        // EMPTY: named, never a bare pair of backticks.
        let text = StateRingVerdict::UnknownState { raw: String::new() }.render();
        assert!(
            text.contains("an empty value") && !text.contains("``"),
            "an empty value is NAMED, never rendered as a bare pair of backticks — which reads \
             as a reader bug rather than as a run that wrote nothing into a key it did \
             create: {text}"
        );
    }
}

/// The trace was attached mid-run: it begins at the first COMPLETE step, and
/// nothing before the attach — departures included — is recoverable.
pub const TRACE_FROM_ATTACH: &str =
    "from the attach point: records before it are outside the window, and a departure that \
     happened before it cannot be known";

/// How a topic's loss accounting should be READ — a three-way split.
///
/// The distinction is not cosmetic: bagd ZEROES `frames_lost`/`gap_events` as
/// unattributable when it disables gap detection, so an UNMEASURED topic and a
/// verified-clean one carry identical counters and can only be told apart by
/// the `sequence_anomaly` / `multi_publisher` flags. Rendering both as
/// `LOST 0 / GAPS 0` tells an operator their capture was verified when nothing
/// verified it. `cerulion_cli_engine::replay_engine` already classifies exactly
/// this case as `Lossy` with dedicated text; this is the same rule at the
/// recorder's own summary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LossReading {
    /// Gap detection ran and found nothing.
    Clean,
    /// Gap detection ran and measured loss.
    Lost { frames: u64, gaps: u64 },
    /// Gap detection was DISABLED at runtime by a wire-sequence anomaly, so the
    /// counters were zeroed as unattributable — loss is UNKNOWN, not zero.
    UnmeasuredAnomaly,
    /// The recorder kept NO per-tap health record for this topic at all.
    UnmeasuredNoRecord,
    /// The topic's sequence baseline RESET after its first frame (head
    /// loss), so the gap arithmetic could not attribute what was missed.
    UnmeasuredBaselineReset { resets: u64 },
    /// The topic has MORE THAN ONE writer, so gap detection was stood down
    /// (interleaved per-publisher sequences would false-positive). Loss is
    /// UNKNOWN by construction, not by failure.
    ///
    /// Reached two ways, and they read the same because the reason is
    /// the same — the topic was DECLARED `multi_publisher_topics` (never armed),
    /// or a second writer was OBSERVED at record time (`label_catch_up`, which
    /// bagd sets only at the arming transition). Which one it was is in the
    /// health record's own fields; what an operator must not be told either way
    /// is a verified `0`.
    UnmeasuredMultiPublisher,
}

impl LossReading {
    /// The column text for this reading. `"?"` where a number would be a lie.
    pub fn cells(self) -> (String, String) {
        match self {
            LossReading::Clean => ("0".to_string(), "0".to_string()),
            LossReading::Lost { frames, gaps } => (frames.to_string(), gaps.to_string()),
            LossReading::UnmeasuredAnomaly
            | LossReading::UnmeasuredMultiPublisher
            | LossReading::UnmeasuredNoRecord
            | LossReading::UnmeasuredBaselineReset { .. } => ("?".to_string(), "?".to_string()),
        }
    }
    /// True when nothing measured this topic's loss.
    pub fn is_unmeasured(self) -> bool {
        matches!(
            self,
            LossReading::UnmeasuredAnomaly
                | LossReading::UnmeasuredMultiPublisher
                | LossReading::UnmeasuredNoRecord
                | LossReading::UnmeasuredBaselineReset { .. }
        )
    }
}

/// Classify one topic's health. PURE — the oracle for the rendering above.
///
/// Order matters: the UNMEASURED arms are checked FIRST, because in exactly
/// those cases the counters were zeroed and would otherwise read as `Clean`.
pub fn classify_loss(health: Option<&cerulion_bagd::TopicHealth>) -> LossReading {
    let Some(h) = health else {
        // L9: no per-tap record at all is its OWN cause — rendering it as a
        // sequence anomaly would state a reason that never happened.
        return LossReading::UnmeasuredNoRecord;
    };
    if h.sequence_anomaly {
        return LossReading::UnmeasuredAnomaly;
    }
    // An OBSERVED second writer routes here too, and the
    // AUTHORITATIVE signal for it is the stand-down REASON — the field bagd
    // writes at the same instant it disables the arithmetic. `label_catch_up`
    // is an EMISSION flag (a catch-up record was handed to a writable writer),
    // and the two DIVERGE by construction: the stand-down is drain-side while
    // the emission rides a batch, so a dropped batch, and window-only recording
    // that mints no continuous bag at all, leave the arithmetic stood down with
    // no record written. `label_catch_up` and `multi_publisher` are kept as
    // belt-and-braces — the reason string is the documented-stable JSON spelling
    // (`GapDisableReason::as_str`), sanctioned to branch on, and three
    // independent terms is what stops one of them going quietly absent.
    //
    // Without this arm such a topic falls through (to `Clean` when its counters
    // are zero — which they are, since the stand-down zeroes nothing but stops
    // advancing them — or to whatever a genuinely-lossy prefix already
    // recorded) and reports a verified number for a stream whose gap arithmetic
    // bagd had already abandoned.
    if h.multi_publisher
        || h.label_catch_up
        || h.gap_detection_disabled_reason.as_deref() == Some("observed_multi_publisher")
    {
        return LossReading::UnmeasuredMultiPublisher;
    }
    if h.frames_lost > 0 || h.gap_events > 0 {
        return LossReading::Lost {
            frames: h.frames_lost,
            gaps: h.gap_events,
        };
    }
    // L2: baseline resets are head-loss evidence bagd records
    // separately from the gap counters. A topic whose baseline reset after its
    // first frame lost frames the gap arithmetic could not attribute, so
    // reporting a bare "0" would again claim a verification that did not
    // happen.
    if h.baseline_resets_after_first > 0 {
        return LossReading::UnmeasuredBaselineReset {
            resets: h.baseline_resets_after_first,
        };
    }
    LossReading::Clean
}

/// Human-readable run summary for [`bag_record`], rendered from bagd's own
/// per-tap accounting so nothing is re-derived (and nothing can disagree with
/// the `__cerulion/record_health.json` attachment in the bag).
///
/// The row source is `record_health.topics`, NOT `per_topic`. `per_topic` is
/// built per FRAME on bagd's inline path, so a tapped topic that published
/// nothing contributes no key and its row VANISHED — leaving an operator unable
/// to tell "never selected" from "selected but silent", which on a `-a` capture
/// is the difference between a working robot and a dead sensor. `build_record_health`
/// maps over the taps themselves, so every tap has a row including the silent ones.
pub fn render_record_summary(summary: &cerulion_bagd::BagdSummary) -> String {
    let mut out = String::new();
    out.push_str("\nbag record finalized:\n");
    for p in &summary.bag_paths {
        out.push_str(&format!("  {}\n", p.display()));
    }
    out.push_str(&format!(
        "{:<44} {:>10}  {:>10}  {:>7}\n",
        "TOPIC", "FRAMES", "LOST", "GAPS"
    ));

    let mut silent = Vec::new();
    let mut unmeasured = Vec::new();
    // EVERY tap gets a row — `record_health.topics` has one per tap, silent
    // topics included.
    for (topic, health) in &summary.record_health.topics {
        let reading = classify_loss(Some(health));
        let (lost, gaps) = reading.cells();
        out.push_str(&format!(
            "{:<44} {:>10}  {:>10}  {:>7}\n",
            topic, health.frames_recorded, lost, gaps
        ));
        if health.frames_recorded == 0 {
            silent.push(topic.as_str());
        }
        if reading.is_unmeasured() {
            unmeasured.push((topic.as_str(), reading));
        }
    }

    out.push_str(&format!(
        "total: {} frame(s), {} byte(s) over {} chunk(s).\n",
        summary.messages, summary.bytes, summary.chunks
    ));
    if summary.frames_lost > 0 || summary.dropped_unwritten > 0 || summary.headerless > 0 {
        // The middle clause must not read "dropped before the writer
        // existed": nothing is evicted before the
        // writer exists, so that phrase would name a state
        // the number cannot come from — and this is the FIFTH surface
        // carrying that claim, reachable in production through ordinary
        // tap-queue overflow (the guard's `frames_lost > 0` arm), which would
        // give one quantity three different names inside a single run.
        // It uses the SAME words as replay's rule-5e catch-all.
        out.push_str(&format!(
            "loss: {} frame(s) lost to tap-queue overflow, {} drained but never written to the \
             bag, {} recorded without a parseable wire header.\n",
            summary.frames_lost, summary.dropped_unwritten, summary.headerless
        ));
    }
    if !silent.is_empty() {
        out.push_str(&format!(
            "silent: {} topic(s) were tapped but never published to during the window — {}. A tap \
             has no listener, so it is never sent late-joiner history: it records only what is \
             published after it attaches.\n",
            silent.len(),
            silent.join(", ")
        ));
    }
    for (topic, reading) in &unmeasured {
        match reading {
            LossReading::UnmeasuredAnomaly => out.push_str(&format!(
                "unmeasured: '{topic}' shows LOST/GAPS as '?' — a wire-sequence anomaly at record \
                 DISABLED loss detection there, so its frame counts are unattributable. Loss on \
                 this topic is UNKNOWN, not zero.\n"
            )),
            // This arm is now reached by an OBSERVED second writer as
            // well as a declared one, so it may no longer say "declared" — on the
            // observed route the graph declared nothing and the sentence would be
            // affirmatively false. Which route it was is in record_health.json
            // (`multi_publisher` vs `label_catch_up`); the verdict is the same
            // either way, and the verdict is what this line exists to give.
            // The sentence names the ROUTE, never the fact: a topic DECLARED
            // `multi_publisher` stands its gap detection down at open whether or
            // not a second writer ever attaches, so "it has more than one
            // writer" would be affirmatively false on exactly those rows.
            LossReading::UnmeasuredMultiPublisher => out.push_str(&format!(
                "unmeasured: '{topic}' shows LOST/GAPS as '?' — it is declared multi_publisher, \
                 or a second writer was observed on it while recording, so gap detection was \
                 stood down (interleaved per-publisher sequences would false-positive). Loss on \
                 this topic is UNKNOWN by construction.\n"
            )),
            LossReading::UnmeasuredNoRecord => out.push_str(&format!(
                "unmeasured: '{topic}' shows LOST/GAPS as '?' — the recorder kept no per-tap \
                 health record for it, so nothing measured its loss.\n"
            )),
            LossReading::UnmeasuredBaselineReset { resets } => out.push_str(&format!(
                "unmeasured: '{topic}' shows LOST/GAPS as '?' — its wire-sequence baseline reset \
                 {resets} time(s) after the first frame (head loss), so the gap arithmetic could \
                 not attribute what was missed.\n"
            )),
            _ => {}
        }
    }
    // The ABSORBANCE shortfall, on the same surface
    // as the three lines above it and for the same reason — it belongs to the
    // "here is what you did not get" family, and this summary is what the
    // operator reads at the moment they can still act on it.
    //
    // Conditional on a SHORT verdict, so a healthy recording's summary is
    // byte-unchanged: a run whose every tap absorbed the stalls it measured has
    // nothing to report here, and the full per-topic verdict (including the
    // topics that absorbed) is in the health attachment this function already
    // names below.
    //
    // Only SHORT is listed. `unrankable` says two numbers could not be
    // RANKED and `no_claim` says nothing was comparable — neither is a
    // shortfall an operator can act on, and printing them here would put three
    // different epistemic states under one heading.
    //
    // EVER short, not short at the end: nothing on a verdict row is sticky
    // except `short_evaluations`, so a tap that could not absorb for most of the
    // run and recovered before shutdown would leave this line absent entirely.
    let short: Vec<(&str, &cerulion_bagd::TopicAbsorbance)> = summary
        .record_health
        .topics
        .iter()
        .filter_map(|(topic, h)| {
            let a = h.absorbance.as_ref()?;
            // The SAME discipline as `bag info`'s reader, on the same type.
            //
            // This surface reads an IN-PROCESS `BagdSummary` whose producer is
            // `debug_assert`ed consistent, so it is not exposed to a foreign
            // document today — but the type and the trust argument are shared,
            // and one document classified oppositely by two surfaces of one
            // program is the class this file's own comments keep naming. A row
            // that disagrees with itself is reported below rather than silently
            // included or silently dropped.
            if a.inconsistency_against(Some(&summary.record_health.drain_gaps.edges_us))
                .is_some()
            {
                return None;
            }
            (a.verdict == cerulion_bagd::AbsorbanceVerdict::Short || a.short_evaluations > 0)
                .then_some((topic.as_str(), a))
        })
        .collect();
    // …and named, on their own line, rather than vanishing between the two
    // filters above.
    let contradictory: Vec<&str> = summary
        .record_health
        .topics
        .iter()
        .filter_map(|(topic, h)| {
            h.absorbance.as_ref().and_then(|a| {
                a.inconsistency_against(Some(&summary.record_health.drain_gaps.edges_us))
                    .map(|_| topic.as_str())
            })
        })
        .collect();
    if !contradictory.is_empty() {
        out.push_str(&format!(
            "absorbance: {} topic(s) carry a verdict that DISAGREES WITH ITS OWN NUMBERS and \
             are reported by neither count above — {}. Read the record_health.json \
             attachment (absorbance) for what disagrees.\n",
            contradictory.len(),
            contradictory.join(", ")
        ));
    }
    if !short.is_empty() {
        // The REMEDY comes off each row rather than being one sentence, because
        // `CERULION_FLASHBACK_TAP_BUDGET_MB` moves a BUDGETED tap and nothing
        // else — and this verb (`cerulion bag record`) never builds a Flashback
        // plane, so every tap it opens is ceiling-deep and the budget knob would
        // be wrong on this surface every single time.
        let named = short
            .iter()
            .map(|(topic, a)| format!("{topic} ({})", a.remedy()))
            .collect::<Vec<_>>()
            .join("; ");
        // CAPABILITY-scoped. `Short` compares depth x rate against
        // the recorder-wide gap TAIL — a quantile over the whole run — so a
        // topic that did not publish across that stall lost nothing, and this
        // line must not assert a loss its own `loss:` sibling reports as 0.
        out.push_str(&format!(
            "absorbance: {} topic(s) had a tap queue too shallow to absorb the drain stalls this \
             run MEASURED — a topic that published through such a stall dropped frames at its \
             own SHM queue — {named}. Three \
             things the numbers cannot see: the rate behind each verdict is a WINDOW AVERAGE, \
             so a bursty topic is worse than its figure; the gaps are DRIVE-LOOP gaps, so a \
             topic whose own drain was skipped saw a longer stall than the one shown; and loss \
             before a tap's first drained frame is invisible to the counts above, so a zero \
             there means nothing was COUNTED.\n",
            short.len(),
        ));
    }
    // The trace degrade, on the same surface as the three
    // lines above it.
    //
    // `silent:` / `loss:` / `unmeasured:` are all "here is what you did not
    // get", and a scheduler trace the recorder could not open is the same class
    // — a whole artifact missing rather than some frames. Leaving it out made
    // this summary's silence ambiguous in exactly the way the qualified-verdict finding made
    // `bag info`'s silence ambiguous: a reader who saw three flavours of
    // shortfall reported and nothing about the trace could reasonably conclude
    // the trace was fine.
    //
    // It matters HERE and not only in the manifest because of WHEN each is
    // read: this block is what the operator sees at the moment they can still
    // act (re-attach, re-run), while `bag info` is read later and the `warn!`
    // goes to stderr, which is routinely not where this summary is going.
    // Conditional, so a healthy recording's summary is byte-unchanged.
    //
    // It reports only what `BagdSummary` actually carries — the names and the
    // transport's own errors. The DECLARED count (and therefore the NONE-vs-
    // PARTIAL fraction `bag info` renders) lives in `record_coverage.json`, so
    // this line points there rather than inventing a denominator.
    if !summary.rings_unavailable.is_empty() {
        out.push_str(&format!(
            "trace: {} declared scheduler-trace ring(s) could not be opened, so this bag's \
             scheduler trace is incomplete or absent — `cerulion bag info` renders the full \
             verdict. The run was most likely exiting: a ring's shared-memory name is unlinked \
             the moment its owner drops. The frames are unaffected.\n",
            summary.rings_unavailable.len()
        ));
        for (ring, err) in &summary.rings_unavailable {
            out.push_str(&format!(
                "  {:<42} {}\n",
                crate::topic_cmd::sanitize_display(ring),
                crate::topic_cmd::sanitize_display(err)
            ));
        }
    }
    // ALWAYS name the artifact. Pointing at it only when a counter is non-zero
    // hides it in exactly the case where the summary is least able to answer
    // the question — an unmeasured topic has all-zero counters.
    out.push_str(&format!(
        "per-topic detail (incl. first/last sequence and the reconciliation terms) is in the \
         bag's `{}` attachment.\n",
        cerulion_bagd::RECORD_HEALTH_ATTACHMENT
    ));
    if let Some(first) = summary.bag_paths.first() {
        out.push_str(&format!(
            "play it back with: cerulion bag play {}\n",
            first.display()
        ));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scan_of(topics: &[(&str, u64)]) -> BagScan {
        BagScan {
            channels: topics
                .iter()
                .map(|(t, n)| ChannelScan {
                    topic: (*t).to_string(),
                    schema_name: "geometry_msgs/Vector3".to_string(),
                    schema_hash: Some(builtin_hash()),
                    frames: *n,
                    max_frame_len: 64,
                    first_log_time: Some(0),
                    last_log_time: Some(1),
                })
                .collect(),
            total_frames: topics.iter().map(|(_, n)| *n).sum(),
            backwards_log_times: 0,
            headerless_frames: 0,
            span_ns: Some(1),
            completeness: "finalized".to_string(),
            finalized: true,
        }
    }

    /// Drive `pace_step` over a frame sequence with a FIXED (hand-supplied)
    /// wall clock, returning `(sleep_ns, slip_ns)` per frame.
    fn drive(stamps: &[(u64, u64)], rate: f64) -> Vec<(u64, u64)> {
        drive_full(stamps, rate)
            .into_iter()
            .map(|s| (s.sleep_ns, s.slip_ns))
            .collect()
    }

    /// As [`drive`], but returning the whole [`PaceStep`] so an arm can assert
    /// on `re_anchored` too.
    fn drive_full(stamps: &[(u64, u64)], rate: f64) -> Vec<PaceStep> {
        let mut schedule = 0u64;
        let mut prev = None;
        let mut out = Vec::new();
        for &(elapsed, ts) in stamps {
            let step = pace_step(elapsed, &mut schedule, prev, ts, rate);
            prev = Some(ts);
            out.push(step);
        }
        out
    }

    #[test]
    fn the_schedule_is_anchored_not_a_chain_of_sleeps() {
        // Three frames 40ms apart, with the wall clock held at 0 — the schedule
        // is cumulative from the anchor, so the sleeps are 0 / 40ms / 80ms, NOT
        // 0 / 40ms / 40ms. That difference is the whole point: chained sleeps
        // let every overshoot compound, so a 1kHz stream would drift slower
        // forever.
        let got = drive(&[(0, 1_000_000), (0, 41_000_000), (0, 81_000_000)], 1.0);
        assert_eq!(
            got,
            vec![(0, 0), (40_000_000, 0), (80_000_000, 0)],
            "the schedule must accumulate against the anchor"
        );
    }

    #[test]
    fn rate_scales_the_schedule() {
        // Hand oracle: 40 ms apart at 2x is 20 ms; at 0.5x, 80 ms.
        assert_eq!(drive(&[(0, 0), (0, 40_000_000)], 2.0)[1].0, 20_000_000);
        assert_eq!(drive(&[(0, 0), (0, 40_000_000)], 0.5)[1].0, 80_000_000);
    }

    #[test]
    fn a_late_frame_slips_and_is_never_delayed_further() {
        // The wall clock is ALREADY 100 ms past when frame 2's 40 ms slot
        // arrives: it must publish immediately (sleep 0) and report exactly
        // 60 ms of slippage. Nothing is dropped and nothing is reordered.
        let got = drive(&[(0, 0), (100_000_000, 40_000_000)], 1.0);
        assert_eq!(got[1], (0, 60_000_000));
        // And the schedule is NOT re-anchored to the late wall time: the next
        // frame's slot is still measured from the original anchor, so a
        // recovered machine catches back up rather than staying 60 ms behind.
        let got = drive(
            &[
                (0, 0),
                (100_000_000, 40_000_000),
                (100_000_000, 200_000_000),
            ],
            1.0,
        );
        assert_eq!(got[2], (100_000_000, 0));
    }

    #[test]
    fn pace_step_clamps_the_three_pathologies() {
        // 1. No predecessor — the first frame of a pass never waits.
        assert_eq!(drive(&[(0, 999_999_999_999)], 1.0), vec![(0, 0)]);
        // 2. Backwards stamps (a cross-epoch bag) publish immediately, never
        //    wait and never underflow.
        assert_eq!(drive(&[(0, 9_000_000_000), (0, 1_000)], 1.0)[1], (0, 0));
        // 3. A gap beyond the cap plays as exactly the cap — and the rate
        //    divide is applied AFTER the clamp, so --rate still speeds it up.
        assert_eq!(
            drive(&[(0, 0), (0, 3_600_000_000_000)], 1.0)[1].0,
            MAX_FRAME_GAP_NS
        );
        assert_eq!(
            drive(&[(0, 0), (0, 3_600_000_000_000)], 5.0)[1].0,
            MAX_FRAME_GAP_NS / 5
        );
        // Boundary: exactly at the cap is NOT clamped.
        assert_eq!(
            drive(&[(0, 0), (0, MAX_FRAME_GAP_NS)], 1.0)[1].0,
            MAX_FRAME_GAP_NS
        );
        // Identical stamps (a burst inside one clock tick) do not wait.
        assert_eq!(drive(&[(0, 500), (0, 500)], 1.0)[1], (0, 0));
    }

    #[test]
    fn a_backwards_stamp_re_anchors_instead_of_reporting_lag_forever() {
        // ONE clock domain whose PRODUCER RESTARTED — the only way a domain's
        // own stamps go backwards (a `--loop` wrap is the other). Frames at
        // 900 ms, then the restart drops to 100 ms, then 200 ms.
        //
        // NOTE — this test previously drove TWO PRODUCERS on unrelated
        // clocks through one schedule and asserted the result as correct. That
        // encoded a defect: producer A's 200 ms of recorded timeline was
        // stretched 5x and the assertion pinned it. Two producers are now two
        // DOMAINS with two schedules and never meet here; `pace_step` is
        // documented as operating on ONE domain, and the multi-domain contract
        // is pinned by `each_clock_domain_keeps_its_own_recorded_rate`.
        //
        // With a bare zero-clamp the schedule stalls while the wall clock keeps
        // moving, so the restart frame AND every frame after it report slip.
        // MEASURED on a real 79 s Go2 bag: 147 "behind schedule" frames, worst
        // 53.6 ms, while `topic hz` measured a faithful 20 Hz.
        //
        // The wall clock here TRACKS the schedule (the player sleeps until it),
        // which is what a healthy run looks like.
        let steps = drive_full(
            &[
                (0, 900_000_000),          // frame 0 — anchors at wall 0
                (0, 950_000_000),          // +50ms
                (50_000_000, 100_000_000), // RESTART — stamps drop
                (50_000_000, 200_000_000), // +100ms on the new epoch
            ],
            1.0,
        );
        assert!(!steps[1].re_anchored, "a forward delta is not a restart");
        assert!(steps[2].re_anchored, "a backwards stamp must re-anchor");
        assert_eq!(
            steps[2].slip_ns, 0,
            "a re-anchored frame is due NOW by construction — never 'late'"
        );
        // THE headline: the frame AFTER the restart is paced by its OWN 100ms
        // delta, not drowned in inherited lag.
        assert_eq!(
            (steps[3].sleep_ns, steps[3].slip_ns),
            (100_000_000, 0),
            "after a re-anchor the next 100ms delta must pace as 100ms with NO slip"
        );
        assert!(!steps[3].re_anchored);
    }

    #[test]
    fn the_runs_first_frame_is_never_reported_as_late() {
        // The first frame has no predecessor, and without a run
        // origin the schedule sits at 0 while the wall clock has ALREADY
        // advanced through `user_frames()` (a footer read), `next_user_frame()`
        // and the header parse. `schedule <= elapsed` then reports slip, so
        // EVERY run — however healthy — would print "N frame(s) published behind
        // schedule … Lower --rate" on a run that was perfectly on time.
        //
        // The caller seeds the schedule with the RUN ORIGIN (the wall instant
        // the first frame was reached), so 3 ms of walk-open costs nothing.
        const ORIGIN: u64 = 3_000_000;
        let mut schedule = ORIGIN;
        let first = pace_step(ORIGIN, &mut schedule, None, 500_000_000, 1.0);
        assert_eq!(
            first.slip_ns, 0,
            "the run's first frame is first BY CONSTRUCTION — it cannot be late"
        );
        assert_eq!(first.sleep_ns, 0, "and it publishes immediately");
        // The next frame's 40ms delta is measured from the ORIGIN, not from
        // time zero (which would make it instantly due).
        let second = pace_step(ORIGIN, &mut schedule, Some(500_000_000), 540_000_000, 1.0);
        assert_eq!(
            (second.sleep_ns, second.slip_ns),
            (40_000_000, 0),
            "the second frame must be due 40ms after the ORIGIN"
        );
    }

    #[test]
    fn a_later_channel_shares_the_run_origin_rather_than_drifting() {
        // A recorder writes each flush grouped by TOPIC, so channel B's frames
        // are REACHED after channel A's have been paced. Anchoring B at its own
        // arrival instant would bake that offset in PERMANENTLY — up to (K-1)
        // flush windows for K topics, about 7 s on a 75-topic `ros2 attach` bag.
        //
        // Sharing the run origin makes it a transient catch-up instead: B's
        // early frames are already due and publish at once, then B tracks its
        // own cadence.
        const ORIGIN: u64 = 0;
        // B's first frame is reached 200ms in (after A's first batch).
        let mut b = ORIGIN;
        let s0 = pace_step(200_000_000, &mut b, None, 0, 1.0);
        assert_eq!(s0.sleep_ns, 0, "an already-due frame never waits");
        assert_eq!(
            s0.slip_ns, 200_000_000,
            "the catch-up is REPORTED, not hidden"
        );
        // B catches up across its batch…
        for i in 1..=25u64 {
            let _ = pace_step(
                200_000_000,
                &mut b,
                Some((i - 1) * 10_000_000),
                i * 10_000_000,
                1.0,
            );
        }
        // …and is then AHEAD of the wall, i.e. pacing normally on the shared
        // origin rather than trailing it forever.
        assert!(
            b > 200_000_000,
            "after catching up, B's schedule must lead the wall clock (got {b})"
        );
    }

    #[test]
    fn each_channel_keeps_its_own_recorded_rate() {
        // THE contract, at the level the first defect lived AND the level the
        // second defect lived: a channel's schedule advances ONLY by its own deltas, so
        // neither another producer's clock offset nor the recorder's write
        // batching can reach it.
        //
        // Channel A: 20 Hz (50 ms apart) starting at 0.
        // Channel B: 3 Hz, on a clock 5 SECONDS ahead.
        let a: Vec<u64> = (0..5).map(|i| i * 50_000_000).collect();
        let b: Vec<u64> = (0..5).map(|i| 5_000_000_000 + i * 333_000_000).collect();

        let mut clock_a = 0u64;
        let mut clock_b = 0u64;
        let mut prev_a = None;
        let mut prev_b = None;
        let mut a_targets = Vec::new();
        for i in 0..5 {
            let _ = pace_step(0, &mut clock_a, prev_a, a[i], 1.0);
            prev_a = Some(a[i]);
            a_targets.push(clock_a);
            let _ = pace_step(0, &mut clock_b, prev_b, b[i], 1.0);
            prev_b = Some(b[i]);
        }
        let a_deltas: Vec<u64> = a_targets.windows(2).map(|w| w[1] - w[0]).collect();
        assert_eq!(
            a_deltas,
            vec![50_000_000; 4],
            "per-channel pacing must reproduce channel A's own 50ms cadence"
        );

        // ONE shared schedule (BOTH the naive single-clock shape AND what any
        // cross-channel inference degrades to) — the same stamps in file order
        // through a single clock. Run inline so the oracle above is falsifiable
        // rather than merely satisfied.
        let mut shared = 0u64;
        let mut prev = None;
        let mut shared_a_targets = Vec::new();
        for i in 0..5 {
            let _ = pace_step(0, &mut shared, prev, a[i], 1.0);
            prev = Some(a[i]);
            shared_a_targets.push(shared);
            let _ = pace_step(0, &mut shared, prev, b[i], 1.0);
            prev = Some(b[i]);
        }
        let shared_deltas: Vec<u64> = shared_a_targets.windows(2).map(|w| w[1] - w[0]).collect();
        assert_ne!(
            shared_deltas,
            vec![50_000_000; 4],
            "a SINGLE shared schedule must NOT reproduce A's cadence — if it does, this \
             oracle cannot detect the defect it exists to catch"
        );
        assert!(
            shared_deltas.iter().all(|d| *d >= 1_000_000_000),
            "the shared-schedule leg must be distorted by whole seconds, got {shared_deltas:?}"
        );
    }

    #[test]
    fn tap_grouped_write_batching_is_not_a_clock_break() {
        // At the unit level: `cerulion_bagd` writes each flush
        // TAP-GROUPED, so file order is per-tap arrival order: A's whole batch,
        // then B's whole batch, and B's first stamp sits BELOW A's last. Reading
        // that as a clock break stretches a 590ms bag to 1.255s.
        //
        // Per-channel pacing cannot see it: A's clock only ever sees A's stamps.
        let a_batch: Vec<u64> = (0..5).map(|i| i * 10_000_000).collect();
        let b_batch: Vec<u64> = (0..5).map(|i| i * 10_000_000).collect();

        let mut clock_a = 0u64;
        let mut prev_a = None;
        let mut restarts = 0;
        for &ts in &a_batch {
            let step = pace_step(0, &mut clock_a, prev_a, ts, 1.0);
            prev_a = Some(ts);
            if step.re_anchored {
                restarts += 1;
            }
        }
        // B's batch follows A's in the FILE, and its first stamp is far below
        // A's last — but B has its own clock, so nothing regresses.
        let mut clock_b = 0u64;
        let mut prev_b = None;
        for &ts in &b_batch {
            let step = pace_step(0, &mut clock_b, prev_b, ts, 1.0);
            prev_b = Some(ts);
            if step.re_anchored {
                restarts += 1;
            }
        }
        assert_eq!(
            restarts, 0,
            "write batching must produce NO timeline restarts"
        );
        // Each channel's span is its OWN recorded span, not a multiple of it.
        assert_eq!(clock_a, 40_000_000);
        assert_eq!(clock_b, 40_000_000);
    }

    #[test]
    fn a_skipped_channels_restart_is_still_counted_and_still_re_anchors() {
        // A filtered (`--topics`) or REFUSED (occupied slot) channel
        // paced with a HARDCODED elapsed of 0 makes the
        // re-anchor a no-op (`max(schedule, 0) == schedule`) and throws the
        // result away — so the phantom-lag defect would stay live whenever the
        // regressing channel is the skipped one, and the operator would lose the
        // restart counter that explains the timing.
        //
        // Driven at the `pace_step` level with a REAL elapsed clock: the
        // regression must re-anchor and must be observable.
        let mut schedule = 900_000_000u64;
        let step = pace_step(
            120_000_000,
            &mut schedule,
            Some(900_000_000),
            100_000_000,
            1.0,
        );
        assert!(
            step.re_anchored,
            "the skipped channel's restart must be visible"
        );
        assert_eq!(
            schedule, 900_000_000,
            "max() keeps the schedule ahead of the wall clock"
        );

        // With a hardcoded 0 the re-anchor could not move anything:
        let mut schedule = 900_000_000u64;
        let _ = pace_step(0, &mut schedule, Some(900_000_000), 100_000_000, 1.0);
        assert_eq!(
            schedule, 900_000_000,
            "documents WHY elapsed=0 was inert: max(schedule, 0) is always the schedule"
        );
    }

    #[test]
    fn the_wall_overrun_survives_a_re_anchor_that_resets_the_slip() {
        // `max_slip_ns` restarts at every timeline discontinuity, so
        // on a bag full of them it measures lag since the LAST restart. The
        // overrun compares the wall against the recording's own duration and
        // cannot be reset by anything.
        let mut summary = PlaySummary {
            topics: Vec::new(),
            passes: 1,
            refused: Vec::new(),
            // 30s wall for a 10s recording — 20s behind.
            elapsed: Duration::from_secs(30),
            max_slip_ns: 4_000_000,
            slipped_frames: 2,
            timeline_restarts: 500,
            scheduled_span_ns: 10_000_000_000,
            catchup_channels: 0,
        };
        assert_eq!(summary.wall_overrun_ns(), Some(20_000_000_000));
        let text = render_play_summary(&summary);
        assert!(text.contains("20.000s BEHIND"), "{text}");
        assert!(text.contains("cannot reset"), "{text}");

        // A run that kept up reports NO overrun — the floor absorbs the setup
        // cost plus the final publish, which is not lag.
        summary.elapsed = Duration::from_secs(10);
        assert_eq!(summary.wall_overrun_ns(), None);
        assert!(!render_play_summary(&summary).contains("BEHIND"));
        // Boundary, both sides.
        summary.elapsed = Duration::from_nanos(10_000_000_000 + WALL_OVERRUN_REPORT_FLOOR_NS);
        assert_eq!(
            summary.wall_overrun_ns(),
            Some(WALL_OVERRUN_REPORT_FLOOR_NS)
        );
        summary.elapsed = Duration::from_nanos(10_000_000_000 + WALL_OVERRUN_REPORT_FLOOR_NS - 1);
        assert_eq!(summary.wall_overrun_ns(), None);
    }

    #[test]
    fn validate_rate_rejects_zero_negative_and_non_finite() {
        assert!(validate_rate(1.0).is_ok());
        assert!(validate_rate(0.001).is_ok());
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let err = validate_rate(bad).unwrap_err().to_string();
            assert!(
                err.contains("--rate must be a finite number greater than 0"),
                "rate {bad} must be refused loudly, got: {err}"
            );
        }
    }

    #[test]
    fn empty_filter_selects_every_channel_in_order() {
        let scan = scan_of(&[("/a", 1), ("/b", 2)]);
        let got: Vec<&str> = select_channels(&scan, &[])
            .unwrap()
            .iter()
            .map(|c| c.topic.as_str())
            .collect();
        assert_eq!(got, vec!["/a", "/b"]);
    }

    #[test]
    fn a_named_filter_selects_only_those_and_dedups() {
        let scan = scan_of(&[("/a", 1), ("/b", 2), ("/c", 3)]);
        let got: Vec<&str> = select_channels(
            &scan,
            &["/c".to_string(), "/a".to_string(), "/c".to_string()],
        )
        .unwrap()
        .iter()
        .map(|c| c.topic.as_str())
        .collect();
        // Selection follows the FILTER's order (what the user asked for), and a
        // repeat selects once.
        assert_eq!(got, vec!["/c", "/a"]);
    }

    #[test]
    fn an_unknown_topic_in_the_filter_is_a_loud_error_naming_it_and_the_bag() {
        let scan = scan_of(&[("/a", 1), ("/b", 2)]);
        let err = select_channels(&scan, &["/a".to_string(), "/nope".to_string()])
            .unwrap_err()
            .to_string();
        assert!(err.contains("/nope"), "must name the missing topic: {err}");
        assert!(
            err.contains("/a") && err.contains("/b"),
            "must list what the bag DOES hold: {err}"
        );
    }

    fn test_walker() -> cerulion_core::codegen::FrameWalker {
        crate::topic_cmd::local_walker_from_workspace(None)
    }

    /// A channel whose hash IS a local built-in.
    fn builtin_hash() -> u64 {
        use cerulion_core::message::ShmMessage;
        native_ros2_messages::geometry_msgs::Vector3::SCHEMA_HASH
    }

    #[test]
    fn schema_resolution_keys_off_the_wire_hash_not_the_recorded_name() {
        // An attach-mode recorder — which is every bag
        // `cerulion bag record` writes — records the literal
        // `cerulion_bagd::ATTACH_MODE_SCHEMA_NAME` ("unknown"), because the wire
        // carries no name, while recording the HASH exactly. A viewer resolves
        // by hash; the bag's name never reaches it.
        //
        // Keying the "a viewer will render nothing" banner off the NAME
        // therefore fired on every bag this verb's own record -> play workflow
        // produces, including bags full of ordinary sensor_msgs frames that
        // render perfectly.
        let walker = test_walker();
        let mut c = scan_of(&[("/a", 1)]).channels.remove(0);

        // THE pin: name "unknown", hash a real built-in ⇒ resolves LOCALLY.
        c.schema_name = ATTACH_SCHEMA_NAME.to_string();
        c.schema_hash = Some(builtin_hash());
        let reading = resolve_schema(&walker, None, &c);
        assert!(
            matches!(reading, SchemaReading::Local { .. }),
            "an attach-mode bag's built-in frames must resolve: {reading:?}"
        );
        // …and the display recovers the real name the bag could not record.
        let shown = display_schema_name(&c, &reading);
        assert!(
            shown.contains("geometry_msgs/Vector3") && shown.contains("resolved by hash"),
            "{shown}"
        );

        // A hash nothing knows is genuinely unresolvable, whatever the name says.
        c.schema_name = "unitree_go/LowState".to_string();
        c.schema_hash = Some(0xDEAD_BEEF_DEAD_BEEF);
        assert_eq!(
            resolve_schema(&walker, None, &c),
            SchemaReading::Unresolvable
        );

        // No descriptor at all is its own state — there is no hash to resolve.
        c.schema_hash = None;
        assert_eq!(
            resolve_schema(&walker, None, &c),
            SchemaReading::NoDescriptor
        );
    }

    /// A catalog holding one vendor type (text + binding) and one NAMED-only
    /// binding — the corpus-skew shape, where the recorder knew what the type was
    /// called but assumed every reader would have its text.
    fn bag_catalog() -> cerulion_bag::BagSchemaCatalog {
        cerulion_bag::BagSchemaCatalog::new(
            vec![cerulion_core::SchemaDoc {
                qualified: "go/LowState".into(),
                encoding: cerulion_core::SchemaEncoding::Msg,
                text: "uint8 head\n".into(),
                deps: vec![],
            }],
            vec![
                cerulion_core::SchemaHashName {
                    schema_hash: 0x1111,
                    qualified: "go/LowState".into(),
                },
                cerulion_core::SchemaHashName {
                    schema_hash: 0x2222,
                    qualified: "sensor_msgs/FromAnotherCorpus".into(),
                },
            ],
        )
    }

    /// The bag's own provenance resolves what this machine cannot, and
    /// the three outcomes it produces are kept apart.
    #[test]
    fn the_bags_own_schema_records_resolve_what_this_machine_cannot() {
        let walker = test_walker();
        let catalog = bag_catalog();
        let mut c = scan_of(&[("/a", 1)]).channels.remove(0);
        c.schema_name = ATTACH_SCHEMA_NAME.to_string();

        // (1) Text in the bag ⇒ decodable HERE, even though nothing local knows it.
        c.schema_hash = Some(0x1111);
        assert_eq!(
            resolve_schema(&walker, None, &c),
            SchemaReading::Unresolvable,
            "without the bag there is nothing to resolve — the control that makes \
             the next assertion mean something"
        );
        let reading = resolve_schema(&walker, Some(&catalog), &c);
        assert_eq!(
            reading,
            SchemaReading::FromBag {
                name: "go/LowState".into()
            }
        );
        assert!(reading.is_resolvable());

        // (2) A binding with no text is NAMED but still undecodable — a different
        //     condition with a different remedy, so it gets its own arm.
        c.schema_hash = Some(0x2222);
        let reading = resolve_schema(&walker, Some(&catalog), &c);
        assert_eq!(
            reading,
            SchemaReading::NamedButUndecodable {
                name: "sensor_msgs/FromAnotherCorpus".into()
            }
        );
        assert!(
            !reading.is_resolvable(),
            "the bag NAMES this type but carries no text for it, so nothing here \
             can DECODE it — a name alone is not resolution"
        );
        assert_eq!(
            reading.name(),
            Some("sensor_msgs/FromAnotherCorpus"),
            "…and yet it IS named, which is the whole reason this arm exists \
             apart from Unresolvable"
        );

        // (3) A hash the bag says nothing about stays unresolvable.
        c.schema_hash = Some(0x9999);
        assert_eq!(
            resolve_schema(&walker, Some(&catalog), &c),
            SchemaReading::Unresolvable
        );

        // (4) LOCAL WINS: when both can name a hash, the local answer describes
        //     what will actually happen, and the bag never shadows it.
        c.schema_hash = Some(builtin_hash());
        let shadowing = cerulion_bag::BagSchemaCatalog::new(
            vec![cerulion_core::SchemaDoc {
                qualified: "go/Impostor".into(),
                encoding: cerulion_core::SchemaEncoding::Msg,
                text: "uint8 x\n".into(),
                deps: vec![],
            }],
            vec![cerulion_core::SchemaHashName {
                schema_hash: builtin_hash(),
                qualified: "go/Impostor".into(),
            }],
        );
        assert_eq!(
            resolve_schema(&walker, Some(&shadowing), &c),
            SchemaReading::Local {
                name: "geometry_msgs/Vector3".into()
            }
        );
    }

    /// A recorded name that DISAGREES with what the hash resolves to
    /// must carry a marker — printing the bag's name silently next to a
    /// hash meaning something else is exactly the shape that renders
    /// nothing while looking fine.
    #[test]
    fn a_recorded_name_that_disagrees_with_the_hash_is_reported() {
        let walker = test_walker();
        let mut scan = scan_of(&[("/a", 3)]);
        scan.channels[0].schema_name = "vendor/TheyCalledItThis".to_string();
        scan.channels[0].schema_hash = Some(builtin_hash());
        let text = render_scan(&scan, Path::new("/tmp/x.mcap"), &walker, None);
        assert!(
            text.contains("but the hash is geometry_msgs/Vector3 here"),
            "a name/hash disagreement must be visible: {text}"
        );

        // Anti-tautology: an AGREEING name carries no such marker.
        scan.channels[0].schema_name = "geometry_msgs/Vector3".to_string();
        let agree = render_scan(&scan, Path::new("/tmp/x.mcap"), &walker, None);
        assert!(!agree.contains("but the hash is"), "{agree}");
        // Nor does the attach-mode placeholder, which is an ABSENCE of a name
        // rather than a competing claim — it renders "(resolved by hash)".
        scan.channels[0].schema_name = ATTACH_SCHEMA_NAME.to_string();
        let placeholder = render_scan(&scan, Path::new("/tmp/x.mcap"), &walker, None);
        assert!(!placeholder.contains("but the hash is"), "{placeholder}");
        assert!(placeholder.contains("resolved by hash"), "{placeholder}");
    }

    /// The banner distinguishes "the bag brought its own definition" from
    /// "nobody has this type", because the two mean opposite things for whether
    /// anything will render.
    #[test]
    fn the_banner_separates_a_bag_supplied_type_from_an_unknown_one() {
        let walker = test_walker();
        let catalog = bag_catalog();
        let mut scan = scan_of(&[("/a", 3)]);
        scan.channels[0].schema_name = ATTACH_SCHEMA_NAME.to_string();
        scan.channels[0].schema_hash = Some(0x1111);

        let text = render_scan(&scan, Path::new("/tmp/x.mcap"), &walker, Some(&catalog));
        assert!(text.contains("the BAG carries its definition"), "{text}");
        assert!(text.contains("go/LowState"), "{text}");
        assert!(
            !text.contains("resolves to NOTHING"),
            "a type the bag defines must NOT be reported as unrenderable: {text}"
        );

        // The SAME channel with no catalog is the unknown case — and the two
        // notes must not be confusable.
        let bare = render_scan(&scan, Path::new("/tmp/x.mcap"), &walker, None);
        assert!(bare.contains("resolves to NOTHING"), "{bare}");
        assert!(!bare.contains("the BAG carries its definition"), "{bare}");

        // A named-but-textless binding gets the skew note, not either of those.
        scan.channels[0].schema_hash = Some(0x2222);
        let skew = render_scan(&scan, Path::new("/tmp/x.mcap"), &walker, Some(&catalog));
        assert!(skew.contains("cannot decode"), "{skew}");
        assert!(skew.contains("sensor_msgs/FromAnotherCorpus"), "{skew}");
        assert!(!skew.contains("the BAG carries its definition"), "{skew}");
    }

    /// A definition the bag carries in a form the VIEWER cannot seed must not be
    /// reported as renderable.
    ///
    /// `cerulion-vizd` parses only ROS `.msg`; the workspace-YAML parser lives in
    /// the CLI engine, which the daemon must not depend on. So a YAML-encoded
    /// definition decodes for `topic echo` / `bag info` and renders NOTHING —
    /// and reporting it beside a `.msg` one would re-create exactly the
    /// success-shaped failure this work removes.
    #[test]
    fn a_bag_definition_the_viewer_cannot_seed_is_not_claimed_renderable() {
        let walker = test_walker();
        let yaml_catalog = cerulion_bag::BagSchemaCatalog::new(
            vec![cerulion_core::SchemaDoc {
                qualified: "ws/YamlType".into(),
                encoding: cerulion_core::SchemaEncoding::Yaml,
                text: "name: YamlType\nfields: []\n".into(),
                deps: vec![],
            }],
            vec![cerulion_core::SchemaHashName {
                schema_hash: 0x7777,
                qualified: "ws/YamlType".into(),
            }],
        );
        let mut scan = scan_of(&[("/a", 3)]);
        scan.channels[0].schema_name = ATTACH_SCHEMA_NAME.to_string();
        scan.channels[0].schema_hash = Some(0x7777);

        let text = render_scan(
            &scan,
            Path::new("/tmp/x.mcap"),
            &walker,
            Some(&yaml_catalog),
        );
        assert!(
            text.contains("the viewer cannot use that form"),
            "the row must say the viewer cannot read this form: {text}"
        );
        assert!(
            text.contains("render NOTHING for them"),
            "the note must state the consequence: {text}"
        );
        assert!(
            !text.contains("the BAG carries its definition"),
            "it must NOT be reported alongside the definitions that DO render: {text}"
        );
        // The type is still NAMED and still decodable by `topic echo` — the
        // reading itself is unchanged, only the renderability claim differs.
        assert_eq!(
            resolve_schema(&walker, Some(&yaml_catalog), &scan.channels[0]),
            SchemaReading::FromBag {
                name: "ws/YamlType".into()
            }
        );
        assert!(text.contains("ws/YamlType"), "{text}");

        // ANTI-TAUTOLOGY: the SAME shape with a `.msg` encoding IS claimed
        // renderable, so the split above is about the encoding and nothing else.
        let msg_catalog = cerulion_bag::BagSchemaCatalog::new(
            vec![cerulion_core::SchemaDoc {
                qualified: "ws/YamlType".into(),
                encoding: cerulion_core::SchemaEncoding::Msg,
                text: "int32 x\n".into(),
                deps: vec![],
            }],
            yaml_catalog.hashes.clone(),
        );
        let msg = render_scan(&scan, Path::new("/tmp/x.mcap"), &walker, Some(&msg_catalog));
        assert!(msg.contains("the BAG carries its definition"), "{msg}");
        assert!(!msg.contains("the viewer cannot use that form"), "{msg}");
    }

    /// A `.msg` doc named after a BUILT-IN is not renderable either
    /// — `cerulion-vizd` refuses it by NAME before its walker ever sees it — so
    /// the banner must not promise that it "decodes and renders here".
    ///
    /// Reachable with NO corpus skew: a workspace holding
    /// `schemas/sensor_msgs/msg/Image.msg` is a first-class, documented shadow
    /// (`SchemaStore::builtin_shadows`, warned about at `ros_cmd`), the store
    /// copy WINS at resolution so the robot publishes under the SHADOW's hash,
    /// and `build_record_schema_catalog` ships that text unfiltered — correctly,
    /// since it is the only definition that explains the recorded hash.
    ///
    /// The FIXTURE HASH IS NOT `builtin_hash()` ON PURPOSE. A shadow drifts from
    /// the built-in, so its frames carry a DIFFERENT hash — which is also the
    /// only way to reach the `FromBag` arm at all, since `resolve_schema` answers
    /// `Local` first for any hash this machine already knows.
    #[test]
    fn a_bag_definition_named_after_a_builtin_is_not_claimed_renderable() {
        let walker = test_walker();
        const SHADOW_HASH: u64 = 0x5152_5354_5556_5758;
        let shadow_catalog = cerulion_bag::BagSchemaCatalog::new(
            vec![cerulion_core::SchemaDoc {
                qualified: "sensor_msgs/Image".into(),
                encoding: cerulion_core::SchemaEncoding::Msg,
                text: "uint32 height\nuint32 width\n".into(),
                deps: vec![],
            }],
            vec![cerulion_core::SchemaHashName {
                schema_hash: SHADOW_HASH,
                qualified: "sensor_msgs/Image".into(),
            }],
        );
        let mut scan = scan_of(&[("/cam", 3)]);
        scan.channels[0].schema_name = ATTACH_SCHEMA_NAME.to_string();
        scan.channels[0].schema_hash = Some(SHADOW_HASH);

        // PRECONDITION: this really is the `FromBag` arm — the reading is
        // unchanged, only the RENDERABILITY claim differs. (Without this the
        // absence assertions below would pass for the wrong reason.)
        assert_eq!(
            resolve_schema(&walker, Some(&shadow_catalog), &scan.channels[0]),
            SchemaReading::FromBag {
                name: "sensor_msgs/Image".into()
            },
            "the fixture must reach FromBag, or this test proves nothing"
        );
        assert!(
            !bag_doc_is_viewable(Some(&shadow_catalog), "sensor_msgs/Image"),
            "a doc named after a built-in is refused by the daemon, so it is not viewable"
        );

        let text = render_scan(
            &scan,
            Path::new("/tmp/x.mcap"),
            &walker,
            Some(&shadow_catalog),
        );
        assert!(
            !text.contains("the BAG carries its definition"),
            "THE PIN: the banner must not promise rendering the daemon structurally \
             refuses: {text}"
        );
        assert!(
            !text.contains("from the bag's own schema records"),
            "…nor mark the ROW as one that renders: {text}"
        );
        assert!(
            text.contains("under a BUILT-IN name"),
            "the row must say WHY: {text}"
        );
        assert!(
            text.contains("REFUSES a definition named after a built-in"),
            "the note must name the refusal: {text}"
        );
        assert!(
            text.contains("render NOTHING for them"),
            "the note must state the consequence: {text}"
        );
        assert!(text.contains("/cam"), "{text}");
        // Its remedy is the OPPOSITE of the YAML arm's ("express it as a .msg"),
        // so the two must not share a note.
        assert!(
            !text.contains("the viewer cannot use that form"),
            "a built-in-named .msg is not a wrong-FORM problem: {text}"
        );

        // ANTI-TAUTOLOGY: the SAME doc under a CUSTOM name IS claimed renderable,
        // so the split is about the name and nothing else.
        let custom = cerulion_bag::BagSchemaCatalog::new(
            vec![cerulion_core::SchemaDoc {
                qualified: "go/Image".into(),
                encoding: cerulion_core::SchemaEncoding::Msg,
                text: "uint32 height\nuint32 width\n".into(),
                deps: vec![],
            }],
            vec![cerulion_core::SchemaHashName {
                schema_hash: SHADOW_HASH,
                qualified: "go/Image".into(),
            }],
        );
        let ok = render_scan(&scan, Path::new("/tmp/x.mcap"), &walker, Some(&custom));
        assert!(ok.contains("the BAG carries its definition"), "{ok}");
        assert!(!ok.contains("under a BUILT-IN name"), "{ok}");
        assert!(
            bag_doc_is_viewable(Some(&custom), "go/Image"),
            "a custom-named .msg doc IS viewable — the predicate is not stuck at false"
        );
    }

    /// A channel with NO Cerulion descriptor is not the same
    /// condition as one whose hash resolves to nothing, and it must not inherit
    /// that note's remedy.
    ///
    /// Landing both in one `unresolvable` bucket would have `bag info` tell an
    /// operator their descriptor-less channel "carries a wire schema hash that
    /// resolves to NOTHING here" (it carries no hash at all) and prescribe
    /// re-recording for the definitions / pointing `DDS_BRIDGE_CONFIG` at a
    /// `msg_dirs` — neither of which can help a channel `open_route` refuses
    /// outright.
    #[test]
    fn a_descriptorless_channel_is_reported_apart_from_an_unresolvable_hash() {
        let walker = test_walker();
        let mut scan = scan_of(&[("/nohdr", 2)]);
        scan.channels[0].schema_hash = None;

        let text = render_scan(&scan, Path::new("/tmp/x.mcap"), &walker, None);
        assert!(
            text.contains("[no Cerulion descriptor — unplayable]"),
            "the ROW marker was already right: {text}"
        );
        assert!(
            text.contains("carry NO Cerulion descriptor at all"),
            "…and now the NOTE agrees with it: {text}"
        );
        assert!(
            !text.contains("resolves to NOTHING here"),
            "THE PIN: a channel with no hash must not be described as one whose hash \
             resolves to nothing: {text}"
        );
        assert!(
            !text.contains("DDS_BRIDGE_CONFIG"),
            "…nor be given a remedy that cannot help it: {text}"
        );
        assert!(text.contains("/nohdr"), "{text}");

        // ANTI-TAUTOLOGY: a channel that DOES carry an unresolvable hash still
        // gets the hash note, with its remedy intact.
        let mut scan = scan_of(&[("/unknown", 2)]);
        scan.channels[0].schema_hash = Some(0xDEAD_BEEF_DEAD_BEEF);
        let hash = render_scan(&scan, Path::new("/tmp/x.mcap"), &walker, None);
        assert!(hash.contains("resolves to NOTHING here"), "{hash}");
        assert!(hash.contains("DDS_BRIDGE_CONFIG"), "{hash}");
        assert!(
            !hash.contains("carry NO Cerulion descriptor at all"),
            "{hash}"
        );

        // BOTH in one bag: two notes, each naming only its OWN topic.
        let mut scan = scan_of(&[("/nohdr", 2), ("/unknown", 2)]);
        scan.channels[0].schema_hash = None;
        scan.channels[1].schema_hash = Some(0xDEAD_BEEF_DEAD_BEEF);
        let both = render_scan(&scan, Path::new("/tmp/x.mcap"), &walker, None);
        let descriptor_note = both
            .split("carry NO Cerulion descriptor at all")
            .nth(1)
            .expect("the descriptor note is present");
        assert!(descriptor_note.contains("/nohdr"), "{both}");
        let hash_note = both
            .split("resolves to NOTHING here")
            .nth(1)
            .expect("the hash note is present");
        assert!(hash_note.contains("/unknown"), "{both}");
        assert!(
            !hash_note
                .split("\n\n")
                .next()
                .unwrap_or("")
                .contains("/nohdr"),
            "the hash note must not claim the descriptor-less topic: {both}"
        );
    }

    #[test]
    fn the_banner_warns_only_when_the_hash_really_resolves_to_nothing() {
        let walker = test_walker();
        let mut scan = scan_of(&[("/a", 3)]);
        // An attach-mode channel carrying BUILT-IN frames: no warning at all.
        scan.channels[0].schema_name = ATTACH_SCHEMA_NAME.to_string();
        scan.channels[0].schema_hash = Some(builtin_hash());
        let text = render_scan(&scan, Path::new("/tmp/x.mcap"), &walker, None);
        assert!(
            !text.contains("resolves to NOTHING"),
            "an attach-mode bag of built-in frames must NOT be warned about: {text}"
        );
        assert!(text.contains("geometry_msgs/Vector3"), "{text}");

        // A genuinely unknown hash IS warned about, by TOPIC.
        scan.channels[0].schema_hash = Some(0xDEAD_BEEF_DEAD_BEEF);
        let text = render_scan(&scan, Path::new("/tmp/x.mcap"), &walker, None);
        assert!(text.contains("resolves to NOTHING"), "{text}");
        assert!(text.contains("/a"), "{text}");
        assert!(text.contains("DDS_BRIDGE_CONFIG"), "{text}");
        assert!(text.contains("BEFORE vizd starts"), "{text}");
    }

    #[test]
    fn the_banner_reports_a_cross_epoch_bag_rather_than_hiding_it() {
        let mut scan = scan_of(&[("/a", 4)]);
        scan.backwards_log_times = 2;
        let text = render_scan(&scan, Path::new("/tmp/x.mcap"), &test_walker(), None);
        assert!(
            text.contains("2 frame(s) carry a wire timestamp EARLIER"),
            "{text}"
        );
        // Anti-tautology: a monotone bag carries no such note.
        let clean = render_scan(
            &scan_of(&[("/a", 4)]),
            Path::new("/tmp/x.mcap"),
            &test_walker(),
            None,
        );
        assert!(!clean.contains("EARLIER"), "{clean}");
    }

    fn live() -> Vec<String> {
        [
            "/imu",
            "/camera/image",
            "/bagd/status",
            "/__cerulion/mirrors",
            "/tf",
        ]
        .iter()
        .map(|s| s.to_string())
        .collect()
    }

    fn no_mirrors() -> BTreeMap<String, String> {
        BTreeMap::new()
    }

    fn rec_opts(f: impl FnOnce(&mut RecordOptions)) -> RecordOptions {
        let mut o = RecordOptions::default();
        f(&mut o);
        o
    }

    #[test]
    fn all_selects_every_live_topic_except_the_internal_ones() {
        // Hand oracle: `/bagd/status` (the recorder's own channel — recording
        // it is a recorder recording itself) and the reserved `__cerulion/`
        // namespace are never auto-selected. The result is sorted, so a given
        // (live set, flags) always yields the same bag channel order.
        let (got, sel) =
            derive_record_topics(&live(), &no_mirrors(), &rec_opts(|o| o.all = true)).unwrap();
        assert_eq!(got, vec!["/camera/image", "/imu", "/tf"]);
        assert_eq!(sel, TopicSelection::All);
    }

    #[test]
    fn auto_selection_and_the_listing_share_one_internal_predicate() {
        // Hand oracle over names the PREFIX LIST alone gets wrong. The bare
        // `/__cerulion` namespace token starts with no listed prefix (each one
        // ends in `/`), so a filter walking `AUTO_SELECT_EXCLUDED_PREFIXES`
        // would auto-select it while `topic list` hides it: the two verbs
        // would disagree about one name. `/__cerulionx` and `/my/bagd/status`
        // are the near misses that must stay USER topics.
        let names: Vec<String> = [
            "/__cerulion",
            "/__cerulion/mirrors",
            "/__cerulionx",
            "/bagd/status",
            "/my/bagd/status",
            "__cerulion/recorder.json",
        ]
        .iter()
        .map(|s| s.to_string())
        .collect();
        assert_eq!(
            auto_selectable(&names),
            vec!["/__cerulionx".to_string(), "/my/bagd/status".to_string()],
            "exactly the two near-miss USER topics survive, in input order"
        );
        // The anti-tautology half: the bare token really is a name the prefix
        // list cannot spell, so the assertion above is about the predicate and
        // not a restatement of the list.
        assert!(
            !AUTO_SELECT_EXCLUDED_PREFIXES
                .iter()
                .any(|p| "/__cerulion".starts_with(p)),
            "if a prefix ever matches the bare token, this test no longer \
             separates the predicate from the list; re-derive its oracle"
        );
        // Every name is classified the same way by the listing's predicate:
        // one function, so the default listing hides exactly what is skipped.
        for name in &names {
            assert_eq!(
                crate::topic_cmd::is_internal_topic(name),
                !auto_selectable(std::slice::from_ref(name)).contains(name),
                "{name}: hidden by `topic list` must mean never auto-selected"
            );
        }
    }

    #[test]
    fn regex_matches_against_the_same_auto_selectable_set() {
        let (got, sel) = derive_record_topics(
            &live(),
            &no_mirrors(),
            &rec_opts(|o| o.regex = Some("^/(imu|tf)$".to_string())),
        )
        .unwrap();
        assert_eq!(got, vec!["/imu", "/tf"]);
        assert_eq!(sel, TopicSelection::Regex);
    }

    #[test]
    fn a_regex_that_only_matches_internal_topics_selects_nothing_loudly() {
        // The internal-prefix filter applies to --regex too, so this selects
        // nothing — and an empty selection is an ERROR, never a recording of
        // nothing that looks like it worked.
        let err = derive_record_topics(
            &live(),
            &no_mirrors(),
            &rec_opts(|o| o.regex = Some("bagd".to_string())),
        )
        .unwrap_err()
        .to_string();
        assert!(
            err.contains("selection is empty"),
            "an empty selection must be loud: {err}"
        );
    }

    #[test]
    fn exclude_narrows_any_selection_including_an_explicit_list() {
        let (got, _) = derive_record_topics(
            &live(),
            &no_mirrors(),
            &rec_opts(|o| {
                o.all = true;
                o.exclude = vec!["^/camera".to_string()];
            }),
        )
        .unwrap();
        assert_eq!(got, vec!["/imu", "/tf"]);

        let (got, sel) = derive_record_topics(
            &live(),
            &no_mirrors(),
            &rec_opts(|o| {
                o.topics = vec!["/imu".to_string(), "/tf".to_string()];
                o.exclude = vec!["tf".to_string()];
            }),
        )
        .unwrap();
        assert_eq!(got, vec!["/imu"]);
        assert_eq!(sel, TopicSelection::Explicit);
    }

    #[test]
    fn an_explicit_topic_that_is_not_local_names_the_reality() {
        // The decision, pinned: no network fallback, and the message must
        // say where the topic is actually recorded.
        let err = derive_record_topics(
            &live(),
            &no_mirrors(),
            &rec_opts(|o| o.topics = vec!["/robot/lowstate".to_string()]),
        )
        .unwrap_err()
        .to_string();
        assert!(err.contains("/robot/lowstate"), "{err}");
        assert!(
            err.contains("never pulls a topic's frames across the network"),
            "{err}"
        );
        assert!(err.contains("ON the robot"), "{err}");
        // It also lists what IS here, so the operator can act immediately.
        assert!(err.contains("/imu"), "{err}");
    }

    #[test]
    fn exactly_one_topic_source_is_allowed() {
        for opts in [
            rec_opts(|o| {
                o.all = true;
                o.topics = vec!["/imu".to_string()];
            }),
            rec_opts(|o| {
                o.all = true;
                o.regex = Some("imu".to_string());
            }),
            rec_opts(|o| {
                o.topics = vec!["/imu".to_string()];
                o.regex = Some("imu".to_string());
            }),
        ] {
            let err = derive_record_topics(&live(), &no_mirrors(), &opts)
                .unwrap_err()
                .to_string();
            assert!(err.contains("exactly ONE topic source"), "{err}");
        }
        // And NO source at all is equally loud.
        let err = derive_record_topics(&live(), &no_mirrors(), &rec_opts(|_| {}))
            .unwrap_err()
            .to_string();
        assert!(err.contains("needs a topic set"), "{err}");
    }

    #[test]
    fn a_bad_pattern_is_refused_by_the_flag_that_carried_it() {
        let err = derive_record_topics(
            &live(),
            &no_mirrors(),
            &rec_opts(|o| o.regex = Some("[".to_string())),
        )
        .unwrap_err()
        .to_string();
        assert!(
            err.contains("--regex") && err.contains("not a valid regex"),
            "{err}"
        );
        let err = derive_record_topics(
            &live(),
            &no_mirrors(),
            &rec_opts(|o| {
                o.all = true;
                o.exclude = vec!["[".to_string()];
            }),
        )
        .unwrap_err()
        .to_string();
        assert!(
            err.contains("--exclude") && err.contains("not a valid regex"),
            "{err}"
        );
    }

    #[test]
    fn the_selection_is_sorted_and_deduped_so_channel_order_is_stable() {
        let (got, _) = derive_record_topics(
            &live(),
            &no_mirrors(),
            &rec_opts(|o| {
                o.topics = vec!["/tf".to_string(), "/imu".to_string(), "/tf".to_string()]
            }),
        )
        .unwrap();
        assert_eq!(got, vec!["/imu", "/tf"]);
    }

    #[test]
    fn sleep_interruptible_returns_false_when_the_flag_clears() {
        let running = AtomicBool::new(false);
        // A one-hour sleep must return immediately (false) on a cleared flag.
        let t = std::time::Instant::now();
        assert!(!sleep_interruptible(3_600_000_000_000, &running));
        assert!(t.elapsed() < Duration::from_secs(1));
        // Zero delay on a LIVE flag is true and does not sleep.
        let live = AtomicBool::new(true);
        assert!(sleep_interruptible(0, &live));
    }

    #[test]
    fn a_netd_mirror_is_never_auto_selected_as_a_local_topic() {
        // `TransportManager::list_topics` is a RAW `{topic}/data`
        // service scan with no provenance filter, and a
        // `cerulion-netd` mirror of a REMOTE robot's topic IS a local
        // `{topic}/data` service. So `bag record -a` on a desk with a standing
        // `cerulion viz --robot go2` recorded that robot's stream into a bag the
        // operator was told was a local capture.
        let mut mirrors = BTreeMap::new();
        mirrors.insert("/camera/image".to_string(), "go2".to_string());

        let (got, _) =
            derive_record_topics(&live(), &mirrors, &rec_opts(|o| o.all = true)).unwrap();
        assert_eq!(
            got,
            vec!["/imu", "/tf"],
            "a netd mirror must be folded OUT of --all"
        );
        // ANTI-TAUTOLOGY: with NO provenance the same topic IS selected, so the
        // exclusion is the mirror map's doing and not an unrelated filter.
        let (got, _) =
            derive_record_topics(&live(), &no_mirrors(), &rec_opts(|o| o.all = true)).unwrap();
        assert_eq!(got, vec!["/camera/image", "/imu", "/tf"]);
    }

    #[test]
    fn a_regex_matching_only_a_mirror_selects_nothing_loudly() {
        let mut mirrors = BTreeMap::new();
        mirrors.insert("/camera/image".to_string(), "go2".to_string());
        let err = derive_record_topics(
            &live(),
            &mirrors,
            &rec_opts(|o| o.regex = Some("camera".to_string())),
        )
        .unwrap_err()
        .to_string();
        assert!(err.contains("selection is empty"), "{err}");
        // The count must be of GENUINELY-LOCAL topics, not the raw scan.
        assert!(
            err.contains("2 selectable local"),
            "the considered count must exclude the mirror AND the internals (/imu + /tf = 2): {err}"
        );
    }

    #[test]
    fn an_explicitly_named_mirror_is_recorded_and_the_refusal_lists_it_separately() {
        let mut mirrors = BTreeMap::new();
        mirrors.insert("/camera/image".to_string(), "go2".to_string());

        // Naming it explicitly RECORDS it — the operator asked for it.
        let (got, sel) = derive_record_topics(
            &live(),
            &mirrors,
            &rec_opts(|o| o.topics = vec!["/camera/image".to_string()]),
        )
        .unwrap();
        assert_eq!(got, vec!["/camera/image"]);
        assert_eq!(sel, TopicSelection::Explicit);

        // And a refusal must not present the mirror as a local topic: it is
        // listed on its own line, attributed to its origin robot.
        let err = derive_record_topics(
            &live(),
            &mirrors,
            &rec_opts(|o| o.topics = vec!["/nope".to_string()]),
        )
        .unwrap_err()
        .to_string();
        let local_line = err
            .lines()
            .find(|l| l.contains("shows what is live here"))
            .expect("the local line");
        assert!(
            !local_line.contains("/camera/image"),
            "a mirror must NOT be listed as live-local: {local_line}"
        );
        assert!(
            err.contains("STREAMED FROM ANOTHER ROBOT") && err.contains("(from go2)"),
            "the mirror must be named separately with its origin: {err}"
        );
    }

    fn health(
        frames: u64,
        lost: u64,
        gaps: u64,
        anomaly: bool,
        multi: bool,
    ) -> cerulion_bagd::TopicHealth {
        cerulion_bagd::TopicHealth {
            frames_recorded: frames,
            frames_lost: lost,
            gap_events: gaps,
            multi_publisher: multi,
            sequence_anomaly: anomaly,
            first_seq: None,
            last_seq: None,
            baseline_resets_after_first: 0,
            gap_detection_disabled_reason: None,
            classify_calls: 0,
            defer_count: 0,
            staging_full_passes: 0,
            // Additive producer-label fields, vacant here.
            producer_labels: 0,
            label_catch_up: false,
            // This fixture opened no tap, so it knows
            // neither depth nor pinned cost — UNKNOWN, not zero.
            tap_buffer_depth: None,
            shm_pinned_bytes: None,
            // Nor its per-row coverage basis.
            loss_counting_basis: None,
            // This fixture's subject is not the verdict;
            // an ABSENT one is UNKNOWN, never a passing one.
            absorbance: None,
        }
    }

    /// A second writer OBSERVED at record time is UNMEASURED, even
    /// though the graph declared nothing.
    ///
    /// The sibling arm below drives `multi_publisher` — the DECLARED flag — so
    /// it is structurally blind to this one: bagd stands gap detection down the
    /// moment it sees a second publisher identity on an undeclared topic, and
    /// from that instant its counters STOP ADVANCING. It does NOT zero them (the
    /// runtime-anomaly path does; `note_observed_plurality` deliberately keeps
    /// what the single-writer prefix earned, because that arithmetic was over a
    /// genuine single stream). Either way the counters no longer describe the
    /// topic — a `0` means "nothing went wrong up to the second writer", not
    /// "nothing went wrong" — so without the observed-plurality terms such a
    /// topic falls through and reports a verified number for a stream whose gap
    /// arithmetic was abandoned.
    ///
    /// The anti-tautology half is in the same body: the same counters with the
    /// marker OFF really do read `Clean`, so this is a claim about the flag and
    /// not about zeroed counters.
    #[test]
    fn a_runtime_observed_second_writer_is_unmeasured_not_clean() {
        let mut h = health(10, 0, 0, false, false);
        assert_eq!(
            classify_loss(Some(&h)),
            LossReading::Clean,
            "control: these counters ALONE are a verified clean run"
        );
        h.label_catch_up = true;
        assert_eq!(
            classify_loss(Some(&h)),
            LossReading::UnmeasuredMultiPublisher,
            "the recorder SAW a second writer, so the zeroed counters measured nothing"
        );
        assert!(classify_loss(Some(&h)).is_unmeasured());
        assert_eq!(
            classify_loss(Some(&h)).cells(),
            ("?".to_string(), "?".to_string()),
            "the columns must say `?` where a number would be a lie"
        );
    }

    /// The authoritative observed-plurality signal is the
    /// gap stand-down REASON, not the catch-up EMISSION flag.
    ///
    /// The two diverge by construction. `note_observed_plurality` runs on the
    /// DRAIN thread the instant a second publisher identity is seen, while
    /// `label_catch_up` is set only when a batch carrying the catch-up record is
    /// handed to a WRITABLE writer — so a dropped batch, and a window-only
    /// recorder that mints no continuous bag at all, both leave the arithmetic
    /// stood down with no record written.
    ///
    /// This is the exact field triple `cerulion_bagd`'s ARM 10
    /// (`a_force_dropped_arming_batch_trips_the_impossible_triple_tripwire`)
    /// produces over real transport: labels earned, nothing declared, no
    /// catch-up. Reading only the emission flag reports it `Clean` — a verified
    /// `0` for a topic whose gap arithmetic bagd abandoned six frames in.
    ///
    /// The anti-tautology half is in the same body: the same counters with the
    /// reason ABSENT really do read `Clean`.
    #[test]
    fn a_stood_down_topic_with_no_catch_up_record_is_still_unmeasured() {
        let mut h = health(10, 0, 0, false, false);
        h.producer_labels = 6;
        assert_eq!(
            classify_loss(Some(&h)),
            LossReading::Clean,
            "control: these counters ALONE are a verified clean run"
        );
        h.gap_detection_disabled_reason = Some("observed_multi_publisher".to_string());
        assert!(
            !h.label_catch_up,
            "the shape under test: the emission flag is OFF (its batch was dropped)"
        );
        assert_eq!(
            classify_loss(Some(&h)),
            LossReading::UnmeasuredMultiPublisher,
            "the recorder stood the arithmetic down, so its counters measured nothing after that"
        );
        assert_eq!(
            classify_loss(Some(&h)).cells(),
            ("?".to_string(), "?".to_string())
        );
    }

    /// When both causes are present the anomaly verdict wins,
    /// and both are unmeasured either way.
    ///
    /// They can coexist — a topic can go non-monotone and only later be seen to
    /// have two writers (the reason field keeps the FIRST cause, which is the
    /// one an operator can act on), and `sequence_anomaly` is the flag that
    /// survives it. The columns are `?` on both routes, so what the precedence
    /// decides is only which SENTENCE the summary prints; it must be the one
    /// that matches the reason bagd recorded.
    #[test]
    fn a_sequence_anomaly_outranks_an_observed_second_writer() {
        let mut h = health(10, 0, 0, true, false);
        h.label_catch_up = true;
        h.gap_detection_disabled_reason = Some("runtime_nonmonotonic_sequence".to_string());
        assert_eq!(
            classify_loss(Some(&h)),
            LossReading::UnmeasuredAnomaly,
            "the anomaly is the first thing that went wrong and the one with a remedy"
        );
        assert!(classify_loss(Some(&h)).is_unmeasured());
        // …and the multi-publisher route is unmeasured too, so the precedence
        // can never turn an unmeasured topic into a measured one.
        let mut m = health(10, 0, 0, false, false);
        m.label_catch_up = true;
        assert!(classify_loss(Some(&m)).is_unmeasured());
    }

    #[test]
    fn loss_classification_never_reports_an_unmeasured_topic_as_clean() {
        // bagd ZEROES frames_lost/gap_events when it disables
        // gap detection, so an unmeasured topic and a verified-clean one carry
        // identical counters — the flags are the only discriminator.
        assert_eq!(
            classify_loss(Some(&health(10, 0, 0, false, false))),
            LossReading::Clean
        );
        assert_eq!(
            classify_loss(Some(&health(10, 7, 2, false, false))),
            LossReading::Lost { frames: 7, gaps: 2 }
        );
        // THE pin: zeroed counters + the anomaly flag is UNKNOWN, not clean.
        assert_eq!(
            classify_loss(Some(&health(10, 0, 0, true, false))),
            LossReading::UnmeasuredAnomaly
        );
        assert_eq!(
            classify_loss(Some(&health(10, 0, 0, false, true))),
            LossReading::UnmeasuredMultiPublisher
        );
        // A missing record is UNKNOWN too, with its OWN cause: reporting it
        // as a sequence anomaly would state a reason that never happened.
        assert_eq!(classify_loss(None), LossReading::UnmeasuredNoRecord);
        // L2: a baseline reset after the first frame is head loss the
        // gap arithmetic could not attribute — also UNKNOWN, not a clean zero.
        let mut h = health(10, 0, 0, false, false);
        h.baseline_resets_after_first = 2;
        assert_eq!(
            classify_loss(Some(&h)),
            LossReading::UnmeasuredBaselineReset { resets: 2 }
        );
        // The columns say "?" where a number would be a lie.
        assert_eq!(
            LossReading::UnmeasuredAnomaly.cells(),
            ("?".to_string(), "?".to_string())
        );
        assert_eq!(
            LossReading::Clean.cells(),
            ("0".to_string(), "0".to_string())
        );
    }

    fn summary_with(topics: Vec<(&str, cerulion_bagd::TopicHealth)>) -> cerulion_bagd::BagdSummary {
        let mut s = cerulion_bagd::BagdSummary {
            bag_paths: vec![std::path::PathBuf::from("/tmp/x.mcap")],
            // A healthy recording never loses a run-vanished
            // capture, so the fixture's value is the healthy one.
            run_vanished_captures_lost: 0,
            messages: 0,
            bytes: 0,
            chunks: 0,
            per_topic: BTreeMap::new(),
            capture_close_max: std::time::Duration::ZERO,
            captures_closed: 0,
            ring_records: 0,
            rings_unavailable: Vec::new(),
            frozen_burst_markers: 0,
            dropped_unwritten: 0,
            headerless: 0,
            status_unavailable: false,
            frames_lost: 0,
            record_health: cerulion_bagd::RecordHealth {
                version: 1,
                dropped_unwritten: 0,
                drive_passes: 0,
                max_pass_duration_us: 0,
                // Likewise no cadence to report.
                drive_span_us: 0,
                drain_gaps: cerulion_bagd::DrainGapHistogram::default(),
                // No claim about what the loss numbers
                // above could see.
                loss_counting_basis: None,
                topics: BTreeMap::new(),
            },
            record_coverage: cerulion_bagd::RecordCoverage::default(),
            // This fixture never ran a drive loop, so the correct value
            // for "how long the channel set stayed open" is `None`.
            channel_set_closed_after: None,
            // Likewise it drove no loop, so it counts no passes and
            // its worst pass has no duration.
            drive_passes: 0,
            max_pass_duration: std::time::Duration::ZERO,
            // No loop, so no span and no gap distribution.
            drive_span: std::time::Duration::ZERO,
            drain_gaps: cerulion_bagd::DrainGapHistogram::default(),
            // And it was bound to no run, so it claims nothing
            // about how any run ended.
            run_ended: None,
            // It recorded no node-state anchors and was
            // configured for none, so it makes no checkpoint claim either.
            state_records: 0,
            state_coverage: None,
            // It opened no taps, so it pinned nothing and
            // has nothing it failed to price.
            shm_pinned_bytes: 0,
            unpriced_taps: 0,
        };
        for (t, h) in topics {
            s.messages += h.frames_recorded;
            s.record_health.topics.insert(t.to_string(), h);
        }
        s
    }

    /// The recorder's own stdout summary reports a
    /// trace it could not open.
    ///
    /// `silent:` / `loss:` / `unmeasured:` are all "here is what you did not
    /// get"; a scheduler trace whose ring vanished is the same class — a whole
    /// artifact missing rather than some frames — and this surface said nothing
    /// about it. The degrade was loud on stderr and durable in the manifest, but
    /// stderr routinely goes somewhere else and `bag info` is read LATER, while
    /// this block is what the operator sees while they can still re-attach.
    ///
    /// The CONTROL is in the same body and is the load-bearing half: without it,
    /// "a trace line appears" is satisfied by a renderer that always prints one,
    /// which would put a scary line on every healthy recording.
    #[test]
    fn a_trace_ring_the_recorder_could_not_open_is_named_in_the_record_summary() {
        // CONTROL — a healthy recording says nothing about the trace.
        let clean =
            render_record_summary(&summary_with(vec![("/a", health(10, 0, 0, false, false))]));
        assert!(
            !clean.contains("trace:"),
            "a recording with no ring trouble must not mention the trace:\n{clean}"
        );

        let mut s = summary_with(vec![("/a", health(10, 0, 0, false, false))]);
        s.rings_unavailable = vec![
            (
                "/cer_rg_dead0".to_string(),
                "shm_open: No such file or directory".to_string(),
            ),
            (
                "/cer_rg_dead1".to_string(),
                "RecordSizeMismatch".to_string(),
            ),
        ];
        let text = render_record_summary(&s);

        let line = text
            .lines()
            .find(|l| l.starts_with("trace: "))
            .unwrap_or_else(|| panic!("no trace line in:\n{text}"));
        assert!(
            line.contains("2 declared scheduler-trace ring(s)"),
            "the count is the fact an operator scans: {line}"
        );
        // The frames are the part that cannot be re-obtained, so the line must
        // say they survived — otherwise it reads as a failed recording.
        assert!(
            line.contains("frames are unaffected"),
            "the line must scope its own bad news: {line}"
        );
        // Each ring is NAMED with the transport's own error, the same discipline
        // `bag info` uses — a count alone cannot be acted on.
        for (ring, err) in &s.rings_unavailable {
            assert!(
                text.contains(ring.as_str()) && text.contains(err.as_str()),
                "every unopened ring must be named with its error: {ring} / {err}\n{text}"
            );
        }
        // …and it points at the surface that CAN state the NONE-vs-PARTIAL
        // fraction, since `BagdSummary` carries no declared count.
        assert!(
            line.contains("cerulion bag info"),
            "the fraction lives in the manifest; this line must say where: {line}"
        );
    }

    /// The `loss:` line names each mechanism in the SHARED vocabulary,
    /// and its three operands stay in their declared order.
    ///
    /// This line was the fifth operator-facing surface carrying the retired
    /// "before the writer existed" claim, and the only one nothing pinned — the
    /// rename edited the two OPERANDS of this very `format!` and left the prose
    /// between them, which is exactly the shape a compiler cannot see. It is
    /// reachable in production on any recording with ordinary tap-queue
    /// overflow (the guard is an OR), so an operator could read one run and be
    /// handed three names for one quantity: `dropped_unwritten` in
    /// `record_health.json`, "drained but never written to the bag" from
    /// replay, and "dropped before the writer existed" here.
    ///
    /// The three counts are DISTINCT primes so a swapped operand cannot pass a
    /// phrase-only check, and the whole clause is matched as one substring so
    /// the count stays attached to its own noun.
    #[test]
    fn the_loss_line_names_each_mechanism_in_the_shared_vocabulary() {
        let mut s = summary_with(vec![("/a", health(10, 7, 1, false, false))]);
        s.frames_lost = 7;
        s.dropped_unwritten = 11;
        s.headerless = 13;
        let text = render_record_summary(&s);

        assert!(
            text.contains("7 frame(s) lost to tap-queue overflow"),
            "the queue-overflow clause keeps its own count: {text}"
        );
        assert!(
            text.contains("11 drained but never written to the bag"),
            "the unwritten-drop clause must use the SAME words as replay's catch-all, with \
             its own count: {text}"
        );
        assert!(
            text.contains("13 recorded without a parseable wire header"),
            "the headerless clause keeps its own count: {text}"
        );
        // The retired vocabulary, in every spelling it ever had here.
        for retired in [
            "before the writer existed",
            "pre-writer",
            "dropped_pre_writer",
        ] {
            assert!(
                !text.contains(retired),
                "the summary must not name a state the counter can no longer come from \
                 ({retired}): {text}"
            );
        }

        // ANTI-TAUTOLOGY: a clean recording renders NO loss line at all, so the
        // assertions above are about a line that is genuinely conditional
        // rather than one that is always present.
        let clean =
            render_record_summary(&summary_with(vec![("/a", health(10, 0, 0, false, false))]));
        assert!(
            !clean.contains("loss:"),
            "a clean recording must not print a loss line: {clean}"
        );
    }

    #[test]
    fn a_selected_but_silent_topic_still_gets_a_row() {
        // A renderer that iterates `per_topic`, which bagd's
        // INLINE path builds PER FRAME, drops a tapped topic that published
        // nothing: it contributes no key and its row VANISHES, leaving the operator
        // unable to tell "never selected" from "selected but silent".
        let text = render_record_summary(&summary_with(vec![
            ("/loud", health(100, 0, 0, false, false)),
            ("/silent", health(0, 0, 0, false, false)),
        ]));
        assert!(text.contains("/loud"), "{text}");
        assert!(
            text.contains("/silent"),
            "a tapped-but-silent topic must still have a row: {text}"
        );
        assert!(text.contains("silent: 1 topic(s)"), "{text}");
        assert!(text.contains("never sent late-joiner history"), "{text}");
        // Anti-tautology: an all-publishing capture carries no silent note.
        let text = render_record_summary(&summary_with(vec![(
            "/loud",
            health(100, 0, 0, false, false),
        )]));
        assert!(!text.contains("silent:"), "{text}");
    }

    #[test]
    fn an_unmeasured_topic_is_rendered_differently_from_a_measured_zero() {
        let text = render_record_summary(&summary_with(vec![
            ("/verified", health(50, 0, 0, false, false)),
            ("/anomalous", health(50, 0, 0, true, false)),
        ]));
        // The rows differ — "?" vs "0" — so the operator can SEE the difference.
        let anomalous_row = text
            .lines()
            .find(|l| l.starts_with("/anomalous"))
            .expect("row");
        let verified_row = text
            .lines()
            .find(|l| l.starts_with("/verified"))
            .expect("row");
        assert!(
            anomalous_row.contains('?'),
            "an unmeasured topic must not render a number: {anomalous_row}"
        );
        assert!(
            !verified_row.contains('?'),
            "a measured-clean topic must render 0: {verified_row}"
        );
        assert!(
            text.contains("Loss on this topic is UNKNOWN, not zero"),
            "{text}"
        );
        // The multi-publisher arm carries its OWN reason. A later change reworded it:
        // the arm is now reached by an OBSERVED second writer too, so it may no
        // longer claim the topic was DECLARED one.
        let text =
            render_record_summary(&summary_with(vec![("/tf", health(50, 0, 0, false, true))]));
        assert!(
            text.contains("it is declared multi_publisher, or a second writer was observed")
                && text.contains("UNKNOWN by construction"),
            "{text}"
        );
    }

    #[test]
    fn the_record_health_artifact_is_always_named() {
        // A renderer that points at the artifact only when a counter is
        // non-zero never does so in the unmeasured case, which is exactly when
        // the summary is least able to answer the question.
        let clean =
            render_record_summary(&summary_with(vec![("/a", health(10, 0, 0, false, false))]));
        assert!(clean.contains("record_health.json"), "{clean}");
        let anomalous =
            render_record_summary(&summary_with(vec![("/a", health(10, 0, 0, true, false))]));
        assert!(anomalous.contains("record_health.json"), "{anomalous}");
    }

    #[test]
    #[tracing_test::traced_test]
    fn the_explicit_mirror_carve_out_announces_its_origin_robot() {
        // The carve-out lets an EXPLICITLY named netd
        // mirror be recorded, and its whole justification is that the operator
        // is TOLD. Without this arm, deleting the warn leaves the suite green, so the carve-out
        // could silently widen into "mirrors record like local topics".
        let mut mirrors = BTreeMap::new();
        mirrors.insert("/camera/image".to_string(), "go2".to_string());
        let (got, _) = derive_record_topics(
            &live(),
            &mirrors,
            &rec_opts(|o| o.topics = vec!["/camera/image".to_string()]),
        )
        .unwrap();
        assert_eq!(got, vec!["/camera/image"]);
        assert!(
            logs_contain("cerulion-netd MIRROR"),
            "recording a mirror must SAY it is a mirror"
        );
        assert!(
            logs_contain("go2"),
            "and must name the origin robot it came from"
        );
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_genuinely_local_explicit_topic_announces_nothing() {
        // The anti-tautology half: without it the assertions above would pass a
        // reporter that warned on every explicit topic.
        let mut mirrors = BTreeMap::new();
        mirrors.insert("/camera/image".to_string(), "go2".to_string());
        let (got, _) = derive_record_topics(
            &live(),
            &mirrors,
            &rec_opts(|o| o.topics = vec!["/imu".to_string()]),
        )
        .unwrap();
        assert_eq!(got, vec!["/imu"]);
        assert!(
            !logs_contain("cerulion-netd MIRROR"),
            "a genuine local topic must not be announced as a mirror"
        );
    }

    // -----------------------------------------------------------------
    // The coverage manifest, as `bag info` renders it
    // -----------------------------------------------------------------

    /// Build a manifest from the fields this renderer reads. `..Default::default()`
    /// is deliberate: the manifest is an ADDITIVE artifact, and a field added to
    /// it must not force this file to change (nor let a test silently start
    /// asserting on a default it never chose).
    ///
    /// `pub(super)`: those arms live in their
    /// own module (they pin a defect that spans two crates and three surfaces,
    /// and burying them in this one would hide that) and must build their
    /// fixtures through this SAME helper — a second copy is how two test
    /// modules' idea of a manifest drifts apart.
    pub(super) fn coverage_of(
        enumerated: bool,
        tapped: &[(&str, cerulion_bagd::TapSource, u64, bool)],
        untapped: &[(&str, cerulion_bagd::UntappedReason)],
    ) -> cerulion_bagd::RecordCoverage {
        cerulion_bagd::RecordCoverage {
            version: cerulion_bagd::RECORD_COVERAGE_VERSION,
            enumerated,
            discovery_requested: enumerated,
            tapped: tapped
                .iter()
                .map(|(topic, source, frames_recorded, attached_late)| {
                    (
                        (*topic).to_string(),
                        cerulion_bagd::TappedTopic {
                            source: *source,
                            frames_recorded: *frames_recorded,
                            attached_late: *attached_late,
                            prefix_lost: None,
                            schema_source: None,
                        },
                    )
                })
                .collect(),
            untapped: untapped
                .iter()
                .map(|(topic, reason)| ((*topic).to_string(), reason.clone()))
                .collect(),
            ..Default::default()
        }
    }

    /// The row for `topic` in the rendered coverage output — the tapped table's
    /// rows start at column 0, the untapped list's are indented, so the match is
    /// on the row's FIRST token either way (never a bare `contains`, which would
    /// be satisfied by the topic and the claim landing on different lines).
    fn coverage_row<'a>(text: &'a str, topic: &str) -> &'a str {
        text.lines()
            .find(|l| l.split_whitespace().next() == Some(topic))
            .unwrap_or_else(|| panic!("no coverage row for {topic} in:\n{text}"))
    }

    #[test]
    fn a_discovered_tap_and_an_untapped_producer_are_both_rendered() {
        // The motivating shape end to end: one DECLARED tap, one topic nobody
        // declared that discovery picked up mid-run, and one live producer that
        // is simply NOT in the bag. All three must be visible, and they must be
        // distinguishable — a discovered late tap that rendered like a declared
        // one would imply coverage from frame zero that it does not have.
        let text = render_coverage(&coverage_of(
            true,
            &[
                ("/imu", cerulion_bagd::TapSource::Declared, 400, false),
                ("/lowstate", cerulion_bagd::TapSource::Discovered, 90, true),
            ],
            &[
                (
                    "/camera/h264",
                    cerulion_bagd::UntappedReason::AppearedAfterBagCreation,
                ),
                (
                    "/bagd/status",
                    cerulion_bagd::UntappedReason::ExcludedInternal,
                ),
                // A netd MIRROR of another robot's stream is the SECOND
                // exclusion-by-rule, and it is the one a hand-rolled
                // `!= ExcludedInternal` partition gets wrong.
                (
                    "/go2/lidar",
                    cerulion_bagd::UntappedReason::RemoteMirror {
                        robot: "go2".to_string(),
                    },
                ),
            ],
        ));
        assert!(
            text.contains("2 topic(s) tapped (1 declared, 1 discovered)"),
            "{text}"
        );
        assert!(text.contains("enumeration RAN"), "{text}");
        assert!(coverage_row(&text, "/imu").contains("declared"), "{text}");
        let discovered = coverage_row(&text, "/lowstate");
        assert!(discovered.contains("discovered"), "{discovered}");
        assert!(
            discovered.contains("no back-fill"),
            "a tap that attached mid-run must SAY its topic is not covered from the start: \
             {discovered}"
        );
        assert!(
            !coverage_row(&text, "/imu").contains("no back-fill"),
            "a tap present at arm time must not carry the late marker: {text}"
        );
        // The gap count is ONE: an exclusion by rule is not a coverage gap, and
        // inflating the number operators act on would train them to skim it.
        assert!(
            text.contains("coverage: INCOMPLETE — 1 live producer(s)"),
            "{text}"
        );
        assert!(
            coverage_row(&text, "/camera/h264").contains("appeared_after_bag_creation"),
            "{text}"
        );
        assert!(
            text.contains("--discovery-settle-ms"),
            "an untapped topic must carry the remedy for its OWN reason: {text}"
        );
        assert!(
            text.contains(
                "by rule (not a coverage gap): /bagd/status [excluded_internal], /go2/lidar \
                 [remote_mirror (robot go2)]"
            ),
            "BOTH exclusions by rule are still ACCOUNTED FOR, just not as gaps — and the mirror \
             names the robot it belongs to (F10): {text}"
        );
        assert!(text.contains("record_coverage.json"), "{text}");
        // The clean claim is exactly what this bag may NOT make.
        assert!(!text.contains("coverage: COMPLETE"), "{text}");
    }

    #[test]
    fn a_fully_covered_recording_says_so_and_prints_no_incomplete_line() {
        // The anti-tautology half of the test above: without it, a renderer that
        // printed INCOMPLETE unconditionally would pass every assertion there.
        let text = render_coverage(&coverage_of(
            true,
            &[("/imu", cerulion_bagd::TapSource::Declared, 400, false)],
            &[],
        ));
        assert!(text.contains("coverage: COMPLETE"), "{text}");
        assert!(!text.contains("INCOMPLETE"), "{text}");
        assert!(!text.contains("by rule"), "{text}");
        // Named on a CLEAN run too — the reader who needs the detail is often
        // the one whose summary looks fine (the `record_health.json` precedent).
        assert!(text.contains("record_coverage.json"), "{text}");
    }

    #[test]
    fn an_unenumerated_manifest_makes_no_coverage_claim_either_way() {
        // `enumerated: false` reaches the reader by TWO routes with OPPOSITE
        // remedies, and neither may render as a clean bill of health.
        let opted_out = render_coverage(&cerulion_bagd::RecordCoverage {
            enumerated: false,
            discovery_requested: false,
            ..coverage_of(
                true,
                &[("/imu", cerulion_bagd::TapSource::Declared, 400, false)],
                &[],
            )
        });
        assert!(opted_out.contains("enumeration did NOT run"), "{opted_out}");
        assert!(
            opted_out.contains("FIXED by the caller before the recording started"),
            "{opted_out}"
        );
        // This arm is reached by `bag record --all`, whose set was DERIVED
        // from a live scan, so it may not claim the caller named its topics.
        assert!(
            !opted_out.contains("explicit selection"),
            "an auto-selected `--all` bag reaches this arm too, so it must not be described as \
             an explicit selection: {opted_out}"
        );
        assert!(
            !opted_out.contains("coverage: COMPLETE"),
            "an empty untapped list that nobody looked for is not a coverage claim: {opted_out}"
        );
        // Nor is it a WARNING: the caller chose its own tap set, which is not a
        // defect. This arm gets no verdict word at all — the same call the
        // recorder's own `is_incomplete` makes when it decides whether to warn.
        assert!(!opted_out.contains("coverage: INCOMPLETE"), "{opted_out}");

        let all_scans_failed = render_coverage(&cerulion_bagd::RecordCoverage {
            enumerated: false,
            discovery_requested: true,
            enumeration_failures: 7,
            ..coverage_of(
                true,
                &[("/imu", cerulion_bagd::TapSource::Declared, 400, false)],
                &[],
            )
        });
        assert!(
            all_scans_failed.contains("REQUESTED but every attempt"),
            "a run that asked and could not be answered must not read as an opt-out: \
             {all_scans_failed}"
        );
        assert!(
            !all_scans_failed.contains("coverage: COMPLETE"),
            "{all_scans_failed}"
        );
        // A run that ASKED and could not be answered IS a defect, and the
        // recorder's terminal line warns about it — so the reader must reach
        // the same verdict rather than printing an ambiguous silence.
        assert!(
            all_scans_failed.contains("coverage: INCOMPLETE — no producer is KNOWN to be missing"),
            "{all_scans_failed}"
        );
    }

    /// A bag whose recorder could not establish the mirror picture says
    /// so, and withholds the COMPLETE claim — while a bag that established it,
    /// and an older bag that carries no verdict at all, are unchanged.
    ///
    /// The manifest field is only worth carrying if a reader surfaces it: the
    /// recorder's own warning scrolls away with the run, and this is the durable
    /// half. All three values are rendered here because the two silent ones are
    /// where a wrong default would hide — `None` in particular must stay silent,
    /// since every bag recorded before this change deserializes to it.
    #[test]
    fn a_recording_that_could_not_read_the_mirror_registry_says_so_and_withholds_complete() {
        let tapped = [(
            "/utlidar/cloud",
            cerulion_bagd::TapSource::Discovered,
            900,
            false,
        )];
        const MIRROR_LINE: &str = "could NOT establish which local topics are mirrors";

        let unverified = render_coverage(&cerulion_bagd::RecordCoverage {
            mirrors_established: Some(false),
            ..coverage_of(true, &tapped, &[])
        });
        assert!(unverified.contains(MIRROR_LINE), "{unverified}");
        assert!(
            unverified.contains("may really be another robot's re-injected stream"),
            "the reader must learn what the doubt IS — that a topic listed as tapped may not be \
             this machine's data: {unverified}"
        );
        assert!(
            !unverified.contains("coverage: COMPLETE"),
            "a bag that cannot rule out recording another robot's stream is not COMPLETE: \
             {unverified}"
        );
        // The verdict line must name the mirror doubt. The generic
        // INCOMPLETE sentence says "the recorder could not establish what was
        // live", which on this arm is FALSE (enumeration ran and succeeded) and
        // contradicts the "enumeration RAN" line two rows above it.
        assert!(
            unverified.contains("coverage: UNVERIFIED"),
            "the verdict must name THIS doubt, not the enumeration one: {unverified}"
        );
        assert!(
            !unverified.contains("could not establish what was live"),
            "enumeration RAN on this arm — a verdict claiming otherwise contradicts the block's \
             own first line: {unverified}"
        );
        assert!(
            unverified.contains("every live producer the recorder enumerated is in this bag"),
            "on THIS path the enumeration claim is true and belongs: {unverified}"
        );

        // Established: the shipping healthy path. Silent about mirrors, and
        // still COMPLETE — the anti-tautology half.
        let established = render_coverage(&cerulion_bagd::RecordCoverage {
            mirrors_established: Some(true),
            ..coverage_of(true, &tapped, &[])
        });
        assert!(!established.contains(MIRROR_LINE), "{established}");
        assert!(established.contains("coverage: COMPLETE"), "{established}");

        // No verdict (an older bag, or discovery never requested): silence is
        // the correct rendering, and the bag keeps whatever verdict it had.
        let pre_978 = render_coverage(&cerulion_bagd::RecordCoverage {
            mirrors_established: None,
            ..coverage_of(true, &tapped, &[])
        });
        assert!(
            !pre_978.contains(MIRROR_LINE),
            "a bag that makes no claim must not be reported as having failed one: {pre_978}"
        );
        assert!(pre_978.contains("coverage: COMPLETE"), "{pre_978}");
    }

    /// The `UNVERIFIED` verdict says only what is
    /// TRUE ON THE PATH THAT REACHED IT — the whole reachable
    /// (enumerated × mirrors_established × gaps) table, in one body.
    ///
    /// The first verdict opened with "every live producer the recorder
    /// enumerated is in this bag" UNCONDITIONALLY, and the first review's own fix
    /// made `cerulion bag record` reach that arm — where discovery is OFF, the
    /// claim is FALSE, and it printed three lines under this block's own
    /// "enumeration did NOT run" / "makes NO claim about what else was live"
    /// text. That is the contradicting-verdict class F2 was raised to fix,
    /// re-created on a second path by the fix for it, which is why the table is
    /// enumerated here rather than sampled.
    ///
    /// `mirrors_established: None` and the gap-bearing rows are covered by the
    /// sibling arms; this one owns every row where the mirror doubt is live.
    /// **The producer claim on a CAPTURE, across every arm that makes
    /// it.**
    ///
    /// "Every live producer the recorder enumerated is in this bag" is a sentence
    /// about a RECORDING; a capture holds a rolling WINDOW of each of those
    /// producers, and printing it under the capture caveat contradicts the
    /// paragraph the block just emitted. The COMPLETE gate was corrected for that
    /// first — and THREE further arms built the same clause and were missed,
    /// which is why the claim now comes from one function.
    ///
    /// Both arms driven here are reachable on a SHIPPING capture, not in theory:
    /// the always-on window recorder is run-bound (so a failed or never-heard
    /// watcher fires the run-binding arm) and `mirrors_established: Some(false)`
    /// LATCHES for the recorder's whole life, which on that recorder is the whole
    /// `graph run`.
    ///
    /// Each arm carries its RECORDING twin in the same body over the same fixture
    /// — without them, a renderer that dropped the claim everywhere passes.
    #[test]
    fn no_arm_tells_a_capture_that_every_producer_is_in_this_bag() {
        const RECORDING_CLAIM: &str = "every live producer the recorder enumerated is in this bag";
        const CAPTURE_CLAIM: &str = "has a channel in this bag";
        let tapped = [("/imu", cerulion_bagd::TapSource::Discovered, 9, false)];

        // ARM: the MIRROR verdict.
        let mirror = |window: bool| {
            render_coverage(&cerulion_bagd::RecordCoverage {
                window_capture: window,
                mirrors_established: Some(false),
                ..coverage_of(true, &tapped, &[])
            })
        };
        let recording = mirror(false);
        assert!(recording.contains("coverage: UNVERIFIED"), "{recording}");
        assert!(
            recording.contains(RECORDING_CLAIM),
            "CONTROL: a recording still makes the claim — without this, a renderer that dropped \
             it everywhere passes: {recording}"
        );
        let capture = mirror(true);
        assert!(
            capture.contains("coverage: UNVERIFIED"),
            "the verdict itself is unchanged — only the producer clause is: {capture}"
        );
        assert!(
            !capture.contains(RECORDING_CLAIM),
            "a capture holds a WINDOW of every producer, so it may not be told they are `in this \
             bag` three lines under the caveat saying otherwise: {capture}"
        );
        assert!(capture.contains(CAPTURE_CLAIM), "{capture}");

        // ARM: the RUN-BINDING verdict. `never_heard` is what makes it ambiguous,
        // and it is the state the always-on recorder reaches when its run dies
        // before first contact — the crash a black box exists for.
        let bound = |window: bool| {
            render_coverage(&cerulion_bagd::RecordCoverage {
                window_capture: window,
                run_binding: Some(cerulion_bagd::RunBindingCoverage {
                    run_id: 0x1420,
                    ended: None,
                    watch_failed: false,
                    unheard_for_ms: 900,
                    unattributed_frames: 9,
                    successor_seen: false,
                    never_heard: true,
                }),
                ..coverage_of(true, &tapped, &[])
            })
        };
        let recording = bound(false);
        assert!(recording.contains("coverage: INCOMPLETE"), "{recording}");
        assert!(recording.contains(RECORDING_CLAIM), "CONTROL: {recording}");
        assert!(
            recording.contains("the RECORDER was stopped"),
            "CONTROL: on a RECORDING, an unobserved end really does mean the recorder stopped \
             first: {recording}"
        );
        let capture = bound(true);
        assert!(capture.contains("coverage: INCOMPLETE"), "{capture}");
        assert!(!capture.contains(RECORDING_CLAIM), "{capture}");
        assert!(
            !capture.contains("the RECORDER was stopped"),
            "a capture is CUT from a live window while the run keeps going, so this is \
             affirmatively false — and it is the ORDINARY case, since the always-on window \
             recorder is run-bound and every trigger-fired capture of a healthy run lands on \
             that arm: {capture}"
        );
        assert!(
            capture.contains("cut from the live window"),
            "…and it says what actually happened: {capture}"
        );
    }

    /// **The NO GAPS sentence names EVERY qualifier that applies.**
    ///
    /// A capture's own qualifier (the WINDOW) is stated ABOVE the table; the
    /// schema and trace qualifiers print BELOW it. A capture can carry either —
    /// `build_capture_coverage` grades `replay_grade` from the capture's own
    /// descriptors and deliberately carries `rings_declared` / `rings_unavailable`
    /// — so a sentence that named only the window sent an operator away from a
    /// `trace: NONE` line printed immediately under it.
    #[test]
    fn a_captures_no_gaps_sentence_points_at_the_lines_that_actually_print() {
        const BELOW: &str = "line(s) below";
        let tapped = [("/imu", cerulion_bagd::TapSource::Discovered, 9, false)];

        // WINDOW alone: the qualifier is above, and there is nothing below.
        let window_only = render_coverage(&cerulion_bagd::RecordCoverage {
            window_capture: true,
            ..coverage_of(true, &tapped, &[])
        });
        assert!(window_only.contains("coverage: NO GAPS"), "{window_only}");
        assert!(window_only.contains("by the WINDOW"), "{window_only}");
        assert!(
            !window_only.contains(BELOW),
            "nothing prints below on this one, so the sentence must not send the reader there: \
             {window_only}"
        );

        // WINDOW *and* a degraded trace: BOTH are named.
        let mut with_trace = cerulion_bagd::RecordCoverage {
            window_capture: true,
            rings_declared: 1,
            ..coverage_of(true, &tapped, &[])
        };
        with_trace
            .rings_unavailable
            .insert("cer_trace_ring".into(), "vanished".into());
        let text = render_coverage(&with_trace);
        assert!(text.contains("coverage: NO GAPS"), "{text}");
        assert!(text.contains("by the WINDOW"), "{text}");
        assert!(
            text.contains(BELOW),
            "a capture whose trace ring never opened prints a `trace:` line under this verdict, \
             so the sentence must point at it: {text}"
        );
        assert!(
            text.contains("trace:"),
            "ANTI-VACUITY: the line it points at really is printed: {text}"
        );

        // CONTROL: a RECORDING's sentence is unchanged.
        let recording = render_coverage(&cerulion_bagd::RecordCoverage {
            ..coverage_of(true, &tapped, &[])
        });
        assert!(
            recording.contains("coverage: COMPLETE"),
            "CONTROL: a clean recording still earns COMPLETE: {recording}"
        );
    }

    #[test]
    fn the_unverified_verdict_states_only_what_is_true_on_the_path_that_reached_it() {
        let tapped = [(
            "/utlidar/cloud",
            cerulion_bagd::TapSource::Discovered,
            9,
            false,
        )];
        const ENUM_CLAIM: &str = "every live producer the recorder enumerated is in this bag";
        const ENUM_DOUBT: &str = "could not establish what was live";
        const NO_CLAIM: &str = "makes no claim about what else was live";

        // ROW 1 — `graph run --record`, enumeration RAN: the enumeration claim
        // is true and belongs.
        let enumerated = render_coverage(&cerulion_bagd::RecordCoverage {
            mirrors_established: Some(false),
            ..coverage_of(true, &tapped, &[])
        });
        assert!(enumerated.contains("coverage: UNVERIFIED"), "{enumerated}");
        assert!(enumerated.contains(ENUM_CLAIM), "{enumerated}");
        assert!(!enumerated.contains(ENUM_DOUBT), "{enumerated}");

        // ROW 2 — `cerulion bag record`: discovery was never REQUESTED, so
        // there is no enumeration claim to make in either direction. This is
        // the row F0/F3 found: it must not assert the enumeration claim, and
        // must not contradict the block's own "did NOT run" line above it.
        let never_asked = render_coverage(&cerulion_bagd::RecordCoverage {
            enumerated: false,
            discovery_requested: false,
            mirrors_established: Some(false),
            ..coverage_of(true, &tapped, &[])
        });
        assert!(
            never_asked.contains("coverage: UNVERIFIED"),
            "{never_asked}"
        );
        assert!(
            !never_asked.contains(ENUM_CLAIM),
            "discovery was OFF — claiming every ENUMERATED producer is in the bag contradicts \
             this block's own 'enumeration did NOT run' line: {never_asked}"
        );
        assert!(
            never_asked.contains(NO_CLAIM),
            "the correct opening for this path is that no enumeration claim is made: {never_asked}"
        );

        // ROW 3 — asked and could NOT be answered: BOTH doubts are real, and
        // the verdict names both rather than dropping one.
        let asked_and_failed = render_coverage(&cerulion_bagd::RecordCoverage {
            enumerated: false,
            discovery_requested: true,
            mirrors_established: Some(false),
            ..coverage_of(true, &tapped, &[])
        });
        assert!(
            asked_and_failed.contains("coverage: UNVERIFIED"),
            "{asked_and_failed}"
        );
        assert!(asked_and_failed.contains(ENUM_DOUBT), "{asked_and_failed}");
        assert!(!asked_and_failed.contains(ENUM_CLAIM), "{asked_and_failed}");

        // ROW 4 — a real GAP outranks the mirror doubt: the gap arm names the
        // producers that are missing, which is the more actionable verdict, and
        // the mirror line above it still carries the other doubt.
        let with_gap = render_coverage(&cerulion_bagd::RecordCoverage {
            mirrors_established: Some(false),
            ..coverage_of(
                true,
                &tapped,
                &[(
                    "/late",
                    cerulion_bagd::UntappedReason::AppearedAfterBagCreation,
                )],
            )
        });
        assert!(
            with_gap.contains("coverage: INCOMPLETE — 1 live producer(s)"),
            "a KNOWN missing producer is the verdict an operator can act on: {with_gap}"
        );
        assert!(
            with_gap.contains("could NOT establish which local topics are mirrors"),
            "and the mirror doubt is still reported above it: {with_gap}"
        );

        // ROW 5 — the anti-tautology control: with the picture ESTABLISHED, no
        // row above may fire, and the clean claim returns.
        let established = render_coverage(&cerulion_bagd::RecordCoverage {
            mirrors_established: Some(true),
            ..coverage_of(true, &tapped, &[])
        });
        assert!(
            !established.contains("coverage: UNVERIFIED"),
            "{established}"
        );
        assert!(established.contains("coverage: COMPLETE"), "{established}");
    }

    /// A bag missing the HEAD of a topic it contains must SAY so —
    /// on the row, in a detail line, and in the verdict word.
    ///
    /// `is_incomplete()` gained the prefix term, so without a verdict arm of its
    /// own this bag falls through to the generic INCOMPLETE sentence ("no
    /// producer is KNOWN to be missing, but the recorder could not establish
    /// what was live"), which on this path is FALSE twice over — enumeration RAN
    /// and the loss is measured. That is the contradicting-verdict class this
    /// block already carries two fixes for.
    #[test]
    fn a_bag_missing_the_start_of_a_topic_says_so_on_the_row_and_in_the_verdict() {
        let with_prefix_loss = |topic: &str, n: u64| {
            let mut c = cerulion_bagd::RecordCoverage {
                armed_before_producers: true,
                ..coverage_of(
                    true,
                    &[
                        ("/burst", cerulion_bagd::TapSource::Declared, 8, false),
                        ("/steady", cerulion_bagd::TapSource::Declared, 400, false),
                    ],
                    &[],
                )
            };
            c.tapped.get_mut(topic).expect("fixture row").prefix_lost = Some(n);
            c
        };

        let text = render_coverage(&with_prefix_loss("/burst", 392));
        // (1) The ROW carries the count — an operator scanning the table sees
        //     WHICH topic begins mid-stream, and by how much.
        let burst = coverage_row(&text, "/burst");
        assert!(
            burst.contains("missing the first 392 frame(s)"),
            "the row must name the frames the wire proves absent: {burst}"
        );
        assert!(
            !coverage_row(&text, "/steady").contains("missing the first"),
            "a topic recorded from its first frame must carry no such marker: {text}"
        );
        // (2) The DETAIL line explains why `frames_lost = 0` is not the whole
        //     story: the exact reading the original head-loss report got.
        assert!(
            text.contains("392 frame(s) across 1 topic(s) were committed BEFORE"),
            "{text}"
        );
        assert!(
            text.contains("invisible to record_health.json's frames_lost"),
            "{text}"
        );
        // (3) The VERDICT names THIS loss, and does not borrow a sentence that
        //     is false here.
        assert!(
            text.contains("missing the START of their stream"),
            "the verdict must name the head loss: {text}"
        );
        assert!(
            !text.contains("could not establish what was live"),
            "enumeration RAN on this arm: {text}"
        );
        assert!(!text.contains("coverage: COMPLETE"), "{text}");

        // (4) The ANTI-TAUTOLOGY control: the same fixture with nothing lost is
        //     silent about head loss and COMPLETE again. Without it, a renderer
        //     that printed the marker unconditionally passes everything above.
        let clean = render_coverage(&cerulion_bagd::RecordCoverage {
            armed_before_producers: true,
            ..coverage_of(
                true,
                &[("/burst", cerulion_bagd::TapSource::Declared, 400, false)],
                &[],
            )
        });
        assert!(!clean.contains("missing the first"), "{clean}");
        assert!(!clean.contains("were committed BEFORE"), "{clean}");
        assert!(clean.contains("coverage: COMPLETE"), "{clean}");

        // (5) The enumeration half is stated PER PATH (the mirror arm's rule):
        //     `armed_before_producers` is independent of `discover_live`, so a
        //     caller can reach this verdict with discovery OFF, where "every
        //     live producer the recorder enumerated is in this bag" would print
        //     under the block's own "enumeration did NOT run" line.
        let mut never_asked = with_prefix_loss("/burst", 7);
        never_asked.enumerated = false;
        never_asked.discovery_requested = false;
        let text = render_coverage(&never_asked);
        assert!(text.contains("missing the START of their stream"), "{text}");
        assert!(
            !text.contains("every live producer the recorder enumerated is in this bag"),
            "discovery was OFF — that claim contradicts this block's own first line: {text}"
        );
        assert!(
            text.contains("makes no claim about what else was live"),
            "{text}"
        );

        // (6) A real GAP outranks it: the missing-producer verdict is the more
        //     actionable one, and the head-loss DETAIL line still prints above.
        let mut with_gap = with_prefix_loss("/burst", 392);
        with_gap.untapped.insert(
            "/late".to_string(),
            cerulion_bagd::UntappedReason::AppearedAfterBagCreation,
        );
        let text = render_coverage(&with_gap);
        assert!(
            text.contains("coverage: INCOMPLETE — 1 live producer(s)"),
            "{text}"
        );
        assert!(text.contains("392 frame(s) across 1 topic(s)"), "{text}");
    }

    #[test]
    fn a_recording_that_lost_an_enumeration_withholds_the_complete_claim() {
        // Enumeration ran AND a scan failed: the untapped list is real but
        // incomplete by an unknown amount, so "COMPLETE" would be a claim the
        // recorder is not in a position to make.
        let text = render_coverage(&cerulion_bagd::RecordCoverage {
            enumeration_failures: 3,
            ..coverage_of(
                true,
                &[("/imu", cerulion_bagd::TapSource::Declared, 400, false)],
                &[],
            )
        });
        assert!(text.contains("3 enumeration(s) FAILED"), "{text}");
        assert!(
            !text.contains("coverage: COMPLETE"),
            "a lost scan means the empty gap list is not the whole truth: {text}"
        );
        assert!(
            text.contains("coverage: INCOMPLETE — no producer is KNOWN to be missing"),
            "and the verdict must MATCH the one the recorder's own is_incomplete reached: {text}"
        );
    }

    #[test]
    fn every_untapped_reason_class_carries_its_own_remedy() {
        let text = render_coverage(&coverage_of(
            true,
            &[],
            &[
                (
                    "/late",
                    cerulion_bagd::UntappedReason::AppearedAfterBagCreation,
                ),
                (
                    "/over",
                    cerulion_bagd::UntappedReason::BudgetExhausted { budget: 256 },
                ),
                (
                    "/refused",
                    cerulion_bagd::UntappedReason::AttachFailed {
                        error: "slots exhausted".to_string(),
                    },
                ),
            ],
        ));
        assert!(
            text.contains("coverage: INCOMPLETE — 3 live producer(s)"),
            "{text}"
        );
        for tag in [
            "appeared_after_bag_creation",
            "budget_exhausted",
            "attach_failed",
        ] {
            assert!(text.contains(tag), "every gap is named by its tag: {text}");
        }
        assert!(text.contains("--discovery-settle-ms"), "{text}");
        assert!(
            text.contains(&cerulion_bagd::DISCOVERY_MAX_TAPS.to_string()),
            "the budget remedy must name the ceiling that was hit: {text}"
        );
        assert!(text.contains("could not open a tap"), "{text}");
    }

    #[test]
    fn every_remedy_names_something_a_graph_run_record_user_can_reach() {
        // Discovery is ON by default on exactly ONE path —
        // `graph run --record`, whose `--topics-json` tap universe is INFERRED —
        // and `graph_cmd::spawn_bagd_recorder` builds that recorder's whole argv
        // internally. So a remedy phrased as a bagd FLAG is unactionable for the
        // reader most likely to be holding one of these bags. Both remedies
        // are pinned here as more than a flag:
        //
        //  - `appeared_after_bag_creation` must not say only "raise `cerulion bagd
        //    --discovery-settle-ms`". It must lead with the
        //    ENV form, which reaches a spawned recorder.
        //  - `budget_exhausted` must not say "name the topics you need explicitly", which
        //    on that path names a list the CLI GENERATES from the graph.
        let text = render_coverage(&coverage_of(
            true,
            &[],
            &[
                (
                    "/late",
                    cerulion_bagd::UntappedReason::AppearedAfterBagCreation,
                ),
                (
                    "/over",
                    cerulion_bagd::UntappedReason::BudgetExhausted { budget: 256 },
                ),
            ],
        ));
        assert!(
            text.contains(RECORD_DISCOVERY_SETTLE_ENV_NAME),
            "the settle remedy must name the ENVIRONMENT form — it is the only one that reaches \
             `graph run --record`, which builds bagd's argv itself: {text}"
        );
        // The flag stays named: a hand-run `cerulion bagd` is the other way to
        // reach this gap, and there the flag IS the direct knob.
        assert!(
            text.contains("cerulion bagd --discovery-settle-ms"),
            "{text}"
        );
        assert!(
            !text.contains("Name the topics you need explicitly"),
            "the budget remedy must not tell a `graph run --record` user to edit a tap list the \
             CLI generates for them: {text}"
        );
        assert!(
            text.contains("DECLARED taps are exempt"),
            "the budget remedy must say what IS true — declared taps do not count against the \
             ceiling: {text}"
        );
    }

    #[test]
    fn a_remote_mirror_and_an_attach_failure_name_their_payloads() {
        // `tag()` is payload-free by design, and printing
        // ONLY the tag withheld the two fields an operator acts on. A desk
        // holding two robots' mirrors cannot tell them apart from
        // `remote_mirror` alone, and `attach_failed` alone does not distinguish
        // slot exhaustion from a vanished service.
        let text = render_coverage(&coverage_of(
            true,
            &[],
            &[
                (
                    "/go2/lidar",
                    cerulion_bagd::UntappedReason::RemoteMirror {
                        robot: "go2".to_string(),
                    },
                ),
                (
                    "/spot/lidar",
                    cerulion_bagd::UntappedReason::RemoteMirror {
                        robot: "spot".to_string(),
                    },
                ),
                (
                    "/refused",
                    cerulion_bagd::UntappedReason::AttachFailed {
                        error: "subscriber slots exhausted (max_subscribers = 8)".to_string(),
                    },
                ),
                (
                    "/over",
                    cerulion_bagd::UntappedReason::BudgetExhausted { budget: 256 },
                ),
            ],
        ));
        // The two mirrors are DISTINGUISHABLE — the whole point of the variant
        // carrying a robot at all.
        assert!(
            text.contains("/go2/lidar [remote_mirror (robot go2)]"),
            "{text}"
        );
        assert!(
            text.contains("/spot/lidar [remote_mirror (robot spot)]"),
            "{text}"
        );
        // The transport's own message rides the GAP row, beside its topic.
        assert!(
            coverage_row(&text, "/refused")
                .contains("attach_failed: subscriber slots exhausted (max_subscribers = 8)"),
            "{text}"
        );
        // And the ceiling rides its own row, so it reads standalone.
        assert!(
            coverage_row(&text, "/over").contains("budget_exhausted (ceiling 256)"),
            "{text}"
        );
    }

    #[test]
    fn a_lan_supplied_robot_name_cannot_inject_terminal_escapes() {
        // The mirror's robot identity is announced by a REMOTE machine and the
        // transport error interpolates a topic name that, for a mirror, is
        // likewise remote-supplied — the same untrusted-text class `topic list`
        // sanitizes. Rendering either verbatim would let a LAN peer write
        // ANSI/CSI into an operator's terminal through a BAG.
        let text = render_coverage(&coverage_of(
            true,
            &[],
            &[
                (
                    "/hostile/mirror",
                    cerulion_bagd::UntappedReason::RemoteMirror {
                        robot: "go2\u{1b}[2Kevil".to_string(),
                    },
                ),
                (
                    "/hostile/attach",
                    cerulion_bagd::UntappedReason::AttachFailed {
                        error: "open failed on /x\u{7}\u{1b}[31m".to_string(),
                    },
                ),
            ],
        ));
        assert!(
            !text.contains('\u{1b}') && !text.contains('\u{7}'),
            "no control character may survive into the rendered coverage block: {text:?}"
        );
        // Anti-tautology: the payloads really are being rendered (a renderer
        // that dropped them entirely would also pass the assertion above).
        assert!(text.contains("go2\u{fffd}[2Kevil"), "{text:?}");
        assert!(
            text.contains("open failed on /x\u{fffd}\u{fffd}[31m"),
            "{text:?}"
        );
    }

    #[test]
    fn the_bag_info_example_in_docs_bag_md_is_what_the_renderer_prints() {
        // `docs/bag.md`'s "How to read it" block shows a
        // `bag info` coverage section rendered from the JSON printed directly
        // above it — and it was hand-written, so it had silently dropped the
        // by-rule line its own manifest produces (`/bagd/status`, which that
        // JSON lists under `untapped`). A reader comparing the two would
        // conclude an exclusion-by-rule is not shown at all.
        //
        // This renders the doc's OWN manifest and requires the result to appear
        // in the file verbatim, so the example cannot drift from the renderer
        // again. On failure, paste what this prints.
        let text = render_coverage(&coverage_of(
            true,
            &[
                ("/imu", cerulion_bagd::TapSource::Declared, 400, false),
                ("/lowstate", cerulion_bagd::TapSource::Discovered, 91, true),
            ],
            &[
                (
                    "/camera/h264",
                    cerulion_bagd::UntappedReason::AppearedAfterBagCreation,
                ),
                (
                    "/bagd/status",
                    cerulion_bagd::UntappedReason::ExcludedInternal,
                ),
            ],
        ));
        let doc_path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("..")
            .join("docs")
            .join("bag.md");
        let doc = std::fs::read_to_string(&doc_path)
            .unwrap_or_else(|e| panic!("cannot read {}: {e}", doc_path.display()));
        let block = text.trim_start_matches('\n');
        assert!(
            doc.contains(block),
            "docs/bag.md's `bag info` coverage example does not match the renderer. Replace the \
             example block with EXACTLY this:\n---8<---\n{block}--->8---"
        );
    }

    #[test]
    fn a_capture_whose_pre_close_drain_did_not_complete_says_so_and_withholds_complete() {
        let capture = |incomplete: Option<bool>| cerulion_bagd::RecordCoverage {
            window_capture: true,
            pre_close_drain_incomplete: incomplete,
            pre_close_drain_stops: if incomplete == Some(true) {
                vec![
                    cerulion_bagd::PreCloseDrainStop::StagingFull {
                        topic: "/cam".into(),
                    },
                    cerulion_bagd::PreCloseDrainStop::ReceiveError {
                        error: "receive failed on /imu".into(),
                    },
                ]
            } else {
                Vec::new()
            },
            ..coverage_of(
                true,
                &[("/cam", cerulion_bagd::TapSource::Declared, 40, false)],
                &[],
            )
        };
        let text = render_coverage(&capture(Some(true)));
        assert!(
            text.contains("pre_close_drain_incomplete: true"),
            "the incomplete drain must be stated above the table, keyed by the manifest field \
             an operator can grep: {text}"
        );
        assert!(
            text.contains(
                "the close's final drain did not complete: /cam was left at the \
                           recorder's staging bound with frames still queued; a receive error \
                           (receive failed on /imu)."
            ),
            "every recorded stop is rendered, in order: {text}"
        );
        assert!(
            text.contains("the END of this capture may be missing frames"),
            "{text}"
        );
        assert!(text.contains("coverage: INCOMPLETE"), "{text}");
        assert!(!text.contains("coverage: COMPLETE"), "{text}");
        assert!(!text.contains("coverage: NO GAPS"), "{text}");
        assert!(
            text.contains("has a channel in this bag (a WINDOW of it"),
            "enumeration RAN on this arm, so the verdict keeps the capture's producer claim: \
             {text}"
        );
        for clean in [Some(false), None] {
            let text = render_coverage(&capture(clean));
            assert!(
                !text.contains("pre_close_drain") && !text.contains("final drain did not complete"),
                "ANTI-TAUTOLOGY: a close whose drain succeeded, or a manifest that makes no \
                 drain claim, must not be marked: {text}"
            );
            assert!(
                !text.contains("the END of this capture may be missing frames"),
                "{text}"
            );
            assert!(
                !text.contains("coverage: INCOMPLETE") && text.contains("coverage: NO GAPS"),
                "a capture with every other term clean reaches no INCOMPLETE verdict on this \
                 arm: {text}"
            );
        }
    }

    #[test]
    fn the_two_excluded_prefix_lists_are_the_same_list() {
        // `AUTO_SELECT_EXCLUDED_PREFIXES` (this crate, for
        // `bag record --all` / `--regex`) and `cerulion_bagd::EXCLUDED_TOPIC_PREFIXES`
        // (bagd, for live discovery) are two hand-typed literals in two crates,
        // and both docs claim they are the same list. Nothing checked it: the
        // `const _` guard beside each only pins the `__cerulion/` element against
        // `cerulion_bag::RESERVED_PREFIX`, so either list could grow a fourth
        // entry the other lacked and both guards would stay green.
        //
        // ORDER is asserted too, not just set membership: these are `starts_with`
        // prefixes evaluated in order and the lists are read side by side by
        // anyone auditing what a recording excludes, so a silent reorder is drift
        // worth failing on.
        assert_eq!(
            AUTO_SELECT_EXCLUDED_PREFIXES,
            cerulion_bagd::EXCLUDED_TOPIC_PREFIXES,
            "the auto-select exclusions and bagd's discovery exclusions must stay the SAME list — \
             a topic bagd discovers but `bag record --all` refuses (or vice versa) means one \
             recording verb captures the framework's own channels and the other does not"
        );
    }

    #[test]
    fn an_absent_or_unreadable_manifest_never_reads_as_clean_coverage() {
        // The four readings answer four different questions. Collapsing any of
        // them into "nothing was missed" would re-create the original defect one
        // layer up, in the reader.
        let pre942 = render_coverage_section(&CoverageReading::AbsentPre942);
        assert!(pre942.contains("carries no"), "{pre942}");
        assert!(pre942.contains("record_coverage.json"), "{pre942}");
        assert!(
            pre942.contains("NOT a clean-coverage claim"),
            "an absent manifest must be stated as an ABSENCE: {pre942}"
        );
        // The CLAIM itself, pinned POSITIVELY rather than only via the
        // disclaimer above. Those two assertions live on different sentences, so
        // a rewrite that turned this arm into a clean-coverage claim while
        // leaving the trailing disclaimer intact satisfies every other assertion
        // here, so this sentence needs its own pin.
        // Re-recording is also the only remedy this arm can offer: no
        // file path exists to point at.
        assert!(
            pre942.contains("never measured what else was live"),
            "the pre-942 arm must state what the recorder did NOT do: {pre942}"
        );
        assert!(
            pre942.contains("re-record"),
            "…and name the only remedy it has: {pre942}"
        );

        let unreadable = render_coverage_section(&CoverageReading::IndexUnreadable);
        assert!(
            unreadable.contains("not finalized"),
            "a non-finalized bag could not be LOOKED AT — a different fact from carrying \
             nothing: {unreadable}"
        );
        assert!(
            unreadable.contains("complete OR incomplete"),
            "{unreadable}"
        );
        assert_ne!(
            pre942, unreadable,
            "the two absences carry different remedies and must not render identically"
        );

        let malformed =
            render_coverage_section(&CoverageReading::Malformed("expected value".to_string()));
        assert!(malformed.contains("MALFORMED"), "{malformed}");
        assert!(malformed.contains("expected value"), "{malformed}");

        for text in [&pre942, &unreadable, &malformed] {
            assert!(!text.contains("COMPLETE —"), "{text}");
            assert!(!text.contains("INCOMPLETE"), "{text}");
        }
        // Anti-tautology: the Present arm DOES delegate to the real renderer, so
        // the absence assertions above are not passing on an empty string.
        let present = render_coverage_section(&CoverageReading::Present(coverage_of(
            true,
            &[("/imu", cerulion_bagd::TapSource::Declared, 1, false)],
            &[],
        )));
        assert!(present.contains("coverage: COMPLETE"), "{present}");
    }

    #[test]
    fn an_explicit_list_names_every_untappable_topic_not_just_the_first() {
        // Naming only the first makes an operator fix them one
        // run at a time.
        let probe = TapProbe {
            attachable: Vec::new(),
            refused: vec![
                ("/a".to_string(), "slots exhausted".to_string()),
                ("/b".to_string(), "does not exist".to_string()),
                ("/c".to_string(), "borrow budget".to_string()),
            ],
        };
        let err = explicit_untappable_error(&probe).expect("must refuse");
        for t in ["/a", "/b", "/c"] {
            assert!(err.contains(t), "every offender must be named: {err}");
        }
        for r in ["slots exhausted", "does not exist", "borrow budget"] {
            assert!(err.contains(r), "with its own reason: {err}");
        }
        assert!(err.contains("3 named topic(s)"), "{err}");
        // Nothing refused ⇒ no error at all.
        assert!(explicit_untappable_error(&TapProbe {
            attachable: vec!["/a".to_string()],
            refused: Vec::new(),
        })
        .is_none());
    }

    /// A verdict row, hand-built. CEILING-deep unless a budget is given.
    fn absorbance_row(
        depth: u64,
        v: cerulion_bagd::AbsorbanceVerdict,
    ) -> cerulion_bagd::TopicAbsorbance {
        use cerulion_bagd::AbsorbanceVerdict as V;
        // Every fixture this mints is a row a PRODUCER could emit, which since
        // the reader validates rows is now load-bearing: a helper that carried a
        // comparison onto a `NoClaim` was building a self-contradicting row, and
        // arms using it were asserting about a state that cannot occur.
        //
        // `Unrankable` has no tail (the ladder-overflow shape) and `NoClaim` has
        // nothing at all — the two absences the vocabulary keeps apart.
        //
        // An `Unrankable` row is producible only ABOVE the ladder's
        // last edge, so a caller asking for one must pass a depth whose
        // `depth * 10_000` us clears `DRAIN_GAP_BUCKET_EDGES_US.last()` — the
        // assert below is what says so if it does not.
        //
        // The tail tracks the DEPTH, never a constant: the verdict IS
        // `absorbance >= tail`, so a fixed tail makes the helper sound at some
        // depths and self-contradicting at others (a depth-9 `Short` against a
        // fixed 25 ms is the shape that would have shipped).
        //
        // It is also a RUNG of that depth's ladder, not a number
        // computed beside the absorbance. A served tail is always a bucket EDGE
        // (`quantile_us` returns one, and has no other exit that yields a
        // value), so the tail is picked FROM the ladder — the nearest rung on
        // whichever side the requested verdict needs — and the required depth
        // and the shortfall then follow from that rung, as they do at the
        // producer.
        let absorbance_us = depth * 10_000;
        let rungs = cerulion_bagd::DRAIN_GAP_BUCKET_EDGES_US;
        let measured_tail_us = match v {
            // The deepest rung this depth covers…
            V::Absorbs => rungs.iter().rev().find(|e| **e <= absorbance_us).copied(),
            // …and the shallowest it does not.
            V::Short => rungs.iter().find(|e| **e > absorbance_us).copied(),
            _ => None,
        };
        assert_eq!(
            measured_tail_us.is_some(),
            matches!(v, V::Absorbs | V::Short),
            "no rung of the shipped ladder gives depth {depth} a {v:?} verdict at 100 Hz"
        );
        let row = cerulion_bagd::TopicAbsorbance {
            tap_buffer_depth: depth,
            rate_mhz: matches!(v, V::NoClaim).then_some(0).xor(Some(100_000)),
            rate_is_floor: false,
            absorbance_us: (!matches!(v, V::NoClaim)).then_some(absorbance_us),
            measured_tail_us,
            // `ceil(rate x tail)`, the producer's own arithmetic — one slot is
            // 10 ms at the 100 Hz this helper mints.
            required_depth: measured_tail_us.map(|t| t.div_ceil(10_000)),
            shortfall_at_least_us: measured_tail_us
                .filter(|t| *t > absorbance_us)
                .map(|t| t - absorbance_us),
            verdict: v,
            short_evaluations: 0,
            budget_bytes: None,
            over_budget: None,
            budget_unpriced: false,
        };
        assert_eq!(
            row.inconsistency(),
            None,
            "this helper may only mint rows a producer could emit: {row:?}"
        );
        row
    }

    // -----------------------------------------------------------------------
    // The record-summary ABSORBANCE line
    // -----------------------------------------------------------------------

    /// `bag record`'s terminal summary names the short topics AND the fix that
    /// applies to each — which on THIS verb is never the budget knob.
    ///
    /// `render_record_summary`'s only production caller is `cerulion bag record`
    /// (`crates/cerulion_cli/src/main.rs`), and that verb never builds a Flashback
    /// plane — so every tap it opens is ceiling-deep and a line telling the
    /// operator to "raise CERULION_FLASHBACK_TAP_BUDGET_MB, or record this topic
    /// explicitly" would be wrong on both halves, every single time.
    #[test]
    fn the_record_summary_names_short_topics_with_the_remedy_that_applies() {
        use cerulion_bagd::AbsorbanceVerdict;
        let mut h = health(10, 0, 0, false, false);
        h.absorbance = Some(absorbance_row(2, AbsorbanceVerdict::Short));
        let text = render_record_summary(&summary_with(vec![("/shallow", h)]));
        assert!(text.contains("absorbance: 1 topic(s)"), "{text}");
        assert!(text.contains("/shallow"), "{text}");
        assert!(text.contains("subscriber_buffer_size"), "{text}");
        assert!(
            !text.contains("CERULION_FLASHBACK_TAP_BUDGET_MB"),
            "a ceiling-deep tap must not be sent to the budget knob: {text}"
        );
        assert!(text.contains("nothing was COUNTED"), "{text}");
        assert!(text.contains("DRIVE-LOOP gaps"), "{text}");

        // ...and a BUDGETED tap gets the knob that does move it.
        let mut b = health(10, 0, 0, false, false);
        b.absorbance = Some(cerulion_bagd::TopicAbsorbance {
            budget_bytes: Some(64 * 1024 * 1024),
            ..absorbance_row(2, AbsorbanceVerdict::Short)
        });
        let budgeted = render_record_summary(&summary_with(vec![("/win", b)]));
        assert!(
            budgeted.contains("CERULION_FLASHBACK_TAP_BUDGET_MB"),
            "{budgeted}"
        );
    }

    /// ANTI-TAUTOLOGY: only SHORT (or ever-short) rows reach that line.
    ///
    /// `Unrankable` says two numbers could not be RANKED and `NoClaim` says
    /// nothing was comparable — neither is a shortfall an operator can act on,
    /// and printing them here would put three different epistemic states under
    /// one heading.
    #[test]
    fn the_record_summary_absorbance_line_is_absent_when_nothing_fell_short() {
        use cerulion_bagd::AbsorbanceVerdict;
        let mut rows = Vec::new();
        // The DEPTH is per-verdict, because `Unrankable` is only producible
        // above the ladder's last edge (the validator now holds it to
        // the producer's whole image, and at the fixed 100 Hz this helper mints,
        // depth 9 derives 90 ms — a value the ladder ranks perfectly well).
        for (topic, depth, v) in [
            ("/fine", 9, AbsorbanceVerdict::Absorbs),
            ("/unrankable", 4096, AbsorbanceVerdict::Unrankable),
            ("/young", 9, AbsorbanceVerdict::NoClaim),
        ] {
            let mut h = health(10, 0, 0, false, false);
            h.absorbance = Some(absorbance_row(depth, v));
            rows.push((topic, h));
        }
        let text = render_record_summary(&summary_with(rows));
        assert!(
            !text.contains("absorbance:"),
            "a run with no shortfall says nothing here: {text}"
        );

        // ...and a row that RECOVERED still reports, so the line is not keyed on
        // the final verdict alone.
        let mut healed = health(10, 0, 0, false, false);
        healed.absorbance = Some(cerulion_bagd::TopicAbsorbance {
            short_evaluations: 12,
            ..absorbance_row(9, AbsorbanceVerdict::Absorbs)
        });
        let text = render_record_summary(&summary_with(vec![("/healed", healed)]));
        assert!(text.contains("absorbance: 1 topic(s)"), "{text}");
        assert!(text.contains("/healed"), "{text}");
    }
}

/// `bag record --run` — the pure halves of the attach path.
///
/// The behavioural half needs a live registry writer, a live producer and a run
/// directory, and lives in `tests/bag_record_run_attach_test.rs`. What is here
/// is everything a wrong answer would express as a wrong VALUE: which run gets
/// picked, what the refusals say, and what the run-identity attachment carries.
#[cfg(test)]
mod run_attach {
    use super::*;
    use cerulion_core::transport::run_registry::{RunRecord, RunState};

    fn rec(id: u128, graph: &str, pid: u32) -> RunRecord {
        RunRecord {
            run_id: id,
            supervisor_pid: pid,
            run_started_at_ns: 1_753_000_000_000_000_000,
            state: RunState::Live,
            graph_name: graph.to_string(),
            run_dir: format!("/tmp/runs/{graph}-{id:032x}"),
        }
    }

    #[test]
    fn a_sole_run_is_attached_to_and_an_empty_machine_falls_back_rather_than_failing() {
        assert!(matches!(
            select_run(&[], &RunTarget::Sole),
            RunSelection::NoLiveRun
        ));
        match select_run(&[rec(7, "go2", 11)], &RunTarget::Sole) {
            RunSelection::Attach(r) => assert_eq!(r.run_id, 7),
            other => panic!("a single live run must be attached to, got {other:?}"),
        }
    }

    #[test]
    fn several_live_runs_and_no_id_is_a_refusal_that_names_every_candidate() {
        let runs = [rec(1, "go2", 11), rec(2, "arm", 12)];
        let selection = select_run(&runs, &RunTarget::Sole);
        let RunSelection::Ambiguous(listed) = &selection else {
            panic!("two live runs with no --run id must REFUSE, got {selection:?}");
        };
        assert_eq!(listed.len(), 2);

        let msg = run_selection_error(&selection).expect("an ambiguity must produce a refusal");
        // Every candidate must be nameable from the message alone — the retry is
        // a copy-paste of the id column, so a refusal listing only some of them
        // sends the operator back to guessing.
        for (id, graph) in [(1u128, "go2"), (2, "arm")] {
            let hex = format!("0x{id:032x}");
            assert!(msg.contains(&hex), "the refusal must carry {hex}:\n{msg}");
            assert!(
                msg.contains(graph),
                "the refusal must carry {graph}:\n{msg}"
            );
        }
        assert!(
            msg.contains("--run"),
            "the refusal must name the flag that resolves it:\n{msg}"
        );
    }

    #[test]
    fn a_named_run_matches_either_id_spelling_or_the_graph_name() {
        let runs = [rec(0xAB, "go2_attach", 11), rec(0xCD, "arm", 12)];
        for want in [
            "0x000000000000000000000000000000ab",
            "000000000000000000000000000000AB",
            "go2_attach",
            // Trimmed: an id pasted out of a log arrives with whitespace.
            "  go2_attach  ",
        ] {
            match select_run(&runs, &RunTarget::Named(want.to_string())) {
                RunSelection::Attach(r) => assert_eq!(r.run_id, 0xAB, "for `{want}`"),
                other => panic!("`{want}` must match run 0xAB, got {other:?}"),
            }
        }
    }

    #[test]
    fn two_runs_of_one_graph_are_ambiguous_by_name_and_separable_by_id() {
        // The normal way to reach ambiguity, and the case where guessing is
        // most harmful: the same graph restarted, or two of it.
        let runs = [rec(1, "go2", 11), rec(2, "go2", 12)];
        let by_name = select_run(&runs, &RunTarget::Named("go2".to_string()));
        assert!(
            matches!(&by_name, RunSelection::Ambiguous(l) if l.len() == 2),
            "one NAME matching two runs must refuse, got {by_name:?}"
        );
        match select_run(&runs, &RunTarget::Named(format!("0x{:032x}", 2))) {
            RunSelection::Attach(r) => assert_eq!(r.supervisor_pid, 12),
            other => panic!("the id must still separate them, got {other:?}"),
        }
    }

    #[test]
    fn a_named_run_that_is_not_live_lists_what_is_and_explains_an_empty_machine() {
        let runs = [rec(1, "go2", 11)];
        let selection = select_run(&runs, &RunTarget::Named("nope".to_string()));
        let msg = run_selection_error(&selection).expect("a miss must refuse");
        assert!(msg.contains("nope"), "the ask is echoed back:\n{msg}");
        assert!(msg.contains("go2"), "what IS live is listed:\n{msg}");

        // The empty-machine arm says something DIFFERENT, because the likely
        // cause is different: an older build announces nothing at all, and
        // an operator staring at a running graph needs to be told that.
        let empty = select_run(&[], &RunTarget::Named("nope".to_string()));
        let msg = run_selection_error(&empty).expect("a miss must refuse");
        assert!(
            msg.contains("too old to publish a run record"),
            "an empty machine must explain the pre-981 case rather than listing \
             nothing:\n{msg}"
        );
    }

    #[test]
    fn an_ending_run_is_still_a_candidate() {
        // A run that has announced its exit is still worth recording the tail
        // of, and refusing it would make the verb's behaviour depend on a race
        // with the run's own `Drop`.
        let mut ending = rec(5, "go2", 11);
        ending.state = RunState::Ending;
        match select_run(&[ending], &RunTarget::Sole) {
            RunSelection::Attach(r) => assert_eq!(r.run_id, 5),
            other => panic!("an Ending run must remain attachable, got {other:?}"),
        }
    }

    #[test]
    fn the_run_identity_attachment_carries_the_attach_facts_and_the_runs_own_manifest() {
        let run = rec(0x1234, "go2", 4711);
        let run_json = br#"{"version":1,"run_id":"0x00000000000000000000000000001234",
            "partition":{"provenance":"derived-in-memory","process_groups":true}}"#;
        let bytes = render_attach_run_json(
            &run,
            Some(run_json),
            999,
            Some(41882),
            TRACE_FROM_ATTACH,
            STATE_RINGS_FROM_ATTACH,
            &[],
        );
        let doc: serde_json::Value = serde_json::from_slice(&bytes).expect("valid JSON");

        assert_eq!(doc["run_id"], "0x00000000000000000000000000001234");
        assert_eq!(doc["graph_name"], "go2");
        assert_eq!(doc["supervisor_pid"], 4711);
        assert_eq!(doc["attached_at_ns"], 999);
        assert_eq!(
            doc["attached_mid_run"], true,
            "the one fact a reader must never have to infer"
        );
        assert_eq!(doc["first_step_recorded"], 41882);
        assert_eq!(doc["trace"], TRACE_FROM_ATTACH);
        // The run's OWN manifest is preserved verbatim under its own key, not
        // merged: a reader must always be able to tell what the RUN said from
        // what the RECORDER said.
        assert_eq!(doc["run"]["partition"]["provenance"], "derived-in-memory");
        assert_eq!(doc["run"]["partition"]["process_groups"], true);
        assert!(
            doc.get("run_manifest_unparsed").is_none(),
            "a parseable manifest is carried structurally"
        );
    }

    #[test]
    fn an_unparseable_run_manifest_is_carried_as_text_rather_than_dropped() {
        let run = rec(9, "go2", 1);
        let bytes = render_attach_run_json(
            &run,
            Some(b"{ this is not json"),
            1,
            None,
            "none: x",
            STATE_RINGS_UNKNOWN_NO_MANIFEST,
            &[],
        );
        let doc: serde_json::Value = serde_json::from_slice(&bytes).expect("valid JSON");
        assert_eq!(
            doc["run_manifest_unparsed"], "{ this is not json",
            "dropping it would lose the run's own description; inventing a \
             parsed shape would be worse"
        );
        assert!(doc.get("run").is_none());
        // The recorder's own facts survive a manifest it could not read.
        assert_eq!(doc["run_id"], "0x00000000000000000000000000000009");
        assert_eq!(doc["attached_mid_run"], true);
        assert_eq!(doc["first_step_recorded"], serde_json::Value::Null);
    }

    #[test]
    fn a_missing_run_manifest_still_yields_the_recorders_own_identity() {
        let run = rec(3, "solo", 2);
        // The trace verdict is `UNKNOWN`, not `NONE_NO_RINGS`: this is the
        // MISSING-manifest case, so what the run declared was never read. The
        // production selector picks it the same way — see the arm below.
        let bytes = render_attach_run_json(
            &run,
            None,
            5,
            None,
            TRACE_UNKNOWN_NO_MANIFEST,
            STATE_RINGS_UNKNOWN_NO_MANIFEST,
            &[(
                "run.json".to_string(),
                "No such file or directory".to_string(),
            )],
        );
        let doc: serde_json::Value = serde_json::from_slice(&bytes).expect("valid JSON");
        assert_eq!(doc["run_id"], "0x00000000000000000000000000000003");
        assert_eq!(doc["trace"], TRACE_UNKNOWN_NO_MANIFEST);
        assert!(doc.get("run").is_none() && doc.get("run_manifest_unparsed").is_none());
        // What could not be read is IN THE BAG, not only in a log line that
        // scrolls away. Without it, an absent `graph.yaml` is ambiguous between
        // "the run had none" and "the recorder could not read it".
        assert_eq!(
            doc["artifacts_unreadable"]["run.json"],
            "No such file or directory"
        );
    }

    /// An UNREADABLE run manifest and a manifest that genuinely
    /// declared no rings are DIFFERENT states, and the bag must not conflate
    /// them.
    ///
    /// `RunArtifacts::rings` reads exclusively from the manifest, so both yield
    /// an empty ring vector — a selector keyed on that vector alone
    /// would stamp "the run's manifest declared no trace rings" onto a bag whose
    /// recorder never opened the manifest. That is a positive claim about the
    /// RUN drawn from the recorder's own failure to read it.
    ///
    /// Nothing else catches it. The reconciliation for a frozen `trace` verdict
    /// — `record_coverage.json`'s `rings_unavailable` and `bag info`'s `trace:
    /// NONE|PARTIAL` line — is gated on `trace_degraded()`, which is
    /// `!rings_unavailable.is_empty()`; with no manifest there are zero DECLARED
    /// rings, so nothing is ever unavailable and that line never fires.
    ///
    /// Driven through `read_run_artifacts` — the PRODUCTION reader — against a
    /// directory that does not exist, so the empty-rings state is REACHED rather
    /// than asserted, and the selector expression is the shipping one.
    #[test]
    fn an_unreadable_run_manifest_says_the_trace_is_unknown_not_that_none_was_declared() {
        let run = rec(0x99, "exiting", 31337);
        // A run directory that has already been removed — the ordinary shape of
        // attaching to a run that is shutting down, not a corner case.
        let gone = std::path::Path::new("/nonexistent_run_dir");
        let artifacts = read_run_artifacts(gone);
        assert!(
            artifacts.run_json.is_none() && artifacts.rings.is_empty(),
            "precondition: an unreadable manifest yields NO rings — which is what \
             makes it indistinguishable from a run that declared none"
        );

        // The PRODUCTION selector, verbatim — the shipping pure function, so
        // this test cannot drift away from what a bag really carries. (It used
        // to be a HAND COPY of the four-arm chain, which is one of the reasons
        // it was extracted.)
        let trace = choose_trace_verdict(TraceVerdictFacts {
            manifest_read: artifacts.run_json.is_some(),
            manifest_parsed: artifacts.manifest_parsed(),
            trace_rings: &artifacts.trace_rings,
            declared_unavailable: &artifacts.declared_unavailable,
            rings_declared: artifacts.rings.len(),
        })
        .render();
        let bytes = render_attach_run_json(
            &run,
            None,
            7,
            None,
            &trace,
            STATE_RINGS_UNKNOWN_NO_MANIFEST,
            &artifacts.unreadable,
        );
        let doc: serde_json::Value = serde_json::from_slice(&bytes).expect("valid JSON");

        assert_eq!(doc["trace"], TRACE_UNKNOWN_NO_MANIFEST);
        assert_ne!(
            doc["trace"], TRACE_NONE_NO_RINGS,
            "THE discrimination: stating the run 'declared no trace rings' when its \
             manifest was never read asserts a fact about the run on the recorder's \
             own failure to read it"
        );
        assert!(
            doc["artifacts_unreadable"]["run.json"]
                .as_str()
                .is_some_and(|s| !s.is_empty()),
            "the reason must be carried, not just the fact: got {}",
            doc["artifacts_unreadable"]
        );
        // Absence stays NO-CLAIM. A "fix" that invented a `run` object to fill
        // the hole would be worse than the ambiguity it closed.
        assert!(doc.get("run").is_none() && doc.get("run_manifest_unparsed").is_none());
        // The recorder's own facts — the ones that came off the REGISTRY RECORD
        // and were genuinely observed — survive a directory it could not read.
        assert_eq!(doc["run_id"], "0x00000000000000000000000000000099");
        assert_eq!(doc["graph_name"], "exiting");
        assert_eq!(doc["attached_mid_run"], true);
    }

    /// ANTI-TAUTOLOGY for the arm above: a run whose manifest WAS read and
    /// declared no rings keeps the `NONE_NO_RINGS` verdict and carries no
    /// `artifacts_unreadable` key at all.
    ///
    /// Without this, `an_unreadable_run_manifest_...` is satisfied by returning
    /// [`TRACE_UNKNOWN_NO_MANIFEST`] unconditionally, and the byte-identical-
    /// when-healthy property is unpinned.
    #[test]
    fn a_readable_manifest_declaring_no_rings_keeps_the_no_rings_verdict() {
        let run = rec(0x7, "plain", 42);
        let run_json = br#"{"version":1,"run_id":"0x00000000000000000000000000000007","rings":[]}"#;
        let artifacts = RunArtifacts {
            run_json: Some(run_json.to_vec()),
            ..Default::default()
        };
        // The PRODUCTION selector, verbatim — the shipping pure function, so
        // this test cannot drift away from what a bag really carries. (It used
        // to be a HAND COPY of the four-arm chain, which is one of the reasons
        // it was extracted.)
        let trace = choose_trace_verdict(TraceVerdictFacts {
            manifest_read: artifacts.run_json.is_some(),
            manifest_parsed: artifacts.manifest_parsed(),
            trace_rings: &artifacts.trace_rings,
            declared_unavailable: &artifacts.declared_unavailable,
            rings_declared: artifacts.rings.len(),
        })
        .render();
        let bytes = render_attach_run_json(
            &run,
            artifacts.run_json.as_deref(),
            9,
            None,
            &trace,
            STATE_RINGS_UNKNOWN_LEGACY,
            &artifacts.unreadable,
        );
        let doc: serde_json::Value = serde_json::from_slice(&bytes).expect("valid JSON");

        assert_eq!(
            doc["trace"], TRACE_NONE_NO_RINGS,
            "a manifest we READ that declares no rings is a fact about the run, and \
             saying so is the correct answer for the ordinary `graph run`"
        );
        assert!(
            doc.get("artifacts_unreadable").is_none(),
            "a healthy attach must stay byte-identical: the key's PRESENCE is the \
             signal, so an always-emitted empty object would defeat it"
        );
    }

    #[test]
    fn the_topic_set_is_what_the_runs_own_graph_declares() {
        // Prefix + node id + an absolute `topic:` override — the three shapes a
        // resolved output name takes. Oracle written by hand from the YAML, so
        // this cannot pass by echoing the resolver.
        let yaml = br#"
name: demo
prefix: demo
nodes:
  - id: cam
    type: camera
    outputs:
      - name: image
        schema: sensor_msgs/Image
  - id: det
    type: detector
    outputs:
      - name: boxes
        schema: vision_msgs/Detection3DArray
      - name: tf
        schema: tf2_msgs/TFMessage
        topic: /tf
"#;
        let got = run_declared_topics(yaml).expect("the graph parses");
        assert_eq!(
            got,
            vec![
                "/demo/cam/image".to_string(),
                "/demo/det/boxes".to_string(),
                "/tf".to_string(),
            ],
            "the declared set is the run's own outputs, sorted and deduped — \
             NOT everything live on the machine"
        );
    }

    #[test]
    fn a_graph_that_does_not_parse_is_a_loud_refusal_not_an_empty_set() {
        // An empty set would silently record NOTHING while looking like a
        // successful attach.
        let err = run_declared_topics(b"nodes: [ this is not yaml").unwrap_err();
        assert!(
            format!("{err}").contains("graph.yaml"),
            "the refusal must name what it could not read: {err}"
        );
        assert!(
            run_declared_topics(&[0xff, 0xfe]).is_err(),
            "invalid UTF-8 too"
        );
    }

    #[test]
    fn reading_a_run_directory_reports_what_it_could_not_read_rather_than_substituting() {
        let tmp = tempfile::tempdir().expect("tempdir");
        std::fs::write(
            tmp.path().join(crate::run_dir::RUN_GRAPH_FILE),
            b"name: x\n",
        )
        .expect("write");
        std::fs::write(tmp.path().join(crate::run_dir::RUN_ENV_FILE), b"{}").expect("write");

        let got = read_run_artifacts(tmp.path());
        assert_eq!(got.graph_yaml.as_deref(), Some(&b"name: x\n"[..]));
        assert_eq!(got.env_json.as_deref(), Some(&b"{}"[..]));
        assert!(got.recorder_json.is_none() && got.run_json.is_none());
        let missing: Vec<&str> = got.unreadable.iter().map(|(n, _)| n.as_str()).collect();
        assert_eq!(
            missing,
            vec![
                crate::run_dir::RUN_RECORDER_FILE,
                crate::run_dir::RUN_MANIFEST_FILE
            ],
            "an artifact that could not be read is NAMED, so a thin bag is \
             explained rather than merely thin"
        );

        // A directory that is not there at all (the run exited and removed it)
        // degrades the same way — every artifact absent, all four reported.
        let gone = read_run_artifacts(&tmp.path().join("vanished"));
        assert_eq!(gone.unreadable.len(), 4);
        assert!(gone.graph_yaml.is_none());
    }

    #[test]
    fn the_ring_tags_come_from_the_run_manifest_and_a_missing_key_means_no_rings() {
        // The object form, a bare-string form, and every way an entry can
        // be unusable — in one oracle, because the tolerance is the point: this
        // manifest is written by another process and possibly another version.
        let doc = br#"{"rings":[
            {"tag":"cer_rec_go2_11_r0","rank":0,"generation":7},
            "cer_rec_go2_11_r1",
            {"rank":2},
            {"tag":""},
            {"tag":"   "},
            17
        ]}"#;
        assert_eq!(
            run_manifest_ring_tags(doc),
            vec![
                "cer_rec_go2_11_r0".to_string(),
                "cer_rec_go2_11_r1".to_string()
            ],
            "usable tags are kept in order; an entry with no tag, an empty one \
             and a non-object are skipped rather than refused"
        );

        // A run that declares nothing, a manifest that will not parse, and a
        // `rings` that is not an array all mean the SAME thing to a recorder —
        // no ring to attach — and none of them is a reason to refuse a
        // recording.
        for absent in [
            &br#"{"version":1}"#[..],
            &br#"{ not json"#[..],
            &br#"{"rings":"nope"}"#[..],
            &br#"{"rings":[]}"#[..],
        ] {
            assert!(
                run_manifest_ring_tags(absent).is_empty(),
                "must yield no tags: {}",
                String::from_utf8_lossy(absent)
            );
        }
    }

    #[test]
    fn a_run_directory_whose_manifest_declares_rings_surfaces_them() {
        // The wiring `read_run_artifacts` owes the recorder: the tags reach
        // `RunArtifacts`, not just the parser.
        let tmp = tempfile::tempdir().expect("tempdir");
        std::fs::write(
            tmp.path().join(crate::run_dir::RUN_MANIFEST_FILE),
            br#"{"version":1,"rings":[{"tag":"cer_rec_x_1_r0","rank":0}]}"#,
        )
        .expect("write");
        assert_eq!(
            read_run_artifacts(tmp.path()).rings,
            vec!["cer_rec_x_1_r0".to_string()]
        );
        // And a run directory with no manifest at all declares none.
        let bare = tempfile::tempdir().expect("tempdir");
        assert!(read_run_artifacts(bare.path()).rings.is_empty());
    }

    #[test]
    fn a_run_target_replaces_the_topic_set_requirement_but_never_an_explicit_one() {
        // Without `--run`, a bare invocation still refuses (unchanged).
        assert!(validate_record_options(&RecordOptions::default()).is_err());
        // With it, the run supplies the set.
        assert!(validate_record_options(&RecordOptions {
            run: Some(RunTarget::Sole),
            ..RecordOptions::default()
        })
        .is_ok());
        // And `--run` does NOT license combining sources: the one-source rule is
        // about `--all` vs `--regex` vs names, which `--run` does not join.
        assert!(validate_record_options(&RecordOptions {
            run: Some(RunTarget::Sole),
            all: true,
            regex: Some("x".to_string()),
            ..RecordOptions::default()
        })
        .is_err());
    }
}

#[cfg(test)]
mod run_degradation {
    //! `--run` degrades per topic.
    //!
    //! The rule was documented on [`TopicSelection::Run`] from the start and
    //! implemented nowhere: the run-derived names were substituted into
    //! `opts.topics`, so they reached [`derive_record_topics`]' EXPLICIT branch
    //! and one quiet declared output refused the entire recording. These arms
    //! pin the rule where it now lives, on both sides of its one boundary.

    use super::*;

    fn opts() -> RecordOptions {
        RecordOptions {
            run: Some(RunTarget::Sole),
            ..RecordOptions::default()
        }
    }

    /// THE FIX: a declared topic with no producer costs THAT TOPIC, and the
    /// live ones still record.
    ///
    /// The `not_live` half is asserted as well as `selected`, because dropping
    /// a quiet topic SILENTLY would satisfy any assertion about the recorded
    /// set while leaving the bag claiming, by omission, that the run declared
    /// exactly what the bag contains.
    #[test]
    fn a_declared_topic_with_no_producer_costs_that_topic_and_nothing_else() {
        let live = vec!["/a".to_string(), "/c".to_string()];
        let declared = vec!["/a".to_string(), "/b".to_string(), "/c".to_string()];
        let got = derive_run_topics(&live, &declared, &opts(), "0xdead (g)")
            .expect("a partially-live run must RECORD, never refuse");
        assert_eq!(
            got.selected,
            vec!["/a".to_string(), "/c".to_string()],
            "every declared topic with a live producer must be recorded"
        );
        assert_eq!(
            got.not_live,
            vec!["/b".to_string()],
            "and the quiet one must be REPORTED, not silently dropped"
        );
    }

    /// The ANTI-TAUTOLOGY control: a fully-live declared set is unchanged and
    /// reports nothing. Without it, "the live ones record" is satisfied by an
    /// implementation that also invents a `not_live` entry for everything.
    #[test]
    fn a_fully_live_declared_set_records_everything_and_reports_nothing() {
        let live = vec!["/a".to_string(), "/b".to_string(), "/z".to_string()];
        let declared = vec!["/b".to_string(), "/a".to_string()];
        let got = derive_run_topics(&live, &declared, &opts(), "0xdead (g)").expect("all live");
        assert_eq!(got.selected, vec!["/a".to_string(), "/b".to_string()]);
        assert!(
            got.not_live.is_empty(),
            "nothing was missing, so nothing may be reported missing"
        );
    }

    /// THE BOUNDARY: zero live declared topics REFUSES, and names the run.
    ///
    /// Recording a bag with no channels while claiming to describe a run would
    /// be worse than saying so — at that point nothing about the run is being
    /// captured, and a silent empty bag is the failure mode this whole verb
    /// exists to make impossible.
    #[test]
    fn a_run_whose_every_declared_topic_is_quiet_refuses_and_names_what_is_missing() {
        let err = derive_run_topics(
            &["/other".to_string()],
            &["/a".to_string(), "/b".to_string()],
            &opts(),
            "0x00000000000000000000000000000abc (percept)",
        )
        .expect_err("nothing live means nothing to record");
        let msg = err.to_string();
        for needle in ["0x00000000000000000000000000000abc", "percept", "/a", "/b"] {
            assert!(
                msg.contains(needle),
                "the refusal must name the run and every missing topic; `{needle}` absent \
                 from:\n{msg}"
            );
        }
    }

    /// `--exclude` removes a topic from BOTH halves: an excluded topic is
    /// neither recorded nor reported as a gap, because the operator took it out
    /// on purpose. Reporting it would put a permanent `declared_not_live` row in
    /// the manifest for a deliberate omission.
    #[test]
    fn exclude_removes_a_declared_topic_from_the_report_as_well_as_the_recording() {
        let live = vec!["/keep".to_string(), "/drop".to_string()];
        let declared = vec![
            "/keep".to_string(),
            "/drop".to_string(),
            "/quiet".to_string(),
        ];
        let o = RecordOptions {
            exclude: vec!["drop".to_string(), "quiet".to_string()],
            ..opts()
        };
        let got = derive_run_topics(&live, &declared, &o, "0xdead (g)").expect("one topic left");
        assert_eq!(got.selected, vec!["/keep".to_string()]);
        assert!(
            got.not_live.is_empty(),
            "an EXCLUDED quiet topic is not a coverage gap — the operator removed it"
        );
        // And excluding EVERYTHING is the empty-selection refusal, not a bag
        // with no channels.
        let all_out = RecordOptions {
            exclude: vec![".".to_string()],
            ..opts()
        };
        assert!(derive_run_topics(&live, &declared, &all_out, "0xdead (g)").is_err());
    }

    /// Both halves come back SORTED and DEDUPLICATED, so a given
    /// `(live, declared)` pair always yields the same bag channel order. The
    /// declared set arrives reverse-sorted with a duplicate, so a passing result
    /// cannot be an accident of input order.
    #[test]
    fn both_halves_are_sorted_and_deduplicated() {
        let live = vec!["/b".to_string(), "/a".to_string()];
        let declared = vec![
            "/z".to_string(),
            "/b".to_string(),
            "/a".to_string(),
            "/b".to_string(),
            "/y".to_string(),
            "/z".to_string(),
        ];
        let got = derive_run_topics(&live, &declared, &opts(), "0xdead (g)").expect("two live");
        assert_eq!(got.selected, vec!["/a".to_string(), "/b".to_string()]);
        assert_eq!(got.not_live, vec!["/y".to_string(), "/z".to_string()]);
    }

    /// A `declared_not_live` verdict must be TERMINAL: `true` here means the
    /// ledger entry is never reconsidered.
    ///
    /// `rescan_discovery` PRUNES the untapped ledger down to terminal verdicts
    /// plus topics that are currently LIVE — and this verdict's entire subject
    /// is a topic that is NOT live, so a non-terminal spelling would be swept
    /// off the ledger by the first rescan and the manifest would silently lose
    /// the one fact it was seeded to carry. It is also NOT a coverage gap: the
    /// gap count answers "which LIVE producer is missing from this bag?", and
    /// counting a declared-but-never-fired output would escalate
    /// `is_incomplete()` on every ordinary `--run` recording.
    #[test]
    fn declared_not_live_is_terminal_so_the_rescan_prune_cannot_erase_it() {
        use cerulion_bagd::UntappedReason as R;
        assert!(
            R::DeclaredNotLive.is_terminal(),
            "a non-terminal verdict for a NOT-LIVE topic is pruned on the first rescan"
        );
        assert!(
            !R::DeclaredNotLive.is_coverage_gap(),
            "a topic nothing was producing is not a LIVE producer missing from the bag"
        );
        assert_eq!(R::DeclaredNotLive.tag(), "declared_not_live");
        // The CONTROL: a reason that IS a live-producer gap still counts, so the
        // assertion above is about this variant rather than about the method.
        assert!(R::AppearedAfterBagCreation.is_coverage_gap());
    }
}

#[cfg(test)]
mod fix_wave_2 {
    //! The two cross-fix defects the first fix composed into
    //! existence, plus the two labelling corrections that fall out of them.
    //!
    //! Both blockers are the hardest class to catch: a fix can be correct in
    //! isolation while every test exercises only its own half alone, so the
    //! suite stays green while the PAIR is broken.

    use super::tests::coverage_of;
    use super::*;

    /// **BLOCKING 1, the SEMANTICS half.** One `--exclude` flag filters TWO
    /// halves of the selection in TWO crates, and they must agree topic for
    /// topic.
    ///
    /// `derive_run_topics` filters the DECLARED set here (`compile_patterns` →
    /// `Regex::is_match`); `discovery::plan_discovery` filters the DISCOVERED
    /// set inside bagd (`ExcludePatterns::matched`). If the two ever disagreed,
    /// one flag would take a topic out of one half and leave it in the other —
    /// which is the exact shape of the blocker this fix closes, re-created by
    /// drift rather than by omission.
    ///
    /// Driven over an adversarial vector rather than a happy pattern: anchors,
    /// alternation, an unanchored substring, a character class, and a regex
    /// METACHARACTER that a naive "treat the pattern as a prefix"
    /// implementation would get wrong.
    #[test]
    fn both_halves_of_one_exclude_flag_agree_about_what_a_pattern_means() {
        let patterns: Vec<String> = ["^/camera", "lidar|radar", "_raw$", "/imu[0-9]", "a.c"]
            .iter()
            .map(|s| s.to_string())
            .collect();
        let topics: Vec<String> = [
            "/camera/front",
            "/front/camera",
            "/go2/lidar/points",
            "/radar",
            "/depth_raw",
            "/depth_raw/meta",
            "/imu0",
            "/imux",
            "/abc",
            "/odom",
        ]
        .iter()
        .map(|s| s.to_string())
        .collect();

        let declared_half = compile_patterns(&patterns, "--exclude").expect("valid patterns");
        let discovered_half = cerulion_bagd::discovery::ExcludePatterns::compile(&patterns)
            .expect("bagd must accept exactly what this verb accepts");

        // The DECLARED half's verdict is "does any pattern match?" (its own
        // `kept` closure, inverted); the DISCOVERED half's is "WHICH pattern
        // matched?". They must answer the same QUESTION on every topic.
        for topic in &topics {
            let excluded_here = declared_half.iter().any(|re| re.is_match(topic));
            let excluded_there = discovered_half.matched(topic).is_some();
            assert_eq!(
                excluded_here, excluded_there,
                "the two halves of one --exclude disagree about `{topic}`: \
                 derive_run_topics says excluded={excluded_here}, plan_discovery \
                 says excluded={excluded_there}"
            );
        }

        // ANTI-TAUTOLOGY: the vector must exercise BOTH verdicts, or "they
        // agree" is satisfied by a pair that excludes nothing.
        let excluded = topics
            .iter()
            .filter(|t| discovered_half.matched(t).is_some())
            .count();
        assert!(
            excluded > 0 && excluded < topics.len(),
            "the fixture must exercise BOTH verdicts; got {excluded} of {} excluded",
            topics.len()
        );
    }

    /// **NB1:** the two not-a-gap classes render on their OWN lines.
    ///
    /// `declared_not_live` under a heading that says a RULE excluded the topic
    /// is a lie: nothing excluded it — it was asked for and had no producer.
    /// The oracle reads each LINE rather than the whole block, because the
    /// defect is precisely that the two were rendered together.
    #[test]
    fn a_never_live_topic_renders_apart_from_the_topics_a_rule_excluded() {
        use cerulion_bagd::UntappedReason as R;
        let text = render_coverage(&coverage_of(
            true,
            &[("/a", cerulion_bagd::TapSource::Declared, 9, false)],
            &[
                ("/bagd/status", R::ExcludedInternal),
                (
                    "/camera/front",
                    R::ExcludedByRequest {
                        pattern: "^/camera".to_string(),
                    },
                ),
                ("/quiet", R::DeclaredNotLive),
            ],
        ));

        let by_rule = text
            .lines()
            .find(|l| l.starts_with("also untapped, by rule"))
            .unwrap_or_else(|| panic!("no by-rule line in:\n{text}"));
        let never_live = text
            .lines()
            .find(|l| l.starts_with("also untapped, declared by the run but never live"))
            .unwrap_or_else(|| panic!("no never-live line in:\n{text}"));

        assert!(
            by_rule.contains("/bagd/status") && by_rule.contains("/camera/front"),
            "both rule exclusions belong on the by-rule line: {by_rule}"
        );
        assert!(
            !by_rule.contains("/quiet"),
            "a topic nothing excluded must NOT be filed under `by rule`: {by_rule}"
        );
        assert!(
            never_live.contains("/quiet") && !never_live.contains("/bagd/status"),
            "the never-live line carries exactly the never-live topics: {never_live}"
        );
        // …and the operator's exclusion NAMES the pattern that caught it, which
        // is the only fact they can act on when several patterns are in play.
        assert!(
            by_rule.contains("excluded_by_request") && by_rule.contains("^/camera"),
            "an operator exclusion must name its pattern: {by_rule}"
        );
        // Neither class is a coverage gap, so the verdict stays COMPLETE.
        assert!(
            text.contains("coverage: COMPLETE"),
            "not-a-gap rows must not change the verdict:\n{text}"
        );
    }

    /// **BLOCKING 2:** `bag info` is the AUTHORITATIVE trace surface, because
    /// `run.json`'s verdict is frozen before the rings are ever opened.
    ///
    /// Three shapes in one body: every ring lost (NONE), some lost (PARTIAL),
    /// and the healthy CONTROL that must say nothing at all — without which
    /// "the trace line appears" is satisfied by a renderer that always prints
    /// it.
    #[test]
    fn a_bag_whose_rings_vanished_renders_the_outcome_and_withholds_complete() {
        let healthy = coverage_of(
            true,
            &[("/a", cerulion_bagd::TapSource::Declared, 9, false)],
            &[],
        );

        // CONTROL: nothing declared, nothing lost, nothing said — and COMPLETE.
        let text = render_coverage(&healthy);
        assert!(
            !text.contains("trace:"),
            "a recording with no ring trouble must render no trace verdict:\n{text}"
        );
        assert!(text.contains("coverage: COMPLETE"), "{text}");

        // NONE: the only declared ring could not be opened.
        let mut none = healthy.clone();
        none.rings_declared = 1;
        none.rings_unavailable.insert(
            "/cer_rg_deadbeef".to_string(),
            "shm_open: No such file or directory".to_string(),
        );
        let text = render_coverage(&none);
        let verdict = text
            .lines()
            .find(|l| l.starts_with("trace: "))
            .unwrap_or_else(|| panic!("no trace verdict in:\n{text}"));
        assert!(
            verdict.starts_with("trace: NONE"),
            "zero rings opened is NONE, not PARTIAL: {verdict}"
        );
        assert!(
            verdict.contains("run.json"),
            "the line must reconcile itself with run.json's FROZEN verdict, \
             which still claims the trace was attached: {verdict}"
        );
        assert!(
            text.contains("/cer_rg_deadbeef") && text.contains("shm_open"),
            "each unopened ring must be named with the transport's own error:\n{text}"
        );
        // COMPLETE is WITHHELD: an operator stops reading at that word, and the
        // trace line sits below it.
        assert!(
            !text.contains("coverage: COMPLETE"),
            "COMPLETE must be withheld while the trace verdict is unhappy:\n{text}"
        );

        // PARTIAL: one of two rings opened — a different fact, different action.
        let mut partial = none.clone();
        partial.rings_declared = 2;
        let text = render_coverage(&partial);
        let verdict = text
            .lines()
            .find(|l| l.starts_with("trace: "))
            .unwrap_or_else(|| panic!("no trace verdict in:\n{text}"));
        assert!(
            verdict.starts_with("trace: PARTIAL"),
            "one of two rings opened is PARTIAL, not NONE: {verdict}"
        );
    }

    /// Withholding COMPLETE must not withhold the
    /// coverage verdict ITSELF — an absent `coverage:` line already means
    /// something else.
    ///
    /// Before this, the chain's last arm was `enumerated && !schemas_unresolved
    /// && !trace_degraded`, with NO `else`. So a run with a clean producer
    /// picture and an unhappy trace-or-schema verdict rendered no coverage line
    /// at all — colliding with the meaning ~30 lines up, where an absent line
    /// says "this caller opted out of discovery and can claim nothing". One
    /// blank standing for two different facts is the class this block's own
    /// contradicting-verdict rule exists to prevent, and the arm's comment
    /// claimed the two surfaces "still agree line for line" while one of them
    /// printed nothing.
    ///
    /// Four shapes in ONE body, because the property is about which of them
    /// produce which verdict — sampling one proves nothing about the others.
    #[test]
    fn a_clean_producer_picture_still_states_its_verdict_when_complete_is_withheld() {
        let healthy = coverage_of(
            true,
            &[("/a", cerulion_bagd::TapSource::Declared, 9, false)],
            &[],
        );

        // (1) CONTROL — nothing qualifies it, so the word is COMPLETE and the
        // qualified line must NOT appear. Without this arm, "NO GAPS is
        // rendered" is satisfied by a renderer that prints it unconditionally.
        let clean = render_coverage(&healthy);
        assert!(clean.contains("coverage: COMPLETE"), "{clean}");
        assert!(
            !clean.contains("coverage: NO GAPS"),
            "a clean run states COMPLETE and nothing weaker:\n{clean}"
        );

        // (2) TRACE degraded — COMPLETE withheld, verdict still STATED.
        let mut trace = healthy.clone();
        trace.rings_declared = 1;
        trace
            .rings_unavailable
            .insert("/cer_rg_dead".to_string(), "ENOENT".to_string());
        let text = render_coverage(&trace);
        assert!(
            !text.contains("coverage: COMPLETE"),
            "COMPLETE is still withheld:\n{text}"
        );
        assert!(
            text.contains("coverage: NO GAPS"),
            "…but the producer verdict must still be STATED, or its absence \
             collides with the opted-out meaning:\n{text}"
        );

        // (3) SCHEMAS unresolved — the same shape, so the two are consistent
        // rather than one carrying a verdict and the other a blank.
        let mut schemas = healthy.clone();
        schemas.schema_demand_requested = true;
        schemas.replay_grade = Some(cerulion_bagd::ReplayGrade::Observability);
        let text = render_coverage(&schemas);
        assert!(!text.contains("coverage: COMPLETE"), "{text}");
        assert!(
            text.contains("coverage: NO GAPS"),
            "the schema arm must qualify the verdict the same way the trace arm \
             does:\n{text}"
        );

        // (4) The one case an absent line is ALLOWED to mean: discovery never
        // ran, so this bag is entitled to no producer claim in EITHER direction.
        // `enumerated`-guarded on the precedent that governs every other arm.
        let opted_out = cerulion_bagd::RecordCoverage {
            enumerated: false,
            discovery_requested: false,
            rings_declared: 1,
            rings_unavailable: [("/cer_rg_dead".to_string(), "ENOENT".to_string())]
                .into_iter()
                .collect(),
            ..coverage_of(false, &[], &[])
        };
        let text = render_coverage(&opted_out);
        assert!(
            !text.contains("coverage: NO GAPS") && !text.contains("coverage: COMPLETE"),
            "a run that never enumerated must claim NOTHING about producers, \
             however unhappy its other artifacts are:\n{text}"
        );
        assert!(
            text.contains("trace:"),
            "…while the trace line, which needs no enumeration, still prints:\n{text}"
        );
    }

    /// Every `UntappedReason` lands in exactly one of
    /// the three reporting buckets, and the two shipped predicates agree with
    /// the classification they are derived from.
    ///
    /// The exhaustiveness itself is compile-enforced (`class()` has no
    /// wildcard). What a test can add is TOTALITY over the shipped variants plus
    /// the derivation: a `class()` that disagreed with `is_coverage_gap` /
    /// `is_excluded_by_rule` would keep both call sites compiling while making
    /// the recorder's counts and `bag info`'s lines describe different sets.
    #[test]
    fn every_untapped_reason_has_exactly_one_class_and_the_predicates_agree() {
        use cerulion_bagd::UntappedClass as C;
        use cerulion_bagd::UntappedReason as R;

        // Hand-written, one entry per shipped variant. A NEW variant is a
        // compile error in `class()`; this is the oracle for the seven that
        // exist, and is deliberately spelled out rather than derived.
        let cases: &[(R, C)] = &[
            (R::ExcludedInternal, C::ExcludedByRule),
            (
                R::RemoteMirror {
                    robot: "r".to_string(),
                },
                C::ExcludedByRule,
            ),
            (
                R::ExcludedByRequest {
                    pattern: "^/x".to_string(),
                },
                C::ExcludedByRule,
            ),
            (R::DeclaredNotLive, C::DeclaredNotLive),
            (R::BudgetExhausted { budget: 256 }, C::Gap),
            (R::AppearedAfterBagCreation, C::Gap),
            (
                R::AttachFailed {
                    error: "e".to_string(),
                },
                C::Gap,
            ),
        ];

        for (reason, want) in cases {
            assert_eq!(reason.class(), *want, "{} classified wrong", reason.tag());
            // The derivation, both directions: exactly one predicate holds for
            // the two named classes, and NEITHER holds for the third.
            assert_eq!(
                reason.is_coverage_gap(),
                *want == C::Gap,
                "is_coverage_gap disagrees with class() for {}",
                reason.tag()
            );
            assert_eq!(
                reason.is_excluded_by_rule(),
                *want == C::ExcludedByRule,
                "is_excluded_by_rule disagrees with class() for {}",
                reason.tag()
            );
            assert!(
                !(reason.is_coverage_gap() && reason.is_excluded_by_rule()),
                "a reason cannot be both a gap and a rule exclusion: {}",
                reason.tag()
            );
        }

        // …and the fixture really covers all three buckets, so the loop above is
        // not vacuously asserting one of them seven times.
        for want in [C::Gap, C::ExcludedByRule, C::DeclaredNotLive] {
            assert!(
                cases.iter().any(|(_, c)| *c == want),
                "the oracle must exercise every bucket; {want:?} is missing"
            );
        }
    }

    /// The manifest fields are ADDITIVE in BOTH directions, so no
    /// `RECORD_COVERAGE_VERSION` bump is needed (the `prefix_lost` precedent).
    ///
    /// A manifest that declares NO rings must serialize byte-identically to
    /// the older format, and an older manifest must decode with the new fields at
    /// their defaults — which is what makes an absent trace verdict mean
    /// "nothing to report" rather than "an old bag that cannot be read".
    ///
    /// The NONZERO shape is exercised too. The
    /// byte-identity property holds for recordings that declare no rings, NOT
    /// for every healthy one — `graph run --record` declares its own rings, so
    /// `rings_declared` is serialized into every one of those manifests. A test
    /// that drives only the `0` shape would let a doc claiming
    /// `0` "on every non-attach recording" stand, so the doc and the coverage
    /// would agree on something false.
    #[test]
    fn the_new_manifest_fields_are_additive_in_both_directions() {
        let healthy = coverage_of(
            true,
            &[("/a", cerulion_bagd::TapSource::Declared, 9, false)],
            &[],
        );
        let json = serde_json::to_string(&healthy).expect("serialize");
        assert!(
            !json.contains("rings_declared") && !json.contains("rings_unavailable"),
            "a healthy manifest must be byte-identical to one written before these keys existed: {json}"
        );

        // …and an OLD manifest (no such keys) decodes to the correct defaults.
        let old: cerulion_bagd::RecordCoverage =
            serde_json::from_str(&json).expect("an old manifest must still decode");
        assert_eq!(old.rings_declared, 0);
        assert!(old.rings_unavailable.is_empty());
        assert!(!old.trace_degraded(), "no claim is not a degraded claim");
        assert_eq!(old.rings_opened(), 0);

        // The nonzero shape — a healthy `graph run --record`
        // manifest, which declares rings and loses none. `rings_declared` is
        // serialized (so byte-identity does NOT hold here, which is the fact this
        // arm pins), `rings_unavailable` is still omitted,
        // and the round trip must preserve BOTH.
        let mut with_rings = healthy.clone();
        with_rings.rings_declared = 3;
        let json = serde_json::to_string(&with_rings).expect("serialize");
        assert!(
            json.contains("\"rings_declared\":3"),
            "a recording that declared rings must SAY how many: {json}"
        );
        assert!(
            !json.contains("rings_unavailable"),
            "…while an empty failure map stays omitted, so a healthy attach adds \
             one key and not two: {json}"
        );
        let back: cerulion_bagd::RecordCoverage = serde_json::from_str(&json).expect("round trip");
        assert_eq!(back.rings_declared, 3);
        assert!(back.rings_unavailable.is_empty());
        assert!(
            !back.trace_degraded(),
            "three rings declared and none lost is not a degraded trace"
        );
        assert_eq!(
            back.rings_opened(),
            3,
            "the fraction a reader needs: all three opened"
        );

        // An older READER of that same manifest ignores the unknown key and
        // behaves exactly as it did before — the other direction of `additive`,
        // which the `0` shape cannot exercise because it emits no key to ignore.
        let mut doc: serde_json::Value = serde_json::from_str(&json).expect("value");
        doc.as_object_mut()
            .expect("object")
            .remove("rings_declared");
        let as_old_reader_sees_it: cerulion_bagd::RecordCoverage =
            serde_json::from_value(doc).expect("an old reader drops the key it does not know");
        assert_eq!(as_old_reader_sees_it.rings_declared, 0);
        assert!(!as_old_reader_sees_it.trace_degraded());
    }

    /// `rings_opened()` SATURATES rather than panicking a reader.
    ///
    /// `bag info` must render every bag it can decode, including one hand-edited
    /// or written by a future recorder whose two counts disagree.
    #[test]
    fn a_self_contradictory_ring_count_renders_rather_than_panicking() {
        let mut c = coverage_of(true, &[], &[]);
        c.rings_declared = 1;
        c.rings_unavailable
            .insert("/a".to_string(), "e".to_string());
        c.rings_unavailable
            .insert("/b".to_string(), "e".to_string());
        assert_eq!(
            c.rings_opened(),
            0,
            "saturating, never a wrapped huge count"
        );
        let text = render_coverage(&c);
        assert!(text.contains("trace: NONE"), "{text}");
    }
}

/// The STATE section of `bag info`.
///
/// PURE renderer arms — the behavioural half (a real recorder, a real state
/// ring, a real bag) is `cerulion_bagd/tests/state_ring_e2e_test.rs`. What is
/// here is everything a wrong answer would express as wrong TEXT: which arm a
/// reading takes, and what each arm may and may not claim.
#[cfg(test)]
mod state_section {
    use super::*;
    use std::collections::BTreeMap;

    fn node(
        complete: u64,
        torn: u64,
        skipped: u64,
        last: Option<u64>,
    ) -> cerulion_bagd::StateNodeCoverage {
        cerulion_bagd::StateNodeCoverage {
            ring: "/cer_st_r0".into(),
            node_idx: None,
            anchors_complete: complete,
            anchors_torn: torn,
            anchors_skipped: skipped,
            anchors_skipped_after_complete: 0,
            last_complete_step: last,
            bytes: 0,
            skip_causes: BTreeMap::new(),
        }
    }

    fn state_of(
        nodes: &[(&str, cerulion_bagd::StateNodeCoverage)],
    ) -> cerulion_bagd::StateCoverage {
        cerulion_bagd::StateCoverage {
            version: cerulion_bagd::STATE_COVERAGE_VERSION,
            attached_mid_run: false,
            armed: None,
            rings_declared: 1,
            ranks_discovered: Vec::new(),
            ranks_missing: Vec::new(),
            rings_unavailable: BTreeMap::new(),
            records: 4,
            head_records_discarded: 0,
            malformed_records: 0,
            foreign_run_records: 0,
            nodes: nodes
                .iter()
                .map(|(id, n)| ((*id).to_string(), n.clone()))
                .collect(),
            unattributed_indices: BTreeMap::new(),
        }
    }

    /// A healthy manifest names every node and its resume point, and does NOT
    /// print the incomplete verdict.
    #[test]
    fn a_clean_manifest_names_every_node_and_its_resume_point() {
        let s = state_of(&[("alpha", node(2, 0, 0, Some(20)))]);
        let out = render_state_coverage_section(&StateCoverageReading::Present(s));
        assert!(
            out.contains("node state: 4 record(s) across 1 node(s)"),
            "{out}"
        );
        assert!(out.contains("alpha: 2 complete"), "{out}");
        assert!(out.contains("last complete anchor at step 20"), "{out}");
        assert!(
            out.contains("no armed capture plane was seen"),
            "a bag that saw no plane must say it claims no cadence — and must not say \
             the RECORDER armed nothing, which was never the recorder's to claim \
             (by design): {out}"
        );
        assert!(!out.contains("INCOMPLETE"), "{out}");
    }

    /// A node with NO anchor is named as such, and the incomplete verdict fires
    /// only once an armed plane was seen.
    ///
    /// Graph ownership of the plane re-pointed WHOSE arming decides it: the graph's, read
    /// off its word — but not the rule: anchors nobody was ever due to take are
    /// not anchors anybody missed.
    #[test]
    fn a_starved_node_is_named_and_the_verdict_fires_only_when_a_plane_was_armed() {
        let mut s = state_of(&[
            ("alpha", node(1, 0, 0, Some(5))),
            ("beta", node(0, 0, 0, None)),
        ]);
        let unarmed = render_state_coverage_section(&StateCoverageReading::Present(s.clone()));
        assert!(unarmed.contains("beta: 0 complete"), "{unarmed}");
        assert!(unarmed.contains("NO complete anchor"), "{unarmed}");
        assert!(
            !unarmed.contains("CHECKPOINT COVERAGE INCOMPLETE"),
            "a bag that saw no armed plane knows of no anchor anybody was due to take: \
             {unarmed}"
        );

        s.armed = Some(cerulion_bagd::StateArmCoverage {
            tag: "run-1".into(),
            cadence_steps: 30_000,
            first_anchor_step: 1,
        });
        let armed = render_state_coverage_section(&StateCoverageReading::Present(s));
        assert!(
            armed.contains("the capture plane was ARMED: every 30000 step(s) from step 1"),
            "{armed}"
        );
        assert!(armed.contains("CHECKPOINT COVERAGE INCOMPLETE"), "{armed}");
    }

    /// TORN and SKIPPED render differently, because they mean different things:
    /// a tear is damage, a skip is the mechanism naming a cause.
    #[test]
    fn a_torn_anchor_and_a_skipped_one_read_differently() {
        let torn = node(1, 1, 0, Some(5));
        let mut skipped = node(1, 0, 2, Some(5));
        skipped.skip_causes = [("contended".to_string(), 2u64)].into_iter().collect();
        let s = state_of(&[("t", torn), ("s", skipped)]);
        let out = render_state_coverage_section(&StateCoverageReading::Present(s));
        assert!(out.contains("t: 1 complete, 1 TORN"), "{out}");
        assert!(
            out.contains("s: 1 complete, 2 skipped (contended x2)"),
            "{out}"
        );
        assert!(
            out.contains("CHECKPOINT COVERAGE INCOMPLETE"),
            "a tear escalates even un-armed: {out}"
        );
    }

    /// A mid-run attach explains the leading records it discarded — they are in
    /// the bag, and they are not loss.
    #[test]
    fn a_mid_run_attach_explains_its_discarded_head_records() {
        let mut s = state_of(&[("alpha", node(1, 0, 0, Some(9)))]);
        s.attached_mid_run = true;
        s.head_records_discarded = 3;
        let out = render_state_coverage_section(&StateCoverageReading::Present(s));
        assert!(out.contains("attached MID-RUN"), "{out}");
        assert!(out.contains("3 leading record(s)"), "{out}");
        assert!(out.contains("they are not loss"), "{out}");
    }

    /// An UNAVAILABLE ring is named with its reason and escalates — the bag was
    /// asked for those anchors and could not even look.
    #[test]
    fn an_unavailable_ring_is_named_with_its_reason_and_escalates() {
        let mut s = state_of(&[("alpha", node(1, 0, 0, Some(2)))]);
        s.rings_unavailable
            .insert("/cer_st_gone".into(), "No such file or directory".into());
        let out = render_state_coverage_section(&StateCoverageReading::Present(s));
        assert!(out.contains("ring /cer_st_gone UNAVAILABLE"), "{out}");
        assert!(out.contains("No such file or directory"), "{out}");
        assert!(out.contains("CHECKPOINT COVERAGE INCOMPLETE"), "{out}");
    }

    /// EVERY manifest-derived string reaches the terminal SANITIZED.
    ///
    /// A bag is written by another process — on a `ros2 attach` robot, by another
    /// version, possibly across a network — and this block prints four kinds of
    /// string straight out of it: a ring's NAME and its failure REASON off the
    /// manifest, a NODE ID off a ring's node table, and a SKIP CAUSE name off a
    /// wire discriminant. Rendering any of them verbatim lets a crafted (or
    /// merely corrupt) bag repaint the screen of whoever runs `bag info` on it —
    /// the hazard the sibling coverage section's topic names and ring reasons
    /// already go through `sanitize_display` for.
    ///
    /// The fixture puts a CSI screen-clear, a CR overwrite and a BEL in each
    /// position at once, and the oracle is that NO control byte survives anywhere
    /// in the output — a per-field allow-list would pass a renderer that
    /// sanitized three of the four.
    #[test]
    fn every_string_the_bag_supplies_is_sanitized_before_it_reaches_a_terminal() {
        const CSI: &str = "\u{1b}[2J";
        let mut hostile = node(1, 0, 2, Some(3));
        hostile.skip_causes = [(format!("contended{CSI}"), 2u64)].into_iter().collect();
        let mut s = state_of(&[(&format!("alpha{CSI}\rBOGUS"), hostile)]);
        s.rings_unavailable.insert(
            format!("/cer_st_r0{CSI}"),
            format!("No such file\u{7}{CSI}"),
        );

        let out = render_state_coverage_section(&StateCoverageReading::Present(s));
        assert!(
            !out.chars().any(|c| c.is_control() && c != '\n'),
            "a control byte from the bag reached the terminal: {out:?}"
        );
        // ANTI-TAUTOLOGY: the sanitizer NEUTERS, it does not delete — the row is
        // still rendered, and the operator still sees which ring and which node.
        assert!(out.contains("alpha\u{fffd}"), "{out}");
        assert!(out.contains("/cer_st_r0\u{fffd}"), "{out}");
        assert!(out.contains("No such file\u{fffd}"), "{out}");
        assert!(out.contains("contended\u{fffd}[2J x2"), "{out}");

        // The MALFORMED arm carries the decoder's message, which interpolates
        // what it was reading — bag bytes — so it goes through the same filter.
        let bad = render_state_coverage_section(&StateCoverageReading::Malformed(format!(
            "expected value{CSI}"
        )));
        assert!(!bad.chars().any(|c| c.is_control() && c != '\n'), "{bad:?}");
        assert!(bad.contains("expected value\u{fffd}"), "{bad}");
    }

    // -----------------------------------------------------------------------
    // The ABSORBANCE block
    // -----------------------------------------------------------------------

    /// A verdict row, hand-built. CEILING-deep unless a budget is given.
    fn verdict(depth: u64, v: cerulion_bagd::AbsorbanceVerdict) -> cerulion_bagd::TopicAbsorbance {
        cerulion_bagd::TopicAbsorbance {
            tap_buffer_depth: depth,
            rate_mhz: Some(100_000),
            rate_is_floor: false,
            absorbance_us: Some(depth * 10_000),
            measured_tail_us: Some(25_000),
            required_depth: Some(3),
            shortfall_at_least_us: (depth < 3).then(|| 25_000 - depth * 10_000),
            verdict: v,
            short_evaluations: 0,
            budget_bytes: None,
            over_budget: None,
            budget_unpriced: false,
        }
    }

    fn present(
        rows: Vec<(String, cerulion_bagd::TopicAbsorbance)>,
        basis: Option<cerulion_bagd::LossCountingBasis>,
    ) -> AbsorbanceReading {
        AbsorbanceReading::Present {
            rows,
            basis,
            scope: AbsorbanceScope::Recording,
            // The SHIPPED ladder: these fixtures build rows the producer really
            // emits, so they must be judged against the vocabulary the producer
            // ranks on. A hand-picked ladder here would make the rows' own
            // soundness a property of the fixture.
            ladder_edges_us: Some(cerulion_bagd::DRAIN_GAP_BUCKET_EDGES_US.to_vec()),
        }
    }

    /// The ABSORBANCE block states the TOTALS over every
    /// topic that carries a verdict, and prints a row only for the ones an
    /// operator can act on.
    ///
    /// The two halves are in one body because the SILENCE about a healthy topic
    /// is the claim under test: a machine-wide recorder taps up to 256 topics,
    /// so a row per topic would be 256 lines of "fine" and nobody would read the
    /// two that are not. The header is what keeps that from hiding anything —
    /// its counts cover the whole set.
    #[test]
    fn the_absorbance_block_counts_every_topic_and_names_only_the_actionable_ones() {
        use cerulion_bagd::AbsorbanceVerdict;
        let out = render_absorbance_section(&present(
            vec![
                (
                    "/healthy".to_string(),
                    verdict(4, AbsorbanceVerdict::Absorbs),
                ),
                (
                    "/also_fine".to_string(),
                    verdict(9, AbsorbanceVerdict::Absorbs),
                ),
                ("/shallow".to_string(), verdict(2, AbsorbanceVerdict::Short)),
                (
                    "/stalled".to_string(),
                    cerulion_bagd::TopicAbsorbance {
                        measured_tail_us: None,
                        required_depth: None,
                        shortfall_at_least_us: None,
                        ..verdict(4096, AbsorbanceVerdict::Unrankable)
                    },
                ),
                (
                    "/young".to_string(),
                    cerulion_bagd::TopicAbsorbance {
                        measured_tail_us: None,
                        required_depth: None,
                        shortfall_at_least_us: None,
                        ..verdict(4, AbsorbanceVerdict::NoClaim)
                    },
                ),
            ],
            Some(cerulion_bagd::LossCountingBasis::PrefixProven),
        ));
        // HAND ORACLE: 5 verdicts — 2 absorb, 1 short, 1 unrankable, 1 no-claim.
        assert!(out.contains("5 topic(s) carry a verdict"), "{out}");
        assert!(out.contains("2 absorb"), "{out}");
        assert!(out.contains("1 fall SHORT"), "{out}");
        assert!(
            out.contains("1 met a stall the histogram cannot measure"),
            "{out}"
        );
        assert!(out.contains("1 make no claim"), "{out}");
        // The actionable rows print…
        assert!(out.contains("/shallow"), "{out}");
        assert!(out.contains("/stalled"), "{out}");
        assert!(out.contains("/young"), "{out}");
        // …and the healthy ones are COUNTED, not printed.
        assert!(
            !out.contains("/healthy"),
            "a topic that absorbed is counted, not printed: {out}"
        );
        assert!(!out.contains("/also_fine"), "{out}");
        // The overflow bucket is RENDERED, never skipped.
        assert!(out.contains("WORSE THAN THE LADDER CAN DESCRIBE"), "{out}");
        // Both caveats ride the header.
        assert!(out.contains("WINDOW AVERAGE"), "{out}");
        assert!(out.contains("DRIVE-LOOP gaps"), "{out}");
        // …and this recording DID prove its prefix, so the counting caveat is
        // the one thing NOT printed.
        assert!(
            !out.contains("nothing was COUNTED"),
            "an armed-before-producers recording accounts its head loss separately: {out}"
        );
    }

    /// An `Absorbs` on a FLOOR rate is an OPTIMISTIC pass, and the block says so
    /// twice — in the header's count and by printing the row.
    ///
    /// A floor rate is a lower bound, so the absorbance derived from it is a
    /// ceiling; folding such a row into the plain "N absorb" total lets a
    /// `multi_publisher` topic (`/tf` on any real robot, whose per-publisher
    /// counters cannot be differenced) be counted beside an exactly-measured one.
    #[test]
    fn an_optimistic_floor_rate_pass_is_counted_apart_and_printed() {
        use cerulion_bagd::AbsorbanceVerdict;
        let floor_row = cerulion_bagd::TopicAbsorbance {
            rate_is_floor: true,
            ..verdict(9, AbsorbanceVerdict::Absorbs)
        };
        let out = render_absorbance_section(&present(
            vec![
                ("/tf".to_string(), floor_row),
                ("/exact".to_string(), verdict(9, AbsorbanceVerdict::Absorbs)),
            ],
            Some(cerulion_bagd::LossCountingBasis::PrefixProven),
        ));
        assert!(out.contains("2 absorb"), "{out}");
        assert!(
            out.contains("1 of them on an OPTIMISTIC floor rate"),
            "{out}"
        );
        assert!(out.contains("/tf"), "an optimistic pass is printed: {out}");
        // ANTI-TAUTOLOGY: the exactly-measured sibling is still suppressed, so
        // the print above is the floor flag and not "every Absorbs row prints".
        assert!(!out.contains("/exact"), "{out}");
    }

    /// A tap that fell short EARLIER and recovered is still reported.
    ///
    /// Nothing on a verdict row is sticky except `short_evaluations`, so without
    /// it a run whose tap could not absorb for twenty minutes and recovered
    /// before shutdown finalizes reading entirely clean.
    #[test]
    fn a_tap_that_fell_short_earlier_is_reported_even_though_it_recovered() {
        use cerulion_bagd::AbsorbanceVerdict;
        let healed = cerulion_bagd::TopicAbsorbance {
            short_evaluations: 37,
            ..verdict(9, AbsorbanceVerdict::Absorbs)
        };
        let out = render_absorbance_section(&present(
            vec![
                ("/healed".to_string(), healed),
                ("/clean".to_string(), verdict(9, AbsorbanceVerdict::Absorbs)),
            ],
            Some(cerulion_bagd::LossCountingBasis::PrefixProven),
        ));
        assert!(
            out.contains("1 fell short earlier in the run and now absorb"),
            "{out}"
        );
        assert!(out.contains("/healed"), "{out}");
        assert!(out.contains("fell short 37 time(s) this run"), "{out}");
        // ANTI-TAUTOLOGY: a genuinely clean row is still silent.
        assert!(!out.contains("/clean"), "{out}");

        // …and RECOVERED means the tap now ABSORBS, never merely "is not short".
        // A tap short all run whose final evaluation could not mint a rate
        // finalizes `NoClaim` with a nonzero count, and calling that a recovery
        // mints a positive health claim out of an absence of evidence. It is
        // counted APART instead, because "was short, now unknown" is worse news
        // than either half alone.
        // A REALISTIC no-claim row: the comparison fields are ABSENT, which is
        // what a stale rate window actually produces (`absorbance_verdict`
        // returns early with none of them set). Carrying them over from a
        // healthy fixture makes a row that disagrees with itself, which the
        // reader now — correctly — reports as inconsistent rather than counting.
        let stranded = cerulion_bagd::TopicAbsorbance {
            short_evaluations: 37,
            rate_mhz: None,
            absorbance_us: None,
            measured_tail_us: None,
            required_depth: None,
            shortfall_at_least_us: None,
            ..verdict(9, AbsorbanceVerdict::NoClaim)
        };
        assert_eq!(
            stranded.inconsistency(),
            None,
            "fixture precondition: this row is a SOUND no-claim, not a contradictory one"
        );
        let out = render_absorbance_section(&present(
            vec![("/stranded".to_string(), stranded)],
            Some(cerulion_bagd::LossCountingBasis::PrefixProven),
        ));
        assert!(
            out.contains("0 fell short earlier in the run and now absorb"),
            "a no-claim row is NOT a recovery: {out}"
        );
        assert!(
            out.contains("1 fell short earlier and can no longer be ranked"),
            "…and it is not silently folded away either: {out}"
        );
        assert!(out.contains("/stranded"), "{out}");
    }

    /// A CAPTURE's verdicts describe the RECORDER's whole run, not the capture
    /// window — and the header says which, because the numbers cannot.
    #[test]
    fn a_captures_verdicts_are_scoped_to_the_recorder_not_to_the_capture() {
        use cerulion_bagd::AbsorbanceVerdict;
        let rows = vec![("/a".to_string(), verdict(2, AbsorbanceVerdict::Short))];
        let recording = render_absorbance_section(&present(rows.clone(), None));
        assert!(recording.contains("this recording measured"), "{recording}");

        let capture = render_absorbance_section(&AbsorbanceReading::Present {
            rows,
            basis: None,
            scope: AbsorbanceScope::CaptureRecorder,
            ladder_edges_us: Some(cerulion_bagd::DRAIN_GAP_BUCKET_EDGES_US.to_vec()),
        });
        assert!(
            capture.contains("the RECORDER that took this capture"),
            "{capture}"
        );
        assert!(
            !capture.contains("this recording measured"),
            "a capture may not claim its own window measured a six-hour run: {capture}"
        );
    }

    /// The counting caveat prints whenever the recording
    /// cannot account for loss before a tap's first frame.
    ///
    /// Driven on BOTH shapes that mean it — an explicit `PrefixInvisible` and a
    /// older document that makes no claim — because a renderer keyed only
    /// on the explicit token would go silent on exactly the older bags that most
    /// need the warning.
    #[test]
    fn the_absorbance_block_states_the_counting_caveat_unless_the_prefix_was_proven() {
        use cerulion_bagd::AbsorbanceVerdict;
        for basis in [
            Some(cerulion_bagd::LossCountingBasis::PrefixInvisible),
            None,
        ] {
            let out = render_absorbance_section(&present(
                vec![("/a".to_string(), verdict(2, AbsorbanceVerdict::Short))],
                basis,
            ));
            assert!(
                out.contains("nothing was COUNTED"),
                "basis {basis:?} cannot prove its head loss, so the caveat must print: {out}"
            );
            assert!(out.contains("prefix_lost"), "{out}");
        }
    }

    /// The over-budget clause reaches the row.
    ///
    /// A tap the joined width put over its budget prints EVEN WHEN IT ABSORBS —
    /// the two are independent facts, and the whole point of carrying the state
    /// per topic is that it outlives the one log line that reports it.
    #[test]
    fn an_over_budget_tap_prints_even_when_it_absorbs() {
        use cerulion_bagd::AbsorbanceVerdict;
        // The occupancy is DERIVED (`capacity == widest x depth`), so
        // the depth here is the one that multiplies out to the capacity — 6
        // slots of 1 MiB against a 4 MiB budget.
        let row = cerulion_bagd::TopicAbsorbance {
            budget_bytes: Some(4 * 1024 * 1024),
            over_budget: Some(cerulion_bagd::OverBudget {
                budget_bytes: 4 * 1024 * 1024,
                capacity_bytes: 6 * 1024 * 1024,
                widest_slot_bytes: 1024 * 1024,
            }),
            ..verdict(6, AbsorbanceVerdict::Absorbs)
        };
        let out = render_absorbance_section(&present(
            vec![("/wide".to_string(), row)],
            Some(cerulion_bagd::LossCountingBasis::PrefixProven),
        ));
        assert!(out.contains("/wide"), "{out}");
        assert!(out.contains("OVER BUDGET"), "{out}");
        // ANTI-TAUTOLOGY: the SAME row without the clause is silent, so the
        // print above is the clause and not the verdict.
        let quiet = render_absorbance_section(&present(
            vec![("/wide".to_string(), verdict(6, AbsorbanceVerdict::Absorbs))],
            Some(cerulion_bagd::LossCountingBasis::PrefixProven),
        ));
        assert!(!quiet.contains("/wide"), "{quiet}");
    }

    /// The three non-Present arms, and the empty one.
    ///
    /// An EMPTY row set renders NOTHING — that is every bag that predates these verdicts, and a
    /// heading over no rows would read as "this recording was checked", which is
    /// the confident-false claim the whole vocabulary exists to prevent.
    #[test]
    fn absent_unreadable_and_verdictless_bags_render_nothing_while_malformed_prints() {
        assert_eq!(render_absorbance_section(&AbsorbanceReading::Absent), "");
        assert_eq!(
            render_absorbance_section(&AbsorbanceReading::IndexUnreadable),
            ""
        );
        assert_eq!(
            render_absorbance_section(&present(
                Vec::new(),
                Some(cerulion_bagd::LossCountingBasis::PrefixInvisible),
            )),
            "",
            "a bag whose taps carry no verdict must not be reported as checked"
        );
        // The MALFORMED arm names the attachment THIS bag holds, so the two
        // scopes are asserted apart: sending an operator to the other one points
        // them at something their bag provably does not contain.
        let bad = render_absorbance_section(&AbsorbanceReading::Malformed(
            AbsorbanceScope::Recording,
            "expected value at line 1".to_string(),
        ));
        assert!(bad.contains("MALFORMED"), "{bad}");
        assert!(bad.contains("expected value at line 1"), "{bad}");
        assert!(
            bad.contains(cerulion_bagd::RECORD_HEALTH_ATTACHMENT),
            "{bad}"
        );
        let bad = render_absorbance_section(&AbsorbanceReading::Malformed(
            AbsorbanceScope::CaptureRecorder,
            "expected value at line 1".to_string(),
        ));
        assert!(
            bad.contains(cerulion_bagd::CAPTURE_RECORDER_HEALTH_ATTACHMENT),
            "a malformed CAPTURE document must name the capture's own attachment: {bad}"
        );
        assert!(
            !bad.contains(cerulion_bagd::RECORD_HEALTH_ATTACHMENT),
            "…and not the recording's, which a capture does not carry: {bad}"
        );
    }

    /// PRINCIPLE #2 at the READER: a decoded row whose verdict contradicts its
    /// own numbers is reported as INCONSISTENT — never counted as healthy, never
    /// silently believed on one side.
    ///
    /// These rows come off a bag written by an unknown robot, so `verdict` and
    /// the numbers beside it are two independent claims. Before this, the
    /// renderer keyed every count on `verdict` and read the numbers separately:
    /// an `Absorbs` row carrying a shortfall landed in the healthy total AND was
    /// filtered off the actionable list (the filter reads the very fields in
    /// dispute), so a bag saying a tap fell short could be summarised as one
    /// where nothing did.
    ///
    /// The degradation is scoped to the ROW, not the document: one
    /// self-contradicting row must not suppress every other topic's verdict, and
    /// `Malformed` already means something else here — "the JSON did not
    /// decode", a different situation with a different remedy.
    #[test]
    fn a_row_whose_verdict_contradicts_its_numbers_is_reported_not_counted_as_healthy() {
        use cerulion_bagd::AbsorbanceVerdict;
        // `absorbs` + a shortfall: a combination `absorbance_verdict` cannot
        // produce, and the exact shape a hand-edited manifest yields.
        let contradictory = cerulion_bagd::TopicAbsorbance {
            shortfall_at_least_us: Some(20_000),
            ..verdict(9, AbsorbanceVerdict::Absorbs)
        };
        assert!(
            contradictory.inconsistency().is_some(),
            "fixture precondition: this row really does disagree with itself"
        );
        let out = render_absorbance_section(&present(
            vec![
                ("/bogus".to_string(), contradictory),
                // A SOUND row beside it — the half that proves the degradation
                // is scoped to the row rather than the document.
                ("/sound".to_string(), verdict(9, AbsorbanceVerdict::Absorbs)),
            ],
            Some(cerulion_bagd::LossCountingBasis::PrefixProven),
        ));

        // NOT in the healthy count: ONE topic absorbs, not two.
        assert!(
            out.contains("1 absorb the drain stalls"),
            "a contradicting row must not be counted as healthy: {out}"
        );
        assert!(
            out.contains("1 DISAGREE WITH THEMSELVES and are counted in none of the above"),
            "…and must be reported in its own bucket: {out}"
        );
        // The ROW names the topic and WHAT disagrees.
        let row = out
            .lines()
            .find(|l| l.contains("/bogus"))
            .unwrap_or_else(|| panic!("no /bogus row: {out}"));
        assert!(row.contains("INCONSISTENT"), "{row}");
        assert!(row.contains("carries a shortfall"), "{row}");
        assert!(row.contains("absorbs"), "…and what it claimed to be: {row}");
        // ANTI-TAUTOLOGY: the sound sibling is still silent, so this is not a
        // renderer that prints every row it is given.
        assert!(!out.contains("/sound"), "{out}");

        // THE BUCKETS PARTITION: every row is counted exactly once, so a reader
        // adding the header's numbers up gets the topic count it was given. The
        // four verdict counts cover `sound` (the enum has exactly those
        // variants) and the disagreement bucket is the rest — an arithmetic that
        // an unaccounted-for row would silently break.
        assert!(out.contains("2 topic(s) carry a verdict"), "{out}");
        let n = |needle: &str| -> usize {
            let at = out
                .find(needle)
                .unwrap_or_else(|| panic!("no `{needle}`: {out}"));
            out[..at]
                .rsplit(|c: char| !c.is_ascii_digit())
                .find(|t| !t.is_empty())
                .and_then(|t| t.parse().ok())
                .unwrap_or_else(|| panic!("no count before `{needle}`: {out}"))
        };
        let sum = n(" absorb the drain stalls")
            + n(" fall SHORT of them")
            + n(" met a stall the histogram cannot measure")
            + n(" make no claim")
            + n(" DISAGREE WITH THEMSELVES");
        assert_eq!(sum, 2, "the buckets must partition the rows: {out}");
    }

    /// The `Unrankable` rules reach THIS renderer.
    ///
    /// The aggregation is verdict-agnostic — it asks
    /// `inconsistency_against` and buckets — so a rule added in `cerulion_bagd`
    /// should surface here for free. "Should" is the reason for the arm: this
    /// renderer's counts are what an operator reads, and `beyond` is the bucket
    /// a loose rule feeds. A row saying "the histogram cannot measure this
    /// stall" while its own numbers say the histogram ranked it is the single
    /// most misleading thing this block can print, and before that fix it was
    /// counted straight into that total.
    #[test]
    fn an_unrankable_row_refused_by_the_producers_image_lands_in_the_inconsistent_bucket() {
        use cerulion_bagd::AbsorbanceVerdict;
        // Producer-true in every respect EXCEPT the required depth, which the
        // overflow arm never stamps (it has no tail to require a depth for).
        // Depth 4096 at 100 Hz derives 40.96 s, well above the ladder's top
        // edge, so the row is unranked-shaped and only this rule can convict it.
        let beyond = cerulion_bagd::TopicAbsorbance {
            measured_tail_us: None,
            required_depth: Some(3),
            shortfall_at_least_us: None,
            ..verdict(4096, AbsorbanceVerdict::Unrankable)
        };
        // …and the SOUND sibling, identical but for that one field.
        let sound = cerulion_bagd::TopicAbsorbance {
            required_depth: None,
            ..beyond
        };
        assert_eq!(
            sound.inconsistency_against(Some(cerulion_bagd::DRAIN_GAP_BUCKET_EDGES_US)),
            None,
            "fixture precondition: the deviation is the required depth and nothing else"
        );

        let out = render_absorbance_section(&present(
            vec![
                ("/forged".to_string(), beyond),
                ("/genuine".to_string(), sound),
            ],
            Some(cerulion_bagd::LossCountingBasis::PrefixProven),
        ));
        // ONE topic met a stall beyond the instrument, not two.
        assert!(
            out.contains("1 met a stall the histogram cannot measure"),
            "a refused row must not be counted as a genuine overflow: {out}"
        );
        assert!(
            out.contains("1 DISAGREE WITH THEMSELVES and are counted in none of the above"),
            "…and must be reported in its own bucket: {out}"
        );
        // The ROW carries its REASON, so the reader is told what disagrees.
        let row = out
            .lines()
            .find(|l| l.contains("/forged"))
            .unwrap_or_else(|| panic!("no /forged row: {out}"));
        assert!(row.contains("INCONSISTENT"), "{row}");
        assert!(row.contains("names the depth"), "{row}");
        // ANTI-TAUTOLOGY: the genuine sibling is still counted and PRINTED as
        // the unrankable row it is (this block names every actionable row), so
        // the assertions above are not satisfied by a renderer that flags
        // everything.
        let genuine = out
            .lines()
            .find(|l| l.contains("/genuine"))
            .unwrap_or_else(|| panic!("no /genuine row: {out}"));
        assert!(!genuine.contains("INCONSISTENT"), "{genuine}");
    }

    /// An EXPLICITLY edge-less ladder survives the decode.
    ///
    /// `drain_gaps` is skipped when vacant, so absent / vacant / explicitly
    /// edge-less all arrive with `edges_us` empty. `counts` is the distinguisher:
    /// the serde default carries both vectors empty, while a histogram that
    /// recorded gaps under NO edges still carries its unbounded bucket. The
    /// decode must not collapse all three to `None`, which would make
    /// `inconsistency_against` fall back to the READER's ladder — leaving the
    /// `Some(&[])` arm it documents unreachable from the production reader.
    #[test]
    fn an_explicitly_edgeless_ladder_is_kept_while_an_absent_one_falls_back() {
        let hist = |edges: Vec<u64>, counts: Vec<u64>| cerulion_bagd::DrainGapHistogram {
            edges_us: edges,
            counts,
        };
        // A real ladder travels verbatim.
        assert_eq!(
            decoded_ladder_edges(&hist(vec![100, 250], vec![1, 0, 0])),
            Some(vec![100, 250])
        );
        // ABSENT / vacant-skipped: the serde default, both vectors empty. No
        // ladder was STATED, so the reader speaks its own.
        assert_eq!(decoded_ladder_edges(&hist(Vec::new(), Vec::new())), None);
        // …and THE FIX: edges declared empty, gaps recorded anyway.
        assert_eq!(
            decoded_ladder_edges(&hist(Vec::new(), vec![7])),
            Some(Vec::new()),
            "a histogram that recorded gaps under no edges STATES an empty ladder"
        );
        // But only in the ONE SHAPE a histogram can hold.
        // `counts.len() == edges_us.len() + 1`, so an edge-less histogram carries
        // exactly its overflow bucket; deserialization enforces nothing, and the
        // earlier decode read `counts` for PRESENCE alone, so any forged length
        // bought a `Some(&[])` ladder and skipped the arms that need edges.
        // `None` is the safe direction for a shape no producer can hold: the
        // arms are asked against the reader's own vocabulary rather than not at
        // all.
        for forged in [vec![1u64, 2], vec![1, 2, 3], vec![0; 9]] {
            assert_eq!(
                decoded_ladder_edges(&hist(Vec::new(), forged.clone())),
                None,
                "{forged:?}: an edge-less histogram holding more than one bucket is a shape \
                 no producer can write"
            );
        }
    }

    /// …and that distinction changes a VERDICT, which is why it is not cosmetic.
    ///
    /// A tail-less `Short` row is the ladder-OVERFLOW arm, and whether it is
    /// producible depends on where the ladder's last edge sits. Ranked against an
    /// EMPTY ladder the question cannot be asked at all and is skipped; ranked
    /// against the reader's own it is convicted — so the collapsed decode
    /// reported a foreign recording's sound rows as self-contradicting.
    #[test]
    fn a_tailless_row_is_skipped_against_an_empty_ladder_and_judged_against_the_readers() {
        use cerulion_bagd::AbsorbanceVerdict;
        let edge = cerulion_bagd::DRAIN_GAP_BUCKET_EDGES_US
            .last()
            .copied()
            .expect("the shipped ladder has edges");
        // The `verdict` helper rates every row at 100 Hz, so its absorbance is
        // `depth * 10_000` us — and the shape rule requires the absorbance to FOLLOW
        // from the depth and rate the row itself reports, so this row reaches
        // past the ladder by being one slot DEEPER rather than by having its
        // absorbance overwritten.
        let past_the_ladder = (edge / 10_000) + 1;
        let tailless = cerulion_bagd::TopicAbsorbance {
            measured_tail_us: None,
            required_depth: None,
            shortfall_at_least_us: Some(1),
            ..verdict(past_the_ladder, AbsorbanceVerdict::Short)
        };
        assert!(
            tailless.absorbance_us.is_some_and(|a| a > edge),
            "precondition: this row's absorbance is above the shipped ladder's last edge"
        );
        // Through the PRODUCTION decode, not a hand-written `Option`: this arm
        // exists to say why the decode's distinction matters, so it must be
        // reading the same function `read_absorbance` does.
        let reading = |edges: Vec<u64>, counts: Vec<u64>| {
            let gaps = cerulion_bagd::DrainGapHistogram {
                edges_us: edges,
                counts,
            };
            AbsorbanceReading::Present {
                rows: vec![("/overflowed".to_string(), tailless)],
                basis: Some(cerulion_bagd::LossCountingBasis::PrefixProven),
                scope: AbsorbanceScope::Recording,
                ladder_edges_us: decoded_ladder_edges(&gaps),
            }
        };
        // STATED empty — no edges, but gaps recorded, so the histogram really
        // did name its (empty) vocabulary. The arm cannot be asked, so the row
        // stands.
        let kept = render_absorbance_section(&reading(Vec::new(), vec![7]));
        assert!(
            !kept.contains("DISAGREE WITH THEMSELVES"),
            "a row ranked on a ladder with no edges must not be convicted by an arm that \
             ladder cannot answer: {kept}"
        );
        assert!(kept.contains("1 fall SHORT of them"), "{kept}");
        // NO ladder stated: the reader falls back to its own vocabulary, and
        // against THAT the row is unproducible. This is the anti-tautology half —
        // without it the assertion above passes on a renderer that convicts
        // nothing.
        let judged = render_absorbance_section(&reading(Vec::new(), Vec::new()));
        assert!(
            judged.contains("1 DISAGREE WITH THEMSELVES"),
            "a document naming no ladder is still checked against the only one its reader \
             speaks: {judged}"
        );
        // THE FORGED SHAPE: no edges beside TWO counts is a histogram
        // no producer can hold — `counts.len() == edges_us.len() + 1` — and
        // reading `counts` for presence alone let it claim the skip arm, so a
        // hand-edited attachment bought its rows an unchecked verdict. It now
        // falls back to the reader's own ladder and is judged exactly like a
        // document that named none.
        let forged = render_absorbance_section(&reading(Vec::new(), vec![1, 2]));
        assert!(
            forged.contains("1 DISAGREE WITH THEMSELVES"),
            "an edge-less histogram carrying two buckets must not buy the skip arm: {forged}"
        );
    }

    /// The SAME reader discipline for a ZERO rate.
    ///
    /// `absorbance_verdict_inner` filters `millihertz > 0` before it builds a
    /// row, so `Some(0)` is exactly as unproducible as `None` on a row that
    /// reaches a verdict — but the reader's guard rejected only the second, and
    /// every arithmetic arm is satisfied by the first. A forged `Absorbs` was
    /// therefore COUNTED as an absorbing topic here while `render_line` printed
    /// it `0.000 Hz`, which is the aggregation-level half of the hole.
    ///
    /// Driven on BOTH verdicts a reader acts on, with a sound sibling for the
    /// partition arithmetic.
    #[test]
    fn a_row_whose_verdict_is_derived_from_a_zero_rate_is_reported_not_counted() {
        use cerulion_bagd::AbsorbanceVerdict;
        let zero_absorbs = cerulion_bagd::TopicAbsorbance {
            rate_mhz: Some(0),
            ..verdict(9, AbsorbanceVerdict::Absorbs)
        };
        let zero_short = cerulion_bagd::TopicAbsorbance {
            rate_mhz: Some(0),
            ..verdict(2, AbsorbanceVerdict::Short)
        };
        for row in [&zero_absorbs, &zero_short] {
            assert!(
                row.inconsistency().is_some(),
                "fixture precondition: a zero rate really does convict this row"
            );
        }
        let out = render_absorbance_section(&present(
            vec![
                ("/zero_absorbs".to_string(), zero_absorbs),
                ("/zero_short".to_string(), zero_short),
                // The SOUND sibling: without it, "0 absorb" would also be
                // satisfied by a renderer that counted nothing at all.
                ("/sound".to_string(), verdict(9, AbsorbanceVerdict::Absorbs)),
            ],
            Some(cerulion_bagd::LossCountingBasis::PrefixProven),
        ));
        // EXCLUDED from the healthy counts: one absorbs, and it is the sound one.
        assert!(
            out.contains("1 absorb the drain stalls"),
            "a zero-rate row must not be counted as absorbing: {out}"
        );
        assert!(
            out.contains("0 fall SHORT of them"),
            "…nor as a shortfall an operator can act on: {out}"
        );
        // …and REPORTED, both of them, with the reason on the row.
        assert!(
            out.contains("2 DISAGREE WITH THEMSELVES and are counted in none of the above"),
            "{out}"
        );
        for topic in ["/zero_absorbs", "/zero_short"] {
            let row = out
                .lines()
                .find(|l| l.contains(topic))
                .unwrap_or_else(|| panic!("no {topic} row: {out}"));
            assert!(row.contains("INCONSISTENT"), "{row}");
            assert!(row.contains("rate of ZERO"), "{row}");
        }
        assert!(
            !out.contains("/sound"),
            "the sound sibling is counted, not printed: {out}"
        );
    }

    /// …and a healthy document says nothing about disagreement at all.
    ///
    /// Without this, the clause could be emitted on every bag ever made and the
    /// assertions above would still pass.
    #[test]
    fn a_document_whose_rows_agree_with_themselves_never_mentions_disagreement() {
        use cerulion_bagd::AbsorbanceVerdict;
        let out = render_absorbance_section(&present(
            vec![
                ("/a".to_string(), verdict(2, AbsorbanceVerdict::Short)),
                ("/b".to_string(), verdict(9, AbsorbanceVerdict::Absorbs)),
            ],
            Some(cerulion_bagd::LossCountingBasis::PrefixProven),
        ));
        assert!(!out.contains("DISAGREE"), "{out}");
        assert!(!out.contains("INCONSISTENT"), "{out}");
        // …while the counts it DOES make are unaffected.
        assert!(out.contains("1 absorb the drain stalls"), "{out}");
        assert!(out.contains("1 fall SHORT"), "{out}");
    }

    /// Remote-supplied text in this block is SANITIZED.
    ///
    /// A topic name comes off a machine this desk did not configure and the
    /// decoder's message interpolates bag bytes, so both go through the shared
    /// filter — the `render_state_coverage_section` precedent.
    #[test]
    fn the_absorbance_block_sanitizes_topic_names_and_decoder_messages() {
        use cerulion_bagd::AbsorbanceVerdict;
        const CSI: &str = "\u{1b}[2J";
        let out = render_absorbance_section(&present(
            vec![(format!("/evil{CSI}"), verdict(2, AbsorbanceVerdict::Short))],
            None,
        ));
        assert!(!out.chars().any(|c| c.is_control() && c != '\n'), "{out:?}");
        assert!(out.contains("/evil\u{fffd}"), "{out}");
        let bad = render_absorbance_section(&AbsorbanceReading::Malformed(
            AbsorbanceScope::Recording,
            format!("expected value{CSI}"),
        ));
        assert!(!bad.chars().any(|c| c.is_control() && c != '\n'), "{bad:?}");
    }

    // -----------------------------------------------------------------------
    // The PRODUCERS block
    // -----------------------------------------------------------------------

    /// The PRODUCERS block names every topic that earned attribution,
    /// distinguishes DECLARED from runtime-OBSERVED, and is silent otherwise.
    ///
    /// Both halves in one body, on the flashback precedent: the silence is the
    /// half that matters most here rather than an edge, because a single-writer
    /// recording earns no labels AT ALL — so a "producers: 0 labelled frames"
    /// line would appear on very nearly every bag ever made and train an operator
    /// to skip the block on the few where it says something.
    ///
    /// The catch-up marker is asserted PER LINE, not over the whole block: with
    /// two rows a bare `contains` is satisfied by the other topic's marker, which
    /// is exactly the confusion the marker exists to prevent.
    #[test]
    fn the_producers_block_names_who_earned_attribution_and_stays_quiet_otherwise() {
        const DECLARED: &str = "/tf";
        const OBSERVED: &str = "/joint_states";
        let out = render_producer_labels_section(&ProducerLabelReading::Present {
            unreportable: 0,
            rows: vec![
                // Declared `multi_publisher`: armed at open, so no prefix was ever
                // unattributed and no catch-up is owed.
                (DECLARED.to_string(), 6, false),
                // Runtime-observed plurality: one catch-up attributes the
                // single-writer run that preceded the second writer.
                (OBSERVED.to_string(), 12, true),
            ],
        });
        assert!(
            out.contains("__cerulion/frame_producers"),
            "the block must name the channel a reader would go and decode: {out}"
        );
        let line_for = |topic: &str| -> String {
            out.lines()
                .find(|l| l.contains(topic))
                .unwrap_or_else(|| panic!("no row for {topic}: {out}"))
                .to_string()
        };
        let declared = line_for(DECLARED);
        assert!(declared.contains("6 labelled frame(s)"), "{declared}");
        assert!(
            !declared.contains("catch-up"),
            "a DECLARED topic owes no catch-up, so its row must not claim one: {declared}"
        );
        let observed = line_for(OBSERVED);
        assert!(observed.contains("12 labelled frame(s)"), "{observed}");
        assert!(
            observed.contains("catch-up record attributing the run before the second writer"),
            "the runtime-armed marker is what says the plurality was OBSERVED: {observed}"
        );

        // The silent arms. EMPTY is the one that matters: it is what nearly every
        // bag carries, and it is a `Present` reading — the manifest was read and
        // it says no topic earned a label.
        assert_eq!(
            render_producer_labels_section(&ProducerLabelReading::Present {
                rows: Vec::new(),
                unreportable: 0
            }),
            "",
            "a single-writer recording must render NOTHING"
        );
        assert_eq!(
            render_producer_labels_section(&ProducerLabelReading::Absent),
            ""
        );
        assert_eq!(
            render_producer_labels_section(&ProducerLabelReading::IndexUnreadable),
            ""
        );

        // A manifest that is THERE and unreadable is a different fact and prints:
        // that bag DOES have an attribution story somebody cannot read.
        let bad = render_producer_labels_section(&ProducerLabelReading::Malformed {
            artifact: cerulion_bagd::RECORD_HEALTH_ATTACHMENT.to_string(),
            detail: "expected value at line 1".to_string(),
        });
        assert!(bad.contains("MALFORMED"), "{bad}");
        assert!(
            bad.contains("The frames and their labels are unaffected"),
            "{bad}"
        );
    }

    /// The MALFORMED arm carries the decoder's message, which interpolates what
    /// it was reading — bag bytes, written by another process on another machine
    /// — so it goes through the same terminal filter every other bag string does.
    #[test]
    fn the_producers_malformed_arm_sanitizes_the_decoders_message() {
        const CSI: &str = "\u{1b}[2J";
        let bad = render_producer_labels_section(&ProducerLabelReading::Malformed {
            artifact: cerulion_bagd::RECORD_HEALTH_ATTACHMENT.to_string(),
            detail: format!("expected value{CSI}"),
        });
        assert!(!bad.chars().any(|c| c.is_control() && c != '\n'), "{bad:?}");
        assert!(bad.contains("expected value\u{fffd}"), "{bad}");
    }

    /// A topic NAME arrives from a bag written by another machine, so the row
    /// goes through the same terminal filter the malformed arm does.
    #[test]
    fn a_producers_row_sanitizes_the_topic_name_it_read_out_of_the_bag() {
        const CSI: &str = "\u{1b}[2J";
        let out = render_producer_labels_section(&ProducerLabelReading::Present {
            unreportable: 0,
            rows: vec![(format!("/evil{CSI}"), 3, false)],
        });
        assert!(!out.chars().any(|c| c.is_control() && c != '\n'), "{out:?}");
        assert!(out.contains("/evil\u{fffd}"), "{out}");
    }

    /// The read's four arms, over real bags.
    ///
    /// The renderer's arms are driven by hand above; what only a real bag can
    /// answer is which arm a given FILE lands on — and the `finalized`
    /// discriminator is the load-bearing half. `BagReader::attachment` walks the
    /// SUMMARY's index, so `Ok(None)` on an UNFINALIZED bag means "could not
    /// look", not "carries nothing": conflating the two would let a torn
    /// recording report that it has no producer attribution, which is a claim.
    /// They render identically today, so no `bag info` output can separate them —
    /// this is the only place the distinction is observable.
    ///
    /// The FILTER is asserted here too, on the same bag: a topic that earned
    /// neither labels nor a catch-up contributes no row.
    #[test]
    fn the_producer_label_reading_separates_a_finalized_bag_from_a_torn_one() {
        use cerulion_bag::{BagWriter, BagWriterConfig, TopicSchema};

        const TOPIC: &str = "/imu";
        const HASH: u64 = 0x1417_1417_1417_1417;

        let dir = tempfile::tempdir().expect("tempdir");
        let schema = || TopicSchema {
            topic: TOPIC.to_string(),
            schema_name: "geometry_msgs/Vector3".to_string(),
            schema_hash: HASH,
            wire_fixed_size: 24,
        };
        let payload = [0u8; 24];
        let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
        WireHeader {
            schema_hash: HASH,
            total_size: (WireHeader::SIZE + payload.len()) as u32,
            offset_table_offset: 0,
            offset_table_count: 0,
            sequence: 0,
            timestamp_ns: 1_000_000_000,
        }
        .write_to_buf(&mut frame[..WireHeader::SIZE]);

        let write = |path: &Path, health: Option<&[u8]>, finalize: bool| {
            let mut w = BagWriter::create(path, BagWriterConfig::default(), &[schema()])
                .expect("create bag");
            if let Some(bytes) = health {
                w.write_attachment(
                    cerulion_bagd::RECORD_HEALTH_ATTACHMENT,
                    "application/json",
                    0,
                    0,
                    bytes,
                )
                .expect("write health");
            }
            w.write_chunk(|c| {
                c.write_message(TOPIC, 0, 1_000_000_000, 1_000_000_000, &[&frame[..]])
            })
            .expect("write frames");
            if finalize {
                w.finalize().expect("finalize");
            }
        };

        // PRESENT — and the filter: `/imu` earned nothing and must not appear,
        // while `/tf` (labels) and `/scan` (catch-up only) both must.
        let health = serde_json::json!({
            "version": 2,
            "dropped_unwritten": 0,
            "topics": {
                "/imu": { "frames_recorded": 9, "frames_lost": 0, "gap_events": 0 },
                "/scan": {
                    "frames_recorded": 9, "frames_lost": 0, "gap_events": 0,
                    "label_catch_up": true
                },
                "/tf": {
                    "frames_recorded": 9, "frames_lost": 0, "gap_events": 0,
                    "producer_labels": 4
                },
            }
        });
        let present = dir.path().join("present.mcap");
        write(
            &present,
            Some(&serde_json::to_vec(&health).expect("encode")),
            true,
        );
        let reader = open_bag(&present).expect("open");
        assert_eq!(
            read_producer_labels(&reader, true),
            ProducerLabelReading::Present {
                rows: vec![
                    ("/scan".to_string(), 0, true),
                    ("/tf".to_string(), 4, false),
                ],
                unreportable: 0,
            },
            "only topics that earned attribution contribute rows, in topic order"
        );

        // COUNTED-AND-EMPTY — finalized, no manifest, but the reserved channel
        // IS registered (every `BagWriter::create` bag carries it),
        // so the reading comes from COUNTING the bag's own
        // records rather than from a manifest, and it finds none.
        //
        // This arm is deliberate, and the fixture
        // is why: it is written by the CURRENT writer, so it is not an
        // "older bag". `Absent` means the
        // strictly older shape — a bag with no producer-label channel at all —
        // which this writer cannot produce. An empty COUNT is a stronger claim
        // than that arm's silence ("this bag labels nothing", read off the bag)
        // and both render identically, so no output changed.
        let counted_empty = dir.path().join("counted_empty.mcap");
        write(&counted_empty, None, true);
        let reader = open_bag(&counted_empty).expect("open");
        assert_eq!(
            read_producer_labels(&reader, true),
            ProducerLabelReading::Present {
                rows: Vec::new(),
                unreportable: 0
            },
            "a finalized bag with no manifest is COUNTED; its channel exists and holds nothing"
        );
        assert!(
            render_producer_labels_section(&read_producer_labels(&reader, true)).is_empty(),
            "…and an empty count renders exactly what `Absent` did: nothing"
        );

        // INDEX-UNREADABLE — a torn recording whose summary was never written.
        // NOT `Absent`: nothing here is evidence about what was labelled.
        let torn = dir.path().join("torn.mcap");
        write(&torn, None, false);
        let reader = open_bag(&torn).expect("open");
        assert_eq!(
            read_producer_labels(&reader, false),
            ProducerLabelReading::IndexUnreadable
        );

        // MALFORMED — present and undecodable, which is a third fact again.
        let bad = dir.path().join("malformed.mcap");
        write(&bad, Some(b"{ this is not json"), true);
        let reader = open_bag(&bad).expect("open");
        assert!(
            matches!(
                read_producer_labels(&reader, true),
                ProducerLabelReading::Malformed { .. }
            ),
            "a manifest that is THERE and will not decode is its own reading"
        );
    }

    // -----------------------------------------------------------------------
    // The FLASHBACK block
    // -----------------------------------------------------------------------

    /// A capture that covers a fraction of what it claims SAYS SO, and one that
    /// covered everything does not explain itself.
    ///
    /// Both halves in one body, because the second is what stops the first being
    /// satisfied by a renderer that prints the shortfall sentence unconditionally
    /// — which would train an operator to skip the block on exactly the bags it
    /// exists for.
    #[test]
    fn the_flashback_block_names_a_shortfall_and_stays_quiet_without_one() {
        let short = cerulion_bagd::FlashbackManifest {
            seq: Some(7),
            pinned: Some(false),
            frames: Some(412),
            span_ms: Some(45_000),
            achieved_span_ms: Some(1_990),
            coverage_shortfall_ms: Some(43_010),
            window_span_ms: Some(30_000),
            window_cap_bytes: Some(335_544_320),
            truncated_frames: Some(63_451),
            topics_with_no_frames: Some(4),
        };
        let out = render_flashback_section(&FlashbackReading::Present(Box::new(short.clone())));
        assert!(out.contains("flashback capture #7"), "{out}");
        assert!(out.contains("covers 2.0s of the 45.0s it claims"), "{out}");
        assert!(out.contains("SHORT BY 43.0s"), "{out}");
        // …with the ceiling that caused it and the lever that moves it — this
        // manifest carries the EVIDENCE (`truncated_frames: 63451`), which is
        // what earns the causal sentence.
        assert!(out.contains("ceiling 320 MiB"), "{out}");
        assert!(out.contains("CERULION_FLASHBACK_WINDOW_MAX_MB"), "{out}");
        assert!(
            out.contains("412 carried, 63451 taken by the byte ceiling"),
            "{out}"
        );
        // A range is not coverage for a topic slower than it.
        assert!(
            out.contains("4 tapped topic(s) contribute NO frame"),
            "{out}"
        );

        // A capture that lost NOTHING: the block is a quiet statement of fact.
        let whole = cerulion_bagd::FlashbackManifest {
            achieved_span_ms: Some(45_000),
            coverage_shortfall_ms: Some(0),
            truncated_frames: Some(0),
            topics_with_no_frames: Some(0),
            ..short
        };
        let out = render_flashback_section(&FlashbackReading::Present(Box::new(whole)));
        assert!(out.contains("covers 45.0s of the 45.0s it claims"), "{out}");
        assert!(!out.contains("SHORT BY"), "{out}");
        assert!(
            !out.contains("contribute NO frame"),
            "no topic was missing, so nothing is said about one: {out}"
        );
    }

    /// The CAUSE is claimed only on evidence.
    ///
    /// The shape: a 45 s claim, 30 s achieved and
    /// `truncated_frames: Some(0)` — which a cause-guessing block would render with the
    /// byte-ceiling sentence and the raise-the-cap remedy. A shortfall does not
    /// imply eviction: a capture triggered inside its first span, a robot whose
    /// topics are sparse, or an interval nobody published in each produce one
    /// with NOTHING evicted, and the remediation is then a wrong instruction
    /// rather than a missing one.
    ///
    /// Three arms in one body over the SAME spans — zero evidence, positive
    /// evidence, and ABSENT evidence (an older bag, which makes no claim
    /// either way and must therefore not have one made for it).
    #[test]
    fn the_flashback_block_blames_the_byte_ceiling_only_when_frames_were_evicted() {
        let base = cerulion_bagd::FlashbackManifest {
            seq: Some(7),
            frames: Some(412),
            span_ms: Some(45_000),
            achieved_span_ms: Some(30_000),
            coverage_shortfall_ms: Some(15_000),
            window_cap_bytes: Some(335_544_320),
            ..Default::default()
        };

        // Zero truncation, real shortfall.
        let none = render_flashback_section(&FlashbackReading::Present(Box::new(
            cerulion_bagd::FlashbackManifest {
                truncated_frames: Some(0),
                ..base.clone()
            },
        )));
        assert!(none.contains("SHORT BY 15.0s"), "{none}");
        assert!(
            !none.contains("evict"),
            "nothing was evicted, so nothing may be blamed on eviction: {none}"
        );
        assert!(
            !none.contains("CERULION_FLASHBACK_WINDOW_MAX_MB"),
            "…and the operator must not be sent to raise a cap that never bound: {none}"
        );
        assert!(!none.contains("ceiling 320 MiB"), "{none}");

        // The SAME spans WITH evidence: the causal sentence and the remedy.
        let evidenced = render_flashback_section(&FlashbackReading::Present(Box::new(
            cerulion_bagd::FlashbackManifest {
                truncated_frames: Some(63_451),
                ..base.clone()
            },
        )));
        assert!(evidenced.contains("SHORT BY 15.0s"), "{evidenced}");
        assert!(
            evidenced.contains("byte ceiling evicted frames during this capture"),
            "{evidenced}"
        );
        assert!(
            evidenced.contains("CERULION_FLASHBACK_WINDOW_MAX_MB"),
            "{evidenced}"
        );
        assert!(evidenced.contains("ceiling 320 MiB"), "{evidenced}");

        // ABSENT evidence (an older bag): the measurement, no cause.
        let unknown = render_flashback_section(&FlashbackReading::Present(Box::new(base)));
        assert!(unknown.contains("SHORT BY 15.0s"), "{unknown}");
        assert!(
            !unknown.contains("evict"),
            "a bag that states no truncation must not have a cause invented for it: {unknown}"
        );
    }

    /// The three arms that render NOTHING or say what they cannot read.
    ///
    /// ABSENT is the ordinary case — most bags are recordings, not captures — and
    /// a paragraph on every one of them trains an operator to skip the block.
    /// A manifest that is THERE and unreadable is a different fact and prints.
    #[test]
    fn the_flashback_block_is_silent_on_a_recording_and_loud_on_a_manifest_it_cannot_read() {
        assert_eq!(render_flashback_section(&FlashbackReading::Absent), "");
        assert_eq!(
            render_flashback_section(&FlashbackReading::IndexUnreadable),
            ""
        );
        let bad = render_flashback_section(&FlashbackReading::Malformed(
            "expected value at line 1".to_string(),
        ));
        assert!(bad.contains("MALFORMED"), "{bad}");
        assert!(
            bad.contains("The frames themselves are unaffected"),
            "{bad}"
        );
    }

    /// A manifest from a recorder that predated these fields renders what it
    /// DOES carry and asserts nothing about what it does not.
    ///
    /// The alternative — printing 0 for an absent `achieved_span_ms` — is the
    /// same defect with the sign flipped: it would report every earlier
    /// capture as having covered nothing at all.
    #[test]
    fn a_pre_1337_manifest_renders_its_claim_and_asserts_nothing_it_did_not_measure() {
        let old = cerulion_bagd::FlashbackManifest {
            seq: Some(2),
            span_ms: Some(45_000),
            window_span_ms: Some(30_000),
            ..Default::default()
        };
        let out = render_flashback_section(&FlashbackReading::Present(Box::new(old)));
        assert!(out.contains("claims 45.0s"), "{out}");
        assert!(
            out.contains("predates achieved-span reporting"),
            "an absent measurement must read as absent, not as zero coverage: {out}"
        );
        assert!(!out.contains("SHORT BY"), "{out}");
        assert!(!out.contains("covers 0.0s"), "{out}");
    }

    /// The MALFORMED arm carries the decoder's message, which interpolates what
    /// it was reading — bag bytes, written by another process on another
    /// machine — so it goes through the same terminal filter every other bag
    /// string does.
    #[test]
    fn the_flashback_malformed_arm_sanitizes_the_decoders_message() {
        const CSI: &str = "\u{1b}[2J";
        let bad =
            render_flashback_section(&FlashbackReading::Malformed(format!("expected value{CSI}")));
        assert!(!bad.chars().any(|c| c.is_control() && c != '\n'), "{bad:?}");
        assert!(bad.contains("expected value\u{fffd}"), "{bad}");
    }

    /// An UNATTRIBUTED index reads like a declared node: same tear wording, same
    /// skip-cause NAMES, and its own ring.
    ///
    /// An unattributed path that prints a bare `, N skipped` while the
    /// declared path prints the causes leaves the rows an operator understands
    /// least told least, with `contended` / `low_memory` / `capture_failed`
    /// (three problems, three different next steps) collapsed into a number the
    /// manifest already knows how to break down. Both go through
    /// `anchor_outcome_suffix`, which is also what stops them drifting.
    ///
    /// The two rings carrying the SAME unknown index are here too: each renders
    /// its own row naming its own ring, which a bare-index keyspace could not do.
    #[test]
    fn an_unattributed_index_renders_its_skip_causes_like_a_declared_node() {
        let mut declared = node(1, 1, 2, Some(4));
        declared.skip_causes = [("contended".to_string(), 2u64)].into_iter().collect();
        let mut s = state_of(&[("alpha", declared)]);

        let mut stray_a = node(0, 1, 3, None);
        stray_a.ring = "/cer_st_a".into();
        stray_a.node_idx = Some(1);
        stray_a.skip_causes = [("low_memory".to_string(), 1u64), ("contended".into(), 2)]
            .into_iter()
            .collect();
        let mut stray_b = node(2, 0, 0, Some(9));
        stray_b.ring = "/cer_st_b".into();
        stray_b.node_idx = Some(1);
        s.unattributed_indices
            .insert("/cer_st_a".into(), [(1, stray_a)].into_iter().collect());
        s.unattributed_indices
            .insert("/cer_st_b".into(), [(1, stray_b)].into_iter().collect());

        let out = render_state_coverage_section(&StateCoverageReading::Present(s));
        // The declared row, unchanged.
        assert!(
            out.contains("alpha: 1 complete, 1 TORN, 2 skipped (contended x2)"),
            "{out}"
        );
        // The unattributed rows: SAME suffix grammar, causes named, ring named.
        assert!(
            out.contains(
                "[no manifest entry] node_idx 1: 0 complete, 1 TORN, 3 skipped \
                 (contended x2, low_memory x1)"
            ),
            "an unattributed index must name its causes exactly as a declared \
             node does: {out}"
        );
        assert!(
            out.contains("index ring /cer_st_a does not declare"),
            "{out}"
        );
        // The SECOND ring's row exists and is its own — a merged keyspace could
        // render only one.
        assert!(
            out.contains("[no manifest entry] node_idx 1: 2 complete — record(s)"),
            "{out}"
        );
        assert!(
            out.contains("index ring /cer_st_b does not declare"),
            "{out}"
        );
        assert_eq!(
            out.matches("[no manifest entry]").count(),
            2,
            "one row per (ring, index): {out}"
        );
    }

    /// The three non-`Present` arms.
    ///
    /// ABSENT and INDEX-UNREADABLE render NOTHING — most bags carry no
    /// checkpoints, and a paragraph on every one of them would train an operator
    /// to skip the block. MALFORMED renders, because that bag DOES have a
    /// checkpoint story somebody cannot read, and it must not read as "clean".
    #[test]
    fn the_non_present_arms_say_only_what_they_know() {
        assert_eq!(
            render_state_coverage_section(&StateCoverageReading::Absent),
            ""
        );
        assert_eq!(
            render_state_coverage_section(&StateCoverageReading::IndexUnreadable),
            ""
        );
        let bad = render_state_coverage_section(&StateCoverageReading::Malformed(
            "expected value".to_string(),
        ));
        assert!(bad.contains("MALFORMED"), "{bad}");
        assert!(bad.contains("expected value"), "{bad}");
        assert!(
            bad.contains("The anchors themselves are unaffected"),
            "a malformed report must not read as lost data: {bad}"
        );
        assert!(
            !bad.contains("complete"),
            "an unreadable manifest may claim nothing: {bad}"
        );
    }
}
