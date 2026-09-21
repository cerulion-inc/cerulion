//! The recorder's FLASHBACK PLANE — the window, the gate, the
//! trigger channel and the capture in flight, in one place.
//!
//! # What lives here and what deliberately does not
//!
//! This is STATE plus the decisions that need only that state. It does not build
//! a `capture::CaptureJob`, because a job needs the
//! recorder's live tap list and resolved channel set — knowledge that arrives
//! from discovery and the schema ladder and belongs to the `Recorder`. So the
//! orchestration (`Recorder::service_flashback`) stays where the taps are, and
//! everything it can answer without them is answered here.
//!
//! # The two modes, and why the window is held in BOTH
//!
//! By design, `cerulion flashback` means one thing. A `--record` run is a
//! serving graph and holds the window exactly like any other; the flag adds a
//! continuous bag, it does not take the black box away. The alternative —
//! `cerulion flashback` producing a bag without the flag and a marker with it —
//! is the misleading-surface class this repo refuses.
//!
//! What differs is only the COST of harvesting: a window-only recorder MOVES each
//! drained batch out of its tap's staging (nothing else wanted it), while a
//! `--record` recorder must CLONE, because its writer thread takes the original.
//! That clone is the copy the design priced in.

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use cerulion_core::flashback::channel::{
    FlashbackOutcome, FlashbackRequestFrame, FlashbackResponder,
};
use cerulion_core::flashback::retention::RetentionCaps;
use cerulion_core::flashback::switch::TriggerPosture;
use cerulion_core::flashback::trigger::{
    CaptureCause, FinishedCapture, FlashbackTriggerGate, TriggerDecision, TriggerKind,
    TriggerPolicy, TriggerStats,
};

use crate::anchor_window::{AnchorFit, AnchorWindow, Checkpoint, NoAnchorReason};
use crate::capture::{CaptureStats, CaptureWriter};
use crate::window::FrameWindow;

/// The rolling anchor retention, shared between the two places a state ring is
/// drained.
///
/// # Why it is shared at all
///
/// The state ring is SPSC: `commit` advances ONE cursor in shared memory, so two
/// consumers steal each other's records. Which thread that single consumer is
/// depends on the mode — a window-only recorder owns its rings for the whole run
/// (there is no continuous bag and so no writer thread), while a `--record` one
/// hands them to the writer thread at bag creation, which drains them zero-copy
/// into `__cerulion/state`.
///
/// So the harvest cannot live on one thread. What is shared is only this
/// buffer; the RULE (which records make one anchor) travels with the ring itself
/// as an `AnchorHarvester`, so neither mode can be wired up wrong or forgotten.
///
/// Contention is negligible by construction: the lock is taken ONCE PER CLOSED
/// ANCHOR (a cadence apart, per node), never per record — a 673 MiB anchor is
/// ~1.4 M records and exactly one acquisition.
pub(crate) type SharedAnchors = Arc<Mutex<AnchorWindow>>;

/// Lock the retention, tolerating POISON.
///
/// A panic elsewhere must not wedge the recorder's drain loop, and it must not
/// silently discard what the retention already holds — the same rule
/// `lock_regime_latch` states for the shared failure latch. Recovering the guard
/// keeps the black box rolling; the alternative is a recorder that stops
/// draining its taps because a capture thread died.
pub(crate) fn lock_anchors(anchors: &SharedAnchors) -> MutexGuard<'_, AnchorWindow> {
    anchors.lock().unwrap_or_else(|e| e.into_inner())
}

/// The rolling SCHEDULER-TRACE retention, shared for exactly
/// the reason [`SharedAnchors`] is.
///
/// The trace ring's single SPSC consumer lives on the WRITER THREAD on a
/// `--record` run (`WriterCore::write_batch` drains it, writes the continuous
/// bag from the same span, and commits). So the retention this feeds cannot live
/// on the drive loop that reads it at capture time.
///
/// # Every multi-process run feeds this now
///
/// Whether this retention has anything to retain is a fact about the RINGS rather
/// than about the retention: a window-only recorder handed no `--ring` at
/// all has nothing to retain from. Every multi-process `graph run`
/// gets per-rank trace rings and hands their names to the window
/// recorder (`flashback_argv` pushes one `--ring` per name), so this retention
/// is fed on the DEFAULT run shape, not only on a recording.
///
/// A window-only recorder handed none is now the exception — a wall-gated run
/// shape, or a run refused its rings — and its captures still report
/// no trace (see [`HANDOFF_TRACE_NONE_NO_RINGS`]).
///
/// Contention is negligible by construction, on the same argument the anchor
/// retention makes: the lock is taken ONCE PER DRAIN BATCH, never per record.
pub(crate) type SharedTraceWindow = Arc<Mutex<crate::trace_window::TraceWindow>>;

/// Lock the trace retention, tolerating POISON — see [`lock_anchors`].
pub(crate) fn lock_trace(
    trace: &SharedTraceWindow,
) -> MutexGuard<'_, crate::trace_window::TraceWindow> {
    trace.lock().unwrap_or_else(|e| e.into_inner())
}

/// Everything an operator or a caller can dial about the plane.
#[derive(Debug, Clone)]
pub struct FlashbackSettings {
    /// How far back the rolling window reaches.
    pub window_span: Duration,
    /// The window's hard byte ceiling.
    pub window_max_bytes: u64,
    /// The ANCHOR retention's hard byte ceiling
    /// (`CERULION_FLASHBACK_ANCHOR_MAX_MB`).
    ///
    /// DERIVED-EQUAL to `window_max_bytes` unless the anchor's own
    /// env says otherwise, so scaling the window scales this with it. There is no
    /// constant to read any more — the equality is made at runtime precisely
    /// because a const-default let the two halves of one priced cost drift apart.
    ///
    /// This is the STARTING ceiling and the plane's total
    /// budget is `window_max_bytes + anchor_max_bytes`. Once a generation has
    /// been MEASURED the two are re-split anchor-first, so the live ceilings are
    /// the retentions' own — see
    /// [`FlashbackPlane::budget_split`](FlashbackPlane::budget_split).
    pub anchor_max_bytes: u64,
    /// Where [`anchor_max_bytes`](Self::anchor_max_bytes) came from.
    ///
    /// The reserve may never grow past a ceiling an
    /// operator STATED, and only the basis can tell an operator's number from a
    /// derived one that happens to have the same value. Carried rather than
    /// re-resolved, so the plane and the spawn-time projection cannot disagree
    /// about which knob is in force.
    pub anchor_cap_basis: cerulion_core::flashback::CapBasis,
    /// The SCHEDULER-TRACE retention's hard byte ceiling
    /// (`CERULION_FLASHBACK_TRACE_MAX_MB`).
    ///
    /// Its own knob rather than a share of `anchor_max_bytes`, because it is a
    /// different order of magnitude: ~13 MB across the shipped window against
    /// the frame window's 320 MiB — see
    /// [`cerulion_core::flashback::DEFAULT_FLASHBACK_TRACE_MAX_MB`]. Past it a
    /// capture loses its own resume boundary and reports itself NOT resimmable
    /// rather than shipping a trace that begins mid-step.
    pub trace_max_bytes: u64,
    /// Where captures land — ALREADY RESOLVED to an absolute path by the caller.
    ///
    /// Resolved one level up on the provenance rule this crate already follows
    /// for `discover_live` and `armed_before_producers`: only the caller knows
    /// whether it has a workspace, and a recorder that resolved a relative path
    /// itself would key it to whatever directory it happened to be spawned in.
    pub dir: PathBuf,
    /// The label every verdict this recorder publishes carries — the graph name,
    /// so an operator on a two-graph desk can tell the answers apart.
    pub label: String,
    /// The dashcam caps.
    pub caps: RetentionCaps,
    /// The anti-spam policy.
    pub policy: TriggerPolicy,
    /// The per-trigger POSTURE (decisions 110-B + 112-G) — which triggers
    /// this robot admits at all.
    ///
    /// Carried on the settings rather than read from the environment inside the
    /// plane, on the same provenance rule `dir` follows: the caller resolves it
    /// once, so a test drives a posture without touching process-global state and
    /// the plane cannot disagree with the observer about what is on.
    pub posture: TriggerPosture,
    /// `true` when this recorder holds ONLY the window — no continuous bag.
    ///
    /// The always-on shape. A `--record` run sets it `false` and gets both.
    pub window_only: bool,
    /// The per-TOPIC byte budget a
    /// WINDOW-ONLY recorder's tap queues are sized to
    /// (`CERULION_FLASHBACK_TAP_BUDGET_MB`, 64 MiB default).
    ///
    /// Read ONLY on the window-only path. A `--record` recorder's taps stay
    /// ceiling-deep whatever this says — the mode gate, and the reason this
    /// field cannot make a replay-grade recording lossy by being mis-set.
    ///
    /// It lives on the FLASHBACK settings rather than on `BagdConfig` because
    /// the standing tap footprint is a property of the always-on plane: a
    /// recorder with no plane has no window to bound, and giving it a budget
    /// field would imply the budget applies to it.
    pub tap_budget_bytes: u64,
    /// Topics the WINDOW does not hold
    /// (`CERULION_FLASHBACK_EXCLUDE_TOPICS`).
    ///
    /// The lever an operator buys window SECONDS back with when the duration
    /// knobs are already spent. It governs the window ONLY — a `--record` run
    /// still records every topic it was asked to, because excluding a camera
    /// from the always-on black box and asking for a recording of that camera
    /// are two different decisions.
    pub exclude_topics: cerulion_core::flashback::ExcludeTopics,
}

/// The unconditional counters (Principle #3).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FlashbackStats {
    /// Requests that reached this recorder over the trigger channel.
    pub requests_seen: u64,
    /// Captures this recorder started.
    pub captures_started: u64,
    /// Captures whose bag was finalized.
    pub captures_written: u64,
    /// Captures whose write FAILED.
    pub captures_failed: u64,
    /// Captures REFUSED because one was already being written — see
    /// [`FlashbackPlane::writer_busy`].
    pub captures_deferred: u64,
    /// Captures REFUSED at finalize because the
    /// plane could not hold one whole checkpoint generation.
    ///
    /// Its own counter, apart from `captures_failed` (the recorder breaking) and
    /// `captures_deferred` (a writer already busy). This one says the MACHINE is
    /// too small for a resimmable black box, which is the only one of the three
    /// an operator fixes with a knob rather than with a repair.
    pub captures_refused_not_resimmable: u64,
    /// Captures still being written when the recorder stopped waiting for them.
    ///
    /// Its own counter, apart from `captures_failed`: a capture that ran out of
    /// shutdown budget may well be a COMPLETE bag on disk a moment later, which
    /// is a different fact from one whose write returned an error.
    pub captures_abandoned: u64,
}

/// What a bounded join found.
///
/// A named three-state answer rather than an `Option<Report>`, because "still
/// writing" is NOT "nothing to report" and must not be handled like it: the
/// requester is owed a verdict and the retention sweep must NOT run over a
/// directory holding a file a live thread is still appending to.
pub(crate) enum JoinOutcome {
    /// Nothing was being written.
    Idle,
    /// It finished (or failed) inside the bound.
    Settled(FinishedCaptureReport),
    /// The bound expired and the thread is STILL writing.
    StillWriting {
        /// Where it is being written.
        path: PathBuf,
        /// Everyone owed a verdict.
        requesters: Vec<u64>,
    },
}

/// A capture that has stopped RECORDING — what the recorder needs to write it.
pub(crate) struct ClosedCapture {
    /// What the gate recorded.
    pub finished: FinishedCapture,
    /// The oldest window instant it promised.
    pub floor_ns: u64,
    /// The instant it was TRIGGERED — see [`ActiveCapture::started_ns`].
    pub started_ns: u64,
    /// Where it lands.
    pub path: PathBuf,
    /// Every request id owed a verdict.
    pub requesters: Vec<u64>,
    /// Frames THIS capture lost to the window's byte ceiling.
    pub truncated_frames: u64,
    /// Trace records THIS capture lost to the TRACE retention's byte ceiling.
    ///
    /// Its own number rather than folded into `truncated_frames`: a lost frame
    /// costs the capture some of what it shows, while a lost trace record can
    /// cost it the boundary a resume BEGINS at — different consequences, and an
    /// operator reading one total could not tell which happened.
    pub truncated_trace_records: u64,
}

/// A capture whose bag is finished — what the recorder needs to report it.
pub(crate) struct FinishedCaptureReport {
    /// The capture's sequence number.
    pub seq: u64,
    /// Where it landed.
    pub path: PathBuf,
    /// Whether it is excluded from retention eviction.
    pub pinned: bool,
    /// Whether `bag play --resim` will accept it. `None` is
    /// UNKNOWN, never a silent "no".
    pub resimmable: Option<bool>,
    /// What the capture CLAIMS to cover and what it CARRIES, so
    /// the `Finished` verdict can say "achieved 2.0 s of the claimed 45.0 s" at
    /// the moment of the incident rather than only inside the bag.
    pub span: cerulion_core::flashback::channel::FinishedSpan,
    /// Every request id owed a verdict.
    pub requesters: Vec<u64>,
    /// Every cause this capture recorded, for the sibling
    /// cause marker.
    pub causes: Vec<CaptureCause>,
    /// What the write produced.
    pub result: Result<CaptureStats, String>,
}

/// The capture currently recording (still collecting its post window).
struct ActiveCapture {
    /// The oldest window instant this capture promised to carry.
    ///
    /// (The capture's `seq` and `pinned` are the GATE's, and are read back off
    /// `FinishedCapture` at close — kept in one place so the two cannot drift.)
    floor_ns: u64,
    /// The instant the capture was TRIGGERED — `T`.
    ///
    /// Carried explicitly rather than recovered from the floor, and here
    /// is why. `floor_ns` is `T - window_span` SATURATING,
    /// so on an EARLY capture — one triggered before a whole span has elapsed —
    /// it pins to zero and stops identifying `T` at all. Every quantity derived
    /// from it then describes a trigger that never happened. See
    /// [`claimed_window_start_ns`](FlashbackPlane::claimed_window_start_ns).
    started_ns: u64,
    /// The window's LIFETIME truncation count when this capture opened.
    ///
    /// Its manifest reports the DELTA. The lifetime counter never
    /// resets, so printing it directly meant that once ANY capture lost a frame
    /// to the byte ceiling, every later capture's manifest repeated that total —
    /// labelling bags truncated that lost nothing.
    truncated_at_start: u64,
    /// The TRACE retention's LIFETIME truncation count when this capture opened.
    ///
    /// Snapshotted for exactly the reason its frame twin above is: the counter
    /// never resets, so reporting it directly would label every later capture
    /// truncated once any capture ever lost a record.
    trace_truncated_at_start: u64,
    /// Where it will land — computed ONCE, at the trigger.
    ///
    /// Frozen rather than re-derived, and the e2e found out why: the name
    /// carries a wall-clock stamp (it outlives the process, so a monotonic
    /// reading would restart near zero at boot), and re-deriving it at finalize
    /// produced a DIFFERENT name a second later — so the operator was told a
    /// path the bag never landed at.
    path: PathBuf,
    /// Every request id that asked for it — one `Finished` verdict each.
    requesters: Vec<u64>,
}

/// The recorder's flashback state. See the module docs.
pub struct FlashbackPlane {
    settings: FlashbackSettings,
    window: FrameWindow,
    anchors: SharedAnchors,
    /// The trace retention, fed by whichever thread owns the
    /// trace rings. See [`SharedTraceWindow`].
    trace: SharedTraceWindow,
    gate: FlashbackTriggerGate,
    /// `None` when the channel could not be opened: the window still rolls (a
    /// process-fault trigger from the same process would still work), but
    /// nothing can ASK for a capture. Degraded, never fatal.
    responder: Option<FlashbackResponder>,
    active: Option<ActiveCapture>,
    writing: Option<CaptureWriter>,
    stats: FlashbackStats,
    /// How the plane's budget is currently split.
    ///
    /// Recomputed only when the MEASURED generation changes (see
    /// [`refresh_budget_split`](Self::refresh_budget_split)), so the ordinary
    /// drive pass costs one comparison.
    split: cerulion_core::flashback::PlaneSplit,
    /// The measurement `split` was derived from, so a re-derivation happens once
    /// per change rather than once per pass.
    split_generation: Option<u64>,
}

impl FlashbackPlane {
    /// Build the plane. `responder` is best-effort — see the field docs.
    pub fn new(settings: FlashbackSettings, responder: Option<FlashbackResponder>) -> Self {
        let window = FrameWindow::new(
            settings.window_span.as_nanos() as u64,
            settings.window_max_bytes,
        );
        // The anchor retention covers the SAME span as the frames, and that is
        // the whole window arithmetic: a capture claims `[T−post, T+post]` and
        // needs an anchor at or before `T−post`, which at cadence `C` is at
        // worst `T−post−C` old — so a span of `post + C`
        // (`DEFAULT_FLASHBACK_WINDOW_MS`) is exactly what reaches one. Giving
        // the anchors a shorter span would leave a capture with frames it cannot
        // resume from; a longer one would retain checkpoints older than the
        // oldest frame, which `AnchorWindow::select` refuses anyway.
        let anchors = AnchorWindow::new(
            settings.window_span.as_nanos() as u64,
            settings.anchor_max_bytes,
        );
        // The trace retention covers the SAME span as the frames and the
        // anchors, and for the same reason: a capture's trace must reach back to
        // the boundary AFTER the anchor it carries, and that anchor is at worst
        // one cadence older than the capture's own pre-window start. A shorter
        // span would leave a capture with an anchor it cannot resume from; a
        // longer one would retain boundaries older than any anchor `select` can
        // serve.
        let trace = crate::trace_window::TraceWindow::new(
            settings.window_span.as_nanos() as u64,
            settings.trace_max_bytes,
        );
        let mut gate = FlashbackTriggerGate::with_posture(settings.policy, settings.posture);
        // SEED the gate's rate-and-floor state from what is
        // already on disk, so the anti-spam machinery describes the ROBOT's last
        // hour rather than this recorder process's.
        //
        // Without it a relaunch loop — the shape a fault produces — mints a fresh
        // 20/hr budget on every launch and the cap never engages at all. Read HERE
        // rather than at first-request time because the answer must be in force
        // before the first decision, and a directory scan at that point would put
        // a filesystem walk on the request path.
        //
        // Best-effort, and the direction of the degradation is stated: an
        // unreadable directory seeds NOTHING, which is exactly pre-marker
        // behaviour, never a refusal to capture.
        let (history, now_wall_ns) = read_capture_history(&settings.dir);
        if !history.is_empty() {
            tracing::info!(
                captures = history.len(),
                dir = %settings.dir.display(),
                "flashback: seeding the trigger gate from captures already \
                 on disk — the rolling-hour cap and each cause's refractory floor \
                 survive this recorder's restart"
            );
        }
        gate.seed_from_history(&history, 0, now_wall_ns);
        // The UNMEASURED split, which is exactly the
        // caps that shipped. Nothing has been measured yet, and sizing a reserve
        // before anything has been measured would be inventing the demand.
        let split = cerulion_core::flashback::split_plane_budget(
            settings.window_max_bytes,
            cerulion_core::flashback::ResolvedCap {
                bytes: settings.anchor_max_bytes,
                basis: settings.anchor_cap_basis,
            },
            None,
            cerulion_core::flashback::FLASHBACK_FRAMES_FLOOR_BYTES,
        );
        Self {
            settings,
            window,
            anchors: Arc::new(Mutex::new(anchors)),
            trace: Arc::new(Mutex::new(trace)),
            gate,
            responder,
            active: None,
            writing: None,
            stats: FlashbackStats::default(),
            split,
            split_generation: None,
        }
    }

    /// Re-split the plane's budget ANCHOR-FIRST when the
    /// measured generation has changed.
    ///
    /// # Why this is polled rather than pushed
    ///
    /// Anchors are admitted by whichever thread owns the state rings — the drive
    /// loop on a window-only run, the WRITER thread on a `--record` one — through
    /// a free function holding only the shared retention. Neither has the plane.
    /// Polling the measurement here costs one comparison on a pass that already
    /// locks the retention to evict it, and it means there is ONE place the split
    /// is applied rather than two call sites that could drift.
    ///
    /// The new ceilings take effect at the next eviction, which is the same pass;
    /// see [`FrameWindow::set_max_bytes`](crate::window::FrameWindow::set_max_bytes).
    fn refresh_budget_split(&mut self, measured: Option<u64>) {
        if measured == self.split_generation {
            return;
        }
        self.split_generation = measured;
        self.split = cerulion_core::flashback::split_plane_budget(
            self.settings.window_max_bytes,
            cerulion_core::flashback::ResolvedCap {
                bytes: self.settings.anchor_max_bytes,
                basis: self.settings.anchor_cap_basis,
            },
            measured,
            cerulion_core::flashback::FLASHBACK_FRAMES_FLOOR_BYTES,
        );
        self.window.set_max_bytes(self.split.frames_bytes);
        lock_anchors(&self.anchors).set_max_bytes(self.split.anchor_bytes);
        tracing::debug!(
            measured_generation_bytes = measured.unwrap_or(0),
            anchor_bytes = self.split.anchor_bytes,
            frames_bytes = self.split.frames_bytes,
            verdict = ?self.split.verdict,
            "flashback: re-split the plane budget anchor-first"
        );
    }

    /// How the plane's budget is currently split, and on what evidence.
    pub fn budget_split(&self) -> cerulion_core::flashback::PlaneSplit {
        self.split
    }

    /// The largest COMPLETE generation this run has held, if one has been
    /// measured. `None` is "nothing measured", never "state costs nothing".
    pub fn measured_generation_bytes(&self) -> Option<u64> {
        lock_anchors(&self.anchors).max_generation_bytes()
    }

    /// The largest anchor-shaped thing the ceiling has
    /// taken, if any: a floor on the demand, used by the ceiling-refusal remedy when
    /// no whole generation has ever completed. `None` is "nothing observed".
    pub fn refused_bytes_floor(&self) -> Option<u64> {
        lock_anchors(&self.anchors).refused_bytes_floor()
    }

    /// Does the WINDOW hold this topic?
    ///
    /// Asked by the harvest rather than by the tap set, so a `--record` run
    /// still records an excluded topic into its continuous bag — the exclusion
    /// is a window budget, never a recording filter.
    pub fn excludes_topic(&self, topic: &str) -> bool {
        self.settings.exclude_topics.excludes(topic)
    }

    /// The shared anchor retention — handed to the writer thread when it takes
    /// ownership of the state rings. See [`SharedAnchors`].
    pub(crate) fn anchors(&self) -> SharedAnchors {
        Arc::clone(&self.anchors)
    }

    /// The shared trace retention — handed to the writer thread with the trace
    /// rings. See [`SharedTraceWindow`].
    pub(crate) fn trace(&self) -> SharedTraceWindow {
        Arc::clone(&self.trace)
    }

    /// Roll the trace retention forward, protecting an active capture's floor.
    ///
    /// Its own call for the reason [`evict_anchors`](Self::evict_anchors) is:
    /// the three retentions are filled by different owners on different threads,
    /// and a caller that could only evict all three would hold every lock on
    /// every frame pass.
    pub(crate) fn evict_trace(&mut self, now_ns: u64) {
        let floor = self.active.as_ref().map(|a| a.floor_ns);
        lock_trace(&self.trace).evict(now_ns, floor);
    }

    /// Trace records the retention is holding.
    pub fn trace_records(&self) -> u64 {
        lock_trace(&self.trace).records()
    }

    /// Bytes the trace retention is holding.
    pub fn trace_bytes(&self) -> usize {
        lock_trace(&self.trace).bytes()
    }

    /// Trace records aged out, lifetime — the ordinary number.
    pub fn trace_aged(&self) -> u64 {
        lock_trace(&self.trace).aged()
    }

    /// How far back the TRACE retention currently reaches, at `now_ns`.
    ///
    /// `None` when it holds nothing. That is now ORDINARILY
    /// TRANSIENT — a window recorder on a multi-process run is handed the run's
    /// rings, so it reaches back as soon as the ranks fire — and it is PERMANENT
    /// only where the run minted no ring at all (a wall-gated shape, or a run
    /// refused its rings). Either way it is exactly the state an operator needs
    /// to be able to see rather than infer from a capture that turned out not to
    /// be resimmable. MEASURED rather than
    /// assumed from the span, for the reason [`coverage_ms`](Self::coverage_ms)
    /// is.
    pub fn trace_coverage_ms(&self, now_ns: u64) -> Option<u64> {
        lock_trace(&self.trace)
            .oldest_ns()
            .map(|oldest| now_ns.saturating_sub(oldest) / 1_000_000)
    }

    /// Trace records the BYTE ceiling took, lifetime.
    ///
    /// Reported apart from ageing for the reason the frame window's twin is: one
    /// is the retention working, the other is a capture losing the boundary a
    /// resume begins at.
    pub fn trace_truncated(&self) -> u64 {
        lock_trace(&self.trace).truncated()
    }

    /// The kept trace for a capture closing now, trimmed to `anchor_step`.
    ///
    /// Built over `snapshot`, the records [`snapshot_trace_from`](Self::snapshot_trace_from)
    /// copied out of the retention at the TOP of the close — before the taps
    /// were drained and the frame window harvested — rather than over a fresh
    /// read of the live retention here, which the drain thread may have pushed
    /// past the frames in the meantime (the ordering is pinned by
    /// `the_capture_close_snapshots_the_trace_before_the_taps_are_drained_and_the_frames_read`).
    /// The copy already travels to the capture WRITER THREAD for the reasons
    /// [`select_anchor`](Self::select_anchor) states: that thread cannot hold a
    /// lock the drive loop needs, and the retention keeps rolling while the
    /// write runs.
    ///
    /// The snapshot was taken from the capture's own frame floor, so the trace
    /// it carries and the frames it carries begin at the same instant.
    pub(crate) fn select_trace(
        &self,
        snapshot: &[cerulion_core::trace_ring::TraceRingRecord],
        anchor_step: u64,
        node_ids: &[String],
    ) -> crate::trace_window::TrimmedTrace {
        let held = lock_trace(&self.trace);
        let mut trimmed =
            crate::trace_window::trim_to_anchor(snapshot.iter().copied(), anchor_step, node_ids);
        // The trim reads `target(S-1)` off the anchor step's own
        // boundary record ON ITS WAY PAST — which only works while that record
        // is among the ones the FRAME floor offers it. A boundary drained a pass
        // before its own anchor sits below the floor and is never offered, and
        // the anchor band's lower edge IS the floor, so the two straddling it is
        // an ordinary interleaving. Recovered from the whole retention here
        // rather than by widening the trim's offer, which would also change what
        // the capture carries — see `TraceWindow::boundary_target_ns`.
        //
        // Still `None` when the retention genuinely does not hold that boundary
        // (the ceiling took it, or the capture predates it), which is the correct
        // answer the manifest and the replay both already handle.
        if trimmed.anchor_target_ns.is_none() {
            trimmed.anchor_target_ns = held.boundary_target_ns(anchor_step);
        }
        trimmed
    }

    /// The kept trace for a capture that has NO anchor to trim to.
    ///
    /// A black box keeps its evidence: an operator can still read what fired,
    /// and the manifest's own verdict is what stops an untrimmed trace being
    /// read as a resume promise.
    ///
    /// Deliberately NOT `select_trace` with a made-up anchor step. Every step
    /// number is a real step: passing 0 would discard the run's own step 0, and
    /// passing `u64::MAX` would discard everything. "There is no anchor" is a
    /// third thing, so it gets its own function rather than a sentinel a reader
    /// has to decode.
    pub(crate) fn select_trace_untrimmed(
        &self,
        snapshot: &[cerulion_core::trace_ring::TraceRingRecord],
        node_ids: &[String],
    ) -> crate::trace_window::TrimmedTrace {
        crate::trace_window::keep_all(snapshot.iter().copied(), node_ids)
    }

    /// Roll the anchor retention forward, protecting an active capture's floor.
    ///
    /// Separate from [`evict`](Self::evict) rather than folded into it, because
    /// the two retentions are drained by different owners in different modes and
    /// a caller that could only evict BOTH would have to hold the anchor lock on
    /// every frame pass.
    pub(crate) fn evict_anchors(&mut self, now_ns: u64) {
        // Re-split BEFORE evicting, so a reserve that has
        // just grown is in force for this very pass. The other order would evict
        // against the OLD, smaller ceiling and drop the checkpoint that grew.
        //
        // Read under its own lock and released, because `refresh_budget_split`
        // takes the lock itself to apply the new ceiling — holding it across
        // both would need a re-entrant lock this crate does not use.
        let measured = lock_anchors(&self.anchors).max_generation_bytes();
        self.refresh_budget_split(measured);
        let floor = self.active.as_ref().map(|a| a.floor_ns);
        lock_anchors(&self.anchors).evict(now_ns, floor);
    }

    /// Checkpoints the anchor retention is holding.
    pub fn anchor_checkpoints(&self) -> usize {
        lock_anchors(&self.anchors).checkpoints()
    }

    /// Bytes the anchor retention is holding.
    pub fn anchor_bytes(&self) -> usize {
        lock_anchors(&self.anchors).bytes()
    }

    /// Anchors admitted to the retention, lifetime.
    pub fn anchors_admitted(&self) -> u64 {
        lock_anchors(&self.anchors).admitted()
    }

    /// Checkpoints the retention aged out, lifetime — the ordinary number, and
    /// the one that says the retention is rolling at all.
    pub fn anchors_aged(&self) -> u64 {
        lock_anchors(&self.anchors).aged()
    }

    /// How far back the CHECKPOINT retention currently reaches, at `now_ns`.
    ///
    /// `None` when it holds nothing. MEASURED rather than assumed from the span,
    /// for the reason [`coverage_ms`](Self::coverage_ms) is: a plane whose
    /// anchors have not started arriving, or whose byte ceiling is biting,
    /// reaches back less far than it promises — and unlike the frame window this
    /// one can legitimately be empty for a whole cadence after arming.
    pub fn anchor_coverage_ms(&self, now_ns: u64) -> Option<u64> {
        lock_anchors(&self.anchors)
            .oldest_ns()
            .map(|oldest| now_ns.saturating_sub(oldest) / 1_000_000)
    }

    /// Times the BYTE CEILING refused or dropped a checkpoint, from either site
    /// — the one number that answers "did the ceiling bite at all", and the
    /// evidence the ceiling-refusal standing alarm carries.
    pub fn anchor_ceiling_refusals(&self) -> u64 {
        lock_anchors(&self.anchors).ceiling_refusals()
    }

    /// Record every node one state ring's manifest declares.
    ///
    /// Called when the ring is OPENED, so a rank that never closes an anchor is
    /// still known to have been expected — see `AnchorWindow::declared_nodes`.
    pub(crate) fn declare_ring_nodes(&self, ring: &str, nodes: &[String]) {
        lock_anchors(&self.anchors).declare_ring_nodes(ring, nodes);
    }

    /// The declared node table, for a capture to judge its checkpoint against.
    pub(crate) fn declared_ring_nodes(&self) -> std::collections::BTreeMap<String, Vec<String>> {
        lock_anchors(&self.anchors).declared_nodes()
    }

    /// Checkpoints the anchor retention's BYTE ceiling took, lifetime — the
    /// degradation counter, reported apart from ordinary ageing because one
    /// is the retention working and the other is a capture losing its resume
    /// point.
    pub fn anchors_truncated(&self) -> u64 {
        lock_anchors(&self.anchors).truncated()
    }

    /// Choose the checkpoint a capture closing at `now_ns` should carry, and
    /// clone it out of the retention.
    ///
    /// CLONED rather than borrowed because the records travel to the capture
    /// WRITER THREAD, which cannot hold a lock the drive loop needs; and because
    /// the retention keeps rolling while that write runs, so a borrow would pin
    /// the whole buffer for the length of a ~155 MB write.
    ///
    /// `floor_ns` is the capture's own FROZEN frame floor, and it is the ONLY
    /// clock input — see [`claimed_window_start_ns`](Self::claimed_window_start_ns).
    ///
    /// # The deadline can never precede the OLDEST FRAME
    ///
    /// The claimed pre-window start is `T − post_window`, which assumes the bag
    /// holds frames that far back. It does not when the window is configured
    /// SHORTER than the post window — `CERULION_FLASHBACK_WINDOW_MS` accepts any
    /// positive value, and the shipped post window is 15 s — because the floor is
    /// `T − window_span`. The deadline then lands EARLIER than the floor, and the
    /// two conditions `select` applies (at or after the floor, at or before the
    /// deadline) become unsatisfiable: no checkpoint could be reported as
    /// covering, INCLUDING one taken exactly at the floor, which covers every
    /// frame the capture carries. A field whose covering arm is unreachable by
    /// construction carries no information.
    ///
    /// So the deadline is CLAMPED to the floor here, at the one place both
    /// numbers are in hand. It is a definitional correction rather than a
    /// loosening: a capture claims `window_span` of pre-incident frames, so with
    /// a short window the start of the window it actually claims IS the floor,
    /// and `T − post` was quoting a promise it never made. Where the window is at
    /// least the post window — every shipped configuration — `floor <= T − post`
    /// and the clamp is a no-op.
    ///
    /// SCOPE: for a checkpoint strictly NEWER than the floor the old label
    /// was already true (a resume from it re-executes less than the bag's frames
    /// hold), so what this changes is the boundary and the reachability of the
    /// covering arm — not the shortfall reporting, which
    /// [`AnchorReport::Embedded::frames_before_anchor_ms`] states exactly either
    /// way.
    ///
    /// The clamp lives HERE and not in `claimed_window_start_ns`, which stays a
    /// pure function of the frozen trigger: what the capture CLAIMED and what its
    /// frames REACH are two different facts, and folding the second into the
    /// first would leave nothing able to state the first.
    pub(crate) fn select_anchor(
        &self,
        floor_ns: u64,
        started_ns: u64,
    ) -> Result<(Checkpoint, AnchorFit), NoAnchorReason> {
        let deadline = self.claimed_window_start_ns(started_ns).max(floor_ns);
        lock_anchors(&self.anchors)
            .select(floor_ns, deadline)
            .map(|(c, fit)| (c.clone(), fit))
    }

    /// PURE: the instant the capture's CLAIMED pre-window starts, derived
    /// entirely from the capture's FROZEN floor.
    ///
    /// # The defect this avoids
    ///
    /// Computing it as `now_ns − post_window` with `now_ns` the CLOSE time is wrong. A
    /// capture closes at `T + post`, so that deadline is `T` — one whole post
    /// window LATE. Every checkpoint taken between `T − post` and `T` would then
    /// satisfy it and be reported `covers_the_claimed_window`, which is a
    /// claim the capture cannot back: a resume from `T − 2 s` covers the last two
    /// seconds before the incident, not the fifteen the bag says it holds.
    ///
    /// The floor is FROZEN at `begin_capture` (`T − window_span`), so
    /// `T = floor + window_span` and the deadline is
    /// `floor + (window_span − post_window)` — no close time, no live clock, and
    /// therefore nothing that can drift between the trigger and the finalize.
    ///
    /// # Why the CLAIMED window and not the frame floor
    ///
    /// `floor_ns` itself is the obvious alternative. That is stricter and it is
    /// the wrong bound, for a reason the shipped constants make concrete:
    /// `DEFAULT_FLASHBACK_WINDOW_MS` is sized as `post + cadence` precisely so
    /// that an anchor at or before `T − post` is REACHABLE, and it guarantees
    /// nothing at or before `T − window_span`. Under `floor_ns` the
    /// `CoversTheClaimedWindow` arm would essentially never fire — every capture
    /// would report the degraded label — and a field that always reads the same
    /// carries no information.
    ///
    /// The concern behind that thread is REAL, though, and is answered by
    /// reporting rather than by tightening: the bag holds `window_span` of frames
    /// while a resume covers from the anchor onward, so
    /// [`AnchorReport::Embedded::frames_before_anchor_ms`] states exactly how much
    /// of the bag's frame span predates the resume point.
    ///
    /// SATURATING both ways. An operator may also configure a window no larger
    /// than the post window, which puts this instant BEFORE the oldest frame the
    /// capture holds — the earlier `floor + (span − post)` expression saturated
    /// to the floor there, and the rewrite below does not. That is
    /// deliberate and the bound is restored where it belongs, at the point of
    /// use: see [`select_anchor`](Self::select_anchor), which clamps the deadline
    /// to the capture's floor. This function answers only "what did the capture
    /// CLAIM", which is a fact about the trigger and the policy alone.
    ///
    /// # Nor from the FLOOR, because the floor SATURATES
    ///
    /// The first fix derived it as `floor + (window_span - post_window)`, on
    /// the reasoning that `floor == T - window_span` makes that identically
    /// `T - post` with no live clock. That identity holds only while the
    /// subtraction does not saturate, and `begin_capture` computes the floor
    /// with `saturating_sub`.
    ///
    /// So on an EARLY capture — triggered before a whole span has elapsed — the
    /// floor pins to ZERO and the deadline becomes `window_span - post_window`,
    /// which is LATER than the trigger itself. At the shipped constants with a
    /// trigger at `T = 5 s`: floor 0, deadline 15 s, and a checkpoint taken at
    /// 10 s — five seconds AFTER the incident — reported as covering the
    /// pre-incident window. That is not a corner: an always-on black box exists
    /// to catch the fault that happens in a run's first seconds, so the early
    /// capture is a shape it must get right.
    ///
    /// Carrying `T` explicitly is both simpler and total. It stays FROZEN (taken
    /// at `begin_capture`), so the first fix's property — no clock read between the
    /// trigger and the finalize — is preserved, and the answer no longer depends
    /// on `window_span` at all.
    pub(crate) fn claimed_window_start_ns(&self, started_ns: u64) -> u64 {
        started_ns.saturating_sub(self.settings.policy.post_window_ns)
    }

    /// The settings in force.
    pub fn settings(&self) -> &FlashbackSettings {
        &self.settings
    }

    /// The lifetime counters.
    pub fn stats(&self) -> FlashbackStats {
        self.stats
    }

    /// The gate's own lifetime counters.
    pub fn trigger_stats(&self) -> TriggerStats {
        self.gate.stats()
    }

    /// Frames the window is holding.
    pub fn window_frames(&self) -> u64 {
        self.window.frames()
    }

    /// Bytes the window is holding.
    pub fn window_bytes(&self) -> usize {
        self.window.bytes()
    }

    /// Frames the byte ceiling took from a capture, lifetime.
    pub fn truncated_frames(&self) -> u64 {
        self.window.truncated_frames()
    }

    /// Frames the window dropped for AGE, lifetime.
    ///
    /// The ordinary number — a window that never aged anything out never
    /// reached its span. Reported beside `truncated_frames` precisely so the two
    /// are read apart: one is the feature working, the other is a capture
    /// covering less than it claims.
    pub fn aged_frames(&self) -> u64 {
        self.window.aged_frames()
    }

    /// How far back the window currently reaches, at `now_ns`.
    ///
    /// `None` on an empty window. This is the operator-facing answer to "is the
    /// black box actually holding anything?", and it is deliberately measured
    /// rather than assumed from the span: a recorder that just armed, or one
    /// whose byte ceiling is biting, reaches back less far than it promises.
    pub fn coverage_ms(&self, now_ns: u64) -> Option<u64> {
        self.window
            .oldest_ns()
            .map(|oldest| now_ns.saturating_sub(oldest) / 1_000_000)
    }

    /// `true` when this recorder writes no continuous bag.
    pub fn window_only(&self) -> bool {
        self.settings.window_only
    }

    /// Take one tap's drained batch into the window.
    pub(crate) fn harvest(
        &mut self,
        topic_idx: usize,
        taken_at_ns: u64,
        frames: crate::StagedFrames,
    ) {
        self.window.push(topic_idx, taken_at_ns, frames);
    }

    /// Roll the window forward, protecting an active capture's floor.
    pub(crate) fn evict(&mut self, now_ns: u64) {
        let floor = self.active.as_ref().map(|a| a.floor_ns);
        self.window.evict(now_ns, floor);
    }

    /// Requests waiting on the trigger channel. Empty when the channel could not
    /// be opened.
    pub fn drain_requests(&mut self) -> Vec<FlashbackRequestFrame> {
        // Drained ONCE and counted from what came back. The channel is a queue:
        // a second call to `drain_requests` would consume a DIFFERENT (empty)
        // set, so counting and returning must read the same vector.
        let frames = self
            .responder
            .as_ref()
            .map(|r| r.drain_requests())
            .unwrap_or_default();
        self.stats.requests_seen += frames.len() as u64;
        frames
    }

    /// Publish a verdict for `request_id`. A no-op when the channel is absent.
    pub fn publish_outcome(&self, request_id: u64, outcome: FlashbackOutcome) {
        if let Some(responder) = self.responder.as_ref() {
            responder.publish_outcome(request_id, outcome);
        }
    }

    /// Ask the gate.
    pub fn decide(
        &mut self,
        request: &cerulion_core::flashback::trigger::CaptureRequest,
        now_ns: u64,
    ) -> TriggerDecision {
        self.gate.decide(request, now_ns)
    }

    /// Tell the gate a cause's condition CLEARED, closing
    /// its regime so a later recurrence captures again.
    ///
    /// Returns how many requests the regime swallowed while it was open, or
    /// `None` when there was no open regime to close (an ordinary answer — a
    /// monitors verdict clears on every row that heals, including rows that
    /// never raised anything this run).
    ///
    /// Delegated rather than exposing the gate, on the same rule as
    /// [`decide`](Self::decide): the gate is this plane's state, and a caller
    /// holding it directly could open a regime the plane's own accounting never
    /// sees.
    pub fn recover(
        &mut self,
        kind: cerulion_core::flashback::trigger::TriggerKind,
        subject: &str,
    ) -> Option<u64> {
        self.gate.recover(kind, subject)
    }

    /// Note that `request_id` is waiting on the ACTIVE capture's outcome.
    pub fn attach_requester(&mut self, request_id: u64) {
        if let Some(active) = self.active.as_mut() {
            if !active.requesters.contains(&request_id) {
                active.requesters.push(request_id);
            }
        }
    }

    /// Open a capture at `now_ns`, remembering the window floor it promised.
    pub fn begin_capture(&mut self, now_ns: u64, path: PathBuf, request_id: Option<u64>) {
        self.stats.captures_started += 1;
        self.active = Some(ActiveCapture {
            path,
            truncated_at_start: self.window.truncated_frames(),
            trace_truncated_at_start: self.trace_truncated(),
            // The floor is the whole pre-window, computed ONCE at the trigger and
            // then frozen: recomputing it per pass would let the horizon walk
            // forward under the capture and quietly shorten the bag it promised.
            floor_ns: now_ns.saturating_sub(self.window.span_ns()),
            started_ns: now_ns,
            requesters: request_id.into_iter().collect(),
        });
    }

    /// Is the active capture due to stop recording?
    pub fn capture_due(&self, now_ns: u64) -> bool {
        self.active.is_some() && self.gate.capture_due_to_end(now_ns)
    }

    /// Is a previous capture still being WRITTEN?
    ///
    /// One writer at a time, deliberately: two concurrent ~155 MB writes on a
    /// Jetson's eMMC contend for exactly the bandwidth the second one needs, and
    /// a plane that could start N of them turns a fault burst into a disk stall.
    /// The rate cap already bounds captures per hour; this bounds them at once.
    pub fn writer_busy(&self) -> bool {
        self.writing.is_some()
    }

    /// Where the active capture will land, if one is recording.
    pub fn active_path(&self) -> Option<&Path> {
        self.active.as_ref().map(|a| a.path.as_path())
    }

    /// The active capture's frozen window floor, if one is recording — the
    /// `floor_ns` `finish_capture` will hand back, readable BEFORE the capture
    /// is finished so the close can snapshot the trace from it first.
    pub(crate) fn active_floor_ns(&self) -> Option<u64> {
        self.active.as_ref().map(|a| a.floor_ns)
    }

    /// Every retained trace record at or after `floor_ns`, COPIED
    /// out of the retention at this instant.
    ///
    /// The capture close reads this BEFORE it drains the taps and harvests the
    /// frame window, so the frames it then reads are never older than the
    /// trace it carries — see `close_capture` for the race this closes. The
    /// copy is the one `select_trace` would take under the lock anyway; it
    /// is taken earlier, not additionally.
    pub(crate) fn snapshot_trace_from(
        &self,
        floor_ns: u64,
    ) -> Vec<cerulion_core::trace_ring::TraceRingRecord> {
        lock_trace(&self.trace)
            .records_from(floor_ns)
            .copied()
            .collect()
    }

    /// The causes the active capture has recorded.
    pub fn active_causes(&self) -> Vec<CaptureCause> {
        self.gate.active_causes()
    }

    /// Close the active capture, handing back what it recorded.
    pub(crate) fn finish_capture(&mut self) -> Option<ClosedCapture> {
        let active = self.active.take()?;
        let finished = self.gate.finish_capture()?;
        Some(ClosedCapture {
            // THIS capture's truncation, not the plane's lifetime total.
            truncated_frames: self
                .window
                .truncated_frames()
                .saturating_sub(active.truncated_at_start),
            truncated_trace_records: self
                .trace_truncated()
                .saturating_sub(active.trace_truncated_at_start),
            finished,
            floor_ns: active.floor_ns,
            started_ns: active.started_ns,
            path: active.path,
            requesters: active.requesters,
        })
    }

    /// Adopt a writer that is now producing the capture's bag.
    pub(crate) fn set_writer(&mut self, writer: CaptureWriter) {
        self.writing = Some(writer);
    }

    /// Note a capture that was decided but could not be started.
    /// Book a capture refused at finalize.
    pub fn note_capture_refused(&mut self) {
        self.stats.captures_refused_not_resimmable += 1;
    }

    pub fn note_capture_deferred(&mut self) {
        self.stats.captures_deferred += 1;
    }

    /// BLOCK for the writer, bounded — the shutdown path's twin of
    /// [`poll_writer`](Self::poll_writer).
    pub(crate) fn join_writer(&mut self, deadline: Duration) -> JoinOutcome {
        let Some(writing) = self.writing.as_mut() else {
            return JoinOutcome::Idle;
        };
        let Some(result) = writing.join_bounded(deadline) else {
            // STILL WRITING. The writer stays in the plane: taking it here would
            // drop its `JoinHandle` and detach a live thread.
            self.stats.captures_abandoned += 1;
            return JoinOutcome::StillWriting {
                path: writing.path.clone(),
                requesters: writing.requesters.clone(),
            };
        };
        let Some(writer) = self.writing.take() else {
            return JoinOutcome::Idle;
        };
        // The job is dropped with the writer, so the batches it
        // pinned are freed and the window's gauge no longer describes anything.
        self.window.note_writer_released();
        match &result {
            Ok(_) => self.stats.captures_written += 1,
            Err(_) => self.stats.captures_failed += 1,
        }
        JoinOutcome::Settled(FinishedCaptureReport {
            seq: writer.seq,
            path: writer.path,
            pinned: writer.pinned,
            resimmable: writer.resimmable,
            span: writer.span,
            requesters: writer.requesters,
            causes: writer.causes,
            result,
        })
    }

    /// The sequence of a capture still being written, if one is.
    ///
    /// Read AFTER a `StillWriting` verdict so the terminal failure can settle the
    /// acceptance it belongs to (the capture identity work) instead of leaving
    /// the requester waiting out its own deadline for a capture nobody will
    /// report again.
    pub(crate) fn abandoned_seq(&self) -> Option<u64> {
        self.writing.as_ref().map(|w| w.seq)
    }

    /// Poll the writer for a finished capture.
    ///
    /// A named struct rather than a five-tuple: three of
    /// its members are a `u64`, a `bool` and a `PathBuf`, so a swap at the one
    /// call site would compile and would publish the wrong verdict to the wrong
    /// requester (the `FinishedCapture` reasoning, applied one layer out).
    pub(crate) fn poll_writer(&mut self) -> Option<FinishedCaptureReport> {
        let result = self.writing.as_mut()?.poll()?;
        let writer = self.writing.take()?;
        // See `join_writer` — the job's `Arc`s die with it.
        self.window.note_writer_released();
        match &result {
            Ok(_) => self.stats.captures_written += 1,
            Err(_) => self.stats.captures_failed += 1,
        }
        Some(FinishedCaptureReport {
            seq: writer.seq,
            path: writer.path,
            pinned: writer.pinned,
            resimmable: writer.resimmable,
            span: writer.span,
            requesters: writer.requesters,
            causes: writer.causes,
            result,
        })
    }

    /// Every held batch at or after `from_ns`, oldest first.
    pub(crate) fn batches_from(
        &self,
        from_ns: u64,
    ) -> impl Iterator<Item = &crate::window::WindowBatch> {
        self.window.batches_from(from_ns)
    }

    /// The oldest instant a capture from `from_ns` will ACTUALLY carry,
    /// or `None` when it will carry nothing.
    ///
    /// The measured twin of the CLAIMED floor. Front-only eviction is what makes
    /// the pair meaningful: everything the window still holds from here on is a
    /// CONTIGUOUS tail, so `[achieved_from, ended]` is a complete range rather
    /// than a hopeful one — accurate up to one drive pass at the boundary, which
    /// is the same bound `harvest_window`'s cadence puts on every other number
    /// here.
    pub fn achieved_from_ns(&self, from_ns: u64) -> Option<u64> {
        self.window.first_stamp_from(from_ns)
    }

    /// How many of `topic_count` taps contribute NO frame
    /// to `[from_ns, ..]`.
    ///
    /// O(#batches) — a capture-close question, never a status field.
    pub fn topics_with_no_frames(&self, from_ns: u64, topic_count: usize) -> usize {
        self.window.topics_with_no_frames(from_ns, topic_count)
    }

    /// The window's byte ceiling.
    pub fn window_cap_bytes(&self) -> usize {
        self.window.max_bytes()
    }

    /// Bytes an in-flight capture writer still holds that the
    /// window has already evicted.
    ///
    /// The accounting that decision D16(a) requires to be stated. Sharing the
    /// window's batches with the writer made a capture's close O(1) instead of a
    /// memcpy of the whole window, and the price is that eviction can no longer
    /// RECLAIM a batch the writer borrowed: dropping it from the window drops one
    /// reference, not the allocation. Peak memory is unchanged — the deep copy
    /// this replaced held the same bytes twice over the same span — but those
    /// bytes stop counting against [`window_cap_bytes`](Self::window_cap_bytes)
    /// while still being resident, so a reader comparing
    /// [`window_bytes`](Self::window_bytes) against the cap would under-report
    /// the plane's real footprint by exactly this number.
    ///
    /// A GAUGE: it rises while a capture is being written and returns to zero
    /// when that writer settles, however it settled.
    pub fn writer_held_bytes(&self) -> usize {
        self.window.writer_held_bytes()
    }

    /// Frames the byte ceiling took with NO capture
    /// active, lifetime.
    ///
    /// The standing cap-bite, apart from [`aged_frames`](Self::aged_frames)
    /// (which also carries it) and from
    /// [`truncated_frames`](Self::truncated_frames) (which is what a CAPTURE
    /// lost). This is the number that says the byte ceiling — not the span — is
    /// what bounds this recorder's look-back.
    pub fn cap_evicted_frames(&self) -> u64 {
        self.window.cap_evicted_frames()
    }

    /// Drop the window AND the anchor retention. Called at teardown, once
    /// nothing can want them.
    ///
    /// Both, in one call: they are one standing cost and one lifetime, and a
    /// teardown that freed the frames while leaving hundreds of megabytes of
    /// checkpoints alive would be a leak nothing observes.
    pub fn clear_window(&mut self) {
        self.window.clear();
        lock_anchors(&self.anchors).clear();
        lock_trace(&self.trace).clear();
    }
}

/// The marker beside a PINNED capture.
///
/// A sibling file rather than a field inside the bag, for one reason: the
/// retention sweep must be able to see the pin WITHOUT opening and parsing every
/// `.mcap` in the directory, and an operator must be able to pin a capture after
/// the fact (`touch foo.mcap.pin`) or unpin it (`rm`) without rewriting a bag
/// whose whole value is that nothing has touched it.
pub(crate) const PIN_MARKER_SUFFIX: &str = ".pin";

/// The marker beside a capture recording WHAT IT WAS ABOUT.
///
/// A sibling file for the SAME reason the pin is one, and it is the reason both
/// halves of that design are affordable at all: diversity-preserving eviction needs a
/// class per capture and cross-restart gate seeding needs every cause's regime
/// key, and a sweep that had to OPEN and parse each `.mcap` to learn them would
/// pay a bag parse per capture on every finalize — on the robot, for policy.
///
/// # Format: one cause per line, `<kind wire word>\t<subject>`
///
/// Line-oriented rather than JSON because the whole file is read by ONE parser
/// on a path that must never fail a recording, and a line is recoverable
/// independently: a truncated write costs the causes after the tear, never the
/// file. The FIRST line is the PRIMARY cause — the request that opened the
/// capture — which is what
/// [`CaptureEntry::cause`](cerulion_core::flashback::retention::CaptureEntry::cause)
/// carries.
///
/// A subject may not contain a tab or a newline (no shipping subject does: the
/// grammars are `{graph}/{group}` and `{condition}:{robot|local}:{topic}`), and a
/// line that breaks that is DROPPED rather than guessed at — a mis-split subject
/// is a regime key that matches nothing, which is silently worse than one cause
/// missing from a bounded list.
pub(crate) const CAUSE_MARKER_SUFFIX: &str = ".cause";

/// Mark a capture as excluded from eviction. Best-effort: a pin that cannot be
/// written is one loud line, because the capture itself is already safely on
/// disk and refusing at this point would achieve nothing.
pub(crate) fn write_pin_marker(bag: &std::path::Path) {
    let marker = pin_marker_path(bag);
    if let Err(e) = std::fs::write(&marker, b"") {
        tracing::warn!(
            path = %marker.display(),
            error = %e,
            "flashback: could not write the pin marker — this capture is NOT protected \
             from retention eviction"
        );
    }
}

/// The pin marker belonging to `bag`.
pub(crate) fn pin_marker_path(bag: &std::path::Path) -> std::path::PathBuf {
    let mut name = bag.as_os_str().to_os_string();
    name.push(PIN_MARKER_SUFFIX);
    std::path::PathBuf::from(name)
}

/// The cause marker belonging to `bag`.
pub(crate) fn cause_marker_path(bag: &std::path::Path) -> std::path::PathBuf {
    let mut name = bag.as_os_str().to_os_string();
    name.push(CAUSE_MARKER_SUFFIX);
    std::path::PathBuf::from(name)
}

/// PURE: render a finished capture's causes as its marker's body.
///
/// Split from the write so the FORMAT is oracle-testable with no filesystem —
/// the same split `plan_retention` and its sweep already use, applied to a file
/// whose two readers (the eviction class and the gate seed) must agree with the
/// writer forever.
pub(crate) fn render_cause_marker(causes: &[CaptureCause]) -> String {
    let mut out = String::new();
    for cause in causes {
        // See `CAUSE_MARKER_SUFFIX`: a subject carrying the separators cannot be
        // written back unambiguously, so it is dropped rather than mangled.
        if cause.subject.contains('\t') || cause.subject.contains('\n') {
            continue;
        }
        out.push_str(cause.kind.as_wire());
        out.push('\t');
        out.push_str(&cause.subject);
        out.push('\n');
    }
    out
}

/// PURE: ONE marker line, when this build recognises it.
fn parse_cause_line(line: &str) -> Option<(TriggerKind, String)> {
    let (kind, subject) = line.split_once('\t')?;
    let kind = TriggerKind::ALL.into_iter().find(|k| k.as_wire() == kind)?;
    Some((kind, subject.to_string()))
}

/// PURE: [`render_cause_marker`]'s reader — every cause the marker records, in
/// order.
///
/// TOLERANT by construction, because this file is policy input and never
/// evidence: an unreadable line is SKIPPED, so a marker written by a newer
/// recorder that added a kind still yields every cause this build understands
/// rather than nothing. The cost is that an unknown kind's floor is not restored,
/// which is bounded at one redundant capture.
///
/// # This is the GATE-SEED reader, and it must not be used to pick the primary
///
/// Skipping is right for seeding — a cause this build cannot name simply seeds no
/// floor — and WRONG for the eviction class, because the skip silently PROMOTES
/// whichever cause happens to be next. See [`parse_primary_cause`].
pub(crate) fn parse_cause_marker(body: &str) -> Vec<(TriggerKind, String)> {
    body.lines().filter_map(parse_cause_line).collect()
}

/// PURE: the marker's PRIMARY cause, its eviction class.
///
/// # The FIRST PHYSICAL LINE, or nothing
///
/// The primary cause is defined positionally: it is the request that OPENED the
/// capture, which [`render_cause_marker`] writes first. So this reads line one and
/// answers `None` if line one is not a cause this build recognises — it never
/// looks further down the file.
///
/// Taking the first line of [`parse_cause_marker`]'s output instead would inherit
/// that reader's SKIP, and the skip is exactly wrong here: a marker written by a
/// NEWER recorder whose primary is a kind this build has never heard of, followed
/// by an ordinary coalesced cause, would promote the COALESCED one to the eviction
/// class. Diversity eviction would then count that capture into a class it does
/// not belong to and target the wrong class's captures — quietly, on the one
/// deployment shape (a mixed-version rollout) where nobody is looking for it.
///
/// `None` is the SAME degradation an absent or unreadable `.cause` already takes:
/// the capture joins the UNKNOWN class, which is one class of its own and
/// behaves exactly as oldest-first did. An explicit unknown, never a guess.
pub(crate) fn parse_primary_cause(body: &str) -> Option<TriggerKind> {
    parse_cause_line(body.lines().next()?).map(|(kind, _)| kind)
}

/// Record what a capture was about, beside the capture.
///
/// Best-effort on the same rule as the pin: the bag is already safely on disk, so
/// refusing at this point would achieve nothing. What is LOST is stated rather
/// than implied — without the marker the capture joins the UNKNOWN eviction class
/// and its causes seed no floor across a restart, which is exactly pre-marker
/// behaviour for that one capture.
pub(crate) fn write_cause_marker(bag: &std::path::Path, causes: &[CaptureCause]) {
    if causes.is_empty() {
        return;
    }
    let marker = cause_marker_path(bag);
    if let Err(e) = std::fs::write(&marker, render_cause_marker(causes)) {
        tracing::warn!(
            path = %marker.display(),
            error = %e,
            "flashback: could not write the cause marker — this capture \
             will be evicted as an UNKNOWN class and its causes will not seed the \
             trigger gate after a restart"
        );
    }
}

/// PURE: read a directory listing into the retention policy's input.
///
/// Split from the sweep on the `plan_discovery` / `plan_connect_set` precedent:
/// the DECISION is `cerulion_core::flashback::retention::plan_retention`, which
/// is oracle-tested with no filesystem, and everything here is either I/O or the
/// mapping between the two.
///
/// `created_ns` is the file's MODIFICATION time — a wall-derived stamp that
/// survives a reboot, which is exactly what
/// [`CaptureEntry::created_ns`](cerulion_core::flashback::retention::CaptureEntry::created_ns)
/// requires and why it cannot be the monotonic clock the trigger gate runs on.
/// A file whose metadata cannot be read is SKIPPED rather than defaulted to
/// zero: a zero would sort it oldest and make it the first thing evicted, i.e.
/// an unreadable stat would silently delete captures.
pub(crate) fn scan_capture_dir(
    dir: &std::path::Path,
) -> Option<Vec<cerulion_core::flashback::retention::CaptureEntry>> {
    use cerulion_core::flashback::retention::CaptureEntry;

    let Ok(read) = std::fs::read_dir(dir) else {
        return Some(Vec::new());
    };
    let mut entries = Vec::new();
    for entry in read {
        // FAIL CLOSED. A skipped entry is a capture the plan
        // cannot see, and `plan_retention` is TOTAL over what it is handed: given
        // a partial listing it happily evicts real captures to satisfy a cap the
        // omitted ones already exceeded. Deleting evidence on an incomplete view
        // is the one outcome the dashcam contract cannot tolerate, so an
        // unreadable entry abandons the whole pass.
        let Ok(entry) = entry else {
            tracing::warn!(
                dir = %dir.display(),
                "flashback: a directory entry could not be read — SKIPPING this \
                 retention sweep rather than planning on a partial listing (the caps may be \
                 exceeded until a later sweep sees the whole directory)"
            );
            return None;
        };
        let path = entry.path();
        if path.extension().and_then(|e| e.to_str()) != Some("mcap") {
            continue;
        }
        let Ok(meta) = entry.metadata() else {
            tracing::warn!(
                path = %path.display(),
                "flashback: a capture's metadata could not be read — SKIPPING this \
                 retention sweep rather than planning without its size"
            );
            return None;
        };
        if !meta.is_file() {
            continue;
        }
        let Ok(created_ns) = meta
            .modified()
            .and_then(|t| {
                t.duration_since(std::time::UNIX_EPOCH)
                    .map_err(|e| std::io::Error::other(e.to_string()))
            })
            .map(|d| d.as_nanos() as u64)
        else {
            tracing::warn!(
                path = %path.display(),
                "flashback: a capture's modification time could not be read — SKIPPING \
                 this retention sweep rather than guessing its age"
            );
            return None;
        };
        let Some(name) = path.file_name().and_then(|n| n.to_str()) else {
            continue;
        };
        // The PRIMARY cause, read from the sibling marker. An
        // unreadable or absent marker is an explicit UNKNOWN, never a guessed
        // class: a capture from a pre-marker recorder genuinely has none, and
        // `plan_retention` treats unknowns as one class, which is exactly the
        // oldest-first behaviour such a directory had before.
        let cause = std::fs::read_to_string(cause_marker_path(&path))
            .ok()
            .and_then(|body| parse_primary_cause(&body));
        entries.push(CaptureEntry {
            name: name.to_string(),
            bytes: meta.len(),
            created_ns,
            pinned: pin_marker_path(&path).exists(),
            cause,
        });
    }
    Some(entries)
}

/// Read the capture history the trigger gate seeds from.
///
/// Returns the history AND the wall reading it was dated against, together,
/// because the two are only meaningful as a pair: `seed_from_history` maps each
/// capture's age onto the monotonic line, so a `now_wall_ns` taken at a different
/// instant from the `mtime`s would shift every age by the gap.
///
/// Best-effort throughout — this is policy input, never evidence. An unreadable
/// directory, an unreadable entry and an unreadable clock each yield an empty
/// history, which restores exactly pre-marker behaviour rather than refusing to
/// capture. That is the OPPOSITE of `scan_capture_dir`'s fail-closed rule, and
/// deliberately: there a partial listing makes the plan DELETE captures it should
/// not, while here a partial history can only fail to withhold budget.
pub(crate) fn read_capture_history(
    dir: &std::path::Path,
) -> (Vec<cerulion_core::flashback::trigger::CaptureHistory>, u64) {
    use cerulion_core::flashback::trigger::CaptureHistory;

    let now_wall_ns = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0);
    let Ok(read) = std::fs::read_dir(dir) else {
        return (Vec::new(), now_wall_ns);
    };
    let mut history = Vec::new();
    for entry in read.flatten() {
        let path = entry.path();
        if path.extension().and_then(|e| e.to_str()) != Some("mcap") {
            continue;
        }
        // The SAME durable stamp the retention plan sorts on — see
        // `CaptureEntry::created_ns` for why it cannot be a monotonic reading.
        let Ok(created_wall_ns) = entry
            .metadata()
            .and_then(|m| m.modified())
            .and_then(|t| {
                t.duration_since(std::time::UNIX_EPOCH)
                    .map_err(|e| std::io::Error::other(e.to_string()))
            })
            .map(|d| d.as_nanos() as u64)
        else {
            continue;
        };
        // A capture with no marker seeds NOTHING rather than a guessed cause: the
        // floor is per-`(kind, subject)`, and a wrong key would withhold budget
        // from a condition that never fired.
        let Ok(body) = std::fs::read_to_string(cause_marker_path(&path)) else {
            continue;
        };
        let causes = parse_cause_marker(&body);
        if causes.is_empty() {
            continue;
        }
        history.push(CaptureHistory {
            created_wall_ns,
            causes,
        });
    }
    (history, now_wall_ns)
}

/// Carry out the dashcam contract on `dir`.
///
/// The loop has NO policy in it — every decision came from `plan_retention` —
/// and every eviction is LOGGED, because a flight recorder that silently deleted
/// evidence would be worse than one that kept none.
pub(crate) fn sweep_capture_dir(dir: &std::path::Path, caps: RetentionCaps) {
    use cerulion_core::flashback::retention::plan_retention;

    let Some(entries) = scan_capture_dir(dir) else {
        // The scan said so itself; nothing further to log.
        return;
    };
    if entries.is_empty() {
        return;
    }
    let plan = plan_retention(&entries, caps);
    for name in &plan.evict {
        let path = dir.join(name);
        match std::fs::remove_file(&path) {
            Ok(()) => {
                // The pin marker cannot exist for an evicted capture (a pinned
                // one is never a candidate), but a STALE marker left by a
                // hand-deleted bag would otherwise accumulate forever.
                let _ = std::fs::remove_file(pin_marker_path(&path));
                // The CAUSE marker, on the other hand, ordinarily DOES exist and
                // must go with its bag — a marker outliving the capture it
                // describes would keep seeding a floor for a cause whose evidence
                // is gone.
                let _ = std::fs::remove_file(cause_marker_path(&path));
                tracing::info!(
                    path = %path.display(),
                    retained_bytes = plan.retained_bytes,
                    retained_count = plan.retained_count,
                    "flashback: evicted the oldest capture to stay inside the retention caps"
                );
            }
            Err(e) => tracing::warn!(
                path = %path.display(),
                error = %e,
                "flashback: could not evict a capture"
            ),
        }
    }
    if plan.pinned_over_cap {
        tracing::warn!(
            dir = %dir.display(),
            retained_bytes = plan.retained_bytes,
            retained_count = plan.retained_count,
            max_bytes = caps.max_bytes,
            max_captures = caps.max_captures,
            "flashback: the retention caps are exceeded and everything left is PINNED — \
             nothing further can be evicted. Unpin a capture (delete its `.pin` marker) or raise \
             the caps; the directory will keep growing until you do"
        );
    }
}

/// What a capture was ABOUT, as the `__cerulion/flashback.json` attachment.
///
/// Hand-rendered rather than `serde`-derived, because the shape is small and
/// fixed and this crate already renders its other manifests by hand at the one
/// site that writes them.
///
/// # `causes_dropped` is rendered with its EXACTNESS
///
/// `FinishedCapture::causes_dropped_exact` is `false` when the gate's dropped-
/// identity memory saturated, which makes the count an UPPER BOUND on the
/// distinct total rather than the distinct total. A reader who cannot see that
/// has no way to know the number weakened — the same rule
/// `RetentionPlan::pinned_over_cap` follows — so both fields are written and the
/// terminal log line carries both.
/// **What this recorder was HANDED**, so a capture can say what it
/// carries — and, where it carries nothing, WHY.
///
/// Every field is a fact about this recorder's OWN INPUTS, which is the only kind
/// of claim it may make (the `RecordCoverage::enumerated` rule: the outcome, never
/// the intent, and never a fact about a peer nobody observed). The rendered
/// vocabulary is CLOSED, on the [`crate::TRACE_NONE_NO_RINGS`]-shaped precedent
/// `cerulion bag record --run` set for exactly this question: "there is no X" has
/// several causes and they are not interchangeable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct CaptureHandoff {
    /// The run's effective graph — what makes a capture RE-EXECUTABLE.
    pub graph_yaml: bool,
    /// The run's environment snapshot.
    pub env_json: bool,
    /// The recording host's identity.
    pub recorder_json: bool,
    /// Whether this recorder was given a capture-plane tag to sweep for.
    pub state_plane: bool,
    /// **Scheduler-trace records THIS CAPTURE CARRIES.**
    ///
    /// The manifest reports what the bag CONTAINS, never what the recorder was
    /// CONFIGURED with, and the two are not the same number — which is the whole
    /// reason this field exists beside
    /// [`trace_rings_configured`](Self::trace_rings_configured). A `--record`
    /// recorder holds the run's trace ring AND, by design, the rolling
    /// window, so a capture written by it had a configured ring and zero trace
    /// records in the bag: `crate::capture::write_capture` writes attachments,
    /// the coverage manifest, the schema catalog and FRAMES, and there is no
    /// trace channel anywhere in it. Reporting the configured count advertised
    /// replay capability the `.mcap` did not contain — a confident-false claim,
    /// and Principle #3 read backwards.
    ///
    /// **SEAM (trace window):** the trace window is what makes a
    /// capture actually CARRY trace, and it flips this verdict by supplying a
    /// real count here — not by editing a string. The `> 0` arm is reachable and
    /// pinned today (`a_capture_that_carries_trace_records_says_how_many`), so
    /// the flip is data-driven by construction.
    pub trace_records_carried: u64,
    /// Trace rings this recorder was HANDED — a fact about its own INPUTS, and
    /// reported under a name that says so.
    ///
    /// Kept because it is genuinely useful (it tells a reader whether the
    /// absence is "nobody gave this recorder a ring" or "it had one and the
    /// capture path does not write trace"), and because dropping a fact to
    /// avoid mis-stating it is the wrong repair. It is NOT the trace verdict.
    pub trace_rings_configured: usize,
    /// Trace records THIS capture's trim discarded as belonging to
    /// steps before the resume.
    ///
    /// Carried for the verdict alone. With a configured ring and nothing
    /// carried, this is what separates "the trim discarded every record it was
    /// offered" from "there was nothing to discard" — two different facts about
    /// the run, with two different things for an operator to look at.
    pub trace_records_trimmed_away: usize,
    /// Trace records THIS capture's byte ceiling took.
    ///
    /// The third cause, for the same reason as the second: an absence the
    /// CEILING produced is the one an operator can act on by raising it, and it
    /// must not be spelled as either of its siblings.
    pub trace_records_truncated: u64,
    /// **Which cursor this recorder's own trace reader started from.**
    ///
    /// `None` when there is no reader to ask — a `--record` recorder (whose
    /// reader is the writer thread, and whose rings are opened at record 0 by
    /// construction) or one handed no ring at all. `Some` is a WINDOW-ONLY
    /// recorder's answer, and the two arms are not interchangeable: a trace read
    /// from record 0 can reach step 0, which is the one shape resim resumes
    /// with no anchor at all, while one read from the live cursor cannot and a
    /// reader must not assume otherwise from a count alone.
    pub trace_attach: Option<crate::trace_drain::TraceAttach>,
    /// **Scheduler-trace records this RECORDER has drained**, lifetime.
    ///
    /// Beside [`trace_records_carried`](Self::trace_records_carried), never
    /// instead of it, because they answer different questions and a reader needs
    /// both: this one says whether the drain is working at all, that one says
    /// what is in THIS bag. A capture whose window holds no fires reports a large
    /// number here and zero there, and that pair is the diagnosis.
    ///
    /// **The active reader's count, in both modes.** A window-only
    /// recorder's reader is its trace-drain thread; a `--record` recorder's is
    /// its WRITER THREAD, and reading only the drain reported a flat `0` for the
    /// whole of that mode — beside a `trace` verdict naming the hundreds of
    /// records the same bag carried, i.e. a manifest contradicting itself about
    /// one run. `Recorder::trace_records_drained` is where the two readers meet,
    /// and it says why the sum needs no mode test.
    ///
    /// Unlike [`trace_attach`](Self::trace_attach) this is NOT nullable: every
    /// recorder that reaches a capture manifest has a reader, so `0` means
    /// "nothing drained" rather than "nobody to ask".
    pub trace_ring_records: u64,
    /// **Holes this recorder watched open in its own read**, lifetime.
    ///
    /// UNCONDITIONAL: it counts every lap of the run, whether or not THIS
    /// capture straddles one. A capture that carries a hole is refused by
    /// `ResimGap::TraceLapped`; a capture taken after the recorder re-attached is
    /// resimmable and still reports a non-zero count here, which is the accurate
    /// pair — the run had a recorder stall, this bag does not contain it.
    pub trace_laps: u64,
}

/// An artifact this capture CARRIES.
pub(crate) const HANDOFF_EMBEDDED: &str = "embedded";

/// No `graph.yaml` was handed over, so the capture cannot be re-executed.
///
/// The consequence is named rather than left to a reader: a bag with no graph is
/// refused by `cerulion bag play --resim` before anything loads, with a message
/// about a corrupt or hand-edited recording — which is exactly the wrong
/// diagnosis for a capture that was simply never given one.
pub(crate) const HANDOFF_GRAPH_ABSENT: &str =
    "absent: this recorder was handed no graph.yaml, so this capture can be VIEWED but not \
     re-executed (`cerulion bag play --resim`). A run writes one into its run directory \
     (~/.cerulion/runs/…) — if that directory could not be written, the run said so at startup.";

/// No `env.json` was handed over.
pub(crate) const HANDOFF_ENV_ABSENT: &str =
    "absent: this recorder was handed no env.json, so this capture carries no record of the \
     environment its run executed in.";

/// No `recorder.json` was handed over.
pub(crate) const HANDOFF_RECORDER_ABSENT: &str =
    "absent: this recorder was handed no recorder.json, so this capture carries no record of the \
     host that wrote it — a reader cannot warn about a cross-arch float/libm skew.";

/// A capture-plane tag WAS handed over.
///
/// Deliberately states only the HANDOFF, not that anchors are in this bag: the
/// tag makes the run's per-rank state rings discoverable, and what a capture then
/// carries of them is the rolling anchor retention's business. Claiming anchors
/// here would be a claim about a mechanism this string cannot see.
///
/// **SEAM (anchor retention):** when a capture really carries
/// anchors, this is the string that says so — and the accounting beside it
/// (`state_coverage.json`'s per-node verdicts) is what it should point at.
pub(crate) const HANDOFF_STATE_PLANE: &str =
    "handed: this recorder was given the run's capture-plane tag, so it discovers the run's \
     per-rank node-state rings.";

/// No capture-plane tag was handed over.
pub(crate) const HANDOFF_STATE_PLANE_ABSENT: &str =
    "absent: this recorder was handed no capture-plane tag, so it sweeps for no node-state ring \
     and this capture carries no anchors. Either the run's plane was refused (the \
     CERULION_FLASHBACK kill switch, or the arm-time memory gate — the run says which at \
     startup), or the run predates the always-on capture plane.";

/// No trace ring was handed over, so there was none to carry.
///
/// The same cause [`crate::TRACE_NONE_NO_RINGS`] gives a mid-run attach — stated
/// here in the recorder's own terms, because this recorder was HANDED the rings
/// it has rather than finding them. It names no flag, by design: the
/// reader is holding a capture from an always-on window recorder that
/// `cerulion graph run` spawned itself, and nothing an operator can type on that
/// command line adds a ring to an already-running run.
///
/// # The cause that reaches this arm has exactly two shapes
///
/// It is NOT that a `graph run` creates rings "only under `--record`": that is
/// not true and is not the cause, since a multi-process run provisions
/// per-rank rings whether or not it records. Two shapes remain.
///
/// The ORDINARY one is a WALL-GATED run — a single-process run, `ros2 attach`,
/// `node run`, or a virtual/external time source. Those mint no ring by design
/// since their gating clock is wall-driven: a trace taken there
/// would carry step boundaries a resume cannot re-advance to, and a capture
/// claiming to be re-executable off one would be confidently wrong.
///
/// The other is a multi-process run that could not HAVE its rings — the
/// free-space gate refused the deployment, or the departure ring itself could
/// not be created. Both leave the run's trace plane `Unavailable`, and it
/// records that reason in its own `run.json`, so the string points there rather
/// than guessing between them.
///
/// A run whose individual RANKS all failed does NOT reach here: the supervisor
/// chains the departure ring unconditionally under a `Declared` plane, so
/// `trace_rings_configured` stays non-zero and such a capture lands on
/// [`HANDOFF_TRACE_NONE_NOT_CARRIED`] instead.
///
/// What can NOT reach this arm is a run that declined its rings: decision
/// 123 stops the window recorder being started at all for such a run, so it
/// takes no captures and there is no manifest to read.
pub(crate) const HANDOFF_TRACE_NONE_NO_RINGS: &str =
    "none: this recorder was handed no trace ring, so this capture carries no scheduler trace and \
     cannot be re-executed. The RUN minted none. A multi-process run provisions per-rank rings \
     whether or not it records, so what reaches this is a wall-gated run — a single-process run, \
     `ros2 attach`, `node run`, or an external time source — which mints none by design because a \
     trace taken on that clock carries step boundaries a resume cannot re-advance to; \
     or a run that was refused its rings, which records that in its own `run.json` under \
     `trace_rings`. A run already under way cannot be given rings. A recorder started with \
     `--flashback-dir` but bound to NO run (no `--run-id`) has no run to be handed rings by, \
     and no `run.json` to consult — its captures carry no trace by construction.";

/// Rings WERE handed over — and the capture still carries no trace.
///
/// The DIFFERENT cause, and the one that must never be spelled as its sibling.
/// Saying "no ring was handed over" here is false about the RECORDER; saying
/// `attached: N ring(s)` is false about the BAG, and worse — it advertises replay
/// capability a reader can act on.
///
/// # What this cause IS
///
/// It must not read "a capture is written by the flashback writer, which persists
/// frames and attachments only — no scheduler trace channel", and must name no
/// tracking issue. The writer persists a trace
/// channel, so that sentence would be a false statement about the very build
/// emitting it, pointing a reader at work that is done.
///
/// What reaches this arm is a capture whose RETENTION yielded nothing, and there
/// are THREE ways to get there, not two.
///
/// A text of "either the trim found no step boundary past the anchor, or the trace
/// retention's byte ceiling held nothing" is an exhaustive-sounding pair that
/// omits the simplest case: a configured ring that produced nothing to retain,
/// which is what a run whose nodes have not fired yet looks like, and what a
/// ring drained empty looks like. Attributing THAT to a trim or a ceiling sends
/// an operator to widen a window or raise a ceiling that were never the problem
/// — the misleading-diagnosis class this whole family of constants exists for.
///
/// So the cause is CHOSEN from the capture's own numbers
/// ([`CaptureHandoff::trace_records_trimmed_away`],
/// [`CaptureHandoff::trace_records_truncated`]) rather than asserted as a
/// disjunction, and this constant is the arm for the case where BOTH are zero:
/// nothing was discarded and nothing was evicted, so nothing was there.
///
/// It still names no flag: on a window recorder `cerulion graph run` spawned
/// itself, the ceiling is not something an operator types on that command line.
pub(crate) const HANDOFF_TRACE_NONE_NOT_CARRIED: &str =
    "none: this recorder holds this run's trace ring(s), but they yielded no records to retain — \
     nothing was trimmed away and nothing was evicted, so nothing was there (a run whose nodes \
     have not fired within this capture's window looks exactly like this). See \
     `trace_rings_configured` in this document for what it was handed. So this capture can be \
     VIEWED but not re-executed.";

/// The retention HELD records and this capture's TRIM discarded all of them.
///
/// A distinct fact with a distinct remedy: the records exist, they simply all
/// belong to steps before the resume point, so no step boundary past the anchor
/// was found and no resume could be derived.
pub(crate) const HANDOFF_TRACE_NONE_TRIMMED_AWAY: &str =
    "none: this recorder holds this run's trace ring(s) and retained records, but every one of \
     them belonged to a step before this capture's anchor — the trim found no step boundary past \
     it, so no resume point could be derived. So this capture can be VIEWED but not re-executed.";

/// The retention's BYTE CEILING took them.
///
/// The one cause of the three an operator can act on directly, so it says which
/// knob — and it is the reason the three are not collapsed into one sentence.
pub(crate) const HANDOFF_TRACE_NONE_CEILING_TOOK_THEM: &str =
    "none: this recorder holds this run's trace ring(s), but this capture's trace retention hit \
     its byte ceiling and evicted every record it had (see `trace_records_truncated` in this \
     document). A longer window needs a larger ceiling: `CERULION_FLASHBACK_TRACE_MAX_MB`. So \
     this capture can be VIEWED but not re-executed.";

impl CaptureHandoff {
    /// The `trace` verdict — a function of what the capture CARRIES, with the
    /// configured-ring count only choosing WHICH absence cause applies.
    ///
    /// Split out so the rule is readable in one place and testable without a
    /// manifest: the whole defect this closes was a renderer that answered from
    /// the wrong number, and a rule stated inline in a `format!` is exactly where
    /// that hides.
    fn trace_verdict(&self) -> String {
        if self.trace_records_carried > 0 {
            // A COUNT, not a bare marker: it is the one number a reader cannot
            // re-derive from the bag without decoding the channel, and it is
            // what makes the trace-window flip visible rather than merely claimed.
            return format!(
                "carried: {} scheduler-trace record(s)",
                self.trace_records_carried
            );
        }
        if self.trace_rings_configured == 0 {
            return HANDOFF_TRACE_NONE_NO_RINGS.to_string();
        }
        // THREE causes, chosen from this capture's own numbers rather
        // than asserted as a disjunction — see `HANDOFF_TRACE_NONE_NOT_CARRIED`.
        //
        // The CEILING is reported ahead of the trim when both fired, on the
        // "report the arm an operator can ACT on" rule the trigger gate and this
        // module's anchor reasons already follow: raising the ceiling is a thing
        // an operator can do, while "every record belonged to an earlier step"
        // is a fact about the run. It also cannot mislead in that order — a
        // capture that hit its ceiling really did lose records to it, whatever
        // the trim then did with what survived.
        if self.trace_records_truncated > 0 {
            HANDOFF_TRACE_NONE_CEILING_TOOK_THEM.to_string()
        } else if self.trace_records_trimmed_away > 0 {
            HANDOFF_TRACE_NONE_TRIMMED_AWAY.to_string()
        } else {
            HANDOFF_TRACE_NONE_NOT_CARRIED.to_string()
        }
    }

    /// Render the manifest's `handoff` object.
    ///
    /// **SEAM (trace window):** the `resimmable` verdict the trace
    /// window reads is a function of exactly these — a capture with no graph and no
    /// carried trace is not resimmable whatever else it holds — so it belongs
    /// BESIDE this object rather than derived from a second set of facts. And it
    /// flips these values by supplying real ones (see
    /// [`trace_records_carried`](CaptureHandoff::trace_records_carried)), never
    /// by editing a string.
    fn render(&self) -> String {
        fn slot(present: bool, absent: &'static str) -> &'static str {
            if present {
                HANDOFF_EMBEDDED
            } else {
                absent
            }
        }
        format!(
            "{{\"graph\":\"{}\",\
             \"env\":\"{}\",\
             \"recorder\":\"{}\",\
             \"state_plane\":\"{}\",\
             \"trace\":\"{}\",\
             \"trace_attach\":{},\
             \"trace_ring_records\":{},\
             \"trace_laps\":{},\
             \"trace_rings_configured\":{}}}",
            esc(slot(self.graph_yaml, HANDOFF_GRAPH_ABSENT)),
            esc(slot(self.env_json, HANDOFF_ENV_ABSENT)),
            esc(slot(self.recorder_json, HANDOFF_RECORDER_ABSENT)),
            esc(if self.state_plane {
                HANDOFF_STATE_PLANE
            } else {
                HANDOFF_STATE_PLANE_ABSENT
            }),
            esc(&self.trace_verdict()),
            // JSON `null` when there is no reader to ask, never a
            // fabricated arm. A `--record` capture and a ringless one both land
            // here, and neither may be read as "this recorder attached at record
            // 0" — the claim `from_start` makes.
            match &self.trace_attach {
                Some(a) => format!("\"{}\"", esc(&a.render())),
                None => "null".to_string(),
            },
            self.trace_ring_records,
            self.trace_laps,
            self.trace_rings_configured,
        )
    }
}

pub(crate) struct CaptureManifest<'a> {
    /// What the gate recorded.
    pub finished: &'a FinishedCapture,
    /// Frames the capture carries.
    pub frames: u64,
    /// Frames THIS capture lost to the window's byte ceiling — never the plane's
    /// lifetime total.
    pub truncated_frames: u64,
    /// Frames whose wire header would not parse, so their sequence and
    /// timestamps in the bag are PLACEHOLDERS rather than the
    /// producer's.
    pub headerless_frames: u64,
    /// The oldest window instant the capture promised.
    ///
    /// The CLAIM. What it actually reaches is
    /// [`achieved_from_ns`](Self::achieved_from_ns), and the two differ by
    /// exactly what the byte ceiling took.
    pub floor_ns: u64,
    /// When it stopped recording.
    pub ended_ns: u64,
    /// The oldest instant this capture ACTUALLY carries, or
    /// `None` when it carries no frames at all.
    ///
    /// `None` is not a corner: the byte pass evicts ≥floor frames too (booked as
    /// truncations), so an overloaded recorder really can finalize a capture
    /// holding nothing of its own window. It renders as JSON `null` rather than
    /// as the floor, because a reader given the floor would believe it had been
    /// told the range.
    pub achieved_from_ns: Option<u64>,
    /// How many tapped topics contribute NO frame to the
    /// achieved range.
    ///
    /// A range implies coverage, and for a topic whose period exceeds it there is
    /// none — this is what stops `[a, b]` being read as "every topic, throughout".
    pub topics_with_no_frames: usize,
    /// The window's byte ceiling while this capture recorded.
    ///
    /// Carried so a shortfall can be READ rather than guessed at: the ceiling and
    /// the span are the two things that bound a look-back, and only one of them
    /// was previously in the manifest.
    pub window_cap_bytes: u64,
    /// What state this capture carries — see [`AnchorReport`].
    pub anchor: AnchorReport,
    /// Trace records THIS capture lost to the trace retention's byte ceiling.
    ///
    /// Its own number rather than folded into `truncated_frames`, for the reason
    /// [`ClosedCapture::truncated_trace_records`] gives: a lost frame costs the
    /// capture some of what it shows, a lost trace record can cost it the
    /// boundary a resume BEGINS at.
    pub truncated_trace_records: u64,
    /// What this recorder was handed, and by cause what it was
    /// not.
    ///
    /// The trace window supplies [`CaptureHandoff::trace_records_carried`], which is
    /// why no `trace_records` field sits beside it here: that is
    /// the ONE spelling of "scheduler-trace records this bag carries",
    /// and a second copy on this struct is how the two come to disagree.
    pub handoff: CaptureHandoff,
    /// Whether `bag play --resim` will accept this bag, judged
    /// by ITS OWN refusal predicates
    /// ([`cerulion_core::flashback::resim::judge_resimmable`]).
    ///
    /// A borrowed VERDICT rather than a `bool` beside a `String`, so the
    /// manifest structurally cannot render a claim its own reason contradicts.
    pub resim: &'a Result<(), cerulion_core::flashback::resim::ResimGap>,
    /// `target(S−1)`: the gating-clock value at the ANCHOR
    /// step's own boundary. `None` when that boundary is not in the retention.
    ///
    /// # Why a capture carries a number the replayer cannot compute
    ///
    /// `replay_engine`'s resume scoping says it in so many words: the band an
    /// external topic must still be served at the first replayed step is
    /// `[target(S−1), target(S))`, and *"`target(S−1)` is exactly the boundary
    /// record the recorder's head-step gate discarded, so no arithmetic here can
    /// recover it"*. So resim leaves external topics entirely unskipped and
    /// accepts a stated cost — that any earlier pre-anchor frame is served
    /// there too, which *"on every shipping path is empty-to-tiny"*.
    ///
    /// A Flashback capture BREAKS that premise: its window is ~30 s deep by
    /// design, so an external topic can carry thousands of pre-anchor frames and
    /// every one of them would inject at the first replayed step.
    ///
    /// The recorder is the one place that CAN recover the number — it holds the
    /// anchor step's boundary record in its retention and reads it on the way
    /// past while trimming. Writing it into the manifest turns resim's
    /// unrecoverable value into a recorded one, which is strictly better than
    /// the alternative of DELETING those frames from the bag: a black box that
    /// discards evidence around an incident is the thing this feature exists not
    /// to be.
    pub anchor_target_ns: Option<u64>,
    /// The gating-clock instant this capture's `resimmable` claim
    /// extends TO — the target of the last authoritative-rank step boundary its
    /// trace carries.
    ///
    /// The range's OTHER endpoint is already stated: `anchor_target_ns` for a
    /// resume, or the run's own step 0 for a from-start capture. What was
    /// missing was the upper one, and its absence is what made the verdict a
    /// confident-false: a capture's frame window ends one writer cycle past its
    /// trace window (see
    /// [`crate::trace_window::TrimmedTrace::last_boundary_target_ns`] for the
    /// mechanism and the measurement), so `resimmable: true` promised
    /// re-execution of frames the bag's own trace could not place.
    ///
    /// `None` when no such boundary is carried, which is exactly the state
    /// `ResimGap::NoBoundary` refuses — so a range is never stated for a bag
    /// nothing can resume.
    pub resim_covered_through_ns: Option<u64>,
}

/// What a capture can claim about the node state it carries.
///
/// # The one thing this vocabulary may never do
///
/// Claim resumability it cannot back. A capture that carries an anchor is not
/// automatically resumable: `cerulion bag play --resim` derives its resume step
/// from the SCHEDULER TRACE (`first_recorded_step − 1`) and refuses a mid-run bag
/// that has none. A plain multi-process `graph run` now DOES mint
/// trace rings, so the common shape now backs a resume — but a wall-gated run
/// mints none, a refused run has none, and a ring that yielded nothing inside
/// this capture's window carries none either. So the two facts are reported apart:
/// `embedded` says what state is IN the bag, and `resimmable` says whether this
/// bag can be resumed from it. A reader is never left to infer the second from
/// the first.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum AnchorReport {
    /// A checkpoint is embedded.
    Embedded {
        /// The run it belongs to.
        run_id: u64,
        /// The step it was taken at.
        step: u64,
        /// How many nodes' anchors it carries.
        nodes: usize,
        /// How many of those are COMPLETE (the rest are deliberate SKIPs, which
        /// tell a reader WHY a node has no state).
        complete: usize,
        /// Records written onto `__cerulion/state`.
        records: usize,
        /// Whether a resume from it covers the whole window the capture claims.
        fit: AnchorFit,
        /// The RECORDER-clock instant the checkpoint was harvested at.
        ///
        /// Carried RAW, because the two derived figures below answer
        /// two different questions and neither can stand in for the other.
        /// `frames_before_anchor_ms` is about the BAG (measured from what it
        /// carries), while "how far past the claimed window start does this
        /// anchor sit" is about the CLAIM — and under truncation those differ.
        /// With `floor_ns` and this instant both in the manifest, a reader
        /// computes either without the recorder having to pick one.
        taken_at_ns: u64,
        /// How much of the bag's FRAME span sits BEFORE the anchor, in ms.
        ///
        /// Reported rather than hidden. A capture
        /// holds frames from its ACHIEVED reach onward while a resume covers only
        /// from the anchor onward, so this is exactly the part of the recording a
        /// resume will NOT re-execute — a reader can see it without recomputing
        /// anything from the range and the step.
        ///
        /// Measured from the ACHIEVED reach, not from the claimed
        /// floor. Under a biting byte ceiling the floor is frozen where the
        /// trigger put it while the frames behind it have been evicted, so a
        /// floor-relative figure OVERSTATES — it counts a stretch of recording
        /// the bag does not carry as recording a resume will skip.
        frames_before_anchor_ms: u64,
        /// How much of the anchor's FORWARD span the bag is
        /// MISSING, in ms — `max(0, achieved_from − anchor_stamp)`.
        ///
        /// The complement of the field above, and the one the anchor accuracy rule
        /// exists for: a resume re-executes from the anchor
        /// onward, so frames between the anchor and the bag's achieved reach are
        /// frames the resume needs and the bag does not have. At most one of the
        /// two is nonzero — the anchor is either inside the achieved range or
        /// before it — so a reader sees which side the gap is on without
        /// arithmetic.
        ///
        /// Both are 0 for a capture carrying NO frames: there is no reach to
        /// measure from, nothing in the bag predates the anchor, and nothing
        /// between them is missing because the bag is missing all of it. That
        /// state is stated by `achieved_from_ns: null` rather than by inventing a
        /// span for it.
        frames_missing_after_anchor_ms: u64,
    },
    /// None is embedded, and why.
    Absent(NoAnchorReason),
}

impl AnchorReport {
    /// The JSON body of the manifest's `anchor` block.
    ///
    /// `resimmable` is deliberately a separate field from `embedded`: `embedded`
    /// says what state is IN the bag, `resimmable` says whether the bag can be
    /// RESUMED from it, and a reader must never have to infer the second from
    /// the first.
    ///
    /// # The claim is a VERDICT, not a re-derivation
    ///
    /// A COUNT test (`trace_records_carried > 0`, read
    /// off [`CaptureHandoff`] so the two halves of one manifest cannot
    /// disagree about whether a trace is there) fixes the SPELLING and
    /// leaves the RULE: a carried trace is one of the six things resim checks, and
    /// not the one a capture most often fails. So the claim is
    /// [`judge_resimmable`](cerulion_core::flashback::resim::judge_resimmable)'s
    /// answer, so a capture and the replayer cannot disagree, and this function
    /// RENDERS that answer rather than deciding anything.
    ///
    /// Note that the ABSENT arm carries the verdict too. Hardcoding `false`
    /// there looks safe and is not: a capture whose window reaches the run's own
    /// step 0 needs no anchor — `resolve_resume` resumes from the start — so a
    /// hardcoded `false` would refuse a bag resim accepts.
    fn render(
        &self,
        resim: &Result<(), cerulion_core::flashback::resim::ResimGap>,
        covered_through_ns: Option<u64>,
    ) -> String {
        let resimmable = resim.is_ok();
        // The POSITIVE claim has two shapes, and one sentence cannot serve both.
        // `RESIMMABLE_REASON` says the capture "carries … a complete
        // checkpoint", which is exactly what the ABSENT arm does NOT have: it is
        // resimmable because its window reaches the run's own step 0 and needs
        // no anchor. Serving the anchor sentence there tells a reader the bag
        // holds a checkpoint it can inspect, and it holds none.
        //
        // Both positive shapes now name the COVERED RANGE, through the
        // one composer — a renderer that appended it to only one of them would
        // publish a verdict whose scope depended on which arm it took.
        let reason = esc(&match (resim, self) {
            (Ok(()), Self::Absent(_)) => {
                cerulion_core::flashback::resim::resimmable_reason(true, covered_through_ns)
            }
            (Ok(()), _) => {
                cerulion_core::flashback::resim::resimmable_reason(false, covered_through_ns)
            }
            (Err(gap), _) => gap.reason(),
        });
        // JSON `null` for "no boundary to end a range at", never 0 —
        // 0 is a REAL gating-clock value (a run's own step 0 runs at it), and
        // the whole point of the field is that a reader can tell a stated range
        // from an absent one. The same rule `anchor_target_ns` is written under.
        let covered = match covered_through_ns {
            Some(ns) => ns.to_string(),
            None => "null".to_string(),
        };
        match self {
            Self::Embedded {
                run_id,
                step,
                nodes,
                complete,
                records,
                fit,
                taken_at_ns,
                frames_before_anchor_ms,
                frames_missing_after_anchor_ms,
            } => {
                let fit = match fit {
                    AnchorFit::CoversTheClaimedWindow => "covers_the_claimed_window",
                    AnchorFit::NewerThanTheClaimedWindow => "newer_than_the_claimed_window",
                };
                format!(
                    "{{\"embedded\":true,\
                     \"run_id\":{run_id},\
                     \"step\":{step},\
                     \"nodes\":{nodes},\
                     \"complete\":{complete},\
                     \"records\":{records},\
                     \"fit\":\"{fit}\",\
                     \"taken_at_ns\":{taken_at_ns},\
                     \"frames_before_anchor_ms\":{frames_before_anchor_ms},\
                     \"frames_missing_after_anchor_ms\":{frames_missing_after_anchor_ms},\
                     \"resimmable\":{resimmable},\
                     \"resim_covered_through_ns\":{covered},\
                     \"resimmable_reason\":\"{reason}\"}}"
                )
            }
            Self::Absent(absent) => format!(
                "{{\"embedded\":false,\
                 \"reason\":\"{}\",\
                 \"resimmable\":{resimmable},\
                 \"resim_covered_through_ns\":{covered},\
                 \"resimmable_reason\":\"{reason}\"}}",
                absent.as_wire()
            ),
        }
    }
}

/// JSON-escape a value that may carry operator- or remote-supplied text.
///
/// Module-level rather than a closure inside the renderer, because
/// [`CaptureHandoff::render`] needs the same treatment: its vocabulary is a
/// closed set of constants today, but a rendered `handoff` value that grew a
/// quote would produce a manifest nothing can parse — the failure being SILENT
/// at write time and total at read time.
fn esc(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out
}

/// The READ shape of `__cerulion/flashback.json` — what
/// `cerulion bag info` needs to say what a capture claims against what it holds.
///
/// # A reader, deliberately not a writer
///
/// The manifest is RENDERED by hand a few lines below, and that stays true: it
/// is written once, by one function, into a bag, and a serde round trip would
/// buy nothing while making the field order (which a reader can diff) an
/// implementation detail of a derive. What this type buys is that the reader is
/// TYPED rather than a pile of `doc["…"]` lookups, and that the renderer's own
/// arms parse through it — so a field renamed on one side fails on the other.
///
/// EVERY field is optional. A bag is written by another process, possibly on
/// another machine, possibly by another version: a manifest from before these
/// fields existed must still render what it does carry, and a manifest from
/// AFTER must not be refused for carrying more (no `deny_unknown_fields`).
#[derive(Debug, Clone, Default, PartialEq, Eq, serde::Deserialize)]
pub struct FlashbackManifest {
    /// The capture's sequence within its recorder.
    #[serde(default)]
    pub seq: Option<u64>,
    /// Whether it is excluded from retention eviction.
    #[serde(default)]
    pub pinned: Option<bool>,
    /// Frames it carries.
    #[serde(default)]
    pub frames: Option<u64>,
    /// What its own endpoints CLAIM, in ms.
    #[serde(default)]
    pub span_ms: Option<u64>,
    /// What its frames REACH, in ms.
    #[serde(default)]
    pub achieved_span_ms: Option<u64>,
    /// The difference — carried rather than re-derived here, so `bag info`
    /// reports the number the recorder wrote rather than a second opinion.
    #[serde(default)]
    pub coverage_shortfall_ms: Option<u64>,
    /// The span the window promised.
    #[serde(default)]
    pub window_span_ms: Option<u64>,
    /// The byte ceiling in force while it recorded.
    #[serde(default)]
    pub window_cap_bytes: Option<u64>,
    /// Frames the byte ceiling took FROM THIS CAPTURE.
    #[serde(default)]
    pub truncated_frames: Option<u64>,
    /// Tapped topics contributing NO frame to the achieved range.
    #[serde(default)]
    pub topics_with_no_frames: Option<u64>,
}

pub(crate) fn render_capture_manifest(m: &CaptureManifest<'_>, plane: &FlashbackPlane) -> Vec<u8> {
    let CaptureManifest {
        finished,
        frames,
        truncated_frames,
        truncated_trace_records,
        headerless_frames,
        floor_ns,
        ended_ns,
        achieved_from_ns,
        topics_with_no_frames,
        window_cap_bytes,
        ref anchor,
        handoff,
        resim,
        anchor_target_ns,
        resim_covered_through_ns,
    } = *m;

    let causes: Vec<String> = finished
        .causes
        .iter()
        .map(|c| {
            format!(
                "{{\"kind\":\"{}\",\"subject\":\"{}\",\"detail\":\"{}\"}}",
                c.kind.as_wire(),
                esc(&c.subject),
                esc(&c.detail)
            )
        })
        .collect();
    // The CLAIMED span — rendered from this capture's OWN endpoints, never from
    // the span constant. This is the wording rule, and the case that forces it is
    // an EARLY trigger: `floor_ns` is `T − window_span` SATURATING, so a capture
    // triggered five seconds into a run claims five seconds, not thirty, and a
    // manifest quoting the constant there would claim a look-back the run had not
    // existed long enough to have.
    let span_ms = ended_ns.saturating_sub(floor_ns) / 1_000_000;
    // What it ACHIEVED. Nothing carried is nothing spanned — 0
    // here is a measurement, not a placeholder, and it sits beside an explicit
    // `achieved_from_ns: null` that says so.
    let achieved_span_ms = achieved_from_ns
        .map(|from| ended_ns.saturating_sub(from) / 1_000_000)
        .unwrap_or(0);
    // Derived, never carried alongside as a third number: a shortfall that could
    // disagree with the two endpoints it is a difference of is worse than no
    // shortfall at all.
    let coverage_shortfall_ms = span_ms.saturating_sub(achieved_span_ms);
    let json = format!(
        "{{\"version\":1,\
         \"seq\":{},\
         \"pinned\":{},\
         \"recorder\":\"{}\",\
         \"frames\":{},\
         \"span_ms\":{},\
         \"achieved_span_ms\":{},\
         \"coverage_shortfall_ms\":{},\
         \"floor_ns\":{},\
         \"achieved_from_ns\":{},\
         \"ended_ns\":{},\
         \"topics_with_no_frames\":{},\
         \"window_span_ms\":{},\
         \"window_cap_bytes\":{},\
         \"truncated_frames\":{},\
         \"headerless_frames\":{},\
         \"causes\":[{}],\
         \"causes_dropped\":{},\
         \"causes_dropped_exact\":{},\
         \"truncated_trace_records\":{},\
         \"anchor_target_ns\":{},\
         \"anchor\":{},\
         \"handoff\":{}}}",
        finished.seq,
        finished.pinned,
        esc(&plane.settings().label),
        frames,
        span_ms,
        achieved_span_ms,
        coverage_shortfall_ms,
        floor_ns,
        // JSON `null` for "this capture carries no frames", never the floor: a
        // reader handed the floor would believe it had been told the range.
        match achieved_from_ns {
            Some(ns) => ns.to_string(),
            None => "null".to_string(),
        },
        ended_ns,
        topics_with_no_frames,
        plane.settings().window_span.as_millis(),
        window_cap_bytes,
        truncated_frames,
        headerless_frames,
        causes.join(","),
        finished.causes_dropped,
        finished.causes_dropped_exact,
        truncated_trace_records,
        // JSON `null` for "not recovered", never 0: 0 is a REAL gating-clock
        // value (a run's own step 0 runs at it), so a reader given 0 would skip
        // nothing and believe it had been told something.
        match anchor_target_ns {
            Some(ns) => ns.to_string(),
            None => "null".to_string(),
        },
        anchor.render(resim, resim_covered_through_ns),
        handoff.render(),
    );
    json.into_bytes()
}

#[cfg(test)]
mod tests {
    use super::*;

    use cerulion_core::flashback::resim::ResimGap;

    /// A handoff in which NOTHING was handed over — the shape a capture takes on
    /// a run whose directory could not be written and whose plane was refused.
    ///
    /// The DEFAULT for the arms that are not about the handoff, deliberately:
    /// their oracles predate it, and giving them the fully-handed shape would
    /// make "everything is embedded" the value every arm in the file renders.
    fn no_handoff() -> CaptureHandoff {
        CaptureHandoff {
            graph_yaml: false,
            env_json: false,
            recorder_json: false,
            state_plane: false,
            trace_records_carried: 0,
            trace_rings_configured: 0,
            // The ordinary shape — nothing trimmed away, nothing
            // evicted, so the absence arm is the nothing-was-there one.
            trace_records_trimmed_away: 0,
            trace_records_truncated: 0,
            // No drain ran in this fixture — see the fields' docs
            // for why `None` is not the same claim as `from_start`.
            trace_attach: None,
            trace_ring_records: 0,
            trace_laps: 0,
        }
    }

    fn settings(dir: std::path::PathBuf) -> FlashbackSettings {
        FlashbackSettings {
            window_span: Duration::from_secs(30),
            window_max_bytes: 1 << 30,
            anchor_max_bytes: 1 << 30,
            // A STATED ceiling, so the anchor-first
            // reserve cannot move these arms' budgets under them.
            anchor_cap_basis: cerulion_core::flashback::CapBasis::Env,
            trace_max_bytes: 1 << 30,
            dir,
            label: "demo".into(),
            caps: RetentionCaps::default(),
            policy: TriggerPolicy::default(),
            posture: TriggerPosture::default(),
            exclude_topics: cerulion_core::flashback::ExcludeTopics::default(),

            window_only: true,
            // The shipped default. These arms drive the
            // WINDOW, never a tap, so the budget is inert here.
            tap_budget_bytes: cerulion_core::flashback::DEFAULT_FLASHBACK_TAP_BUDGET_MB
                * 1024
                * 1024,
        }
    }

    /// The verdict every capture had before captures carried a trace.
    fn no_trace() -> Result<(), ResimGap> {
        Err(ResimGap::NoTrace)
    }

    // These arms are NOT "synthetic manifest test data" in the banned sense,
    // and they are not the only thing standing behind the verdict. Both points below.
    //
    // First: these arms drive the production renderer
    // (`render_capture_manifest`) with hand-built INPUTS, which is the repo's
    // required pattern rather than the banned one. Principle #13 forbids
    // fabricating DATA and presenting it as a real measurement — a benchmark, a
    // report, a claim about a robot. An oracle vector fed through real code is
    // what every pure test in this crate does, and it is what makes the
    // renderer's own branches reachable at all: `MultiRing` needs several state
    // rings, i.e. a real multi-process supervisor, and `NoBoundary` needs a
    // torn ring head. The lineage is stated in this repo's own vocabulary at
    // `cerulion_viz/bin/cerulion_vizd/tests/vizd_e2e_test.rs:112` — "a RECORDING
    // spy `DemandPlane` (a DI test double — Principle #13, not fake data)".
    //
    // Second: a gap these arms cannot close on their own is real:
    // on their own nothing drives the capture->manifest->REPLAY path end to end,
    // so the verdict these arms render and the gate `bag play --resim` applies
    // could disagree about one bag with every test green: a
    // `write_capture` that published `resimmable: true` while writing no
    // `__cerulion/trace_manifest_rank<N>.json` would pass here while `load_trace_manifests`
    // refuses a bag with zero manifests outright.
    //
    // That arm exists — `cerulion_cli/tests/flashback_resim_e2e_test.rs`,
    // over three REAL binaries (`graph run --record` -> `cerulion flashback` ->
    // `bag play --resim all`), asserting exit 0 beside the capture's own
    // `resimmable: true`. It is what makes these vector arms a REFINEMENT of a
    // covered path rather than the only thing standing behind the verdict.
    //
    // The renderer's inputs are additionally covered by the REAL recorder
    // writing REAL manifests:
    //
    //   * `flashback_trace_e2e_test::a_capture_carries_its_trace_trimmed_to_its_
    //     anchor_and_reads_resimmable` — the `Ok(())` arm,
    //   * `...::a_departure_carrying_capture_reads_not_resimmable_naming_fault_
    //     replay` — `FaultReplay`,
    //   * `...::a_capture_with_no_anchor_in_window_reads_not_resimmable_honestly`
    //     and `...::a_run_with_no_trace_ring_still_captures_and_says_it_is_not_
    //     resimmable` — `NoTrace` + the ABSENT anchor arm,
    //   * `...::a_trace_with_no_boundary_past_the_anchor_is_trimmed_away_and_
    //     reads_not_resimmable` — `NoBoundary`,
    //   * `flashback_anchor_e2e_test`'s nine arms — every `AnchorReport` shape.
    //
    // What no e2e can drive is `MultiRing` (several state rings need a real
    // supervisor), so its vector here is DEFENSIVE rendering of a verdict the
    // shared judge really produces, pinned purely in
    // `cerulion_core::flashback::resim`.
    //
    // `AmbiguousRun` is deliberately NOT exercised here: `judge_capture_
    // resimmable` reports 1 run or 0 by construction (a `Checkpoint` is keyed on
    // `(run_id, step)` and `select` returns ONE), so the recorder cannot mint it
    // and a renderer vector for it would assert a combination no capture has.
    // The arm stays in the shared judge because it mirrors resim's own
    // `resolve_run_at` refusal, and it is oracle-tested there.

    /// The manifest carries `causes_dropped` WITH its exactness, which is the
    /// obligation this attachment discharges: an upper bound that reads
    /// like an exact count is a number a reader cannot use.
    #[test]
    fn the_capture_manifest_states_whether_its_dropped_count_is_exact() {
        use cerulion_core::flashback::trigger::{CaptureCause, TriggerKind};

        let plane = FlashbackPlane::new(settings(std::path::PathBuf::from("/tmp/fb")), None);
        let finished = FinishedCapture {
            seq: 7,
            pinned: true,
            causes: vec![CaptureCause {
                kind: TriggerKind::ProcessFault,
                subject: "p1".into(),
                // A detail carrying the two characters JSON cannot hold raw.
                detail: "worker \"p1\" died\nSIGSEGV".into(),
            }],
            causes_dropped: 4,
            causes_dropped_exact: false,
        };
        let bytes = render_capture_manifest(
            &CaptureManifest {
                finished: &finished,
                frames: 900,
                truncated_frames: 12,
                headerless_frames: 3,
                floor_ns: 0,
                ended_ns: 45_000_000_000,
                // No truncation in this fixture — the capture reaches
                // exactly the floor it claims, which is the shape every oracle
                // here predates and still describes.
                achieved_from_ns: Some(0),
                topics_with_no_frames: 0,
                window_cap_bytes: 320 * 1024 * 1024,
                anchor: AnchorReport::Absent(NoAnchorReason::NothingRetained),
                truncated_trace_records: 5,
                handoff: no_handoff(),
                resim: &no_trace(),
                anchor_target_ns: None,
                resim_covered_through_ns: None,
            },
            &plane,
        );
        let text = String::from_utf8(bytes).expect("utf-8");

        assert!(text.contains("\"causes_dropped\":4"), "{text}");
        // PER-CAPTURE, not the plane's lifetime totals. The
        // two numbers are deliberately DIFFERENT so a swap cannot pass, and the
        // plane here has NEVER truncated anything — so a renderer reading
        // `plane.truncated_frames()` would print 0 and fail.
        assert!(text.contains("\"truncated_frames\":12"), "{text}");
        assert!(text.contains("\"headerless_frames\":3"), "{text}");
        assert!(
            text.contains("\"causes_dropped_exact\":false"),
            "an upper bound must SAY it is one: {text}"
        );
        assert!(text.contains("\"seq\":7"));
        assert!(text.contains("\"pinned\":true"));
        assert!(text.contains("\"frames\":900"));
        assert!(text.contains("\"span_ms\":45000"));
        assert!(text.contains("\"process_fault\""));
        // The escaping is real: a quote and a newline in an operator-supplied
        // detail must not produce a manifest nothing can parse.
        assert!(text.contains("worker \\\"p1\\\" died\\nSIGSEGV"), "{text}");
        let parsed: serde_json::Value = serde_json::from_str(&text)
            .unwrap_or_else(|e| panic!("the manifest must be valid JSON: {e}\n{text}"));
        assert_eq!(parsed["causes_dropped_exact"], serde_json::json!(false));
    }

    /// The exact half of the same claim — without this, the arm above passes
    /// against a renderer that hardcodes `false`.
    #[test]
    fn an_ordinary_capture_reports_its_dropped_count_as_exact() {
        let plane = FlashbackPlane::new(settings(std::path::PathBuf::from("/tmp/fb")), None);
        let finished = FinishedCapture {
            seq: 0,
            pinned: false,
            causes: Vec::new(),
            causes_dropped: 0,
            causes_dropped_exact: true,
        };
        let text = String::from_utf8(render_capture_manifest(
            &CaptureManifest {
                finished: &finished,
                frames: 0,
                truncated_frames: 0,
                headerless_frames: 0,
                floor_ns: 0,
                ended_ns: 0,
                // No truncation in this fixture — the capture reaches
                // exactly the floor it claims, which is the shape every oracle
                // here predates and still describes.
                achieved_from_ns: Some(0),
                topics_with_no_frames: 0,
                window_cap_bytes: 320 * 1024 * 1024,
                anchor: AnchorReport::Absent(NoAnchorReason::NothingRetained),
                truncated_trace_records: 0,
                handoff: no_handoff(),
                resim: &no_trace(),
                anchor_target_ns: None,
                resim_covered_through_ns: None,
            },
            &plane,
        ))
        .expect("utf-8");
        assert!(text.contains("\"causes_dropped_exact\":true"), "{text}");
        assert!(text.contains("\"pinned\":false"));
    }

    fn manifest(anchor: AnchorReport, trace_records_carried: u64) -> String {
        manifest_with(anchor, trace_records_carried, &no_trace())
    }

    fn manifest_with(
        anchor: AnchorReport,
        trace_records_carried: u64,
        resim: &Result<(), ResimGap>,
    ) -> String {
        let plane = FlashbackPlane::new(settings(std::path::PathBuf::from("/tmp/fb")), None);
        let finished = FinishedCapture {
            seq: 1,
            pinned: false,
            causes: Vec::new(),
            causes_dropped: 0,
            causes_dropped_exact: true,
        };
        String::from_utf8(render_capture_manifest(
            &CaptureManifest {
                finished: &finished,
                frames: 10,
                truncated_frames: 0,
                headerless_frames: 0,
                floor_ns: 0,
                ended_ns: 1,
                // No truncation in this fixture — the capture reaches
                // exactly the floor it claims, which is the shape every oracle
                // here predates and still describes.
                achieved_from_ns: Some(0),
                topics_with_no_frames: 0,
                window_cap_bytes: 320 * 1024 * 1024,
                anchor,
                truncated_trace_records: 0,
                // The trace count rides `CaptureHandoff`, which is the
                // one place this manifest states it.
                handoff: CaptureHandoff {
                    trace_records_carried,
                    ..no_handoff()
                },
                resim,
                anchor_target_ns: Some(4_321),
                // A DISTINCT value from `anchor_target_ns`, and above
                // it — the two are the range's endpoints, so an arm that read
                // one where it meant the other would still pass against equal
                // numbers.
                resim_covered_through_ns: Some(9_876),
            },
            &plane,
        ))
        .expect("utf-8")
    }

    /// `manifest_with`'s sibling for the arms whose subject IS the
    /// covered range, so they can drive a value of their own.
    ///
    /// Its own helper rather than a fourth parameter on `manifest_with`: that
    /// one is called by nine arms that are not about the range, and threading a
    /// value through them would make every one of them state a range it does not
    /// assert on.
    fn render_manifest_covering(
        anchor: AnchorReport,
        resim: &Result<(), ResimGap>,
        covered_through_ns: Option<u64>,
    ) -> String {
        let plane = FlashbackPlane::new(settings(std::path::PathBuf::from("/tmp/fb")), None);
        let finished = FinishedCapture {
            seq: 1,
            pinned: false,
            causes: Vec::new(),
            causes_dropped: 0,
            causes_dropped_exact: true,
        };
        String::from_utf8(render_capture_manifest(
            &CaptureManifest {
                finished: &finished,
                frames: 10,
                truncated_frames: 0,
                headerless_frames: 0,
                floor_ns: 0,
                ended_ns: 1,
                // No truncation in this fixture — the capture reaches
                // exactly the floor it claims, which is the shape every oracle
                // here predates and still describes.
                achieved_from_ns: Some(0),
                topics_with_no_frames: 0,
                window_cap_bytes: 320 * 1024 * 1024,
                anchor,
                truncated_trace_records: 0,
                handoff: CaptureHandoff {
                    trace_records_carried: 12,
                    ..no_handoff()
                },
                resim,
                anchor_target_ns: Some(4_321),
                resim_covered_through_ns: covered_through_ns,
            },
            &plane,
        ))
        .expect("utf-8")
    }

    /// A resimmable capture states the RANGE its claim covers, in BOTH
    /// verdict shapes, and its reason names it.
    ///
    /// The decision is that a capture keeps every frame and its
    /// `resimmable` claim names the covered PREFIX — so a reader must be able to
    /// see WHERE the claim stops without inferring it from anything. Before this
    /// the manifest said `resimmable: true` and stopped, which is how a bag whose
    /// frames ran past its own trace advertised re-execution of frames the trace
    /// could not place.
    ///
    /// The oracle asserts THREE things a weaker one would not separate:
    ///
    /// * the range is present as a NUMBER, so a machine reader has it;
    /// * it is the CAPTURE'S OWN value and not `anchor_target_ns` — the fixture
    ///   holds the two apart (4321 vs 9876), so a renderer that spliced the
    ///   wrong endpoint fails rather than passing on equal numbers;
    /// * the REASON names it too, because the operator-facing sentence is what
    ///   `cerulion flashback`'s NOT-resimmable hint points at and it must not be
    ///   narrower than the field beside it.
    ///
    /// Driven on the EMBEDDED and the ABSENT arm in one body: the two are
    /// separate `format!` literals, so a field added to one is exactly the shape
    /// this arm exists to refuse.
    #[test]
    fn a_resimmable_capture_states_the_range_its_claim_covers() {
        for (label, anchor) in [
            (
                "embedded",
                AnchorReport::Embedded {
                    run_id: 7,
                    step: 41,
                    nodes: 2,
                    complete: 2,
                    records: 4,
                    fit: AnchorFit::CoversTheClaimedWindow,
                    // These arms do not exercise the anchor instant.
                    taken_at_ns: 0,
                    frames_before_anchor_ms: 100,
                    frames_missing_after_anchor_ms: 0,
                },
            ),
            (
                "absent",
                AnchorReport::Absent(NoAnchorReason::NothingRetained),
            ),
        ] {
            let text = manifest_with(anchor, 12, &Ok(()));
            let parsed: serde_json::Value =
                serde_json::from_str(&text).expect("the manifest is valid JSON");

            assert_eq!(
                parsed["anchor"]["resimmable"],
                serde_json::json!(true),
                "{label}: the fixture must CLAIM resimmable, or the range below qualifies nothing"
            );
            assert_eq!(
                parsed["anchor"]["resim_covered_through_ns"],
                serde_json::json!(9_876),
                "{label}: a resimmable capture states the instant its claim reaches: {text}"
            );
            // …and it is NOT the range's other endpoint. Both are gating-clock
            // instants of the same recording, so only distinct values can tell
            // a spliced-wrong-endpoint renderer from a correct one.
            assert_eq!(
                parsed["anchor_target_ns"],
                serde_json::json!(4_321),
                "{label}: the LOWER endpoint is still its own field: {text}"
            );

            let reason = parsed["anchor"]["resimmable_reason"]
                .as_str()
                .unwrap_or_default();
            assert!(
                reason.contains("9876 ns"),
                "{label}: the operator-facing sentence names the range too: {reason}"
            );
            assert!(
                reason.contains("outside the range"),
                "{label}: …and says the remainder is present but outside it: {reason}"
            );
        }
    }

    /// A capture with no boundary to end a range at states `null`, not 0.
    ///
    /// 0 is a REAL gating-clock instant — a run's own step 0 runs at it — so a
    /// reader handed 0 would take a covered range of exactly the first step and
    /// call the whole recording untrimmed-past-it. The same rule
    /// `anchor_target_ns` is written under, and the same reason.
    ///
    /// Such a capture is refused separately (`NoBoundary`), which is why this
    /// arm drives the refusing verdict: the pairing IS the guarantee — no range
    /// claimed, and a reason that says why there is nothing to claim.
    #[test]
    fn a_capture_with_no_boundary_states_no_covered_range_rather_than_zero() {
        let text = render_manifest_covering(
            AnchorReport::Absent(NoAnchorReason::NothingRetained),
            &Err(ResimGap::NoBoundary),
            None,
        );
        let parsed: serde_json::Value = serde_json::from_str(&text).expect("valid JSON");
        assert_eq!(
            parsed["anchor"]["resim_covered_through_ns"],
            serde_json::Value::Null,
            "no boundary means no range — and NEVER a numeric 0: {text}"
        );
        // The ANTI-TAUTOLOGY half, in the same body: the field really does carry
        // a number when there is one, so `null` above is a measurement and not
        // a field that is always absent.
        let with_range = render_manifest_covering(
            AnchorReport::Absent(NoAnchorReason::NothingRetained),
            &Ok(()),
            Some(0),
        );
        let parsed: serde_json::Value = serde_json::from_str(&with_range).expect("valid JSON");
        assert_eq!(
            parsed["anchor"]["resim_covered_through_ns"],
            serde_json::json!(0),
            "…and a range that really IS 0 renders as the number: {with_range}"
        );
    }

    /// THE over-claim pin: a capture that CARRIES an anchor must not claim it can
    /// be RESUMED from unless the verdict says so.
    ///
    /// `bag play --resim` takes its resume step from the scheduler trace
    /// (`first_recorded_step − 1`) and refuses a mid-run bag with none. So an
    /// operator reading `embedded: true` must be able to see, in the same block,
    /// that this bag cannot be resimmed and why.
    #[test]
    fn an_embedded_anchor_without_a_trace_never_claims_to_be_resimmable() {
        let embedded = AnchorReport::Embedded {
            run_id: 0xABCD,
            step: 4200,
            nodes: 3,
            complete: 2,
            records: 9,
            fit: AnchorFit::CoversTheClaimedWindow,
            // These arms do not exercise the anchor instant.
            taken_at_ns: 0,
            frames_before_anchor_ms: 15_000,
            frames_missing_after_anchor_ms: 0,
        };
        let text = manifest(embedded.clone(), 0);
        assert!(text.contains("\"embedded\":true"), "{text}");
        assert!(text.contains("\"run_id\":43981"), "{text}");
        assert!(text.contains("\"step\":4200"), "{text}");
        assert!(text.contains("\"nodes\":3"), "{text}");
        // COMPLETE is reported apart from the node count, because a SKIP tells a
        // reader why a node has no state where a missing entry says only that
        // one is absent.
        assert!(text.contains("\"complete\":2"), "{text}");
        assert!(text.contains("\"records\":9"), "{text}");
        assert!(
            text.contains("\"fit\":\"covers_the_claimed_window\""),
            "{text}"
        );
        assert!(
            text.contains("\"resimmable\":false"),
            "an anchor with no trace is NOT resimmable, and the manifest must say so: {text}"
        );
        // The pin is on the REASON the manifest carries, not on a
        // tracking issue's id: naming an issue is not naming the reason,
        // and the reason is not "rings are
        // record-only" either. `resimmable_reason` says what is missing,
        // and it must
        // name no flag, by design.
        assert!(
            text.contains("\"resimmable_reason\":\"no scheduler trace in this capture"),
            "…naming the reason, in the manifest's own reason field: {text}"
        );
        assert!(
            !text.contains("--record") && !text.contains("--no-rings"),
            "an absence names the CAUSE, never another verb's flag: {text}"
        );
        // The COUNT the verdict is computed from, in its one spelling: the
        // manifest states it ONCE, inside `handoff`, so the `resimmable` claim
        // and the `trace` verdict cannot disagree about whether a trace is there.
        assert!(
            text.contains(&format!("\"trace\":\"{HANDOFF_TRACE_NONE_NO_RINGS}\"")),
            "{text}"
        );
        let parsed: serde_json::Value = serde_json::from_str(&text)
            .unwrap_or_else(|e| panic!("the manifest must be valid JSON: {e}\n{text}"));
        assert_eq!(parsed["anchor"]["resimmable"], serde_json::json!(false));

        // ANTI-TAUTOLOGY: the SAME anchor with a PASSING verdict reads
        // resimmable — so the claim tracks the judge rather than being
        // hardcoded either way.
        let with_trace = manifest_with(embedded, 900, &Ok(()));
        assert!(with_trace.contains("\"resimmable\":true"), "{with_trace}");
        assert!(
            with_trace.contains("carried: 900 scheduler-trace record(s)"),
            "the SAME number drives both halves — `resimmable` and the `handoff` \
             trace verdict are two renderings of one measured count: {with_trace}"
        );
        assert!(
            !with_trace.contains("\"trace\":\"none:"),
            "a bag that CAN be resimmed must not carry the excuse for one that cannot: \
             {with_trace}"
        );
        assert!(
            with_trace.contains("--resim all"),
            "…and the positive reason tells the operator what to RUN: {with_trace}"
        );
    }

    /// The manifest renders the JUDGE's verdict, not a
    /// re-derivation of it — including on the arms `trace_records > 0` got wrong.
    ///
    /// The earlier rule was `trace_records > 0`, which is TRUE for every one
    /// of these captures. Each carries a trace and is still not resimmable, for
    /// a reason resim would refuse it on — so a renderer that fell back to the
    /// count would print `true` for all four.
    #[test]
    fn a_capture_with_a_trace_still_reports_the_gap_that_stops_it() {
        let embedded = AnchorReport::Embedded {
            run_id: 7,
            step: 40,
            nodes: 2,
            complete: 1,
            records: 4,
            fit: AnchorFit::CoversTheClaimedWindow,
            // These arms do not exercise the anchor instant.
            taken_at_ns: 0,
            // Not what this arm is about — the neutral value.
            frames_before_anchor_ms: 0,
            frames_missing_after_anchor_ms: 0,
        };
        for (gap, marker) in [
            (
                ResimGap::FaultReplay { departures: 3 },
                "fault-degraded recording is not supported yet",
            ),
            (ResimGap::MultiRing { rings: 4 }, "no rank"),
            (ResimGap::NoBoundary, "no step-boundary record"),
            (
                ResimGap::AnchorIncomplete {
                    detail: "node `relay` has no state".to_string(),
                },
                "relay",
            ),
        ] {
            let text = manifest_with(embedded.clone(), 900, &Err(gap.clone()));
            assert!(
                text.contains("carried: 900 scheduler-trace record(s)"),
                "the count is still reported — in the handoff's own vocabulary, \
                 which is the one place this manifest states it: {text}"
            );
            assert!(
                text.contains("\"resimmable\":false"),
                "a capture WITH a trace is still refused on {gap:?}: {text}"
            );
            assert!(
                text.contains(marker),
                "…and the reason names it ({marker}): {text}"
            );
            let parsed: serde_json::Value = serde_json::from_str(&text)
                .unwrap_or_else(|e| panic!("the manifest must be valid JSON: {e}\n{text}"));
            assert_eq!(parsed["anchor"]["resimmable"], serde_json::json!(false));
        }
    }

    /// A reason is an ordinary SENTENCE — quotes, backticks and all — so the
    /// manifest must escape it. Without this the `AnchorIncomplete` arm (which
    /// renders `plan_restore`'s own text, and that text quotes node ids) would
    /// emit JSON nothing can parse.
    #[test]
    fn a_reason_carrying_json_metacharacters_still_parses() {
        let text = manifest_with(
            AnchorReport::Absent(NoAnchorReason::NothingRetained),
            0,
            &Err(ResimGap::AnchorIncomplete {
                detail: "node \"a\\b\" is\nmissing".to_string(),
            }),
        );
        let parsed: serde_json::Value = serde_json::from_str(&text)
            .unwrap_or_else(|e| panic!("the manifest must be valid JSON: {e}\n{text}"));
        let reason = parsed["anchor"]["resimmable_reason"]
            .as_str()
            .expect("a reason string");
        assert!(
            reason.contains("node \"a\\b\" is\nmissing"),
            "the reason survives escaping intact: {reason}"
        );
    }

    /// The trace retention's own truncation is reported PER CAPTURE and apart
    /// from the frame window's — a lost frame and a lost resume boundary are
    /// different failures and one total could not tell them apart.
    #[test]
    fn the_manifest_reports_trace_truncation_apart_from_frame_truncation() {
        // `the_capture_manifest_states_whether_its_dropped_count_is_exact`
        // renders 12 truncated frames and 5 truncated trace records — two
        // DIFFERENT numbers, so a renderer that printed one for the other fails.
        let plane = FlashbackPlane::new(settings(std::path::PathBuf::from("/tmp/fb")), None);
        let finished = FinishedCapture {
            seq: 1,
            pinned: false,
            causes: Vec::new(),
            causes_dropped: 0,
            causes_dropped_exact: true,
        };
        let text = String::from_utf8(render_capture_manifest(
            &CaptureManifest {
                finished: &finished,
                frames: 10,
                truncated_frames: 12,
                truncated_trace_records: 5,
                headerless_frames: 0,
                floor_ns: 0,
                ended_ns: 1,
                // No truncation in this fixture — the capture reaches
                // exactly the floor it claims, which is the shape every oracle
                // here predates and still describes.
                achieved_from_ns: Some(0),
                topics_with_no_frames: 0,
                window_cap_bytes: 320 * 1024 * 1024,
                anchor: AnchorReport::Absent(NoAnchorReason::NothingRetained),
                // The CARRIED count rides the handoff, which is the one place
                // this manifest states it — there is no top-level
                // `trace_records` key to pin any more.
                handoff: CaptureHandoff {
                    trace_records_carried: 3,
                    ..no_handoff()
                },
                resim: &no_trace(),
                anchor_target_ns: None,
                resim_covered_through_ns: None,
            },
            &plane,
        ))
        .expect("utf-8");
        assert!(text.contains("\"truncated_frames\":12"), "{text}");
        assert!(text.contains("\"truncated_trace_records\":5"), "{text}");
        // The third number, in the handoff's own vocabulary. Still DIFFERENT
        // from the two above, so a renderer printing one for another fails.
        assert!(
            text.contains("carried: 3 scheduler-trace record(s)"),
            "{text}"
        );
    }

    // -----------------------------------------------------------------------
    // The achieved span, beside the claimed one
    // -----------------------------------------------------------------------

    /// One frame of `bytes` payload — the window measures payload bytes, so this
    /// is what the ceiling arithmetic below is denominated in.
    fn window_batch(bytes: usize) -> crate::StagedFrames {
        let mut frames = crate::StagedFrames::default();
        // An UNLABELED (single-writer) frame.
        frames.push(
            Some((0, 0)),
            crate::producer_labeling::FrameOrigin::Unlabeled,
            &vec![0xAB; bytes],
        );
        frames
    }

    /// THE headline defect, driven through a REAL window rather than
    /// handed in: under overload the manifest claimed 45.0 s while the bag
    /// carried 1.99 s — a 22.6x overstatement flagged only by an uninterpretable
    /// `truncated_frames` count.
    ///
    /// # The fixture is the model's SHAPE, scaled — not its numbers pasted in
    ///
    /// What decides the achieved reach is cap ÷ rate, so the byte figures are
    /// scaled down by the same factor as the payloads and the arithmetic is
    /// identical to the 320 MiB / ~164 MiB/s scenario the model ran: exactly
    /// `HOLDS` batches fit, one arrives every `CADENCE`, so the window settles at
    /// `(HOLDS − 1) × CADENCE` of reach WHATEVER its span claims. The eviction
    /// that produces it is the production one — nothing here tells the window
    /// what to keep.
    ///
    /// The three numbers are then hand-computed from the fixture's own constants
    /// rather than copied from the memo, so a change to either would fail here
    /// rather than agreeing with a stale literal.
    #[test]
    fn an_overloaded_capture_reports_the_span_it_achieved_beside_the_one_it_claims() {
        const BATCH_BYTES: usize = 1_000;
        const HOLDS: u64 = 200;
        const CADENCE_NS: u64 = 10_000_000;
        const SPAN_NS: u64 = 30_000_000_000;
        const POST_NS: u64 = 15_000_000_000;
        const TRIGGER_NS: u64 = 60_000_000_000;
        const ENDED_NS: u64 = TRIGGER_NS + POST_NS;

        let mut s = settings(std::path::PathBuf::from("/tmp/fb-overload"));
        s.window_span = Duration::from_nanos(SPAN_NS);
        s.window_max_bytes = BATCH_BYTES as u64 * HOLDS;
        s.policy.post_window_ns = POST_NS;
        let mut plane = FlashbackPlane::new(s, None);

        let mut now = 0u64;
        while now <= ENDED_NS {
            if now == TRIGGER_NS {
                plane.begin_capture(
                    now,
                    std::path::PathBuf::from("/tmp/fb-overload/c.mcap"),
                    None,
                );
            }
            plane.harvest(0, now, window_batch(BATCH_BYTES));
            plane.evict(now);
            now += CADENCE_NS;
        }

        // The capture's own two instants, exactly as `close_capture` reads them.
        let floor_ns = TRIGGER_NS - SPAN_NS;
        let achieved_from_ns = plane.achieved_from_ns(floor_ns);

        // The window really is pinned at its ceiling, and the reach is the
        // ceiling's, not the span's.
        assert_eq!(
            achieved_from_ns,
            Some(ENDED_NS - (HOLDS - 1) * CADENCE_NS),
            "the reach is (HOLDS-1) x CADENCE behind the end, whatever the span promises"
        );
        assert!(
            plane.cap_evicted_frames() > 0,
            "the pre-trigger stretch was evicted by the CEILING, not by the span"
        );
        assert!(
            plane.truncated_frames() > 0,
            "…and the post-trigger inflow went on evicting frames the capture wanted"
        );

        // The overload gap, reported: the surviving range is entirely POST-trigger,
        // so this bag holds ZERO pre-incident frames.
        assert!(
            achieved_from_ns.expect("a reach") > TRIGGER_NS,
            "the model's headline: an overloaded capture's lead-up is gone"
        );

        let finished = FinishedCapture {
            seq: 1,
            pinned: false,
            causes: Vec::new(),
            causes_dropped: 0,
            causes_dropped_exact: true,
        };
        let text = String::from_utf8(render_capture_manifest(
            &CaptureManifest {
                finished: &finished,
                frames: plane.batches_from(floor_ns).count() as u64,
                truncated_frames: plane.truncated_frames(),
                truncated_trace_records: 0,
                headerless_frames: 0,
                floor_ns,
                ended_ns: ENDED_NS,
                achieved_from_ns,
                topics_with_no_frames: plane.topics_with_no_frames(floor_ns, 1),
                window_cap_bytes: plane.window_cap_bytes() as u64,
                anchor: AnchorReport::Absent(NoAnchorReason::NothingRetained),
                handoff: no_handoff(),
                resim: &no_trace(),
                anchor_target_ns: None,
                resim_covered_through_ns: None,
            },
            &plane,
        ))
        .expect("utf-8");
        let doc: serde_json::Value = serde_json::from_str(&text)
            .unwrap_or_else(|e| panic!("the manifest must be valid JSON: {e}\n{text}"));

        // Hand-computed from the fixture's constants — the memo's 45.0 / 1.99 /
        // 43.0, derived rather than pasted.
        let claimed_ms = (ENDED_NS - floor_ns) / 1_000_000;
        let achieved_ms = (HOLDS - 1) * CADENCE_NS / 1_000_000;
        assert_eq!(claimed_ms, 45_000);
        assert_eq!(achieved_ms, 1_990);
        assert_eq!(doc["span_ms"], serde_json::json!(claimed_ms));
        assert_eq!(doc["achieved_span_ms"], serde_json::json!(achieved_ms));
        assert_eq!(
            doc["coverage_shortfall_ms"],
            serde_json::json!(claimed_ms - achieved_ms),
            "43.0 s of the claim is not in this bag, and the manifest says so"
        );
        // The endpoints, so a reader can check the arithmetic rather than
        // trusting it.
        assert_eq!(doc["floor_ns"], serde_json::json!(floor_ns));
        assert_eq!(doc["ended_ns"], serde_json::json!(ENDED_NS));
        assert_eq!(
            doc["achieved_from_ns"],
            serde_json::json!(achieved_from_ns.expect("a reach"))
        );
        // …and the ceiling that caused it, which is the number an operator
        // would otherwise have to guess at.
        assert_eq!(
            doc["window_cap_bytes"],
            serde_json::json!(BATCH_BYTES as u64 * HOLDS)
        );

        // ANTI-DRIFT: the same bytes, through the READER `cerulion bag info`
        // uses. The writer is hand-rolled and the reader is a derive, so this is
        // what stops a field renamed on one side from silently reading `None` on
        // the other — which would render as "this bag predates the field".
        let read: FlashbackManifest = serde_json::from_str(&text).expect("the reader parses it");
        assert_eq!(read.span_ms, Some(claimed_ms));
        assert_eq!(read.achieved_span_ms, Some(achieved_ms));
        assert_eq!(
            read.coverage_shortfall_ms,
            Some(claimed_ms - achieved_ms),
            "every field `bag info` renders must be REACHABLE through the reader"
        );
        assert_eq!(read.window_span_ms, Some(30_000));
        assert_eq!(read.window_cap_bytes, Some(BATCH_BYTES as u64 * HOLDS));
        assert_eq!(read.truncated_frames, Some(plane.truncated_frames()));
        assert_eq!(read.topics_with_no_frames, Some(0));
        assert_eq!(read.seq, Some(1));
        assert_eq!(read.pinned, Some(false));
        assert!(read.frames.is_some());
    }

    /// The wording rule: the CLAIMED window is rendered from this
    /// capture's own endpoints, never from the span constant.
    ///
    /// An EARLY trigger is the case that forces it — the floor is
    /// `T − window_span` SATURATING, so a capture fired five seconds into a run
    /// claims five seconds of lead-up and not thirty. A manifest quoting the
    /// constant there would advertise a look-back the run had not existed long
    /// enough to have.
    #[test]
    fn an_early_trigger_renders_the_window_it_claims_not_the_span_constant() {
        let plane = FlashbackPlane::new(settings(std::path::PathBuf::from("/tmp/fb-early")), None);
        let finished = FinishedCapture {
            seq: 1,
            pinned: false,
            causes: Vec::new(),
            causes_dropped: 0,
            causes_dropped_exact: true,
        };
        // Triggered at 5 s on a 30 s window: the floor saturates to 0, and the
        // capture ends at 20 s.
        let text = String::from_utf8(render_capture_manifest(
            &CaptureManifest {
                finished: &finished,
                frames: 4,
                truncated_frames: 0,
                truncated_trace_records: 0,
                headerless_frames: 0,
                floor_ns: 0,
                ended_ns: 20_000_000_000,
                achieved_from_ns: Some(0),
                topics_with_no_frames: 0,
                window_cap_bytes: 1 << 30,
                anchor: AnchorReport::Absent(NoAnchorReason::NothingRetained),
                handoff: no_handoff(),
                resim: &no_trace(),
                anchor_target_ns: None,
                resim_covered_through_ns: None,
            },
            &plane,
        ))
        .expect("utf-8");
        let doc: serde_json::Value = serde_json::from_str(&text).expect("json");
        assert_eq!(
            doc["span_ms"],
            serde_json::json!(20_000),
            "the claim is this capture's OWN endpoints: {text}"
        );
        assert_eq!(
            doc["window_span_ms"],
            serde_json::json!(30_000),
            "the policy's span is still reported, and is a DIFFERENT number: {text}"
        );
        // …and nothing was lost, so there is no shortfall to report.
        assert_eq!(doc["achieved_span_ms"], serde_json::json!(20_000));
        assert_eq!(doc["coverage_shortfall_ms"], serde_json::json!(0));
    }

    /// A capture that carries NO frames says so, rather than reporting the floor
    /// it claimed as though it had reached it.
    ///
    /// Reachable: the byte pass takes at-or-past-floor frames as truncations, so
    /// an overloaded recorder can finalize a capture holding nothing of its own
    /// window. `null` is the whole point — a reader handed the floor would
    /// believe it had been told the range.
    #[test]
    fn a_capture_that_carries_no_frames_reports_no_reach_rather_than_its_floor() {
        let plane = FlashbackPlane::new(settings(std::path::PathBuf::from("/tmp/fb-empty")), None);
        let finished = FinishedCapture {
            seq: 3,
            pinned: false,
            causes: Vec::new(),
            causes_dropped: 0,
            causes_dropped_exact: true,
        };
        let text = String::from_utf8(render_capture_manifest(
            &CaptureManifest {
                finished: &finished,
                frames: 0,
                truncated_frames: 4_000,
                truncated_trace_records: 0,
                headerless_frames: 0,
                floor_ns: 10_000_000_000,
                ended_ns: 55_000_000_000,
                achieved_from_ns: None,
                topics_with_no_frames: 7,
                window_cap_bytes: 4_096,
                anchor: AnchorReport::Absent(NoAnchorReason::NothingRetained),
                handoff: no_handoff(),
                resim: &no_trace(),
                anchor_target_ns: None,
                resim_covered_through_ns: None,
            },
            &plane,
        ))
        .expect("utf-8");
        let doc: serde_json::Value = serde_json::from_str(&text).expect("json");
        assert_eq!(doc["achieved_from_ns"], serde_json::Value::Null, "{text}");
        assert_eq!(doc["achieved_span_ms"], serde_json::json!(0));
        assert_eq!(
            doc["coverage_shortfall_ms"],
            serde_json::json!(45_000),
            "the whole claim is missing, and the shortfall is the whole claim: {text}"
        );
        // The count rides beside the range, so `[a, b]` is never
        // read as "every topic, throughout".
        assert_eq!(doc["topics_with_no_frames"], serde_json::json!(7));
    }

    /// Anchor accuracy: the bag states which SIDE of the anchor
    /// its gap is on, and at most one of the two figures is ever nonzero.
    ///
    /// The pair exists because a resume re-executes from the anchor onward while
    /// the bag holds frames from its ACHIEVED reach onward. When the anchor is
    /// inside the range the difference is recording a resume will skip
    /// (`frames_before_anchor_ms`); when the ceiling has evicted past the anchor
    /// it is recording the resume NEEDS and the bag has not got
    /// (`frames_missing_after_anchor_ms`) — which the old floor-relative figure
    /// could not express at all.
    #[test]
    fn the_anchor_block_says_which_side_of_the_anchor_the_gap_is_on() {
        // The anchor sits INSIDE the achieved range: 3 s of the bag predates it.
        let inside = manifest(
            AnchorReport::Embedded {
                run_id: 1,
                step: 9,
                nodes: 1,
                complete: 1,
                records: 1,
                fit: AnchorFit::CoversTheClaimedWindow,
                // These arms do not exercise the anchor instant.
                taken_at_ns: 0,
                frames_before_anchor_ms: 3_000,
                frames_missing_after_anchor_ms: 0,
            },
            0,
        );
        let doc: serde_json::Value = serde_json::from_str(&inside).expect("json");
        assert_eq!(doc["anchor"]["frames_before_anchor_ms"], 3_000);
        assert_eq!(doc["anchor"]["frames_missing_after_anchor_ms"], 0);

        // The ceiling evicted PAST the anchor: 8 s the resume needs are gone.
        let missing = manifest(
            AnchorReport::Embedded {
                run_id: 1,
                step: 9,
                nodes: 1,
                complete: 1,
                records: 1,
                fit: AnchorFit::CoversTheClaimedWindow,
                // These arms do not exercise the anchor instant.
                taken_at_ns: 0,
                frames_before_anchor_ms: 0,
                frames_missing_after_anchor_ms: 8_000,
            },
            0,
        );
        let doc: serde_json::Value = serde_json::from_str(&missing).expect("json");
        assert_eq!(doc["anchor"]["frames_before_anchor_ms"], 0);
        assert_eq!(
            doc["anchor"]["frames_missing_after_anchor_ms"], 8_000,
            "a capture whose window was cut short of its own anchor must SAY so: {missing}"
        );
    }

    /// The two ABSENT reasons render apart, and neither ever reads resimmable —
    /// a reader that cannot tell "nothing was captured" from "the window was too
    /// short to reach one" cannot act on either.
    #[test]
    fn an_absent_anchor_names_which_absence_it_is() {
        let nothing = manifest(AnchorReport::Absent(NoAnchorReason::NothingRetained), 0);
        assert!(nothing.contains("\"embedded\":false"), "{nothing}");
        assert!(
            nothing.contains("\"reason\":\"no_anchor_retained\""),
            "{nothing}"
        );
        assert!(nothing.contains("\"resimmable\":false"), "{nothing}");

        let older = manifest(
            AnchorReport::Absent(NoAnchorReason::AllOlderThanTheFrames),
            0,
        );
        assert!(
            older.contains("\"reason\":\"anchors_older_than_the_frames\""),
            "{older}"
        );
        // …and an absent anchor WITH a trace is judged, not assumed: a capture
        // that reaches a mid-run resume point needs an anchor and is refused
        // (`AnchorIncomplete`), which is what this asserts.
        let older_with_trace = manifest_with(
            AnchorReport::Absent(NoAnchorReason::AllOlderThanTheFrames),
            900,
            &Err(ResimGap::AnchorIncomplete {
                detail: "node `relay` has no state".to_string(),
            }),
        );
        assert!(
            older_with_trace.contains("\"resimmable\":false"),
            "a trace without state is no more resumable than state without a trace: \
             {older_with_trace}"
        );

        // …but the ABSENT arm renders the VERDICT, so a capture whose window
        // reaches the run's own step 0 — which resim resumes from the start,
        // needing NO anchor — is reported resimmable rather than refused by a
        // hardcoded `false`. This is the arm a "just write false when there is
        // no anchor" implementation gets wrong.
        let from_start = manifest_with(
            AnchorReport::Absent(NoAnchorReason::NothingRetained),
            900,
            &Ok(()),
        );
        assert!(
            from_start.contains("\"embedded\":false"),
            "still no checkpoint: {from_start}"
        );
        assert!(
            from_start.contains("\"resimmable\":true"),
            "a capture that resumes FROM START needs no anchor: {from_start}"
        );
    }

    /// A capture served an anchor NEWER than its claimed pre-window start says
    /// so: the resume covers less than the capture's frames do, and that is the
    /// operator's judgement to make rather than a silent difference.
    #[test]
    fn an_anchor_newer_than_the_claimed_window_is_served_with_that_fact() {
        let text = manifest(
            AnchorReport::Embedded {
                run_id: 1,
                step: 9,
                nodes: 1,
                complete: 1,
                records: 1,
                fit: AnchorFit::NewerThanTheClaimedWindow,
                // These arms do not exercise the anchor instant.
                taken_at_ns: 0,
                frames_before_anchor_ms: 29_000,
                frames_missing_after_anchor_ms: 0,
            },
            0,
        );
        assert!(
            text.contains("\"fit\":\"newer_than_the_claimed_window\""),
            "{text}"
        );
        assert!(text.contains("\"embedded\":true"), "{text}");
    }

    /// A capture that was handed the run's context SAYS SO, and
    /// says how much of a trace it holds.
    ///
    /// This is the ANTI-TAUTOLOGY half of the absence arm below: without it, a
    /// renderer that hardcodes every slot to its absence string passes that arm
    /// character for character while reporting a fully-described capture as
    /// carrying nothing.
    #[test]
    fn a_capture_handed_the_run_context_reports_each_slot_as_carried() {
        let plane = FlashbackPlane::new(settings(std::path::PathBuf::from("/tmp/fb")), None);
        let finished = FinishedCapture {
            seq: 3,
            pinned: false,
            causes: Vec::new(),
            causes_dropped: 0,
            causes_dropped_exact: true,
        };
        let text = String::from_utf8(render_capture_manifest(
            &CaptureManifest {
                // Not what these arms are about — the neutral value, on
                // `no_handoff()`'s own rule.
                anchor: AnchorReport::Absent(NoAnchorReason::NothingRetained),
                truncated_trace_records: 0,
                resim: &no_trace(),
                anchor_target_ns: None,
                resim_covered_through_ns: None,
                finished: &finished,
                frames: 10,
                truncated_frames: 0,
                headerless_frames: 0,
                floor_ns: 0,
                ended_ns: 1_000_000_000,
                // No truncation in this fixture — the capture reaches
                // exactly the floor it claims, which is the shape every oracle
                // here predates and still describes.
                achieved_from_ns: Some(0),
                topics_with_no_frames: 0,
                window_cap_bytes: 320 * 1024 * 1024,
                handoff: CaptureHandoff {
                    graph_yaml: true,
                    env_json: true,
                    recorder_json: true,
                    state_plane: true,
                    // The `--record` shape, and the one that matters
                    // most: this recorder really does hold TWO trace
                    // rings (a multi-process deployment hands one per rank), and
                    // the capture still carries no scheduler trace, because
                    // `write_capture` has no trace channel. The verdict must
                    // describe the BAG.
                    trace_records_carried: 0,
                    trace_rings_configured: 2,
                    // The ordinary shape — nothing trimmed away, nothing
                    // evicted, so the absence arm is the nothing-was-there one.
                    trace_records_trimmed_away: 0,
                    trace_records_truncated: 0,
                    // No drain ran in this fixture — see the fields' docs
                    // for why `None` is not the same claim as `from_start`.
                    trace_attach: None,
                    trace_ring_records: 0,
                    trace_laps: 0,
                },
            },
            &plane,
        ))
        .expect("utf-8");
        let doc: serde_json::Value = serde_json::from_str(&text)
            .unwrap_or_else(|e| panic!("the manifest must be valid JSON: {e}\n{text}"));
        let h = &doc["handoff"];
        assert_eq!(h["graph"], serde_json::json!(HANDOFF_EMBEDDED), "{text}");
        assert_eq!(h["env"], serde_json::json!(HANDOFF_EMBEDDED), "{text}");
        assert_eq!(h["recorder"], serde_json::json!(HANDOFF_EMBEDDED), "{text}");
        assert_eq!(
            h["state_plane"],
            serde_json::json!(HANDOFF_STATE_PLANE),
            "{text}"
        );
        assert_eq!(
            h["trace"],
            serde_json::json!(HANDOFF_TRACE_NONE_NOT_CARRIED),
            "a capture that carries NO trace must say so BY CAUSE, whatever its recorder was \
             configured with — reporting the configured count here advertises replay capability \
             the .mcap does not contain: {text}"
        );
        assert_eq!(
            h["trace_rings_configured"],
            serde_json::json!(2),
            "…and the configured fact survives, under a name that says what it is: {text}"
        );
    }

    /// The trace verdict describes the
    /// BAG, and the two absence causes are DIFFERENT.
    ///
    /// A verdict read off `cfg.rings.len()` is wrong: a
    /// `--record` run's capture would render `attached: 1 trace ring(s)` while
    /// `crate::capture::write_capture` — whose whole input is attachments, the
    /// coverage manifest, the schema catalog and frame batches — persists ZERO
    /// scheduler-trace records. A reader acting on that goes to `bag play
    /// --resim`, which refuses at `BagNoSchedulerTrace`.
    ///
    /// All three arms in ONE body, because the claim is a PARTITION over one
    /// input pair and splitting it would let a renderer satisfy each arm with a
    /// different constant.
    /// `select_trace` recovers `target(S-1)` even when
    /// the anchor step's boundary was drained BELOW the capture's frame floor.
    ///
    /// The rule itself is oracle-tested in `trace_window`; this is the WIRING —
    /// without it the recovery exists and nothing calls it, and every arm over
    /// there stays green (the inert-shipping shape). The straddle it drives is
    /// the ordinary one: `AnchorWindow::select` picks the newest checkpoint at
    /// or before the deadline, and the eligible band's lower edge IS the floor,
    /// so a boundary drained one pass before its own anchor sits below it.
    #[test]
    fn select_trace_recovers_an_anchor_target_drained_below_the_capture_floor() {
        use cerulion_core::trace_ring::{
            TraceRingRecord, RECORD_TYPE_FIRE, RECORD_TYPE_STEP_BOUNDARY,
        };
        const MS: u64 = 1_000_000;
        let rec = |step: u64, kind: u32, target: u64| TraceRingRecord {
            step,
            fire_time_ns: target,
            duration_ns: 0,
            node_idx: 0,
            global_level: 0,
            record_type: kind,
            reserved: 0,
        };

        let plane = FlashbackPlane::new(settings(std::path::PathBuf::from("/tmp/fb-target")), None);
        {
            let shared = plane.trace();
            let mut held = lock_trace(&shared);
            // The anchor step's own boundary, drained BELOW the floor.
            held.push(
                10 * MS,
                vec![
                    rec(3, RECORD_TYPE_STEP_BOUNDARY, 3_000),
                    rec(3, RECORD_TYPE_FIRE, 3_000),
                ],
            );
            // …and the resumed window, drained above it.
            held.push(
                20 * MS,
                vec![
                    rec(4, RECORD_TYPE_STEP_BOUNDARY, 4_000),
                    rec(4, RECORD_TYPE_FIRE, 4_000),
                ],
            );
        }

        let snapshot = plane.snapshot_trace_from(20 * MS);
        let trimmed = plane.select_trace(&snapshot, 3, &["ticker".to_string()]);
        // PRECONDITION: the floor really did hide the anchor's boundary, so the
        // trim could not have read it on its way past.
        assert_eq!(
            trimmed.discarded, 0,
            "PRECONDITION: the floor excluded the anchor step entirely, so the trim discarded \
             nothing — which is exactly why it could not recover the target on its own"
        );
        assert_eq!(
            trimmed.anchor_target_ns,
            Some(3_000),
            "the capture must still state `target(S-1)`; a `None` here writes JSON null, the \
             replay's external-frame prefix skip goes away, and the first resumed step is \
             injected with the whole retained pre-window"
        );
        // …and the kept records are unchanged: the recovery is a lookup, never a
        // widening of what the capture carries.
        assert_eq!(trimmed.records.len(), 2);
        assert_eq!(trimmed.first_recorded_step, Some(4));
    }

    #[test]
    fn the_trace_verdict_describes_the_capture_not_the_recorders_configuration() {
        let plane = FlashbackPlane::new(settings(std::path::PathBuf::from("/tmp/fb")), None);
        let finished = FinishedCapture {
            seq: 0,
            pinned: false,
            causes: Vec::new(),
            causes_dropped: 0,
            causes_dropped_exact: true,
        };
        let verdict_with = |carried: u64,
                            configured: usize,
                            trimmed_away: usize,
                            truncated: u64|
         -> serde_json::Value {
            let text = String::from_utf8(render_capture_manifest(
                &CaptureManifest {
                    // Not what these arms are about — the neutral value, on
                    // `no_handoff()`'s own rule.
                    anchor: AnchorReport::Absent(NoAnchorReason::NothingRetained),
                    truncated_trace_records: truncated,
                    resim: &no_trace(),
                    anchor_target_ns: None,
                    resim_covered_through_ns: None,
                    finished: &finished,
                    frames: 1,
                    truncated_frames: 0,
                    headerless_frames: 0,
                    floor_ns: 0,
                    ended_ns: 1,
                    // No truncation in this fixture — the capture reaches
                    // exactly the floor it claims, which is the shape every oracle
                    // here predates and still describes.
                    achieved_from_ns: Some(0),
                    topics_with_no_frames: 0,
                    window_cap_bytes: 320 * 1024 * 1024,
                    handoff: CaptureHandoff {
                        graph_yaml: true,
                        env_json: true,
                        recorder_json: true,
                        state_plane: true,
                        trace_records_carried: carried,
                        trace_rings_configured: configured,
                        trace_records_trimmed_away: trimmed_away,
                        trace_records_truncated: truncated,
                        // No drain ran in this fixture — see the fields' docs
                        // for why `None` is not the same claim as `from_start`.
                        trace_attach: None,
                        trace_ring_records: 0,
                        trace_laps: 0,
                    },
                },
                &plane,
            ))
            .expect("utf-8");
            serde_json::from_str::<serde_json::Value>(&text)
                .unwrap_or_else(|e| panic!("valid JSON: {e}\n{text}"))["handoff"]["trace"]
                .clone()
        };
        // The ordinary shape: nothing trimmed away, nothing evicted.
        let verdict = |carried: u64, configured: usize| verdict_with(carried, configured, 0, 0);

        // (a) no rings, nothing carried — the always-on shape.
        assert_eq!(
            verdict(0, 0),
            serde_json::json!(HANDOFF_TRACE_NONE_NO_RINGS)
        );
        // (b) rings HELD, nothing carried — the `--record` shape, and the
        // case that matters. It must NOT borrow (a)'s text, which is false about this
        // recorder, and must not claim an attachment, which is false about the
        // bag.
        assert_eq!(
            verdict(0, 1),
            serde_json::json!(HANDOFF_TRACE_NONE_NOT_CARRIED)
        );
        // …and the two causes are DISTINCT, which is the whole point of a closed
        // vocabulary rather than one shared "no trace".
        assert_ne!(HANDOFF_TRACE_NONE_NO_RINGS, HANDOFF_TRACE_NONE_NOT_CARRIED);
        // The cause must name the writer's ACTUAL scope: the writer
        // persists a trace channel, so the
        // text "frames and attachments only" is a claim this build falsifies.
        //
        // In the other direction, this arm must not
        // require the text to name BOTH "trim" and "ceiling", which would be a
        // defect — a disjunction asserted over a case where NEITHER happened.
        // This constant is the arm for exactly that case, so it must name
        // the absence WITHOUT attributing it to either sibling.
        //
        // The needle is the ATTRIBUTION, not the bare word — checking for
        // "trim" and "ceiling" as SUBSTRINGS fails on the fixed text, because
        // a cause that says "nothing was
        // trimmed away and nothing was evicted" DENIES both while containing
        // both words. What was wrong before was the DISJUNCTION ("either the
        // trim … or the … ceiling"), which asserts one of them ran without
        // knowing which, so that is what must not come back.
        for attribution in ["either the trim", "or the trace retention"] {
            assert!(
                !HANDOFF_TRACE_NONE_NOT_CARRIED.contains(attribution),
                "the nothing-was-there cause must not attribute the absence to a trim or a \
                 ceiling that did not run: {HANDOFF_TRACE_NONE_NOT_CARRIED}"
            );
        }
        // …and it must make the positive claim instead, or a reader is left to
        // infer the cause from an absence of one.
        assert!(
            HANDOFF_TRACE_NONE_NOT_CARRIED.contains("nothing was there"),
            "…and must say what DID happen: {HANDOFF_TRACE_NONE_NOT_CARRIED}"
        );
        // …and must never borrow its sibling's cause, which is the confusion the
        // closed vocabulary exists to prevent.
        assert!(
            !HANDOFF_TRACE_NONE_NOT_CARRIED.contains("handed no trace ring"),
            "a recorder that HOLDS rings must not be described as having none: \
             {HANDOFF_TRACE_NONE_NOT_CARRIED}"
        );

        // (c) the three causes, chosen from the capture's own
        //     numbers. The earlier code had one string for all of them, whose
        //     text asserted "either the trim … or the ceiling" — an
        //     exhaustive-sounding disjunction over a case where NEITHER ran,
        //     which is what a configured ring that simply produced nothing looks
        //     like. That sends an operator to widen a window or raise a ceiling
        //     that were never the problem.
        assert_eq!(
            verdict_with(0, 1, 7, 0),
            serde_json::json!(HANDOFF_TRACE_NONE_TRIMMED_AWAY),
            "records were retained and the TRIM discarded them all"
        );
        assert_eq!(
            verdict_with(0, 1, 0, 9),
            serde_json::json!(HANDOFF_TRACE_NONE_CEILING_TOOK_THEM),
            "the byte CEILING evicted them"
        );
        assert_eq!(
            verdict_with(0, 1, 0, 0),
            serde_json::json!(HANDOFF_TRACE_NONE_NOT_CARRIED),
            "neither ran, so nothing was there — and the text must say so rather than guess"
        );
        // The CEILING outranks the trim when both fired: it is the one cause of
        // the three an operator can act on directly, and it cannot mislead in
        // that order (a capture that hit its ceiling really did lose records to
        // it, whatever the trim then did with what survived).
        assert_eq!(
            verdict_with(0, 1, 7, 9),
            serde_json::json!(HANDOFF_TRACE_NONE_CEILING_TOOK_THEM)
        );
        // ANTI-TAUTOLOGY: a capture that CARRIES records reports the count on
        // every one of those inputs, so the three arms above are about the
        // absence rather than about the two new numbers being non-zero.
        for (trimmed_away, truncated) in [(0, 0), (7, 0), (0, 9), (7, 9)] {
            assert_eq!(
                verdict_with(3, 1, trimmed_away, truncated),
                serde_json::json!("carried: 3 scheduler-trace record(s)")
            );
        }
        // …and the three causes are DISTINCT, so an operator can tell which
        // remedy is theirs.
        let absences = [
            HANDOFF_TRACE_NONE_NOT_CARRIED,
            HANDOFF_TRACE_NONE_TRIMMED_AWAY,
            HANDOFF_TRACE_NONE_CEILING_TOOK_THEM,
        ];
        let distinct: std::collections::BTreeSet<&str> = absences.iter().copied().collect();
        assert_eq!(distinct.len(), absences.len(), "{absences:?}");
        // Only the CEILING arm names a knob, because only it has one. The env
        // var is spelled from the constant so a rename cannot leave the message
        // naming a variable nothing reads (the per-verb remedy rule).
        assert!(
            HANDOFF_TRACE_NONE_CEILING_TOOK_THEM
                .contains(cerulion_core::flashback::FLASHBACK_TRACE_MAX_MB_ENV),
            "the ceiling cause must name the knob that raises it: \
             {HANDOFF_TRACE_NONE_CEILING_TOOK_THEM}"
        );
        for other in [
            HANDOFF_TRACE_NONE_NOT_CARRIED,
            HANDOFF_TRACE_NONE_TRIMMED_AWAY,
        ] {
            assert!(
                !other.contains("CERULION_FLASHBACK_TRACE_MAX"),
                "a cause a ceiling did not produce must not send an operator to raise one: {other}"
            );
        }
    }

    /// …and the CARRIED arm is reachable TODAY, which is what makes the trace-window
    /// seam a data flip rather than a string edit.
    ///
    /// Nothing in this build supplies a nonzero count (the flashback writer has
    /// no trace channel), so without this arm the `> 0` branch would be dead
    /// code that the trace window would have to discover works. Driving it here fixes the
    /// rendered shape now: a COUNT, so a reader can tell a one-rank run from a
    /// multi-process deployment, and so the flip is visible rather than claimed.
    #[test]
    fn a_capture_that_carries_trace_records_says_how_many() {
        let plane = FlashbackPlane::new(settings(std::path::PathBuf::from("/tmp/fb")), None);
        let finished = FinishedCapture {
            seq: 0,
            pinned: false,
            causes: Vec::new(),
            causes_dropped: 0,
            causes_dropped_exact: true,
        };
        let text = String::from_utf8(render_capture_manifest(
            &CaptureManifest {
                // Not what these arms are about — the neutral value, on
                // `no_handoff()`'s own rule.
                anchor: AnchorReport::Absent(NoAnchorReason::NothingRetained),
                truncated_trace_records: 0,
                resim: &no_trace(),
                anchor_target_ns: None,
                resim_covered_through_ns: None,
                finished: &finished,
                frames: 1,
                truncated_frames: 0,
                headerless_frames: 0,
                floor_ns: 0,
                ended_ns: 1,
                // No truncation in this fixture — the capture reaches
                // exactly the floor it claims, which is the shape every oracle
                // here predates and still describes.
                achieved_from_ns: Some(0),
                topics_with_no_frames: 0,
                window_cap_bytes: 320 * 1024 * 1024,
                handoff: CaptureHandoff {
                    graph_yaml: true,
                    env_json: true,
                    recorder_json: true,
                    state_plane: true,
                    trace_records_carried: 1234,
                    trace_rings_configured: 2,
                    // The ordinary shape — nothing trimmed away, nothing
                    // evicted, so the absence arm is the nothing-was-there one.
                    trace_records_trimmed_away: 0,
                    trace_records_truncated: 0,
                    // No drain ran in this fixture — see the fields' docs
                    // for why `None` is not the same claim as `from_start`.
                    trace_attach: None,
                    trace_ring_records: 0,
                    trace_laps: 0,
                },
            },
            &plane,
        ))
        .expect("utf-8");
        let doc: serde_json::Value =
            serde_json::from_str(&text).unwrap_or_else(|e| panic!("valid JSON: {e}\n{text}"));
        assert_eq!(
            doc["handoff"]["trace"],
            serde_json::json!("carried: 1234 scheduler-trace record(s)"),
            "{text}"
        );
        assert_eq!(
            doc["handoff"]["trace_rings_configured"],
            serde_json::json!(2)
        );
    }

    /// Every absence is named BY ITS CAUSE, and the five causes
    /// are DIFFERENT — which is the whole point of a closed vocabulary rather
    /// than a shared `"absent"`.
    ///
    /// This is the shape the always-on window recorder took before the argv
    /// handoff existed: no graph, no env, no host identity, no capture-plane tag
    /// and no ring — five distinct things missing for four distinct reasons,
    /// every one of them invisible in the capture.
    #[test]
    fn a_capture_handed_nothing_names_each_absence_by_its_own_cause() {
        let plane = FlashbackPlane::new(settings(std::path::PathBuf::from("/tmp/fb")), None);
        let finished = FinishedCapture {
            seq: 1,
            pinned: false,
            causes: Vec::new(),
            causes_dropped: 0,
            causes_dropped_exact: true,
        };
        let text = String::from_utf8(render_capture_manifest(
            &CaptureManifest {
                // Not what these arms are about — the neutral value, on
                // `no_handoff()`'s own rule.
                anchor: AnchorReport::Absent(NoAnchorReason::NothingRetained),
                truncated_trace_records: 0,
                resim: &no_trace(),
                anchor_target_ns: None,
                resim_covered_through_ns: None,
                finished: &finished,
                frames: 10,
                truncated_frames: 0,
                headerless_frames: 0,
                floor_ns: 0,
                ended_ns: 1_000_000_000,
                // No truncation in this fixture — the capture reaches
                // exactly the floor it claims, which is the shape every oracle
                // here predates and still describes.
                achieved_from_ns: Some(0),
                topics_with_no_frames: 0,
                window_cap_bytes: 320 * 1024 * 1024,
                handoff: no_handoff(),
            },
            &plane,
        ))
        .expect("utf-8");
        let doc: serde_json::Value = serde_json::from_str(&text)
            .unwrap_or_else(|e| panic!("the manifest must be valid JSON: {e}\n{text}"));
        let h = &doc["handoff"];
        assert_eq!(
            h["graph"],
            serde_json::json!(HANDOFF_GRAPH_ABSENT),
            "{text}"
        );
        assert_eq!(h["env"], serde_json::json!(HANDOFF_ENV_ABSENT), "{text}");
        assert_eq!(
            h["recorder"],
            serde_json::json!(HANDOFF_RECORDER_ABSENT),
            "{text}"
        );
        assert_eq!(
            h["state_plane"],
            serde_json::json!(HANDOFF_STATE_PLANE_ABSENT),
            "{text}"
        );
        assert_eq!(
            h["trace"],
            serde_json::json!(HANDOFF_TRACE_NONE_NO_RINGS),
            "{text}"
        );
        assert_eq!(h["trace_rings_configured"], serde_json::json!(0), "{text}");

        // The vocabulary is CLOSED and its members are DISTINCT. A set equality
        // over five strings is what stops the five slots collapsing onto one
        // "absent" that tells an operator which thing is missing but never why.
        let causes = [
            HANDOFF_GRAPH_ABSENT,
            HANDOFF_ENV_ABSENT,
            HANDOFF_RECORDER_ABSENT,
            HANDOFF_STATE_PLANE_ABSENT,
            HANDOFF_TRACE_NONE_NO_RINGS,
            HANDOFF_TRACE_NONE_NOT_CARRIED,
            // The two causes a single NOT-CARRIED string must not
            // assert as a disjunction. They join the CLOSED set here, or the
            // set-equality above would stop being the inventory it claims to be.
            HANDOFF_TRACE_NONE_TRIMMED_AWAY,
            HANDOFF_TRACE_NONE_CEILING_TOOK_THEM,
        ];
        let distinct: std::collections::BTreeSet<&str> = causes.iter().copied().collect();
        assert_eq!(distinct.len(), causes.len(), "{causes:?}");

        // …and each names the CONSEQUENCE, not merely the gap. These are the
        // words an operator holding an un-resimmable capture reads.
        assert!(
            HANDOFF_GRAPH_ABSENT.contains("re-executed"),
            "{HANDOFF_GRAPH_ABSENT}"
        );
        // This pin must not REQUIRE the two things the
        // string may not say: `--record` and a tracking issue's id. That would read
        // "must name the run shape that mints rings" while asserting the
        // opposite: `--record` is not a run shape, it is a flag on another
        // verb, and a reader cannot open a tracking issue. So the assertion is
        // on the CONTENT — what the string must say, and what
        // it must not say — rather than only a prohibition, because a pin that
        // only forbids words is satisfied by a string that says nothing.
        assert!(
            HANDOFF_TRACE_NONE_NO_RINGS.contains("scheduler trace")
                && HANDOFF_TRACE_NONE_NO_RINGS.contains("The RUN minted none")
                && HANDOFF_TRACE_NONE_NO_RINGS.contains("single-process")
                && HANDOFF_TRACE_NONE_NO_RINGS.contains("trace_rings"),
            "the trace absence must name the run SHAPE that mints none and point at the run's \
             own record of why: {HANDOFF_TRACE_NONE_NO_RINGS}"
        );
        assert!(
            !HANDOFF_TRACE_NONE_NO_RINGS.contains("--record")
                && !HANDOFF_TRACE_NONE_NO_RINGS.contains("--no-rings"),
            "project rule: an absence names the CAUSE, never another verb's flag, and \
             every run now mints rings, so `--record` would be false as well as forbidden: \
             {HANDOFF_TRACE_NONE_NO_RINGS}"
        );
        assert!(
            HANDOFF_STATE_PLANE_ABSENT.contains("CERULION_FLASHBACK"),
            "a refused plane has an OPERATOR-visible cause and the string must name it: \
             {HANDOFF_STATE_PLANE_ABSENT}"
        );
    }

    /// The sweep carries out `plan_retention`'s plan and NOTHING else: a pinned
    /// capture survives even when it is the oldest, and the pin is a SIBLING
    /// FILE so the sweep never has to open a bag to see it.
    #[test]
    fn the_sweep_evicts_the_oldest_unpinned_and_never_a_pinned_one() {
        let dir = std::env::temp_dir().join(format!(
            "sweep-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_nanos())
                .unwrap_or(0)
        ));
        std::fs::create_dir_all(&dir).expect("temp dir");

        // Three captures of 100 bytes, written in age order. Their mtimes are
        // non-decreasing by construction, and `plan_retention` breaks a tie by
        // NAME — which here agrees with creation order, so the plan is
        // deterministic whatever the filesystem's timestamp resolution is (this
        // test is about the SWEEP carrying out a plan, not about the ordering,
        // which is oracle-tested with no filesystem in `retention`).
        for name in ["a.mcap", "b.mcap", "c.mcap"] {
            std::fs::write(dir.join(name), vec![0u8; 100]).expect("write");
        }
        write_pin_marker(&dir.join("a.mcap"));

        // A cap of TWO captures. `a` is pinned, so the oldest unpinned goes.
        sweep_capture_dir(
            &dir,
            RetentionCaps {
                max_bytes: u64::MAX,
                max_captures: 2,
            },
        );
        assert!(
            dir.join("a.mcap").exists(),
            "a PINNED capture is never a candidate"
        );
        assert!(
            !dir.join("b.mcap").exists(),
            "the oldest UNPINNED capture is evicted"
        );
        assert!(dir.join("c.mcap").exists());

        std::fs::remove_dir_all(&dir).ok();
    }

    /// A directory the recorder cannot read is not a crash and not a claim: the
    /// sweep does nothing, which is the only safe answer when the listing is
    /// unknown (evicting on an empty listing would delete nothing; PLANNING on
    /// one would be worse if the plan were ever inverted).
    #[test]
    fn a_missing_capture_directory_sweeps_to_a_no_op() {
        let dir = std::env::temp_dir().join("does-not-exist-ever");
        assert_eq!(scan_capture_dir(&dir), Some(Vec::new()));
        sweep_capture_dir(&dir, RetentionCaps::default());
    }

    /// One complete single-record anchor, for the selection arms below.
    fn probe_anchor(step: u64) -> crate::anchor_window::HarvestedAnchor {
        crate::anchor_window::HarvestedAnchor {
            run_id: 1,
            step,
            node_idx: 0,
            ring: "r".into(),
            node: Some("n".into()),
            kind: crate::anchor_window::AnchorKind::Complete,
            records: vec![[0u8; 512]],
        }
    }

    /// The anchor deadline is `T - post`, derived from the
    /// capture's FROZEN TRIGGER instant — never from the close time, and never
    /// from the floor.
    ///
    /// The first defect: a capture closes at `T + post`, so a `now_ns - post` deadline was
    /// `T` — one whole post window LATE.
    ///
    /// The second defect: the first fix derived it as `floor + (span - post)`, on the
    /// identity `floor == T - span`. That identity FAILS whenever the floor
    /// saturates, which is exactly the early-capture case.
    ///
    /// Asserted on the ARITHMETIC (hand-computed instants), not on the shipped
    /// numbers: the property is that only the FROZEN trigger is read.
    #[test]
    fn the_anchor_deadline_is_the_trigger_minus_the_post_window() {
        const MS: u64 = 1_000_000;
        let plane = FlashbackPlane::new(settings(std::path::PathBuf::from("/tmp/fb")), None);

        // A capture triggered at T = 100 s: the claimed pre-window starts at
        // T - post_window (15 s) = 85 s.
        let trigger = 100_000 * MS;
        assert_eq!(plane.claimed_window_start_ns(trigger), 85_000 * MS);
        // The first defect, spelled out: at the close instant (T + post = 115 s)
        // `now - post` is 100 s — a full post window late.
        assert_ne!(
            plane.claimed_window_start_ns(trigger),
            (115_000 * MS) - plane.settings().policy.post_window_ns,
            "a deadline read off the CLOSE time is the first defect"
        );

        // THE SATURATION EDGE. A capture triggered at 5 s, before a 30 s window has
        // elapsed, has floor `5 s - 30 s` SATURATED to zero. The first-fix
        // expression then reads `0 + (30 - 15) = 15 s` — a deadline TEN SECONDS
        // AFTER the trigger, so a checkpoint taken at 10 s (five seconds after the
        // incident) would have been reported as covering the pre-incident window.
        let early = 5_000 * MS;
        let span_ns = plane.settings().window_span.as_nanos() as u64;
        let post_ns = plane.settings().policy.post_window_ns;
        let saturated_floor = early.saturating_sub(span_ns);
        assert_eq!(
            saturated_floor, 0,
            "precondition: the floor really saturates"
        );
        assert_eq!(
            plane.claimed_window_start_ns(early),
            0,
            "a capture triggered inside its own post window can claim nothing earlier \
             than instant zero"
        );
        assert_ne!(
            plane.claimed_window_start_ns(early),
            saturated_floor.saturating_add(span_ns.saturating_sub(post_ns)),
            "the FIRST-FIX floor expression reads 15 s here — later than the trigger itself, \
             which is the second defect"
        );

        // ...and the two agree wherever the floor does NOT saturate, which is why
        // the first-fix expression looked total.
        assert_eq!(
            plane.claimed_window_start_ns(trigger),
            (trigger - span_ns).saturating_add(span_ns.saturating_sub(post_ns)),
            "with an un-saturated floor the two derivations are identical"
        );
    }

    /// The saturation edge through the production `select_anchor`: on an EARLY
    /// capture, a checkpoint taken AFTER the trigger is not reported as covering
    /// the pre-incident window.
    ///
    /// This is not a corner. An always-on black box exists to catch the fault
    /// that happens in a run's first seconds, so the early capture is a shape it
    /// has to get right — and the anchor here was taken five seconds after the
    /// incident it is supposed to precede.
    #[test]
    fn an_early_capture_does_not_claim_coverage_from_an_anchor_taken_after_the_trigger() {
        const MS: u64 = 1_000_000;
        let plane = FlashbackPlane::new(settings(std::path::PathBuf::from("/tmp/fb")), None);

        // Trigger at 5 s on a 30 s window => floor saturates to 0.
        let trigger = 5_000 * MS;
        let floor_ns = trigger.saturating_sub(plane.settings().window_span.as_nanos() as u64);
        assert_eq!(floor_ns, 0, "precondition: the floor really saturates");

        // A checkpoint at 10 s — FIVE SECONDS AFTER the trigger.
        lock_anchors(&plane.anchors).admit(10_000 * MS, probe_anchor(700));
        let (picked, fit) = plane.select_anchor(floor_ns, trigger).expect("selected");
        assert_eq!(picked.step, 700);
        assert_eq!(
            fit,
            AnchorFit::NewerThanTheClaimedWindow,
            "an anchor taken AFTER the incident cannot cover the window before it — under \
             the first-fix floor derivation this read `covers_the_claimed_window`"
        );

        // ANTI-TAUTOLOGY: the same EARLY capture with an anchor genuinely at or
        // before its claimed start still reports covering, so the arm is about the
        // SATURATION and not about an early capture never covering anything.
        let mut tight = settings(std::path::PathBuf::from("/tmp/fb"));
        tight.policy.post_window_ns = 1_000 * MS;
        let later = FlashbackPlane::new(tight, None);
        let trigger = 20_000 * MS;
        let floor_ns = trigger.saturating_sub(later.settings().window_span.as_nanos() as u64);
        assert_eq!(
            floor_ns, 0,
            "still an early capture: 20 s into a 30 s window"
        );
        lock_anchors(&later.anchors).admit(15_000 * MS, probe_anchor(800));
        let (picked, fit) = later.select_anchor(floor_ns, trigger).expect("selected");
        assert_eq!(picked.step, 800);
        assert_eq!(
            fit,
            AnchorFit::CoversTheClaimedWindow,
            "15 s is at or before this capture's claimed start (20 - 1 = 19 s)"
        );
    }

    /// The selection USES that deadline: a checkpoint too new for the claimed
    /// window is reported as such rather than passed off as covering it.
    ///
    /// Driven through the production `select_anchor` over a real retention, so it
    /// fails if the deadline is computed correctly and then not used.
    #[test]
    fn a_checkpoint_newer_than_the_claimed_window_start_is_not_reported_as_covering_it() {
        const MS: u64 = 1_000_000;
        let plane = FlashbackPlane::new(settings(std::path::PathBuf::from("/tmp/fb")), None);
        let floor_ns = 70_000 * MS;
        // A capture triggered at 100 s on a 30 s window: the floor did NOT
        // saturate, so `T` and `floor + span` agree and this arm is about the
        // deadline rather than about the early-capture edge above.
        const TRIGGER_NS: u64 = 100_000 * MS;

        // ONE checkpoint, at 90 s: inside the bag's frames (floor 70 s) but AFTER
        // the claimed window start (85 s). Under the original close-time deadline
        // (100 s) this reported `CoversTheClaimedWindow`.
        lock_anchors(&plane.anchors).admit(90_000 * MS, probe_anchor(500));
        let (picked, fit) = plane.select_anchor(floor_ns, TRIGGER_NS).expect("selected");
        assert_eq!(picked.step, 500);
        assert_eq!(
            fit,
            AnchorFit::NewerThanTheClaimedWindow,
            "a checkpoint taken after the claimed pre-window start covers less than the \
             capture claims, and must SAY so"
        );

        // ANTI-TAUTOLOGY: a checkpoint at or before the claimed start reads as
        // covering it, so the arm above is about the DEADLINE and not about a
        // plane that never reports the covering arm.
        {
            let mut held = lock_anchors(&plane.anchors);
            held.clear();
            held.admit(80_000 * MS, probe_anchor(400));
        }
        let (picked, fit) = plane.select_anchor(floor_ns, TRIGGER_NS).expect("selected");
        assert_eq!(picked.step, 400);
        assert_eq!(fit, AnchorFit::CoversTheClaimedWindow);
    }

    /// On a window configured SHORTER than the post window, a
    /// checkpoint at the capture's own FLOOR covers the whole window it claims —
    /// and must say so.
    ///
    /// `CERULION_FLASHBACK_WINDOW_MS` takes any positive value and the shipped
    /// post window is 15 s, so a 5 s window is a supported configuration. The
    /// floor is then `T − 5 s` while the claimed start is `T − 15 s`, which is
    /// EARLIER — and `select` requires a candidate to be at or after the floor
    /// AND at or before the deadline, two conditions nothing can satisfy at once.
    /// The covering arm was unreachable by construction on such a robot: every
    /// capture reported the degraded label, including one whose anchor covers
    /// every frame in the bag, so the field said the same thing forever and
    /// carried no information.
    ///
    /// The pair is what makes this about the CLAMP rather than about a plane that
    /// reports covering unconditionally: at the floor covers, one millisecond
    /// after it does not.
    #[test]
    fn a_checkpoint_at_the_floor_covers_a_window_shorter_than_the_post_window() {
        const MS: u64 = 1_000_000;
        let mut short = settings(std::path::PathBuf::from("/tmp/fb"));
        short.window_span = Duration::from_secs(5);
        let plane = FlashbackPlane::new(short, None);

        let trigger = 100_000 * MS;
        let floor_ns = trigger - plane.settings().window_span.as_nanos() as u64;
        assert_eq!(
            floor_ns,
            95_000 * MS,
            "precondition: the floor did not saturate"
        );
        assert!(
            plane.claimed_window_start_ns(trigger) < floor_ns,
            "PRECONDITION, and the whole defect: the claimed start (85 s) precedes the \
             oldest frame this capture holds (95 s), so an unclamped deadline is \
             unsatisfiable"
        );

        // AT the floor: this checkpoint covers every frame the capture carries,
        // which is the most any anchor could do.
        lock_anchors(&plane.anchors).admit(floor_ns, probe_anchor(900));
        let (picked, fit) = plane.select_anchor(floor_ns, trigger).expect("selected");
        assert_eq!(picked.step, 900);
        assert_eq!(
            fit,
            AnchorFit::CoversTheClaimedWindow,
            "an anchor at the oldest frame in the bag covers the whole window this capture \
             claims — unclamped, `select` could report no checkpoint as covering at all"
        );

        // ONE MILLISECOND after it: the bag holds frames the resume will not
        // re-execute, so the degraded label is the true one.
        {
            let mut held = lock_anchors(&plane.anchors);
            held.clear();
            held.admit(floor_ns + MS, probe_anchor(901));
        }
        let (picked, fit) = plane.select_anchor(floor_ns, trigger).expect("selected");
        assert_eq!(picked.step, 901);
        assert_eq!(
            fit,
            AnchorFit::NewerThanTheClaimedWindow,
            "the clamp raises the deadline to the floor and no further — a checkpoint that \
             leaves frames un-re-executed still says so"
        );

        // …and it TIGHTENS nothing where the window is at least the post window,
        // which is every shipped configuration: there the claimed start is at or
        // after the floor and the clamp is a no-op, so a checkpoint strictly
        // newer than the floor still covers.
        let ordinary = FlashbackPlane::new(settings(std::path::PathBuf::from("/tmp/fb")), None);
        let floor_ns = 70_000 * MS;
        lock_anchors(&ordinary.anchors).admit(80_000 * MS, probe_anchor(902));
        let (picked, fit) = ordinary.select_anchor(floor_ns, trigger).expect("selected");
        assert_eq!(picked.step, 902);
        assert_eq!(
            fit,
            AnchorFit::CoversTheClaimedWindow,
            "80 s is after the floor (70 s) and at or before the claimed start (85 s) — a \
             clamp that raised the deadline to the floor unconditionally would fail here"
        );
    }

    /// REPORTED: a reader can see how much of the
    /// bag's frame span a resume will NOT re-execute.
    #[test]
    fn an_embedded_anchor_states_how_much_of_the_bag_predates_it() {
        let text = manifest(
            AnchorReport::Embedded {
                run_id: 1,
                step: 9,
                nodes: 1,
                complete: 1,
                records: 1,
                fit: AnchorFit::CoversTheClaimedWindow,
                // These arms do not exercise the anchor instant.
                taken_at_ns: 0,
                frames_before_anchor_ms: 14_500,
                frames_missing_after_anchor_ms: 0,
            },
            0,
        );
        assert!(
            text.contains("\"frames_before_anchor_ms\":14500"),
            "a resume covers from the ANCHOR onward while the bag holds more frames than \
             that, and the difference must be readable: {text}"
        );
    }

    /// An UNKNOWN primary cause is UNKNOWN — never the coalesced cause behind it.
    ///
    /// The eviction class is defined POSITIONALLY (the request that OPENED the
    /// capture, which `render_cause_marker` writes first), so reading it off
    /// `parse_cause_marker`'s output inherits that reader's SKIP: a marker
    /// written by a NEWER recorder whose primary is a kind this build has never
    /// heard of, followed by an ordinary coalesced cause, PROMOTES the coalesced
    /// one. Diversity eviction would then count the capture into a class it does
    /// not belong to and target the wrong class's captures — quietly, on the one
    /// deployment shape (a mixed-version rollout) where nobody is looking for it.
    ///
    /// BOTH readers are driven over the SAME bytes in one body, because the
    /// asymmetry is the whole design: the PRIMARY is `None` (an explicit unknown,
    /// the same degradation an absent `.cause` already takes) while the gate
    /// seed still adopts every line it CAN name, so a cause this build
    /// understands keeps its refractory floor across a restart.
    #[test]
    fn an_unrecognised_primary_cause_is_unknown_not_the_coalesced_one_behind_it() {
        // The shape a NEWER recorder writes: a kind this build cannot name,
        // first, followed by one it can.
        let body = "some_future_kind\tgraph/p0\nmonitor_verdict\tstalled:local:/a\n";

        assert_eq!(
            parse_primary_cause(body),
            None,
            "the eviction class is the FIRST line or nothing — promoting the \
             coalesced cause would file this capture under a class it never had"
        );

        // …while the SEED reader is deliberately tolerant, and keeps what it can.
        assert_eq!(
            parse_cause_marker(body),
            vec![(TriggerKind::MonitorVerdict, "stalled:local:/a".to_string())],
            "a cause this build understands must still seed its floor"
        );

        // ANTI-TAUTOLOGY: the same marker with a RECOGNISED primary reads that
        // primary — otherwise `None` above is satisfied by a reader that answers
        // `None` for everything.
        let known = "process_fault\tgraph/p0\nmonitor_verdict\tstalled:local:/a\n";
        assert_eq!(parse_primary_cause(known), Some(TriggerKind::ProcessFault));

        // A MALFORMED first line (no tab) is the same answer for the same
        // reason: the primary is that line, and it names no cause.
        assert_eq!(
            parse_primary_cause("garbage\nprocess_fault\tgraph/p0\n"),
            None
        );

        // …and an empty marker has no first line at all.
        assert_eq!(parse_primary_cause(""), None);
    }

    /// BOTH writer-settle sites release the writer-held gauge.
    ///
    /// A STRUCTURAL pin: `window.rs`'s own arms call
    /// `note_writer_released` directly, so they are blind to a plane that never
    /// calls it — and a gauge that only ever RISES reports memory nothing holds,
    /// for the rest of the run, on the one number that exists to make the
    /// writer-held memory visible.
    ///
    /// TWO sites, because a capture settles two ways: `poll_writer` on the drive
    /// loop and `join_writer` at shutdown. Pinning one would leave the other free
    /// to drift, which is how a wiring fix half-lands.
    #[test]
    fn both_writer_settle_sites_release_the_writer_held_gauge() {
        let src = include_str!("flashback_plane.rs");
        let code: String = src
            .lines()
            .map(|line| match line.find("//") {
                Some(i) => &line[..i],
                None => line,
            })
            .collect::<Vec<_>>()
            .join("\n");
        for site in ["fn join_writer", "fn poll_writer"] {
            let start = code
                .find(site)
                .unwrap_or_else(|| panic!("{site} must exist"));
            // Generous window: over-reading can only make this STRICTER, and
            // both bodies are well under it.
            let body = &code[start..(start + 1200).min(code.len())];
            assert!(
                body.contains("self.window.note_writer_released()"),
                "{site} settles a capture, so it must release the gauge:\n{body}"
            );
        }
    }

    /// A MEASURED generation re-splits the plane LIVE, and
    /// the cost lands on the frame window.
    ///
    /// This is the wiring the pure `split_plane_budget` oracles cannot see: they
    /// prove the arithmetic, and stay green if nothing ever calls it, if the new
    /// ceilings are never applied, or if they are applied to the wrong retention.
    /// Driven through the real `evict_anchors`, which is the pass a running
    /// recorder takes.
    ///
    /// The window's LIVE cap must shrink — the rule "the cost of big state
    /// lands on window seconds", made observable.
    ///
    /// # What this arm does NOT re-prove, and why
    ///
    /// That the window then EVICTS down to its new cap is `window.rs`'s own
    /// proven behaviour (`the_byte_ceiling_bites_through_a_capture_and_says_that
    /// _it_did`). Re-proving it here would need the window filled past 100 MiB,
    /// because the frames floor is 64 MiB and every realistic plane is far above
    /// it — a heavyweight fixture for a property already pinned. What is NOT
    /// provable anywhere else, and is what this arm exists for, is that the
    /// split reaches the retention's live ceiling at all.
    ///
    /// # The numbers are chosen so the frames genuinely GIVE
    ///
    /// With a DERIVED anchor cap the plane is 2W, so the reserve only takes from
    /// the frames when `3G > W` while `3G + floor <= 2W`. W = 200 MiB and
    /// G = 100 MiB sits inside that band: the reserve is 300 MiB and the frames
    /// fall from 200 MiB to 100 MiB. A fixture outside it would show the frames
    /// GROWING (the anchor's derived half being handed back), which is real
    /// behaviour but not the property named here.
    #[test]
    fn a_measured_generation_re_splits_the_plane_and_the_frames_pay_for_it() {
        const MIB: u64 = 1024 * 1024;
        const REC: usize = 512;
        let dir = std::env::temp_dir().join(format!("split-{}", std::process::id()));
        let mut s = settings(dir.clone());
        // DERIVED on both sides, so the reserve is free to move — an `Env` basis
        // would cap it and this arm would be measuring the clamp instead.
        s.window_max_bytes = 200 * MIB;
        s.anchor_max_bytes = 200 * MIB;
        s.anchor_cap_basis = cerulion_core::flashback::CapBasis::Window;
        let mut plane = FlashbackPlane::new(s, None);

        // Nothing measured: the shipped caps stand, unchanged.
        assert_eq!(
            plane.budget_split().verdict,
            cerulion_core::flashback::PlaneSplitVerdict::Unmeasured
        );
        assert_eq!(plane.window_cap_bytes() as u64, 200 * MIB);
        assert_eq!(
            plane.measured_generation_bytes(),
            None,
            "nothing measured is NOT a measurement of zero"
        );

        // MEASURE a 100 MiB generation: one anchor, one step, whose record count
        // is what `byte_len` reads.
        {
            let shared = plane.anchors();
            let mut retention = lock_anchors(&shared);
            let mut anchor = probe_anchor(1);
            anchor.records = vec![[0u8; REC]; (100 * MIB as usize) / REC];
            retention.admit(9_000_000, anchor);
        }
        plane.evict_anchors(9_000_000);

        assert_eq!(
            plane.measured_generation_bytes(),
            Some(100 * MIB),
            "the demand is the COMPLETE bytes of the biggest checkpoint held"
        );
        assert_eq!(
            plane.budget_split(),
            cerulion_core::flashback::PlaneSplit {
                anchor_bytes: 300 * MIB,
                frames_bytes: 100 * MIB,
                verdict: cerulion_core::flashback::PlaneSplitVerdict::Reserved { generations: 3 },
            },
            "three generations fit beside the floor, so the anchor takes 300 MiB \
             of the 400 MiB plane"
        );

        // THE WIRING CLAIM: both live ceilings moved, not just the bookkeeping.
        assert_eq!(
            plane.window_cap_bytes() as u64,
            100 * MIB,
            "the frame window's LIVE ceiling must be the split's frames budget — \
             a split nobody applied leaves this at 200 MiB"
        );
        assert_eq!(
            lock_anchors(&plane.anchors()).max_bytes_for_test(),
            300 * MIB,
            "…and the anchor retention's, or the reserve is a number in a struct"
        );

        std::fs::remove_dir_all(&dir).ok();
    }

    /// By design a capture triggered seconds after boot, before one whole window
    /// span has elapsed, still selects an anchor.
    ///
    /// # Why this arm is a GUARD and not a fix
    ///
    /// The design requires the state plane to take its first generation at arm
    /// time so a young run can resim. Measured against this tree, that is
    /// ALREADY what happens, by two mechanisms:
    ///
    /// 1. `state_arm_attach::arm_capture_plane` makes its single `arm()` call
    ///    with `first_anchor_step = 0`, and `state_arm::cadence_due` is TRUE at
    ///    step 0 (pinned by `the_shipping_cadence_example_matches_the_design`).
    ///    The boot generation is taken at step 0, not one cadence later.
    /// 2. `AnchorWindow::select` falls back to `NewerThanTheClaimedWindow` when
    ///    no checkpoint precedes the deadline — exactly the young-run shape,
    ///    because the claimed-window start saturates to 0 when the trigger is
    ///    younger than the post window.
    ///
    /// What was pinned NOWHERE is their COMPOSITION: that a young run's boot
    /// anchor survives the deadline arithmetic and is actually served. The two
    /// halves are pinned apart, in two crates, so a change to either could break
    /// the outcome the design names while both stayed green. The outcome
    /// therefore gets its own oracle.
    ///
    /// `NothingRetained`, the older behaviour the brief expected to find, is
    /// reachable ONLY from an empty retention, i.e. a state-plane-less or
    /// arm-failed run, which is the residual the design itself says to keep.
    #[test]
    fn a_capture_triggered_before_one_window_has_elapsed_still_selects_its_boot_anchor() {
        let dir = std::env::temp_dir().join(format!("young-{}", std::process::id()));
        let plane = FlashbackPlane::new(settings(dir.clone()), None);

        // The BOOT anchor: taken a few milliseconds into the run, which is what a
        // step-0 checkpoint really looks like on the recorder's clock.
        const BOOT_NS: u64 = 5_000_000;
        {
            let shared = plane.anchors();
            lock_anchors(&shared).admit(BOOT_NS, probe_anchor(0));
        }

        // A capture triggered 8 s in. The floor saturates to 0 (the run has not
        // lived a whole span) and so does the claimed-window deadline, so the
        // boot anchor is NEWER than the window this capture claims.
        const TRIGGER_NS: u64 = 8_000_000_000;
        let (checkpoint, fit) = plane.select_anchor(0, TRIGGER_NS).expect(
            "project rule: a young run must still resim, since an anchor exists and must be \
             served, not refused for being newer than a window the run is too young \
             to have",
        );
        assert_eq!(
            checkpoint.taken_at_ns, BOOT_NS,
            "and it must be the BOOT anchor, not some other checkpoint"
        );
        assert_eq!(
            fit,
            AnchorFit::NewerThanTheClaimedWindow,
            "…served under the ACCURATE label: it really is newer than the claimed \
             pre-window, and saying otherwise would over-claim coverage"
        );

        // ANTI-TAUTOLOGY: an EMPTY retention still refuses, so the arm above is
        // the fallback doing its job rather than `select_anchor` answering `Ok`
        // to everything.
        let empty = FlashbackPlane::new(settings(dir.clone()), None);
        assert!(
            matches!(
                empty.select_anchor(0, TRIGGER_NS),
                Err(crate::anchor_window::NoAnchorReason::NothingRetained)
            ),
            "a run whose state plane never armed has nothing to serve, and that is the \
             residual the young-run rule keeps"
        );

        std::fs::remove_dir_all(&dir).ok();
    }
}
