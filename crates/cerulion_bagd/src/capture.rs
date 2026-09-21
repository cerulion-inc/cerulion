//! Writing a FLASHBACK — turning the rolling window into one bag.
//!
//! # Why this is not the recorder's writer thread
//!
//! bagd already has a writer thread, and it belongs to the CONTINUOUS bag: it
//! owns rotation, the trace rings, the state rings, the coverage manifests and a
//! chunk clock. A flashback is none of those things — it is a one-shot dump of
//! frames the recorder is already holding, into a file that is finalized
//! immediately and never rotated.
//!
//! It is also written CONCURRENTLY with recording, on the `--record` path, so it
//! cannot borrow that thread even if the shapes matched.
//!
//! # …and not the drive loop either
//!
//! A ~155 MB write on a Jetson's eMMC is seconds. The drive loop is what drains
//! the taps, and each topic's own SHM queue is the loss boundary (0.25 to 1 s
//! on a kHz topic), so a capture written inline would cost the robot
//! exactly the frames the capture exists to preserve. So it goes to its own
//! thread, the drive loop keeps draining, and the outcome is collected on a later
//! pass.
//!
//! `BagWriter` is `!Send`, so it is CREATED on that thread, never moved to it —
//! the same discipline `WriterCore::create` follows for the continuous bag.

use cerulion_core::flashback::trigger::CaptureCause;
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::mpsc::{self, Receiver};
use std::sync::Arc;
use std::thread::JoinHandle;

use cerulion_bag::{
    BagSchemaCatalog, BagWriter, BagWriterConfig, ProducerAttribution, ProducerRecord, TopicSchema,
};

use crate::StagedFrames;

/// The MCAP `library` string a flashback carries.
///
/// Distinct from the continuous bag's, so a reader (and an operator running
/// `cerulion bag info`) can tell a flashback from a recording without inspecting
/// its attachments.
const FLASHBACK_LIBRARY: &str = "cerulion-flashback";

/// The attachment naming what this capture was ABOUT.
///
/// `pub` because `cerulion bag info` READS it: a durable
/// artifact no shipped command ever surfaces is a claim nobody can check (the
/// `record_coverage.json` rule).
pub const FLASHBACK_ATTACHMENT: &str = "__cerulion/flashback.json";

/// The node state a capture carries.
///
/// # Raw records, on the bag's own reserved channel
///
/// The records go onto `__cerulion/state` VERBATIM, which is the same channel
/// and the same bytes a `--record` bag holds for the same anchor — so
/// `replay_state::read_bag_anchors` reads a capture through exactly the path it
/// already reads a recording through, with no new framing and no second reader.
pub(crate) struct CaptureAnchor {
    /// The checkpoint's records, in admission then part order — one entry per
    /// ANCHOR, each SHARED with the retention rather than copied.
    ///
    /// Grouped rather than flat because that is how the retention stores them,
    /// and un-grouping would mean copying the records back out — the very copy
    /// this shape removes. The write order is unchanged: groups in admission
    /// order, records in part order — the order the retention holds them in, and
    /// the order `Checkpoint::record_groups` yields them.
    pub records: Vec<Arc<Vec<crate::anchor_window::StateRecord>>>,
}

impl CaptureAnchor {
    /// Every record, flattened back into the order the bag is written in.
    fn records(&self) -> impl Iterator<Item = &crate::anchor_window::StateRecord> {
        self.records.iter().flat_map(|group| group.iter())
    }
}

/// Everything the writer thread needs, all of it owned and `Send`.
pub(crate) struct CaptureJob {
    /// Where the bag lands.
    pub path: PathBuf,
    /// The channel set, resolved at CAPTURE time rather than at arm time — so a
    /// topic the recorder learned after arming still gets a channel here.
    pub topics: Vec<TopicSchema>,
    /// The catalog for those channels.
    pub catalog: BagSchemaCatalog,
    /// `graph.yaml` / `env.json` / `recorder.json`, if the recorder was given
    /// them — the same attachments the continuous bag carries, so a flashback is
    /// as self-describing as a recording.
    pub attachments: Vec<(String, Vec<u8>)>,
    /// `__cerulion/flashback.json`.
    pub coverage: Vec<u8>,
    /// `__cerulion/record_coverage.json`: the
    /// CAPTURE-SCOPED coverage manifest.
    ///
    /// A DIFFERENT document from [`coverage`](Self::coverage) above, which is the
    /// capture's own `flashback.json` (what the window was ABOUT: its span, its
    /// causes, its resim verdict). This one is the `--record` path's manifest, in
    /// the same type and read by the same readers, answering the one question
    /// `flashback.json` does not: WHICH topics are in this bag and HOW EACH GOT
    /// THERE.
    ///
    /// It is what makes a capture re-executable at all on a shared machine. The
    /// window recorder records what is LIVE on the default iceoryx2 namespace, so
    /// a capture holds any co-tenant topic that was streaming; without a manifest
    /// marking those `source: discovered`, `bag play --resim` refuses the whole
    /// file as corrupt. See `Recorder::build_capture_coverage`.
    ///
    /// EMPTY only if the manifest would not serialize — deliberately still
    /// written, because both readers render an unparseable manifest as MALFORMED
    /// while an ABSENT one reads as "recorded by a build that never measured",
    /// and only the first of those is true here.
    pub record_coverage: Vec<u8>,
    /// Chunk ceiling, mirrored from the recorder's config.
    pub chunk_max_bytes: usize,
    /// The node state to resume from, when the retention had one to give.
    ///
    /// `None` is a real and ordinary answer — see
    /// [`AnchorReport`](crate::flashback_plane::AnchorReport) — and the capture's
    /// own manifest says WHICH absence it is.
    pub anchor: Option<CaptureAnchor>,
    /// `__cerulion/state_coverage.json`.
    ///
    /// # Why this is NOT a field of [`CaptureAnchor`]
    ///
    /// It was, and that lost the catch-up clamp on a capture with no anchor.
    /// The manifest carries TWO independent things: the `node_idx -> node id`
    /// table, without which the state RECORDS are unreadable, and the ARMED
    /// PLANE (`armed.first_anchor_step`), which is where
    /// `replay_state::read_state_arm` reads the decision-41 clamp onset from.
    ///
    /// The first is meaningless without an anchor; the second is not. Hanging
    /// the manifest off the anchor tied them together, so a capture whose
    /// retention held no checkpoint wrote no manifest, and a from-start replay
    /// of it silently ran UNCLAMPED — firing `Period` catch-up bursts the
    /// recording never ran, reported as a divergence the candidate did not
    /// cause.
    ///
    /// `None` means the recorder had nothing to say about either (no anchor and
    /// no armed plane), and then the bag is byte-identical to one written before
    /// this field existed.
    pub state_coverage: Option<Vec<u8>>,
    /// Stage A: the SCHEDULER TRACE, already trimmed to the anchor.
    ///
    /// Written onto the RESERVED `__cerulion/scheduler_trace` channel, which
    /// `BagWriter::create` auto-registers in every bag — so this needs no entry
    /// in [`topics`](Self::topics) and no new framing. The records are the ring's
    /// own 40-byte wire form, so a capture's trace bytes are the bytes a
    /// recording's are and resim reads a capture through the path it already
    /// reads a recording through.
    ///
    /// Empty is the EXCEPTION now, not the ordinary case: a window-only
    /// recorder on a multi-process run is handed that run's trace rings, so a
    /// capture normally carries records. It is still reachable — a wall-gated run
    /// mints no ring, a refused run has none, and a ring can yield nothing inside
    /// this capture's window — and where it is, the capture's manifest says which
    /// rather than implying a resume.
    pub trace: Vec<cerulion_core::trace_ring::TraceRingRecord>,
    /// The frames, OLDEST FIRST, each already paired with its topic name.
    ///
    /// Names rather than indices: the window keys on an index into the
    /// recorder's live tap list, which the writer thread cannot see and which
    /// may grow (discovery attaches taps) while the capture is being written.
    ///
    /// SHARED with the window rather than copied out of it — see
    /// [`WindowBatch::frames`](crate::window::WindowBatch::frames) for the cost
    /// that buys and the accounting change it forces.
    pub batches: Vec<(String, Arc<StagedFrames>)>,
    /// TEST SEAM: hold the capture thread before it writes anything.
    ///
    /// The smallest gate that can produce a STALLED capture writer, which is the
    /// one condition under which the run-vanished latch could hold the drive loop
    /// open forever. `fault_inject_writer_stall_gate` cannot serve: it gates the
    /// CONTINUOUS writer, a different thread on a different path.
    ///
    /// `None` on every production path — `BagdConfig::new` never sets it — and the
    /// field is feature-gated so it does not exist in a shipping recorder.
    #[cfg(any(test, feature = "test-helpers"))]
    pub stall: Option<Arc<crate::WriterStallGate>>,
    /// Decision R2: per topic, the first publisher its tap ever saw.
    ///
    /// Named for what it IS, not for what it is used for. It is READ as the
    /// attribution of the unlabelled run preceding a capture's first labelled
    /// frame — but on a DECLARED `multi_publisher` tap there is no such run
    /// (labelling armed at open, so the first labelled frame sits at ordinal 0
    /// and the `ordinal > 0` guard below never consults this), and a name like
    /// `prefix_origins` would assert a "prefix" semantic the value does not
    /// carry on those topics.
    ///
    /// # Why a capture needs this at all
    ///
    /// A staged frame carries [`FrameOrigin::Labeled`](crate::producer_labeling::FrameOrigin)
    /// only once its topic is BEING LABELLED — declared `multi_publisher`, or
    /// observed to have a second writer. On a topic whose plurality was
    /// OBSERVED, the frames staged before the arming frame carry
    /// `Unlabeled`, and their publisher id was never stored (that is the whole
    /// point of the enum: `Unlabeled` means NO LABEL IS OWED, not "unknown", and
    /// paying 16 bytes a frame on every single-writer topic is the cost it
    /// avoids). A capture window that reaches back past the arming frame
    /// therefore holds a leading run this bag cannot attribute from the frames
    /// alone.
    ///
    /// It CAN attribute it from one number: every frame before arming carried the
    /// tap's FIRST origin, because arming happens on the first frame that did not
    /// match it. So the run is written as ONE
    /// [`ProducerAttribution::CatchUpPrefix`] naming this id — the same record
    /// the continuous writer mints for the same run, for the same reason.
    ///
    /// Keyed by topic NAME, the key [`batches`](Self::batches) already uses, so
    /// there is no parallel index to get wrong. A topic ABSENT here has no first
    /// origin recorded (its tap drained no frame), which is a state a capture
    /// holding its frames cannot be in — and if it somehow were, the capture
    /// writes its `FrameLabel`s and NO catch-up rather than inventing one.
    pub first_origins: HashMap<String, u128>,
}

/// What a finished capture produced.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct CaptureStats {
    /// The bag's size on disk.
    pub bytes: u64,
    /// DATA messages persisted — the capture's frames.
    pub messages: u64,
    /// State records persisted onto `__cerulion/state`.
    ///
    /// Counted APART from `messages`, on the `frames_lost`/`untapped` precedent:
    /// they answer different questions ("how much of the world did this capture
    /// hold?" versus "can it be resumed?"), and folding them would let a capture
    /// with a big checkpoint and no frames look well-covered.
    pub state_records: u64,
    /// Scheduler-trace records persisted onto `__cerulion/scheduler_trace`.
    ///
    /// Counted APART from `messages` and from `state_records`, on the same rule
    /// those two are kept apart by: they answer different questions ("how much of
    /// the world did this hold?", "can it be resumed?", "from WHERE"), and one
    /// total would let a capture with a big trace and no frames look
    /// well-covered.
    pub trace_records: u64,
    /// Decision R2: `FrameLabel` records written onto the reserved
    /// `__cerulion/frame_producers` channel — one per labelled frame.
    ///
    /// Counted APART from `messages` on the same rule `state_records` and
    /// `trace_records` are: they answer a different question ("can a reader say
    /// WHO wrote each frame?"), and folding them into the message total would
    /// let a capture of a shared topic look as though it held twice the frames
    /// it does. ZERO on the overwhelmingly common single-writer capture, which
    /// is what makes a non-zero value mean something.
    ///
    /// FRAME LABELS ONLY, with catch-ups counted separately in
    /// [`catch_up_records`](Self::catch_up_records) — because that is what this
    /// NAME means everywhere else in the crate, and a name that means two
    /// different things in two places is how two surfaces come to disagree about
    /// one bag. `TopicHealth` pairs `producer_labels: u64` with a separate
    /// `label_catch_up: bool`; the recorder's own per-tap accounting counts
    /// `origin.is_labeled()` and `owed.is_some()` apart; and `bag info` renders a
    /// per-topic label COUNT beside a separate catch-up marker. Folding the two
    /// here would have made this capture's log line and its own `bag info`
    /// PRODUCERS block differ by exactly the number of catch-up records — the
    /// same class as the `fs::metadata` re-read below, which exists so an
    /// operator comparing the reported size with `ls` does not find them
    /// different.
    pub producer_labels: u64,
    /// Decision R2: `CatchUpPrefix` records written onto the same
    /// reserved channel — at most ONE per labelled topic, attributing the run of
    /// frames this capture holds from before its topic's arming frame.
    ///
    /// Apart from [`producer_labels`](Self::producer_labels) for the reason
    /// given there. It is also a different QUANTITY: a frame label names one
    /// frame, a catch-up names a RUN, so adding them counts nothing an operator
    /// can act on.
    pub catch_up_records: u64,
    /// Frames whose wire header would not parse.
    ///
    /// Carried out to the manifest. MCAP demands a sequence and two
    /// timestamps for every message, so a headerless frame is written with
    /// zeros — and a reader shown `sequence: 0, log_time: 0` with nothing else to
    /// go on would read INVENTED values as this frame's real chronology and
    /// identity. The payload is still preserved (a black box that dropped the one
    /// malformed frame around an incident would be discarding the evidence); what
    /// changes is that the bag SAYS how many of its messages carry placeholder
    /// metadata, exactly as the continuous recorder's `headerless` counter does.
    pub headerless: u64,
}

/// A capture being written.
pub(crate) struct CaptureWriter {
    /// The capture's sequence number, as the trigger gate assigned it.
    pub seq: u64,
    /// Where it is landing.
    pub path: PathBuf,
    /// Whether it is excluded from retention eviction.
    pub pinned: bool,
    /// Stage A: whether `bag play --resim` will accept this capture.
    ///
    /// Carried on the WRITER rather than recomputed at the publish site,
    /// because the verdict is a property of what was WRITTEN and the writer is
    /// the only thing that outlives `close_capture`'s scope. `None` is UNKNOWN
    /// — see [`FlashbackOutcome::Finished`].
    ///
    /// [`FlashbackOutcome::Finished`]: cerulion_core::flashback::channel::FlashbackOutcome::Finished
    pub resimmable: Option<bool>,
    /// What this capture CLAIMS to cover and what it CARRIES.
    ///
    /// Carried on the writer for exactly the reason `resimmable` is: both are
    /// properties of the closed capture, and the writer is the only thing that
    /// outlives `close_capture`'s scope. NOT an `Option` here — this recorder
    /// always measured both, and the wire's absence arm means "an older recorder
    /// made no claim", which is a different statement and not one this process
    /// can make about its own capture.
    pub span: cerulion_core::flashback::channel::FinishedSpan,
    /// Every request id that asked for this capture — one `Finished` verdict per
    /// requester, so an operator whose request COALESCED into somebody else's
    /// capture still hears how it ended.
    pub requesters: Vec<u64>,
    /// Every cause this capture recorded, carried so the
    /// sibling CAUSE MARKER can be written beside the finished bag.
    ///
    /// On the WRITER for the same reason `resimmable` is: the marker is written
    /// at the publish site, which runs after `close_capture`'s scope is gone, and
    /// the writer is the only thing that outlives it. Re-reading the gate there
    /// would read the NEXT capture's causes.
    pub causes: Vec<CaptureCause>,
    join: Option<JoinHandle<()>>,
    rx: Receiver<Result<CaptureStats, String>>,
}

impl CaptureWriter {
    /// Start writing `job` on its own thread.
    pub(crate) fn spawn(
        seq: u64,
        pinned: bool,
        resimmable: Option<bool>,
        span: cerulion_core::flashback::channel::FinishedSpan,
        requesters: Vec<u64>,
        causes: Vec<CaptureCause>,
        job: CaptureJob,
    ) -> std::io::Result<Self> {
        let path = job.path.clone();
        let (tx, rx) = mpsc::channel();
        let join = std::thread::Builder::new()
            .name("bagd-flashback".to_string())
            .spawn(move || {
                // The result is SENT rather than returned through the join
                // handle, so the drive loop can poll for it without blocking on
                // a join — a capture that hangs on a full disk must not wedge
                // the recorder.
                let _ = tx.send(write_capture(job));
            })?;
        Ok(Self {
            seq,
            path,
            pinned,
            resimmable,
            span,
            requesters,
            causes,
            join: Some(join),
            rx,
        })
    }

    /// Wait for the write, bounded. `None` means STILL WRITING.
    ///
    /// # The timeout must NOT become an `Err`
    ///
    /// An `Err` reads as a SETTLED
    /// capture, so the caller took the writer out of the plane and dropped it —
    /// DETACHING a thread that was still appending — published a terminal
    /// `Failed` for a bag that might yet finalize, and let the retention sweep
    /// run over a directory holding that file. Three contradictions from one
    /// arm, and `captures_abandoned` stayed zero because the arm meant to count
    /// them was unreachable.
    ///
    /// `None` is the only correct answer: the writer is retained, and the caller
    /// decides what to do about a capture that outlived the budget.
    pub(crate) fn join_bounded(
        &mut self,
        deadline: std::time::Duration,
    ) -> Option<Result<CaptureStats, String>> {
        match self.rx.recv_timeout(deadline) {
            Ok(result) => {
                if let Some(join) = self.join.take() {
                    let _ = join.join();
                }
                Some(result)
            }
            // STILL WRITING — the handle is deliberately NOT taken.
            Err(mpsc::RecvTimeoutError::Timeout) => None,
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                if let Some(join) = self.join.take() {
                    let _ = join.join();
                }
                Some(Err("the flashback writer thread panicked".to_string()))
            }
        }
    }

    /// Has the write finished? Non-blocking.
    pub(crate) fn poll(&mut self) -> Option<Result<CaptureStats, String>> {
        match self.rx.try_recv() {
            Ok(result) => {
                // Reap the thread now that it has spoken, so a long-lived
                // recorder does not accumulate un-joined threads across
                // captures.
                if let Some(join) = self.join.take() {
                    let _ = join.join();
                }
                Some(result)
            }
            Err(mpsc::TryRecvError::Empty) => None,
            Err(mpsc::TryRecvError::Disconnected) => {
                // The thread went away without sending — it PANICKED. Reported
                // as a failure rather than treated as "still running", which
                // would leave the requester waiting for a verdict that can never
                // come.
                if let Some(join) = self.join.take() {
                    let _ = join.join();
                }
                Some(Err("the flashback writer thread panicked".to_string()))
            }
        }
    }
}

/// Write one capture. Runs on the capture thread.
fn write_capture(job: CaptureJob) -> Result<CaptureStats, String> {
    // TEST SEAM: block here so a test can observe a STALLED capture writer.
    // Before any file is created, so the stall is total.
    #[cfg(any(test, feature = "test-helpers"))]
    if let Some(gate) = job.stall.as_ref() {
        gate.entered
            .store(true, std::sync::atomic::Ordering::Release);
        while gate.engaged.load(std::sync::atomic::Ordering::Acquire) {
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
    }
    if let Some(parent) = job.path.parent() {
        std::fs::create_dir_all(parent)
            .map_err(|e| format!("could not create {}: {e}", parent.display()))?;
    }
    let cfg = BagWriterConfig {
        chunk_max_bytes: job.chunk_max_bytes,
        library: FLASHBACK_LIBRARY.to_string(),
        ..Default::default()
    };
    let mut writer = BagWriter::create(&job.path, cfg, &job.topics)
        .map_err(|e| format!("could not create the flashback bag: {e}"))?;
    for (name, bytes) in &job.attachments {
        writer
            .write_attachment(name, crate::attachment_media_type(name), 0, 0, bytes)
            .map_err(|e| format!("could not write the {name} attachment: {e}"))?;
    }
    writer
        .write_attachment(
            FLASHBACK_ATTACHMENT,
            "application/json",
            0,
            0,
            &job.coverage,
        )
        .map_err(|e| format!("could not write {FLASHBACK_ATTACHMENT}: {e}"))?;
    // The capture-scoped coverage manifest, under the SAME attachment
    // name a `--record` bag uses — one document, one reader, and replay's
    // existing unmodelled-topic escape applies to a capture with no change at all.
    writer
        .write_attachment(
            crate::RECORD_COVERAGE_ATTACHMENT,
            "application/json",
            0,
            0,
            &job.record_coverage,
        )
        .map_err(|e| format!("could not write {}: {e}", crate::RECORD_COVERAGE_ATTACHMENT))?;
    writer
        .write_schema_catalog(&job.catalog)
        .map_err(|e| format!("could not write the schema catalog: {e}"))?;

    let mut messages = 0u64;
    let mut headerless = 0u64;
    let mut state_records = 0u64;

    // The node state FIRST, so the bag reads in the order a
    // resume uses it — resume from here, then these frames.
    //
    // `log_time` is the recorder's wall clock, read ONCE for the whole
    // checkpoint. A state record carries a STEP and no nanosecond ("a step is
    // not a nanosecond" — `WriterCore::flush_batch`), so the only meaningful stamp
    // is when the recorder wrote it; reading the clock per record would give one
    // checkpoint a spread of times that means nothing. The continuous recorder
    // stamps its state records the same way, so a capture and a recording are
    // consistent here too.
    if let Some(anchor) = &job.anchor {
        let channel = writer.state_channel_id();
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_nanos() as u64);
        for (i, record) in anchor.records().enumerate() {
            writer
                .write_message(channel, i as u32, stamp, stamp, &[record.as_slice()])
                .map_err(|e| format!("could not write a state record: {e}"))?;
            state_records += 1;
        }
    }
    // The manifest WITHOUT which those records are unreadable — written even
    // when the record list is empty, so a reader is told "this capture's
    // checkpoint carried nothing" rather than being left with the same silence
    // a pre-checkpoint bag gives.
    //
    // OUTSIDE the anchor block, deliberately: it also carries the ARMED PLANE a
    // from-start replay reads its catch-up clamp onset from, which is a fact
    // about the RUN rather than about the checkpoint — see
    // [`CaptureJob::state_coverage`].
    if let Some(coverage) = &job.state_coverage {
        writer
            .write_attachment(
                crate::state_coverage::STATE_COVERAGE_ATTACHMENT,
                "application/json",
                0,
                0,
                coverage,
            )
            .map_err(|e| {
                format!(
                    "could not write {}: {e}",
                    crate::state_coverage::STATE_COVERAGE_ATTACHMENT
                )
            })?;
    }
    // Stage A: the SCHEDULER TRACE, after the state and before the
    // frames — the order a resume uses the bag in: resume from here, replay
    // these steps, against these frames.
    //
    // `log_time`/`publish_time` are the record's own `fire_time_ns`, which is
    // what the continuous recorder stamps for the same records. That is a GATING
    // clock rather than a wall one, and using it here is what keeps a capture's
    // trace messages indistinguishable from a recording's — a reader that time-
    // aligned them differently would be reading a different bag.
    let mut trace_records = 0u64;
    for (i, record) in job.trace.iter().enumerate() {
        writer
            .write_scheduler_trace(i as u32, record.fire_time_ns, record.fire_time_ns, record)
            .map_err(|e| format!("could not write a scheduler-trace record: {e}"))?;
        trace_records += 1;
    }
    // Decision R2: the capture's producer-label state.
    //
    // # The ordinal base is 0 AT THIS BAG'S FIRST FRAME OF THE CHANNEL
    //
    // Not a choice this code makes — the definition it inherits. A
    // `FrameLabel`'s ordinal is documented as "that frame's 0-based index in
    // that channel's stream of frames written to THIS BAG" (`ProducerRecord`),
    // and a capture IS a bag. Continuing the recording's numbering would name
    // positions this file does not contain, so a reader of the capture ALONE —
    // which is the whole point of a black box handed to somebody who does not
    // have the recording — could resolve none of them.
    //
    // What that costs, stated: a capture's ordinals do NOT line up with the
    // continuous bag's for the same frames. Nothing reads them across bags, and
    // nothing could: the two files hold different frames.
    let label_channel = writer.frame_producers_channel_id();
    // Per topic, the next ordinal and whether its catch-up is still owed. Keyed
    // by the same topic NAME `batches` is, so no index can drift.
    let mut ordinals: HashMap<&str, u64> = HashMap::new();
    let mut catch_up_owed: HashMap<&str, bool> = HashMap::new();
    let mut producer_labels = 0u64;
    let mut catch_up_records = 0u64;
    // ONE sequence for the reserved channel across every topic, exactly as the
    // continuous writer keeps it: the records share a channel, so they share a
    // stream.
    let mut label_seq = 0u32;
    for (topic, frames) in &job.batches {
        // The DATA channel these records ANNOTATE, resolved ONCE per batch —
        // the continuous writer hoists the identical lookup the same way
        // (`WriterCore::flush_batch`), and doing it per frame paid a `HashMap`
        // probe on every labelled frame of every batch.
        //
        // `None` is UNREACHABLE and needs no arm of its own: `write_message`
        // below resolves the SAME map by name, so a topic with no channel makes
        // the FRAME write fail loudly one statement later with the topic named.
        // (The skip is not because a capture must not abort over its own
        // annotation — the capture does abort on exactly that condition,
        // immediately, on the frame. The skip buys nothing on that front, and is
        // kept only so a future divergence loses the annotation rather than
        // panicking.)
        let data_channel = writer.channel_id(topic);
        // The third item is the frame's PRODUCER LABEL, carried
        // through the window by `StagedFrames` and minted here (decision R2).
        for (header, origin, payload) in frames.iter() {
            // A frame whose wire header would not parse is still WRITTEN, with
            // PLACEHOLDER zeros — and counted, so the manifest can say so rather
            // than letting a reader take the zeros for real chronology. Same rule
            // and same word as the continuous recorder's `headerless`.
            let (seq, ts) = match header {
                Some(pair) => pair,
                None => {
                    headerless += 1;
                    (0, 0)
                }
            };
            let ordinal = *ordinals.get(topic.as_str()).unwrap_or(&0);
            if let Some(publisher_id) = origin.id() {
                if let Some(data_channel) = data_channel {
                    // THE CATCH-UP, at most once per topic: the run of frames
                    // ahead of this one is `[0, ordinal)` in THIS bag's
                    // numbering, and every one of them carried the tap's first
                    // origin (arming happens on the first frame that did not
                    // match it). `ordinal == 0` means this capture's very first
                    // frame of the topic is already labelled — there is no
                    // unlabelled run, so nothing is owed and a zero-length
                    // record would attribute the empty range to somebody.
                    let owed = *catch_up_owed.get(topic.as_str()).unwrap_or(&true);
                    if owed {
                        catch_up_owed.insert(topic.as_str(), false);
                        if ordinal > 0 {
                            if let Some(prefix_origin) =
                                job.first_origins.get(topic.as_str()).copied()
                            {
                                writer
                                    .write_message(
                                        label_channel,
                                        label_seq,
                                        ts,
                                        ts,
                                        &[&ProducerRecord {
                                            attribution: ProducerAttribution::CatchUpPrefix {
                                                prefix_len: ordinal,
                                            },
                                            channel_id: data_channel,
                                            publisher_id: prefix_origin,
                                        }
                                        .encode()],
                                    )
                                    .map_err(|e| {
                                        format!(
                                            "could not write a catch-up record for {topic}: {e}"
                                        )
                                    })?;
                                label_seq = label_seq.wrapping_add(1);
                                catch_up_records += 1;
                            } else {
                                // The tap recorded no first origin, which a tap
                                // holding frames cannot be in. Say so rather
                                // than inventing an attribution or dropping the
                                // capture.
                                // The data rides STRUCTURED FIELDS only — the
                                // message must not also interpolate `topic` and
                                // `ordinal`, which would print each twice and is
                                // what the logging rule forbids.
                                //
                                // UNREACHABLE as the code stands: a tap observes
                                // every frame's origin BEFORE staging it, so a
                                // topic with frames always has a first origin.
                                // Worded as the recorder bug it would be rather
                                // than as an operational condition.
                                tracing::warn!(
                                    topic = %topic,
                                    unlabelled_prefix = ordinal,
                                    "flashback: a capture holds frames that precede \
                                     the frame which armed producer labelling, but the \
                                     recorder has no first-writer identity for the topic, so \
                                     that leading run is written UNATTRIBUTED (later frames \
                                     each carry their own label). This should be unreachable \
                                     — please report the capture."
                                );
                            }
                        }
                    }
                    // The FRAME LABEL goes IN FRONT of the frame it names, in
                    // the same chunk-arena call order the continuous writer uses
                    // and for the same reason: an auto-flush can fall between
                    // them, so the guarantee is the DIRECTION of the damage — a
                    // torn tail can leave a label whose frame is missing, never
                    // a labelled frame whose label is.
                    //
                    // The COST of that ordering, made knowingly: a sink error at
                    // this instant kills the capture ONE FRAME EARLIER than a
                    // frame-first ordering would, because the annotation is
                    // attempted before the evidence. It is the right trade for a
                    // black box — a frame whose label is missing is
                    // indistinguishable from an unlabelled-prefix frame, i.e. a
                    // SILENT attribution hole in the one artifact that exists to
                    // attribute — but it is a trade, not a free invariant. Both carry the frame's own
                    // `ts` (0 for a headerless frame, whose producer is known
                    // regardless: identity is sample metadata, not wire content)
                    // so they time-align with it in the bag.
                    writer
                        .write_message(
                            label_channel,
                            label_seq,
                            ts,
                            ts,
                            &[&ProducerRecord {
                                attribution: ProducerAttribution::FrameLabel {
                                    frame_index: ordinal,
                                },
                                channel_id: data_channel,
                                publisher_id,
                            }
                            .encode()],
                        )
                        .map_err(|e| {
                            format!("could not write a producer label for {topic}: {e}")
                        })?;
                    label_seq = label_seq.wrapping_add(1);
                    producer_labels += 1;
                }
            }
            writer
                .write_message(topic, seq, ts, ts, &[payload])
                .map_err(|e| format!("could not write a frame for {topic}: {e}"))?;
            messages += 1;
            // EVERY frame advances the ordinal — labelled or not, header-parsed
            // or not. The ordinal names a position in the channel's stream, not
            // a count of anything else.
            ordinals.insert(topic.as_str(), ordinal + 1);
        }
    }
    let bytes = writer.bytes_written() + writer.open_chunk_bytes();
    writer
        .finalize()
        .map_err(|e| format!("could not finalize the flashback bag: {e}"))?;
    // The finalized file's real size — `bytes_written` is the pre-summary
    // figure, and an operator comparing the reported number with `ls` should not
    // find them different.
    let bytes = std::fs::metadata(&job.path)
        .map(|m| m.len())
        .unwrap_or(bytes);
    if headerless > 0 {
        tracing::warn!(
            path = %job.path.display(),
            headerless,
            messages,
            "flashback: {headerless} frame(s) in this capture carry PLACEHOLDER metadata \
             (their wire header would not parse) — the payloads are preserved, but their \
             sequence and timestamps are zeros, not the producer's"
        );
    }
    Ok(CaptureStats {
        bytes,
        messages,
        headerless,
        state_records,
        trace_records,
        producer_labels,
        catch_up_records,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A writer whose "write" is a thread we control.
    ///
    /// The production constructor runs a real `write_capture`, so the TIMEOUT
    /// arm is unreachable from it without a slow filesystem — which is exactly
    /// how that arm shipped broken through a full review with every gate
    /// green. This builds the same struct around a thread that finishes when
    /// told, so the arm is drivable in microseconds and deterministically.
    fn controllable() -> (CaptureWriter, std::sync::mpsc::Sender<()>) {
        let (tx, rx) = mpsc::channel::<Result<CaptureStats, String>>();
        let (release_tx, release_rx) = std::sync::mpsc::channel::<()>();
        let join = std::thread::Builder::new()
            .name("bagd-flashback-test".to_string())
            .spawn(move || {
                // Blocks until released, exactly as a slow write would.
                let _ = release_rx.recv();
                let _ = tx.send(Ok(CaptureStats {
                    bytes: 7,
                    messages: 3,
                    headerless: 0,
                    state_records: 0,
                    trace_records: 0,
                    producer_labels: 0,
                    catch_up_records: 0,
                }));
            })
            .expect("spawn");
        (
            CaptureWriter {
                seq: 0,
                path: std::path::PathBuf::from("/tmp/capture-test.mcap"),
                pinned: false,
                resimmable: None,
                // These arms are about the writer lifecycle, not the span.
                span: cerulion_core::flashback::channel::FinishedSpan {
                    claimed_span_ms: 0,
                    achieved_span_ms: 0,
                    truncated_frames: 0,
                },
                requesters: vec![42],
                causes: Vec::new(),
                join: Some(join),
                rx,
            },
            release_tx,
        )
    }

    /// A timeout must return `None` and
    /// keep the handle.
    ///
    /// `Some(Err(..))` reads as a SETTLED capture, so the caller takes the writer
    /// out of the plane and drops it — detaching a live thread — publishes a
    /// terminal failure for a bag that may yet finalize, and lets the retention
    /// sweep run over a file still being appended to. This is the arm that makes
    /// all three of those reachable, and nothing drove it until now.
    #[test]
    fn a_timeout_returns_none_and_keeps_the_writer_joinable() {
        let (mut writer, release) = controllable();

        assert!(
            writer
                .join_bounded(std::time::Duration::from_millis(20))
                .is_none(),
            "a timeout is NOT a settled capture — it must report STILL WRITING"
        );
        assert!(
            writer.join.is_some(),
            "…and it must leave the JoinHandle in place; dropping it here detaches a live \
             thread, which is the whole defect"
        );

        // The thread then finishes, and a LATER bounded wait collects its REAL
        // outcome — so a writer that merely ran late still settles correctly
        // rather than being reported failed on a bound it beat by a moment.
        release.send(()).expect("release");
        let stats = writer
            .join_bounded(std::time::Duration::from_secs(5))
            .expect("finished on the second look")
            .expect("the write really succeeded");
        assert_eq!(stats.bytes, 7);
        assert_eq!(stats.messages, 3);
    }

    /// The ordinary path is unaffected: a write that finishes inside the bound
    /// settles, and the handle is reaped.
    ///
    /// The anti-tautology half — without it, a `join_bounded` that ALWAYS
    /// returned `None` would pass the arm above.
    #[test]
    fn a_write_that_finishes_inside_the_bound_settles() {
        let (mut writer, release) = controllable();
        release.send(()).expect("release");
        let settled = writer
            .join_bounded(std::time::Duration::from_secs(5))
            .expect("finished inside the bound");
        assert_eq!(settled.expect("ok").messages, 3);
        assert!(writer.join.is_none(), "a settled write reaps its thread");
    }

    /// The bound half of the invariant pair.
    ///
    /// Without a bound, a silent detach or an unbounded `join_final` lets a wedged
    /// writer hold a SIGTERM open forever. Both halves are pinned here,
    /// against a thread that is never released:
    ///
    ///   * the wait RETURNS, inside a wall that is a small multiple of the bound
    ///     (a wall is legitimate here: the assertion is that the call is bounded
    ///     AT ALL, and load can only push it up — a hang has no ceiling);
    ///   * it returns `None`, i.e. STILL WRITING, so the caller abandons the
    ///     capture loudly rather than reporting a failure it did not observe or
    ///     sweeping a directory holding a file being appended to.
    ///
    /// The thread is deliberately left blocked: that is the stated residual, and
    /// releasing it here would test a different scenario.
    #[test]
    fn a_wedged_writer_is_abandoned_at_the_bound_rather_than_hanging_the_shutdown() {
        let (mut writer, release) = controllable();
        let started = std::time::Instant::now();
        let verdict = writer.join_bounded(std::time::Duration::from_millis(50));
        let waited = started.elapsed();

        assert!(
            verdict.is_none(),
            "a wedged writer must report STILL WRITING, never a failure nobody observed"
        );
        assert!(
            waited < std::time::Duration::from_secs(5),
            "the shutdown wait must be BOUNDED — it returned only after {waited:?}"
        );
        assert!(
            writer.join.is_some(),
            "…and the handle is still there, so nothing detached it behind the caller's back"
        );
        // Let the fixture's thread go so the test leaves nothing running.
        release.send(()).ok();
    }

    /// A PANICKED writer is a failure, not a timeout: the sender is dropped, so
    /// the channel disconnects and the caller must be told rather than left
    /// waiting out the whole bound.
    #[test]
    fn a_panicked_writer_is_reported_rather_than_waited_out() {
        let (tx, rx) = mpsc::channel::<Result<CaptureStats, String>>();
        let join = std::thread::Builder::new()
            .spawn(move || {
                drop(tx);
            })
            .expect("spawn");
        let mut writer = CaptureWriter {
            seq: 1,
            path: std::path::PathBuf::from("/tmp/capture-panic.mcap"),
            pinned: false,
            resimmable: None,
            // These arms are about the writer lifecycle, not the span.
            span: cerulion_core::flashback::channel::FinishedSpan {
                claimed_span_ms: 0,
                achieved_span_ms: 0,
                truncated_frames: 0,
            },
            requesters: Vec::new(),
            causes: Vec::new(),
            join: Some(join),
            rx,
        };
        let result = writer
            .join_bounded(std::time::Duration::from_secs(5))
            .expect("a disconnected channel answers immediately");
        assert!(result.is_err());
    }
}
