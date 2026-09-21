// SPDX-License-Identifier: AGPL-3.0-only
//! Producer labels end-to-end: a SHARED topic's frames carry, in the bag, the
//! identity of the publisher that committed each one.
//!
//! The wire `sequence` is a PER-PUBLISHER commit counter, so a
//! `multi_publisher_topics:` topic's channel holds an interleaving no reader can
//! attribute after the fact. The recorder CAN see it — iceoryx2 reports a
//! drained sample's `UniquePublisherId` (`OwnedInboundSample::origin`) — so the
//! attribution is WRITTEN DOWN, on the reserved `__cerulion/frame_producers`
//! channel, while it is still knowable.
//!
//! # What these arms assert, and why they are not a self-compare
//!
//! Every label is decoded BY HAND from its 28 bytes (version byte, kind byte,
//! LE `channel_id`, LE `ordinal`, LE `publisher_id`) and the headline arms build
//! the expected 28 bytes by hand too — `ProducerRecord::encode` is deliberately
//! NOT the oracle, since encoding with the same function the recorder encoded
//! with proves only that the function is a function. `ProducerRecord::decode`
//! is used as a SECONDARY cross-check so a hand-decode slip cannot pass either.
//!
//! The `u128` in a label is cross-checked against `CerulionPublisher::publisher_id`
//! — the publisher's own report of the id
//! iceoryx2 stamps on its samples — and against the frame's own BODY, which
//! names the writer that wrote it. So a constant, a swapped, or a dropped
//! origin all fail on a value the recorder never supplied.
//!
//! # Reading the reserved channel
//!
//! Through the upstream `mcap` crate, NOT `cerulion_bag`'s `UserFrameWalk`,
//! which skips `__cerulion/*` BY DESIGN. `MessageStream` yields FILE ORDER,
//! which is what makes the ADJACENCY claim ("the label is written immediately
//! before the frame it names") checkable at all. `IgnoreEndMagic` is passed so
//! the SAME reader serves an intact bag and a truncated one.
//!
//! The data channel's id is resolved from the bag's own channel table by topic
//! NAME — never hardcoded. Reserved ids are derived from the sorted registration
//! (`__cerulion/frame_producers` sorts first of the reserved set),
//! so a literal would encode today's registration order as a contract.
//!
//! # The arms
//!
//! Numbered for the map below; the NAME is what a failure prints, so both are
//! given — a renumbering or a reorder cannot silently invalidate the map.
//!
//! | # | Function |
//! |---|---|
//! | 1 | `declared_multi_publisher_labels_every_frame_from_ordinal_zero` |
//! | 2 | `observed_plurality_emits_one_catch_up_prefix_with_the_exact_prefix_count` |
//! | 3 | `torn_tail_leaves_an_orphan_label_never_an_unlabeled_armed_frame` |
//! | 4 | `single_writer_undeclared_topic_writes_zero_labels_but_the_channel_exists` |
//! | 5 | `salvage_retry_replays_labels_exactly_once` |
//! | 6 | `headerless_frame_on_an_armed_topic_is_labeled_and_consumes_its_ordinal` |
//! | 7 | `two_topics_one_declared_one_observed_label_their_own_channels` |
//! | 8 | `the_label_channel_is_reserved_so_replay_never_sees_it_as_a_user_topic` |
//! | 9 | `a_deferred_arming_batch_keeps_its_catch_up` |
//! | 10 | `a_force_dropped_arming_batch_trips_the_impossible_triple_tripwire` |
//!
//! Arms 1-10 are about the CONTINUOUS recording. Arms 11-16, and their own
//! table, are about a FLASHBACK CAPTURE and live
//! beside them at the foot of this file.
//!
//! # Mutant -> killing arm
//!
//! | Mutant | Killed by |
//! |---|---|
//! | label written AFTER its frame | 1 (adjacency) |
//! | ordinal off by one | 1 (ordinals are exactly `0..N`) |
//! | ordinals count only LABELLED frames | 2 (the prefix count is `N_A`) |
//! | catch-up counts only the CURRENT batch | 2 (the prefix spans >= 3 batches) |
//! | the detector never re-checks past the first origin | 2 (B appears after several drains) |
//! | origin dropped at the drain->writer boundary, or a constant label | 1 (the `u128` cross-check) |
//! | a recovered frame left UNLABELLED by a torn tail | 3 (the one-sided invariant) |
//! | the chunk-boundary STRADDLE made unreachable (a coarser `chunk_max_bytes`) | 3 (`one chunk per message`, asserted structurally — without it the sweep passes on a bag in which the hazard never occurred) |
//! | the truncation sweep sampling arbitrary offsets rather than record boundaries | 3 (the cuts are DERIVED from `record_boundaries`) |
//! | labels minted on a single-writer, undeclared topic | 4 |
//! | the salvage retry does not snapshot/restore label state | 5 |
//! | headerless frames skipped by the labeller | 6 |
//! | any per-tap array read at `[0]` instead of `[idx]` | 7 (two taps in different states) |
//! | the observed-plurality gap stand-down never fires | 2, 7, 9, 10 (each asserts the disable REASON on the durable document; 10 is where it is the ONLY surviving evidence) |
//! | the reserved channel respelled with a leading `/` | 8 (the production user walk) |
//! | a deferred hand-off drops the catch-up it took | 9 |
//! | the labels-without-a-route tripwire deleted | 10 |
//! | the UNACCOUNTED-catch-up (inverse) tripwire deleted | 10 |
//! | the two tripwires sharing ONE latch | 10 (both heads are required in one run) |
//! | `frames_drained_before` off by one (the `- 1`, or the bump moved after the observe) | 10 (the ARMING line's exact value) |
//! | `producer_labels` counted on terms other than `frames_recorded`'s | 1-10 (`assert_labels_bounded_by_frames`) |
//! | `label_catch_up` set on a hand-off that carried none | 9 vs 10 (set after a defer-and-retry, absent after a drop) |
//!
//! Arms 7-10 exist because arms 1-6 are all SINGLE-TAP, CLEAN-HAND-OFF
//! recordings: every parallel-array index in the writer's per-tap walk is `0`,
//! the defer-restore loop is never entered, and the tripwires' conditions are
//! unreachable. Each of those defects leaves arms 1-6 entirely green.
//!
//! What is NOT here, and where it lives: `bag_cmd::classify_loss`'s routing on
//! the stand-down REASON is pinned in `cerulion_cli_engine`
//! (`a_stood_down_topic_with_no_catch_up_record_is_still_unmeasured`), because
//! that crate depends on this one and importing it back would be a package
//! cycle. ARM 10 produces exactly the field triple that arm classifies.
//!
//! Isolated per-test SHM roots ([`common::make_manager`]); `#[serial]` mirrors
//! the sibling recorder suites (the bagd tests are run `--test-threads=1`).

mod common;

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use cerulion_bag::writer::TopicSchema;
use cerulion_bag::{
    BagReader, ProducerAttribution, ProducerRecord, ProducerRecordKind, FRAME_PRODUCERS_TOPIC,
    PRODUCER_RECORD_SIZE, PRODUCER_RECORD_VERSION,
};
use cerulion_bagd::{
    run_bagd, BagdConfig, BagdError, BagdSummary, FlashbackSettings, TapSpec, TopicHealth,
};
use cerulion_core::flashback::channel::{FlashbackOutcome, FlashbackRequester};
use cerulion_core::flashback::switch::TriggerPosture;
use cerulion_core::flashback::trigger::{CaptureRequest, TriggerPolicy};
use cerulion_core::transport::subscriber::DataOnlySubscriber;
use cerulion_core::transport::{TransportManager, RECORDING_WRITER_CHANNEL_BOUND};
use cerulion_core::wire::WireHeader;

use common::*;

/// An arbitrary, stable schema hash for the hand-built frames.
const HASH: u64 = 0x1417_1417_1417_1417;

/// The topic's subscriber queue depth. Deep enough that every arm's whole
/// stimulus fits while the drain is gated, so nothing is lost for a reason that
/// has nothing to do with labelling.
const QUEUE: usize = 16;

/// Wire discriminants, spelled here rather than imported, so a renumbering of
/// `ProducerRecordKind` fails these arms instead of riding along.
const KIND_FRAME_LABEL: u8 = 0;
const KIND_CATCH_UP_PREFIX: u8 = 1;

/// Successful hand-offs needed, after the writer is stalled, to leave the
/// drain->writer channel FULL — so the NEXT batch cannot be handed off at all.
///
/// DERIVED from the production constant rather than picked: the writer takes one
/// batch off the channel and blocks inside it, and the channel then holds
/// `sync_channel(RECORDING_WRITER_CHANNEL_BOUND.max(1))` more. A hand-coded
/// number would silently stop filling the channel if that bound ever moved, and
/// the two arms that stall the writer would quietly become ordinary
/// observed-plurality arms.
const FILL_THE_CHANNEL: u32 = if RECORDING_WRITER_CHANNEL_BOUND > 1 {
    RECORDING_WRITER_CHANNEL_BOUND as u32 + 1
} else {
    // `sync_channel(BOUND.max(1))` — the recorder's own floor.
    2
};

// ===========================================================================
// Harness
// ===========================================================================

fn spawn_bagd(
    mgr: Arc<TransportManager>,
    cfg: BagdConfig,
    shutdown: Arc<AtomicBool>,
) -> JoinHandle<Result<BagdSummary, BagdError>> {
    std::thread::spawn(move || run_bagd(mgr, cfg, shutdown))
}

fn schema_of(topic: &str) -> TopicSchema {
    TopicSchema {
        topic: topic.to_string(),
        schema_name: "probe_msgs/Probe".to_string(),
        schema_hash: HASH,
        wire_fixed_size: 8,
    }
}

/// A DECLARED (exact-mode) tap, optionally carrying the
/// `multi_publisher_topics:` declaration the labeller arms on.
fn tap(topic: &str, multi_publisher: bool) -> TapSpec {
    let mut spec = TapSpec::exact(topic, schema_of(topic));
    spec.multi_publisher = multi_publisher;
    spec
}

/// The `graph run --record` config shape: exact-mode declared taps (so the bag
/// is created on drive-loop pass 1, as it is on that path), discovery OFF (this
/// suite asserts exact sets over topics it created itself), status ON at a fast
/// cadence (the live progress feed the rendezvous below reads).
fn record_cfg(out: PathBuf, taps: Vec<TapSpec>, ready: PathBuf) -> BagdConfig {
    let mut cfg = BagdConfig::new(out, taps);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.ready_file = Some(ready);
    cfg.status_period = Some(Duration::from_millis(20));
    cfg.schema_wait = Duration::from_millis(500);
    cfg.discover_live = false;
    cfg
}

fn finish(
    handle: JoinHandle<Result<BagdSummary, BagdError>>,
    shutdown: &Arc<AtomicBool>,
    what: &str,
) -> BagdSummary {
    shutdown.store(true, Ordering::Relaxed);
    join_bagd(handle, shutdown, what).expect("the recorder must finalize cleanly")
}

/// Signal the recorder to stop when this drops — INCLUDING on an unwind.
///
/// Every arm but ARM 10 runs `run_bagd` on a spawned thread and the TEST thread
/// signals it, so a panic in the assertions still reaches `finish`. ARM 10 is the
/// INVERSE (`run_bagd` on the test thread, stimulus on a helper, because
/// `tracing-test` scopes its capture to the test's span), which puts the shutdown
/// store INSIDE the helper: a stimulus panic there would leave the test thread
/// blocked in the recorder's drive loop FOREVER, i.e. a hung test rather than a
/// failing one — the unbounded cross-thread wait class `shm_ring_test` also
/// guards against. Held by the stimulus closure so the store happens on both exits.
struct ShutdownOnDrop(Arc<AtomicBool>);

impl Drop for ShutdownOnDrop {
    fn drop(&mut self) {
        self.0.store(true, Ordering::Relaxed);
    }
}

// ---------------------------------------------------------------------------
// The live-progress rendezvous
// ---------------------------------------------------------------------------

/// Drain every queued `/bagd/status` frame into parsed JSON (bounded).
fn drain_status(sub: &mut DataOnlySubscriber, out: &mut Vec<serde_json::Value>) {
    let mut held = Vec::new();
    loop {
        let n = sub.drain_owned(1, &mut held).expect("drain status");
        if n == 0 {
            break;
        }
        for sample in &held {
            let frame = sample.payload();
            let body = &frame[WireHeader::SIZE..];
            out.push(serde_json::from_slice(body).expect("status body parses as JSON"));
        }
        held.clear();
    }
}

/// Block until the recorder's own live feed reports it has DRAINED AND WRITTEN
/// at least `want` frames of `topic`.
///
/// # Why this is a condition, and what it BUYS beyond liveness
///
/// It is how the "several separate drain batches" precondition of the
/// observed-plurality arm becomes an OBSERVED FACT rather than a hope.
/// `per_topic` is bumped only on the WRITTEN branch of `flush_threaded`, once
/// per batch, so waiting for it between bursts proves the previous burst was
/// drained and handed to the writer BEFORE the next burst existed — which is
/// exactly what a "count only the frames in this batch" catch-up implementation
/// cannot survive.
///
/// A sleep would be a bet on how much work a loaded runner completes in a
/// window, which load can invert; this can only be DELAYED by load.
fn await_written(
    sub: &mut DataOnlySubscriber,
    topic: &str,
    want: u64,
    what: &str,
) -> Vec<serde_json::Value> {
    let mut seen: Vec<serde_json::Value> = Vec::new();
    // PROGRESS, tracked alongside the condition — see the failure arm below.
    let mut best = 0u64;
    let mut last_increase = Instant::now();
    let ok = await_condition(RECORDER_JOIN_DEADLINE, || {
        drain_status(sub, &mut seen);
        let now_best = written_for(&seen, topic);
        if now_best > best {
            best = now_best;
            last_increase = Instant::now();
        }
        best >= want
    });
    if !ok {
        // ATTRIBUTABLE FAILURE: a ceiling this generous is
        // reached THREE ways, and they are not the same news.
        //
        // A WEDGE — the recorder stopped writing this topic entirely — is a
        // defect, and the count standing still for the whole window is what says
        // so. A runner merely STARVED shows a count that was still climbing when
        // the ceiling expired, and reporting that as a regression sends a reader
        // looking for a bug in code that was working. And a batch that was
        // FORCE-DROPPED is terminal in a third way that neither of those
        // describes: nothing re-publishes it, so the count can never arrive
        // however long the deadline is, and a count standing still says "wedge"
        // when the truth is "the frames are gone".
        //
        // `dropped_unwritten` is the only thing that separates the third from
        // the first, so it travels in BOTH messages rather than being
        // reconstructed from a CI log by hand — a value above the drops an arm
        // deliberately induces is the discriminator.
        let stalled_for = last_increase.elapsed();
        let dropped = seen
            .iter()
            .filter_map(|s| s["dropped_unwritten"].as_u64())
            .max();
        assert!(
            best == 0 || stalled_for >= PROGRESS_STALL_VERDICT,
            "NON-PROBATIVE: {what}: the recorder was STILL WRITING '{topic}' when the \
             {RECORDER_JOIN_DEADLINE:?} ceiling expired — it reported {best} of the {want} \
             frames required, the count last moved {stalled_for:?} ago, and dropped_unwritten \
             stood at {dropped:?}. That is a starved runner, not a recorder or label defect; \
             re-run rather than reading it as a regression"
        );
        // SCOPED TO THE FEED. `best` moves only when a /bagd/status frame says
        // so, and that feed is best-effort (a publish failure is a warn), so a
        // count standing still is evidence about what was REPORTED — claiming
        // the recorder "wrote NOTHING MORE" asserts more than this harness can
        // observe, and a stalled status publisher would have read as a recorder
        // wedge.
        panic!(
            "{what}: the recorder never reported {want} written frames of '{topic}' on its own \
             /bagd/status feed within {RECORDER_JOIN_DEADLINE:?} — it reached {best} and its \
             /bagd/status feed reported nothing more for {stalled_for:?}, which is a recorder \
             wedge or the status feed itself stalling (dropped_unwritten: {dropped:?}). A \
             dropped_unwritten ABOVE the drops this arm deliberately induces means the batch \
             carrying these frames was FORCE-DROPPED rather than written, and nothing \
             re-publishes it; otherwise this is a WEDGE rather than a slow runner. Either way \
             it is a FAILURE of the recorder or of this harness, not of the label assertions \
             below"
        );
    }
    seen
}

/// The highest written-frame count for `topic` in any status frame seen so far.
///
/// `per_topic` is CUMULATIVE, so the maximum over the frames drained is the
/// recorder's own progress — and taking the max rather than the last frame is
/// what makes this immune to the order `drain_status` happens to append in.
fn written_for(seen: &[serde_json::Value], topic: &str) -> u64 {
    seen.iter()
        .filter_map(|s| s["per_topic"][topic].as_u64())
        .max()
        .unwrap_or(0)
}

/// How long a written count must stand COMPLETELY still before a timeout is
/// read as a wedge rather than as a starved runner.
///
/// Sized as a large multiple of the recorder's own flush cadence rather than as
/// a guess: these arms run `flush_interval` at tens of milliseconds, so a
/// recorder that is making any progress at all moves this counter orders of
/// magnitude more often. It only has to separate "moving slowly" from "stopped".
const PROGRESS_STALL_VERDICT: Duration = Duration::from_secs(10);

/// Block until the recorder's own live feed reports at least `want` frames
/// DROPPED (drained but never written).
///
/// The condition form of "the force-drop seam has bitten", and the reason it is
/// a condition rather than a sleep is specific: the arm that uses it needs a
/// FLUSH ATTEMPT to have happened against a full channel while the writer is
/// stalled. A fixed sleep is a bet on how many flush cycles a loaded runner
/// completes in a window — and a runner slow enough to complete none would leave
/// the arming batch un-attempted, which fails the arm's real assertions much
/// later and for a reason those assertions do not name. This can only be DELAYED
/// by load, and when it is genuinely unreachable it says so here.
fn await_dropped(sub: &mut DataOnlySubscriber, want: u64, what: &str) {
    let mut seen: Vec<serde_json::Value> = Vec::new();
    let ok = await_condition(RECORDER_JOIN_DEADLINE, || {
        drain_status(sub, &mut seen);
        seen.iter()
            .any(|s| s["dropped_unwritten"].as_u64().is_some_and(|n| n >= want))
    });
    assert!(
        ok,
        "{what}: the recorder never reported {want} dropped frame(s) on its own /bagd/status feed \
         within {RECORDER_JOIN_DEADLINE:?}, so no hand-off was ever ATTEMPTED against the full \
         channel. This is a FAILURE of the recorder or of this harness, not of the label \
         assertions below"
    );
}

/// Block until the recorder's OWN drive loop reports that `at_least` more time
/// has passed inside it.
///
/// The DEFER path publishes no counter a test can watch (unlike the force-drop
/// path's `dropped_unwritten`), so "several flush cycles happened while the
/// writer was stalled" has to be established some other way — and `sleep` is the
/// wrong one, because it measures the TEST's wall rather than the recorder's
/// progress, so a starved drive loop passes the sleep having attempted no flush
/// at all. `drive_span_us` is the drive loop's own elapsed clock, so this waits
/// for the RECORDER to have run that long: load can only delay it.
fn await_recorder_span_growth(sub: &mut DataOnlySubscriber, at_least: Duration, what: &str) {
    let delta_us = at_least.as_micros() as u64;
    let mut seen: Vec<serde_json::Value> = Vec::new();
    let span = |seen: &[serde_json::Value]| -> Option<u64> {
        seen.iter()
            .filter_map(|s| s["drive_span_us"].as_u64())
            .max()
    };
    // A grounded baseline: without one an empty first read would make `base` 0
    // and satisfy the growth wait against the recorder's whole lifetime.
    let based = await_condition(RECORDER_JOIN_DEADLINE, || {
        drain_status(sub, &mut seen);
        span(&seen).is_some()
    });
    assert!(
        based,
        "{what}: the recorder published no status frame at all within \
         {RECORDER_JOIN_DEADLINE:?}"
    );
    let base = span(&seen).expect("grounded above");
    let ok = await_condition(RECORDER_JOIN_DEADLINE, || {
        drain_status(sub, &mut seen);
        span(&seen).is_some_and(|now| now >= base + delta_us)
    });
    assert!(
        ok,
        "{what}: the recorder's own drive loop never advanced {delta_us}us past {base}us within \
         {RECORDER_JOIN_DEADLINE:?}, so it cannot have attempted the flush cycles this arm needs. \
         This is a FAILURE of the recorder or of this harness, not of the label assertions below"
    );
}

// ===========================================================================
// Reading the bag
// ===========================================================================

/// One message as the upstream `mcap` reader yields it, in FILE ORDER.
#[derive(Debug, Clone)]
struct Msg {
    channel_id: u16,
    topic: String,
    sequence: u32,
    log_time: u64,
    data: Vec<u8>,
}

/// Every message in `bytes`, in file order, plus whether the stream ENDED in an
/// error (a torn tail). Uses the upstream `mcap` crate directly — NOT
/// `cerulion_bag`'s `UserFrameWalk`, which skips `__cerulion/*` by design, so
/// it can never see a producer label.
fn read_stream(bytes: &[u8]) -> (Vec<Msg>, bool) {
    let stream = match mcap::MessageStream::new_with_options(
        bytes,
        mcap::read::Options::IgnoreEndMagic.into(),
    ) {
        Ok(s) => s,
        Err(_) => return (Vec::new(), true),
    };
    let mut out = Vec::new();
    for item in stream {
        match item {
            Ok(m) => out.push(Msg {
                channel_id: m.channel.id,
                topic: m.channel.topic.clone(),
                sequence: m.sequence,
                log_time: m.log_time,
                data: m.data.into_owned(),
            }),
            Err(_) => return (out, true),
        }
    }
    (out, false)
}

fn read_bag(out: &Path) -> Vec<Msg> {
    let bytes = std::fs::read(out).expect("read bag");
    let (msgs, torn) = read_stream(&bytes);
    assert!(!torn, "an intact bag must read to its end without a tear");
    msgs
}

/// The DATA channel id for `topic`, resolved from the messages' own channel
/// records. Never hardcoded: reserved ids shift with the registration.
fn data_channel_id(msgs: &[Msg], topic: &str) -> u16 {
    let ids: Vec<u16> = msgs
        .iter()
        .filter(|m| m.topic == topic)
        .map(|m| m.channel_id)
        .collect();
    let first = *ids
        .first()
        .unwrap_or_else(|| panic!("no message on '{topic}' — the recording captured nothing"));
    assert!(
        ids.iter().all(|id| *id == first),
        "one topic must map to one channel, got {ids:?}"
    );
    first
}

/// The byte offsets at which each TOP-LEVEL MCAP record ENDS.
///
/// An MCAP record is `opcode: u8 | length: u64le | body[length]`, after the
/// 8-byte leading magic — so this needs no reader and no crate API, which is
/// what makes it usable on a bag whose tail this test is about to cut off.
///
/// These are the ONLY truncation points that change what a reader recovers: a
/// truncated record cannot be parsed at all, so a cut anywhere inside a record
/// yields exactly what a cut at its start does. Cutting here is therefore
/// EXHAUSTIVE over distinct outcomes rather than a sample of fractions.
fn record_boundaries(bytes: &[u8]) -> Vec<usize> {
    /// `\x89MCAP0\r\n`.
    const MAGIC_LEN: usize = 8;
    /// opcode + u64 length.
    const HEADER_LEN: usize = 9;

    let mut out = Vec::new();
    let mut at = MAGIC_LEN;
    while at + HEADER_LEN <= bytes.len() {
        let len = u64::from_le_bytes(
            bytes[at + 1..at + HEADER_LEN]
                .try_into()
                .expect("9-byte record header"),
        );
        let Some(end) = usize::try_from(len)
            .ok()
            .and_then(|l| at.checked_add(HEADER_LEN)?.checked_add(l))
        else {
            break;
        };
        if end > bytes.len() {
            break;
        }
        out.push(end);
        at = end;
    }
    out
}

/// Count the TOP-LEVEL `Chunk` records in a (possibly un-finalized) bag.
///
/// `BagWriter::flush_chunk` returns early on an EMPTY chunk, writing no record —
/// which is what makes this a faithful read of "was there a durable prefix?" on
/// the salvage path: that path closes the open chunk and then retries the
/// remainder into a fresh one, so a fault that landed PAST the first message
/// leaves TWO chunks while one that landed at position 0 leaves ONE.
fn top_level_chunks(bytes: &[u8]) -> usize {
    let reader = mcap::read::LinearReader::new_with_options(
        bytes,
        mcap::read::Options::IgnoreEndMagic.into(),
    )
    .expect("linear reader over the bag bytes");
    let mut chunks = 0usize;
    for rec in reader {
        match rec {
            Ok(mcap::records::Record::Chunk { .. }) => chunks += 1,
            // A torn tail is expected on the faulted arm; count what is readable.
            Ok(_) => {}
            Err(_) => break,
        }
    }
    chunks
}

// ---------------------------------------------------------------------------
// `record_health.json` — the DURABLE half of the label accounting
// ---------------------------------------------------------------------------

/// Read + parse the `record_health.json` attachment from a finalized bag.
///
/// The bag's own durable statement about what it recorded, which is where the
/// producer-label counters live — so an arm that asserts on the LABEL RECORDS
/// alone is only half the claim (the counters could be zero, or could describe
/// another topic, with every byte oracle above still green).
fn read_record_health(out: &Path) -> cerulion_bagd::RecordHealth {
    let reader = BagReader::open(out).expect("open bag");
    let att = reader
        .attachment(cerulion_bagd::RECORD_HEALTH_ATTACHMENT)
        .expect("read attachments")
        .expect("record_health.json is present in EVERY finalized bag");
    serde_json::from_slice(&att.data).expect("record_health.json parses")
}

/// One topic's row out of a `record_health.json`, or fail naming what IS there.
fn health_of<'a>(health: &'a cerulion_bagd::RecordHealth, topic: &str) -> &'a TopicHealth {
    health.topics.get(topic).unwrap_or_else(|| {
        panic!(
            "no health row for '{topic}' — rows present: {:?}",
            health.topics.keys().collect::<Vec<_>>()
        )
    })
}

/// The invariant documented on [`TopicHealth::producer_labels`]: a label
/// is minted on EXACTLY the terms a recorded frame is, so a topic can never
/// carry more labels than frames.
fn assert_labels_bounded_by_frames(h: &TopicHealth, topic: &str) {
    assert!(
        h.producer_labels <= h.frames_recorded,
        "{topic}: producer_labels ({}) must never exceed frames_recorded ({}) — they are counted \
         on the same hand-off gate",
        h.producer_labels,
        h.frames_recorded
    );
}

// ---------------------------------------------------------------------------
// Producer labels
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Label {
    kind: u8,
    channel_id: u16,
    ordinal: u64,
    publisher_id: u128,
}

/// Decode ONE 28-byte producer record BY HAND, then cross-check the production
/// decoder agrees. The hand decode is the primary oracle.
fn decode_label(data: &[u8]) -> Label {
    assert_eq!(
        data.len(),
        PRODUCER_RECORD_SIZE,
        "a producer record is a fixed {PRODUCER_RECORD_SIZE}-byte record, got {data:?}"
    );
    assert_eq!(
        data[0], PRODUCER_RECORD_VERSION,
        "unexpected producer-record version byte"
    );
    let by_hand = Label {
        kind: data[1],
        channel_id: u16::from_le_bytes(data[2..4].try_into().unwrap()),
        ordinal: u64::from_le_bytes(data[4..12].try_into().unwrap()),
        publisher_id: u128::from_le_bytes(data[12..28].try_into().unwrap()),
    };
    // SECONDARY cross-check: a hand-decode slip must not pass either.
    let decoded = ProducerRecord::decode(data).expect("the production decoder must read it too");
    assert_eq!(decoded.channel_id, by_hand.channel_id);
    assert_eq!(decoded.publisher_id, by_hand.publisher_id);
    // The wire's `(kind, ordinal)` pair decodes into ONE
    // `ProducerAttribution`, so the cross-check reads both halves back out of
    // the variant the decoder chose.
    let (decoded_kind, decoded_ordinal) = match decoded.attribution {
        ProducerAttribution::FrameLabel { frame_index } => {
            (ProducerRecordKind::FrameLabel.as_u8(), frame_index)
        }
        ProducerAttribution::CatchUpPrefix { prefix_len } => {
            (ProducerRecordKind::CatchUpPrefix.as_u8(), prefix_len)
        }
    };
    assert_eq!(decoded_ordinal, by_hand.ordinal);
    assert_eq!(
        decoded_kind, by_hand.kind,
        "hand-decoded kind byte disagrees with the production decoder"
    );
    by_hand
}

/// Build the expected 28 bytes BY HAND — deliberately not via
/// `ProducerRecord::encode`, so the byte layout is pinned against a literal
/// rather than against the function under test.
fn expected_record_bytes(
    kind: u8,
    channel_id: u16,
    ordinal: u64,
    publisher_id: u128,
) -> [u8; PRODUCER_RECORD_SIZE] {
    let mut b = [0u8; PRODUCER_RECORD_SIZE];
    b[0] = PRODUCER_RECORD_VERSION;
    b[1] = kind;
    b[2..4].copy_from_slice(&channel_id.to_le_bytes());
    b[4..12].copy_from_slice(&ordinal.to_le_bytes());
    b[12..28].copy_from_slice(&publisher_id.to_le_bytes());
    b
}

/// Every producer record in file order, paired with its position in `msgs`.
fn labels_in_order(msgs: &[Msg]) -> Vec<(usize, Label)> {
    msgs.iter()
        .enumerate()
        .filter(|(_, m)| m.topic == FRAME_PRODUCERS_TOPIC)
        .map(|(i, m)| (i, decode_label(&m.data)))
        .collect()
}

/// Every producer record NAMING `channel`, in file order.
///
/// A recording with several labeled taps interleaves their records on the ONE
/// reserved channel, so every per-topic claim below is made over this SUBSET —
/// and the "no label names another topic's channel" half is asserted separately
/// by [`assert_every_label_names`], which is what stops a channel-scoped filter
/// from quietly swallowing a mis-attributed record.
fn labels_on(msgs: &[Msg], channel: u16) -> Vec<(usize, Label)> {
    labels_in_order(msgs)
        .into_iter()
        .filter(|(_, l)| l.channel_id == channel)
        .collect()
}

/// Every producer record in the bag names one of `channels`.
///
/// The complement of [`labels_on`]: that one asks what a channel is owed, this
/// one asks whether anything was written that no channel asked for. A parallel-
/// array index slip (a label built with tap 0's channel id while walking tap 1)
/// shows up HERE when the two topics' ids differ, and in the per-topic ordinal
/// runs when they do not.
fn assert_every_label_names(msgs: &[Msg], channels: &[u16], what: &str) {
    for (pos, label) in labels_in_order(msgs) {
        assert!(
            channels.contains(&label.channel_id),
            "{what}: the producer record at file position {pos} names channel {} — no tap in this \
             recording owns it (expected one of {channels:?}); decoded {label:?}",
            label.channel_id
        );
    }
}

/// `msgs` position -> that frame's 0-based FILE-ORDER index within its own
/// channel. This is what a `FrameLabel`'s `ordinal` names.
fn ordinals_by_position(msgs: &[Msg], channel: u16) -> BTreeMap<usize, u64> {
    let mut out = BTreeMap::new();
    let mut next = 0u64;
    for (i, m) in msgs.iter().enumerate() {
        if m.channel_id == channel {
            out.insert(i, next);
            next += 1;
        }
    }
    out
}

// ---------------------------------------------------------------------------
// Writer-identifying frames
// ---------------------------------------------------------------------------

fn body(writer: char, i: u32) -> Vec<u8> {
    format!("{writer}-{i:04}").into_bytes()
}

fn frame(writer: char, seq: u32) -> Vec<u8> {
    build_frame(HASH, seq, 1_000 + u64::from(seq), &body(writer, seq))
}

/// The writer tag stamped into a recorded frame's BODY — an identity the
/// recorder never supplied, so it cross-checks the label's `u128` against
/// something outside the mechanism under test.
fn writer_tag(frame: &[u8]) -> char {
    assert!(
        frame.len() > WireHeader::SIZE,
        "expected a full wire frame, got {} bytes",
        frame.len()
    );
    let body = &frame[WireHeader::SIZE..];
    let text = std::str::from_utf8(body).expect("frame bodies are UTF-8 in this suite");
    let tag = text
        .strip_prefix("")
        .and_then(|rest| rest.chars().next())
        .unwrap_or_else(|| panic!("unrecognised frame body {text:?}"));
    tag
}

/// Assert every label OF `channel` sits IMMEDIATELY BEFORE the frame it names,
/// and that its `publisher_id` is the id of the writer that frame's body names.
/// Returns the ordinals it saw labelled, in file order.
///
/// CHANNEL-SCOPED (see [`labels_on`]): a recording with two labeled taps
/// interleaves their records, so this asks only what THIS channel is owed. Pair
/// it with [`assert_every_label_names`] — without that, a record naming a
/// channel nobody owns is filtered out here rather than failing.
fn assert_labels_pair_with_their_frames(
    msgs: &[Msg],
    channel: u16,
    ids: &BTreeMap<char, u128>,
    what: &str,
) -> Vec<u64> {
    let ordinals = ordinals_by_position(msgs, channel);
    let mut seen = Vec::new();
    for (pos, label) in labels_on(msgs, channel) {
        if label.kind != KIND_FRAME_LABEL {
            continue;
        }
        let next = pos + 1;
        assert!(
            next < msgs.len(),
            "{what}: the label at file position {pos} (ordinal {}) is the LAST message in the \
             bag — a FrameLabel must be written immediately BEFORE the frame it names",
            label.ordinal
        );
        let frame_ordinal = ordinals.get(&next).copied().unwrap_or_else(|| {
            panic!(
                "{what}: the message after the label at position {pos} is on channel {} ('{}'), \
                 not the labelled data channel {channel} — a label and its frame must be ADJACENT",
                msgs[next].channel_id, msgs[next].topic
            )
        });
        assert_eq!(
            label.ordinal, frame_ordinal,
            "{what}: the label at position {pos} names ordinal {} but the frame immediately after \
             it is ordinal {frame_ordinal}",
            label.ordinal
        );
        let tag = writer_tag(&msgs[next].data);
        let expected = *ids
            .get(&tag)
            .unwrap_or_else(|| panic!("{what}: frame body names unknown writer '{tag}'"));
        assert_eq!(
            label.publisher_id, expected,
            "{what}: ordinal {frame_ordinal} was written by '{tag}' (per its own body) but its \
             label attributes it to {:#x}",
            label.publisher_id
        );
        seen.push(label.ordinal);
    }
    seen
}

// ===========================================================================
// ARM 1 — a DECLARED multi-publisher topic labels every frame, from ordinal 0
// ===========================================================================

/// Two writers interleave on one declared `multi_publisher_topics:` topic; every
/// frame in the bag carries a label naming the writer that committed it.
#[test]
#[serial_test::serial]
fn declared_multi_publisher_labels_every_frame_from_ordinal_zero() {
    const PER_WRITER: u32 = 4;
    const TOTAL: usize = (PER_WRITER as usize) * 2;

    let mgr = make_manager(QUEUE);
    let topic = unique_topic("declared");
    let out = unique_out("declared");
    let ready = unique_ready_file("declared");

    // A creates the service; B opens it by equality (identical caps).
    let mut writer_a = multi_publisher_with_provisioning(&mgr, &topic, QUEUE, 4096);
    let mut writer_b = multi_publisher_with_provisioning(&mgr, &topic, QUEUE, 4096);
    let ids: BTreeMap<char, u128> = [
        ('A', writer_a.publisher_id()),
        ('B', writer_b.publisher_id()),
    ]
    .into_iter()
    .collect();
    assert_ne!(
        ids[&'A'], ids[&'B'],
        "two live publishers must carry distinct iceoryx2 ids"
    );
    assert!(
        ids[&'A'] != 0 && ids[&'B'] != 0,
        "a real publisher id is never zero"
    );

    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(
        mgr.clone(),
        record_cfg(out.clone(), vec![tap(&topic, true)], ready.clone()),
        shutdown.clone(),
    );
    await_bagd_ready(&ready, "declared-multi");
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");
    settle();

    for i in 0..PER_WRITER {
        writer_a.publish_raw(&frame('A', i)).expect("publish A");
        writer_b.publish_raw(&frame('B', i)).expect("publish B");
    }
    await_written(&mut status, &topic, TOTAL as u64, "declared-multi");
    let summary = finish(handle, &shutdown, "declared-multi");
    assert_eq!(summary.messages, TOTAL as u64, "every frame recorded");

    let msgs = read_bag(&out);
    let channel = data_channel_id(&msgs, &topic);
    let frames: Vec<&Msg> = msgs.iter().filter(|m| m.channel_id == channel).collect();
    assert_eq!(frames.len(), TOTAL, "all {TOTAL} frames must be in the bag");
    assert_every_label_names(&msgs, &[channel], "declared-multi");

    let labels = labels_in_order(&msgs);
    assert_eq!(
        labels.len(),
        TOTAL,
        "a DECLARED multi-publisher topic must carry ONE producer label per frame: expected \
         {TOTAL} records on '{FRAME_PRODUCERS_TOPIC}', found {}",
        labels.len()
    );
    assert!(
        labels.iter().all(|(_, l)| l.kind == KIND_FRAME_LABEL),
        "a DECLARED topic is labelled from ordinal 0, so it needs NO CatchUpPrefix: {labels:?}"
    );

    let labelled = assert_labels_pair_with_their_frames(&msgs, channel, &ids, "declared-multi");
    let expected_ordinals: Vec<u64> = (0..TOTAL as u64).collect();
    assert_eq!(
        labelled, expected_ordinals,
        "ordinals must be exactly 0..{TOTAL} in file order"
    );

    // Both writers really are represented (a constant label would pass every
    // count above while attributing the whole topic to one publisher).
    let attributed: std::collections::BTreeSet<u128> =
        labels.iter().map(|(_, l)| l.publisher_id).collect();
    assert_eq!(
        attributed,
        [ids[&'A'], ids[&'B']].into_iter().collect(),
        "the labels must partition the topic between EXACTLY the two live writers"
    );

    // HAND-BUILT byte oracle for the first two labels. The publisher each names
    // is read off the labelled frame's own body, so this pins the LAYOUT and the
    // VALUES without asking the encoder what it thinks it wrote.
    let ordinals = ordinals_by_position(&msgs, channel);
    for (pos, label) in labels.iter().take(2) {
        let tag = writer_tag(&msgs[pos + 1].data);
        let expected =
            expected_record_bytes(KIND_FRAME_LABEL, channel, ordinals[&(pos + 1)], ids[&tag]);
        assert_eq!(
            msgs[*pos].data.as_slice(),
            expected.as_slice(),
            "the producer record at file position {pos} must be byte-identical to the hand-built \
             oracle (decoded as {label:?})"
        );
    }

    // The DURABLE half. The label RECORDS above are what a reader decodes; these
    // counters are what `bag info` and `classify_loss` read, and nothing above
    // can see them disagree.
    let health = read_record_health(&out);
    let h = health_of(&health, &topic);
    assert_eq!(
        h.producer_labels, TOTAL as u64,
        "a declared multi-publisher topic counts one label per recorded frame"
    );
    assert!(
        !h.label_catch_up,
        "a topic armed AT OPEN has no unlabeled prefix, so it can never owe a catch-up"
    );
    assert!(
        h.multi_publisher,
        "the DECLARED flag is what armed this tap"
    );
    assert_eq!(
        h.frames_recorded, TOTAL as u64,
        "the frame counter the label counter is bounded by"
    );
    assert_labels_bounded_by_frames(h, &topic);

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ===========================================================================
// ARM 2 — observed plurality: ONE catch-up prefix, with the exact count
// ===========================================================================

/// A topic NOT declared multi-publisher is labelled from the moment a SECOND
/// origin is drained, and the frames before that are covered by exactly one
/// `CatchUpPrefix` naming the first origin and the exact number of frames.
///
/// The prefix deliberately spans SEVERAL drain batches: each burst is
/// rendezvoused on the recorder's own written count before the next is
/// published (see [`await_written`]), so a catch-up implementation that counts
/// only the frames in the CURRENT batch reports 4 where 12 is required.
#[test]
#[serial_test::serial]
fn observed_plurality_emits_one_catch_up_prefix_with_the_exact_prefix_count() {
    const BURST: u32 = 4;
    const BURSTS: u32 = 3;
    const PREFIX: u64 = (BURST * BURSTS) as u64;
    /// Frames published after the arming frame (which is itself labelled).
    const TAIL: u32 = 3;
    const TOTAL: usize = PREFIX as usize + 1 + TAIL as usize;

    let mgr = make_manager(QUEUE);
    let topic = unique_topic("observed");
    let out = unique_out("observed");
    let ready = unique_ready_file("observed");

    let mut writer_a = multi_publisher_with_provisioning(&mgr, &topic, QUEUE, 4096);

    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(
        mgr.clone(),
        // NOT declared multi-publisher: arming must come from OBSERVATION.
        record_cfg(out.clone(), vec![tap(&topic, false)], ready.clone()),
        shutdown.clone(),
    );
    await_bagd_ready(&ready, "observed-plurality");
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");
    settle();

    let mut seq_a = 0u32;
    for burst in 1..=BURSTS {
        for _ in 0..BURST {
            writer_a.publish_raw(&frame('A', seq_a)).expect("publish A");
            seq_a += 1;
        }
        await_written(
            &mut status,
            &topic,
            u64::from(burst * BURST),
            "observed-plurality prefix burst",
        );
    }

    // The SECOND writer appears, several drains later, and publishes ALONE —
    // rendezvoused, so the arming frame's ordinal is unambiguous.
    let mut writer_b = multi_publisher_with_provisioning(&mgr, &topic, QUEUE, 4096);
    let ids: BTreeMap<char, u128> = [
        ('A', writer_a.publisher_id()),
        ('B', writer_b.publisher_id()),
    ]
    .into_iter()
    .collect();
    writer_b.publish_raw(&frame('B', 0)).expect("publish B");
    await_written(&mut status, &topic, PREFIX + 1, "observed-plurality arming");

    for i in 0..TAIL {
        if i % 2 == 0 {
            writer_a.publish_raw(&frame('A', seq_a)).expect("publish A");
            seq_a += 1;
        } else {
            writer_b.publish_raw(&frame('B', i + 1)).expect("publish B");
        }
    }
    await_written(&mut status, &topic, TOTAL as u64, "observed-plurality tail");
    let summary = finish(handle, &shutdown, "observed-plurality");
    assert_eq!(summary.messages, TOTAL as u64, "every frame recorded");

    let msgs = read_bag(&out);
    let channel = data_channel_id(&msgs, &topic);
    let ordinals = ordinals_by_position(&msgs, channel);
    assert_eq!(ordinals.len(), TOTAL, "all {TOTAL} frames must be recorded");
    assert_every_label_names(&msgs, &[channel], "observed-plurality");

    let records = labels_in_order(&msgs);
    let catch_ups: Vec<&(usize, Label)> = records
        .iter()
        .filter(|(_, l)| l.kind == KIND_CATCH_UP_PREFIX)
        .collect();
    assert_eq!(
        catch_ups.len(),
        1,
        "observed plurality must emit EXACTLY ONE CatchUpPrefix, found {} in {records:?}",
        catch_ups.len()
    );
    let (catch_pos, catch) = *catch_ups[0];
    assert_eq!(
        catch.ordinal, PREFIX,
        "the prefix must cover EVERY frame written before arming — ordinals [0, {PREFIX}) — not \
         just the frames of the batch that armed it"
    );
    assert_eq!(
        catch.publisher_id, ids[&'A'],
        "the prefix belongs to the topic's FIRST origin"
    );
    assert_eq!(
        msgs[catch_pos].data.as_slice(),
        expected_record_bytes(KIND_CATCH_UP_PREFIX, channel, PREFIX, ids[&'A']).as_slice(),
        "the CatchUpPrefix must be byte-identical to the hand-built oracle"
    );

    let first_label = records
        .iter()
        .find(|(_, l)| l.kind == KIND_FRAME_LABEL)
        .expect("the arming frame and everything after it must be labelled");
    assert!(
        catch_pos < first_label.0,
        "the CatchUpPrefix must precede the first per-frame label (positions {catch_pos} vs {})",
        first_label.0
    );

    let labelled = assert_labels_pair_with_their_frames(&msgs, channel, &ids, "observed-plurality");
    let expected: Vec<u64> = (PREFIX..TOTAL as u64).collect();
    assert_eq!(
        labelled, expected,
        "labelling ARMS with the frame that introduced the second origin: ordinals {expected:?} \
         must be labelled, and nothing before them"
    );

    // The DURABLE half: the counters an operator reads, on the SAME run.
    let health = read_record_health(&out);
    let h = health_of(&health, &topic);
    assert_eq!(
        h.producer_labels,
        TOTAL as u64 - PREFIX,
        "exactly the frames from the arming one on are labelled; the prefix is one catch-up record"
    );
    assert!(
        h.label_catch_up,
        "the runtime-armed marker is what says the plurality was OBSERVED, not declared"
    );
    assert!(
        !h.multi_publisher,
        "the graph declared nothing about this topic — that is the whole shape under test"
    );
    assert_eq!(
        h.gap_detection_disabled_reason.as_deref(),
        Some("observed_multi_publisher"),
        "observing a second writer stands the gap accounting down, under its OWN reason"
    );
    assert_eq!(h.frames_recorded, TOTAL as u64);
    assert_labels_bounded_by_frames(h, &topic);

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ===========================================================================
// ARM 3 — a torn tail leaves an ORPHAN label, never an unlabelled armed frame
// ===========================================================================

/// Truncating the bag at many offsets must never produce a recovered frame with
/// no label: at worst the tail holds a label whose frame did not make it.
///
/// A chunk can legitimately close between a label and its frame
/// (`maybe_flush_chunk` runs after every `write_message`), so an orphan label is
/// a real, permitted outcome — which is exactly why the assertion is one-sided.
///
/// # Why the cuts are DERIVED and the chunking is maximal
///
/// A truncated MCAP record cannot be parsed AT ALL, and a chunk is one record —
/// so a cut inside a chunk drops the whole chunk and every distinct recovery
/// outcome is reachable by cutting at a top-level RECORD BOUNDARY. Cutting at
/// arbitrary fractions of the file therefore samples the same handful of
/// outcomes over and over while its "did anything recover?" floor stays green,
/// which is what made the first version's anti-vacuity check nearly free.
///
/// `chunk_max_bytes = 1` closes a chunk after EVERY `write_message`, so every
/// label is separated from the frame it names by a chunk boundary — asserted
/// structurally below (`one chunk per message`) rather than hoped for. That is
/// what makes the straddle the arm exists for REACHABLE at all, and it is
/// MEASURED rather than argued: at this fixture's previous `chunk_max_bytes =
/// 512` the recording was **10 chunks for 80 messages**, and an EXHAUSTIVE sweep
/// of every record boundary produced **62 recovering cuts and ZERO straddling
/// ones** — i.e. no boundary fell between a label and its frame, so the
/// one-sided invariant below was satisfied by a bag in which the hazard it
/// guards never occurred, while the old `recovered_any > 0` floor stayed green.
#[test]
#[serial_test::serial]
fn torn_tail_leaves_an_orphan_label_never_an_unlabeled_armed_frame() {
    const FRAMES: u32 = 40;

    let mgr = make_manager(QUEUE);
    let topic = unique_topic("torn");
    let out = unique_out("torn");
    let ready = unique_ready_file("torn");

    let mut writer_a = multi_publisher_with_provisioning(&mgr, &topic, QUEUE, 4096);
    let mut writer_b = multi_publisher_with_provisioning(&mgr, &topic, QUEUE, 4096);
    let ids: BTreeMap<char, u128> = [
        ('A', writer_a.publisher_id()),
        ('B', writer_b.publisher_id()),
    ]
    .into_iter()
    .collect();

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = record_cfg(out.clone(), vec![tap(&topic, true)], ready.clone());
    // ONE message per chunk: `maybe_flush_chunk` fires whenever the arena is at
    // or over this many bytes, so every write closes a chunk and every label is
    // separated from its frame by a boundary a cut can land on.
    cfg.chunk_max_bytes = 1;
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    await_bagd_ready(&ready, "torn-tail");
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");
    settle();

    for i in 0..FRAMES {
        let writer = if i % 2 == 0 { 'A' } else { 'B' };
        let pubr = if writer == 'A' {
            &mut writer_a
        } else {
            &mut writer_b
        };
        pubr.publish_raw(&frame(writer, i / 2)).expect("publish");
        if i % 8 == 7 {
            await_written(&mut status, &topic, u64::from(i + 1), "torn-tail burst");
        }
    }
    await_written(&mut status, &topic, u64::from(FRAMES), "torn-tail");
    finish(handle, &shutdown, "torn-tail");

    let bytes = std::fs::read(&out).expect("read bag");

    // POSITIVE CONTROL: intact, every frame labelled.
    let intact = read_bag(&out);
    let channel = data_channel_id(&intact, &topic);
    assert_every_label_names(&intact, &[channel], "torn-tail (intact)");
    let intact_labelled =
        assert_labels_pair_with_their_frames(&intact, channel, &ids, "torn-tail (intact)");
    assert_eq!(
        intact_labelled.len(),
        FRAMES as usize,
        "the intact recording must label every one of its {FRAMES} frames"
    );

    // STRUCTURAL: the boundary between a label and its frame really exists in
    // this bag. Without it every cut below lands on a pair-aligned boundary and
    // the one-sided invariant is satisfied by a recording in which the hazard it
    // guards never arises.
    assert_eq!(
        top_level_chunks(&bytes),
        intact.len(),
        "one chunk per message is what puts a boundary between EVERY label and the frame it \
         names — {} messages against {} chunks",
        intact.len(),
        top_level_chunks(&bytes)
    );

    // Cut at every top-level record boundary — the only offsets that change what
    // a reader recovers, and exhaustive over them.
    let cuts = record_boundaries(&bytes);
    assert!(
        cuts.len() > intact.len(),
        "the boundary walk must find at least one record per message, found {} for {} messages",
        cuts.len(),
        intact.len()
    );
    // Cuts that recovered at least one frame, and cuts that landed BETWEEN a
    // label and the frame it names (an orphan trailing label). Counted apart:
    // the first says the sweep is not vacuous, the second that it reaches the
    // shape this arm is named for.
    let mut recovering_cuts = 0usize;
    let mut straddling_cuts = 0usize;
    for cut in cuts {
        let (msgs, _torn) = read_stream(&bytes[..cut]);
        let ordinals = ordinals_by_position(&msgs, channel);
        if !ordinals.is_empty() {
            recovering_cuts += 1;
        }

        // The one-sided claim: every RECOVERED frame is preceded, adjacently, by
        // its own label. A trailing label with no frame is permitted.
        for (pos, ordinal) in &ordinals {
            assert!(
                *pos > 0,
                "cut at {cut}: the frame at ordinal {ordinal} is the FIRST recovered message — a \
                 label must precede it"
            );
            let before = &msgs[pos - 1];
            assert_eq!(
                before.topic, FRAME_PRODUCERS_TOPIC,
                "cut at {cut}: the recovered frame at ordinal {ordinal} (file position {pos}) is \
                 NOT preceded by a producer label — it is preceded by '{}'",
                before.topic
            );
            let label = decode_label(&before.data);
            assert_eq!(
                (label.kind, label.channel_id, label.ordinal),
                (KIND_FRAME_LABEL, channel, *ordinal),
                "cut at {cut}: the label before ordinal {ordinal} names something else"
            );
        }

        // An ORPHAN trailing label — a cut that fell between a label and its
        // frame. Permitted (that is the whole one-sidedness), and COUNTED,
        // because a sweep that never produced one never tested it. `decode_label`
        // already refuses anything that is not a producer record, so there is no
        // separate kind check to make here: a `kind == LABEL || kind ==
        // CATCH_UP` assertion would enumerate the only two values the decoder can
        // return and could not fail.
        if let Some(last) = msgs.last() {
            if last.topic == FRAME_PRODUCERS_TOPIC {
                let orphan = decode_label(&last.data);
                if orphan.kind == KIND_FRAME_LABEL && orphan.channel_id == channel {
                    straddling_cuts += 1;
                }
            }
        }
    }
    assert!(
        recovering_cuts >= FRAMES as usize,
        "each of the {FRAMES} frames' own record boundaries is a cut that recovers at least one \
         frame, so a sweep recovering from only {recovering_cuts} of them is not exercising the \
         invariant"
    );
    assert!(
        straddling_cuts >= FRAMES as usize,
        "with one message per chunk EVERY label's boundary is a cut that lands between it and its \
         frame, so all {FRAMES} must produce an orphan label — got {straddling_cuts}"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ===========================================================================
// ARM 4 — a single-writer, undeclared topic writes ZERO labels
// ===========================================================================

/// The reserved channel is registered UNCONDITIONALLY (the channel table must
/// not be a function of run-time configuration), so its PRESENCE proves nothing
/// — what matters is that an ordinary single-writer topic puts no message on it.
#[test]
#[serial_test::serial]
fn single_writer_undeclared_topic_writes_zero_labels_but_the_channel_exists() {
    const FRAMES: u32 = 12;

    let mgr = make_manager(QUEUE);
    let topic = unique_topic("solo");
    let out = unique_out("solo");
    let ready = unique_ready_file("solo");

    let mut writer = publisher_with_provisioning(&mgr, &topic, QUEUE, QUEUE, 4096);

    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(
        mgr.clone(),
        record_cfg(out.clone(), vec![tap(&topic, false)], ready.clone()),
        shutdown.clone(),
    );
    await_bagd_ready(&ready, "single-writer");
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");
    settle();

    for i in 0..FRAMES {
        writer.publish_raw(&frame('A', i)).expect("publish");
    }
    await_written(&mut status, &topic, u64::from(FRAMES), "single-writer");
    let summary = finish(handle, &shutdown, "single-writer");
    assert_eq!(summary.messages, u64::from(FRAMES));

    let reader = BagReader::open(&out).expect("open bag");
    let channels = reader.channels().expect("channel table");
    assert!(
        channels.iter().any(|c| c.topic == FRAME_PRODUCERS_TOPIC),
        "the reserved producer-label channel is registered unconditionally, so it must be in \
         EVERY bag's channel table: {:?}",
        channels.iter().map(|c| &c.topic).collect::<Vec<_>>()
    );

    let msgs = read_bag(&out);
    let channel = data_channel_id(&msgs, &topic);
    assert_eq!(
        ordinals_by_position(&msgs, channel).len(),
        FRAMES as usize,
        "the recording itself must be intact (anti-vacuity for the zero-label claim)"
    );
    let labels = labels_in_order(&msgs);
    assert!(
        labels.is_empty(),
        "a single-writer topic that was never declared `multi_publisher` must produce NO producer \
         labels, found {labels:?}"
    );

    // The DURABLE half: the counters must agree with the empty channel. A
    // nonzero `producer_labels` here beside zero records would be the two halves
    // of one recording telling different stories.
    let health = read_record_health(&out);
    let h = health_of(&health, &topic);
    assert_eq!(
        h.producer_labels, 0,
        "no record was written, so nothing may be counted"
    );
    assert!(!h.label_catch_up, "nothing armed, so no prefix is owed");
    assert!(!h.multi_publisher);
    assert_eq!(
        h.gap_detection_disabled_reason, None,
        "an ordinary single-writer topic keeps its gap accounting"
    );
    assert_eq!(h.frames_recorded, u64::from(FRAMES));
    assert_labels_bounded_by_frames(h, &topic);

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ===========================================================================
// ARM 5 — the salvage retry replays labels EXACTLY ONCE
// ===========================================================================

/// The label stream a mid-batch write fault produced, and the one the identical
/// stimulus produced with no fault, must be identical.
///
/// The comparison is `(kind, channel, ordinal, writer, mcap sequence)`: an
/// ordinal written twice, an ordinal skipped, or a per-channel sequence that
/// double-counts the salvaged remainder each show up as a stream difference.
#[derive(Debug, Clone, PartialEq, Eq)]
struct LabelRow {
    kind: u8,
    channel_id: u16,
    ordinal: u64,
    writer: char,
    sequence: u32,
}

fn label_rows(msgs: &[Msg], channel: u16, ids: &BTreeMap<char, u128>) -> Vec<LabelRow> {
    let ordinals = ordinals_by_position(msgs, channel);
    labels_in_order(msgs)
        .into_iter()
        .map(|(pos, label)| {
            // Resolve the writer through the frame's own body where there IS a
            // frame (a FrameLabel); a CatchUpPrefix resolves through the id map.
            let writer = if ordinals.contains_key(&(pos + 1)) {
                writer_tag(&msgs[pos + 1].data)
            } else {
                *ids.iter()
                    .find(|(_, id)| **id == label.publisher_id)
                    .map(|(tag, _)| tag)
                    .unwrap_or(&'?')
            };
            LabelRow {
                kind: label.kind,
                channel_id: label.channel_id,
                ordinal: label.ordinal,
                writer,
                sequence: msgs[pos].sequence,
            }
        })
        .collect()
}

/// One capture: the whole stimulus is published while the recorder's drain is
/// HELD, so opening the gate produces ONE batch and the injected fault is
/// unambiguously mid-batch.
fn run_salvage_capture(tag: &str, fault_after: Option<u64>) -> (Vec<LabelRow>, Vec<Vec<u8>>) {
    const PER_WRITER: u32 = 3;
    const TOTAL: usize = (PER_WRITER as usize) * 2;

    let mgr = make_manager(QUEUE);
    let topic = unique_topic(tag);
    let out = unique_out(tag);
    let ready = unique_ready_file(tag);

    let mut writer_a = multi_publisher_with_provisioning(&mgr, &topic, QUEUE, 4096);
    let mut writer_b = multi_publisher_with_provisioning(&mgr, &topic, QUEUE, 4096);
    let ids: BTreeMap<char, u128> = [
        ('A', writer_a.publisher_id()),
        ('B', writer_b.publisher_id()),
    ]
    .into_iter()
    .collect();

    let gate = Arc::new(AtomicBool::new(false));
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = record_cfg(out.clone(), vec![tap(&topic, true)], ready.clone());
    // Far above the whole recording: nothing auto-flushes, so the fault lands
    // inside ONE open chunk and the recovery must close-and-retry rather than
    // discard.
    cfg.chunk_max_bytes = cerulion_bag::DEFAULT_CHUNK_MAX_BYTES;
    cfg.fault_inject_tap_drain_gate = Some(gate.clone());
    cfg.fault_inject_flush_error_after_messages = fault_after;
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    await_bagd_ready(&ready, tag);
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");
    settle();

    // Everything is committed while the drain is held: one queue, one batch.
    for i in 0..PER_WRITER {
        writer_a.publish_raw(&frame('A', i)).expect("publish A");
    }
    for i in 0..PER_WRITER {
        writer_b.publish_raw(&frame('B', i)).expect("publish B");
    }
    gate.store(true, Ordering::Relaxed);

    if fault_after.is_some() {
        // The injected fault terminates the run on its own; the shutdown store
        // below is the liveness fallback, not the expected path.
        let finished = await_condition(RECORDER_JOIN_DEADLINE, || handle.is_finished());
        shutdown.store(true, Ordering::Relaxed);
        let result = join_bagd(handle, &shutdown, tag);
        assert!(
            finished && result.is_err(),
            "{tag}: the injected mid-batch write fault must propagate (finished={finished}, \
             result={result:?})"
        );
    } else {
        await_written(&mut status, &topic, TOTAL as u64, tag);
        finish(handle, &shutdown, tag);
    }

    let bytes = std::fs::read(&out).expect("read bag");
    if fault_after.is_some() {
        // PRECONDITION, not the pin: the salvage path closes the open
        // chunk and retries the batch's un-accepted remainder into a FRESH one,
        // and `BagWriter::flush_chunk` writes nothing for an EMPTY chunk — so
        // two chunks is exactly "the first attempt left a durable prefix", i.e.
        // the retry really was PARTIAL. A fault that fired at the batch's first
        // message replays the whole thing, which the label assertions below
        // would still pass while testing none of the counter-restore this arm
        // exists for. The threshold counts whatever `write_batch` accounts, so
        // this stays correct if that basis ever changes.
        assert!(
            top_level_chunks(&bytes) >= 2,
            "{tag}: the injected fault must land PAST this batch's first message, leaving a \
             non-empty durable prefix for the salvage retry to skip — found {} chunk(s)",
            top_level_chunks(&bytes)
        );
    }
    let (msgs, _torn) = read_stream(&bytes);
    let channel = data_channel_id(&msgs, &topic);
    let ordinals = ordinals_by_position(&msgs, channel);
    assert_eq!(
        ordinals.len(),
        TOTAL,
        "{tag}: every published frame must land in the bag exactly once (the salvage retry \
         re-writes the un-accepted remainder)"
    );
    let bodies: Vec<Vec<u8>> = msgs
        .iter()
        .filter(|m| m.channel_id == channel)
        .map(|m| m.data[WireHeader::SIZE..].to_vec())
        .collect();
    let rows = label_rows(&msgs, channel, &ids);

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
    (rows, bodies)
}

#[test]
#[serial_test::serial]
fn salvage_retry_replays_labels_exactly_once() {
    // Fires on the 4th accounted message of the batch — a genuine MID-batch
    // fault, so the recovery must close the durable prefix and retry the
    // remainder. The threshold counts whatever `write_batch` accounts, and the
    // arm does not care WHICH: the whole stimulus is one batch (the drain is
    // held until every frame is committed), so the fault lands inside it either
    // way and nothing published after it can be lost.
    let (faulted, faulted_bodies) = run_salvage_capture("salvage_fault", Some(3));
    let (control, control_bodies) = run_salvage_capture("salvage_control", None);

    assert!(
        !control.is_empty(),
        "the control run must produce labels, or the equality below is vacuous"
    );
    assert_eq!(
        faulted_bodies, control_bodies,
        "PRECONDITION: both runs must record the same frames in the same order — otherwise the \
         label comparison below is comparing two different recordings"
    );
    assert_eq!(
        faulted, control,
        "a mid-batch write fault plus its salvage retry must replay the label stream EXACTLY \
         ONCE: no duplicated ordinal, no skipped ordinal, no doubled per-channel sequence"
    );

    // And the stream really is the complete, gap-free ordinal run (a pair of
    // identically-broken streams would satisfy the equality alone).
    let ordinals: Vec<u64> = faulted.iter().map(|r| r.ordinal).collect();
    assert_eq!(
        ordinals,
        (0..faulted.len() as u64).collect::<Vec<_>>(),
        "ordinals must be exactly 0..N with no repeat"
    );
    let sequences: Vec<u32> = faulted.iter().map(|r| r.sequence).collect();
    assert_eq!(
        sequences,
        (0..faulted.len() as u32).collect::<Vec<_>>(),
        "the reserved channel's own per-channel sequence must be gap-free too"
    );
}

// ===========================================================================
// ARM 6 — a headerless frame on an armed topic is labelled and consumes its
//         ordinal
// ===========================================================================

/// A payload too short for a wire header is recorded WHOLE with fabricated
/// seq/time 0. On an armed topic it is labelled like any other frame, and the
/// ordinals after it are UNSHIFTED — a labeller that skipped it would leave the
/// rest of the stream naming the wrong frames.
#[test]
#[serial_test::serial]
fn headerless_frame_on_an_armed_topic_is_labeled_and_consumes_its_ordinal() {
    let mgr = make_manager(QUEUE);
    let topic = unique_topic("headerless");
    let out = unique_out("headerless");
    let ready = unique_ready_file("headerless");

    let mut writer = multi_publisher_with_provisioning(&mgr, &topic, QUEUE, 4096);
    let id = writer.publisher_id();

    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(
        mgr.clone(),
        record_cfg(out.clone(), vec![tap(&topic, true)], ready.clone()),
        shutdown.clone(),
    );
    await_bagd_ready(&ready, "headerless");
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");
    settle();

    // A single writer keeps the queue FIFO, so file order IS publish order and
    // the expected ordinal list can be exact.
    let garbage: Vec<u8> = vec![0xBA; 10]; // < WireHeader::SIZE => headerless
    writer.publish_raw(&frame('A', 0)).expect("publish 0");
    writer.publish_raw(&garbage).expect("publish garbage");
    writer.publish_raw(&frame('A', 1)).expect("publish 1");
    writer.publish_raw(&frame('A', 2)).expect("publish 2");
    await_written(&mut status, &topic, 4, "headerless");
    let summary = finish(handle, &shutdown, "headerless");
    assert_eq!(
        summary.headerless, 1,
        "exactly one headerless frame counted"
    );
    assert_eq!(summary.messages, 4, "good + garbage frames all recorded");

    let msgs = read_bag(&out);
    let channel = data_channel_id(&msgs, &topic);
    let ordinals = ordinals_by_position(&msgs, channel);
    assert_eq!(ordinals.len(), 4, "all four frames must be recorded");

    // The headerless frame is at ordinal 1, carries the fabricated stamp 0, and
    // the ordered frame payloads are unchanged.
    let frames: Vec<&Msg> = msgs.iter().filter(|m| m.channel_id == channel).collect();
    assert_eq!(
        frames[1].data, garbage,
        "the headerless frame is recorded WHOLE, at ordinal 1"
    );
    assert_eq!(
        frames[1].log_time, 0,
        "a headerless frame carries a fabricated timestamp of 0"
    );

    let labels = labels_in_order(&msgs);
    assert_eq!(
        labels.len(),
        4,
        "an armed topic labels EVERY recorded frame, headerless ones included: expected 4 \
         records on '{FRAME_PRODUCERS_TOPIC}', found {}",
        labels.len()
    );
    let listed: Vec<u64> = labels.iter().map(|(_, l)| l.ordinal).collect();
    assert_eq!(
        listed,
        vec![0, 1, 2, 3],
        "the headerless frame CONSUMES its ordinal, so the frames after it are unshifted"
    );
    assert!(
        labels.iter().all(|(_, l)| l.publisher_id == id),
        "every frame on this topic came from the one live writer"
    );

    // The headerless frame's own label sits immediately before it, and its
    // log_time follows the frame's fabricated 0.
    let (pos, label) = labels
        .iter()
        .find(|(_, l)| l.ordinal == 1)
        .expect("ordinal 1 must be labelled");
    assert_eq!(
        msgs[pos + 1].data,
        garbage,
        "the ordinal-1 label must be written immediately before the headerless frame"
    );
    assert_eq!(
        msgs[*pos].log_time, 0,
        "the label of a headerless frame is stamped with the frame's fabricated timestamp"
    );
    assert_eq!(
        msgs[*pos].data.as_slice(),
        expected_record_bytes(KIND_FRAME_LABEL, channel, 1, id).as_slice(),
        "the headerless frame's label must be byte-identical to the hand-built oracle \
         (decoded as {label:?})"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ===========================================================================
// ARM 7 — TWO taps, one DECLARED and one OBSERVED, label their OWN channels
// ===========================================================================

/// One recording, two labeled taps: every per-tap quantity a producer record
/// carries is read out of a PARALLEL ARRAY indexed by the tap, and with a single
/// tap in the bag every one of those indices is `0`.
///
/// So this is the arm the whole index-slip class needs. Each of `frame_ordinals`,
/// `batch.catch_up`, the accounting's `catch_up_owed`, and the per-tap DATA
/// CHANNEL is asked for by index inside the writer's per-tap walk, and a `[0]`
/// where an `[idx]` belongs is invisible to every other arm in this file: it
/// answers with the ONLY tap's own value and every byte oracle stays green.
///
/// The two taps are deliberately in DIFFERENT states, which is what makes each
/// slip observable rather than merely present:
///
/// * `/…declared_a` is DECLARED `multi_publisher`, so it labels from ordinal 0,
///   owes no catch-up, and is at its FINAL ordinal by the time the other tap
///   arms — a `frame_ordinals[0]` slip therefore stamps a FROZEN number onto
///   every one of the other tap's labels.
/// * `/…observed_b` is UNDECLARED and goes plural mid-run, so it owes exactly
///   one `CatchUpPrefix` — which the declared tap's slot never holds, so a
///   `catch_up[0]` slip drops it entirely and a `catch_up_owed[0]` slip leaves
///   the runtime-armed marker off the manifest.
///
/// Both halves of the recording are asserted: the label RECORDS in the bag, and
/// the `record_health.json` counters an operator reads.
#[test]
#[serial_test::serial]
fn two_topics_one_declared_one_observed_label_their_own_channels() {
    /// Frames on the DECLARED tap, from two interleaved writers.
    const A_PER_WRITER: u32 = 3;
    const A_TOTAL: usize = (A_PER_WRITER as usize) * 2;
    /// The UNDECLARED tap's single-writer prefix, over two rendezvoused bursts.
    const B_BURST: u32 = 4;
    const B_BURSTS: u32 = 2;
    const B_PREFIX: u64 = (B_BURST * B_BURSTS) as u64;
    /// Frames published after the arming frame (which is itself labeled).
    const B_TAIL: u32 = 3;
    const B_TOTAL: usize = B_PREFIX as usize + 1 + B_TAIL as usize;

    let mgr = make_manager(QUEUE);
    // `unique_topic` prefixes the base, so `declared_a` sorts before
    // `observed_b` — the declared tap is index 0 whichever way the recorder
    // orders its taps, which is what puts the OBSERVED tap's catch-up in a slot
    // a `[0]` slip can never read.
    let topic_a = unique_topic("declared_a");
    let topic_b = unique_topic("observed_b");
    let out = unique_out("two_topics");
    let ready = unique_ready_file("two_topics");

    let mut a1 = multi_publisher_with_provisioning(&mgr, &topic_a, QUEUE, 4096);
    let mut a2 = multi_publisher_with_provisioning(&mgr, &topic_a, QUEUE, 4096);
    let mut b1 = multi_publisher_with_provisioning(&mgr, &topic_b, QUEUE, 4096);
    let ids_a: BTreeMap<char, u128> = [('P', a1.publisher_id()), ('Q', a2.publisher_id())]
        .into_iter()
        .collect();

    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(
        mgr.clone(),
        record_cfg(
            out.clone(),
            vec![tap(&topic_a, true), tap(&topic_b, false)],
            ready.clone(),
        ),
        shutdown.clone(),
    );
    await_bagd_ready(&ready, "two-topics");
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");
    settle();

    // The DECLARED tap runs to completion FIRST, so its ordinal is frozen at
    // A_TOTAL for the whole of the other tap's life.
    for i in 0..A_PER_WRITER {
        a1.publish_raw(&frame('P', i)).expect("publish P");
        a2.publish_raw(&frame('Q', i)).expect("publish Q");
    }
    await_written(&mut status, &topic_a, A_TOTAL as u64, "two-topics declared");

    // The UNDECLARED tap's single-writer prefix, over SEVERAL drain batches.
    let mut seq_b = 0u32;
    for burst in 1..=B_BURSTS {
        for _ in 0..B_BURST {
            b1.publish_raw(&frame('R', seq_b)).expect("publish R");
            seq_b += 1;
        }
        await_written(
            &mut status,
            &topic_b,
            u64::from(burst * B_BURST),
            "two-topics prefix burst",
        );
    }

    // The second writer appears, ARMS labeling, and publishes ALONE — so the
    // arming frame's ordinal is unambiguous.
    let mut b2 = multi_publisher_with_provisioning(&mgr, &topic_b, QUEUE, 4096);
    let ids_b: BTreeMap<char, u128> = [('R', b1.publisher_id()), ('S', b2.publisher_id())]
        .into_iter()
        .collect();
    b2.publish_raw(&frame('S', 0)).expect("publish S");
    await_written(&mut status, &topic_b, B_PREFIX + 1, "two-topics arming");

    for i in 0..B_TAIL {
        if i % 2 == 0 {
            b1.publish_raw(&frame('R', seq_b)).expect("publish R");
            seq_b += 1;
        } else {
            b2.publish_raw(&frame('S', i + 1)).expect("publish S");
        }
    }
    await_written(&mut status, &topic_b, B_TOTAL as u64, "two-topics tail");
    let summary = finish(handle, &shutdown, "two-topics");
    assert_eq!(
        summary.messages,
        (A_TOTAL + B_TOTAL) as u64,
        "every frame of both topics recorded"
    );

    let msgs = read_bag(&out);
    let chan_a = data_channel_id(&msgs, &topic_a);
    let chan_b = data_channel_id(&msgs, &topic_b);
    assert_ne!(
        chan_a, chan_b,
        "two topics are two channels — equal ids would make every per-channel claim below vacuous"
    );
    // Nothing was written naming a channel no tap owns.
    assert_every_label_names(&msgs, &[chan_a, chan_b], "two-topics");

    let all_ids: BTreeMap<char, u128> = ids_a
        .iter()
        .chain(ids_b.iter())
        .map(|(k, v)| (*k, *v))
        .collect();

    // ---- the DECLARED tap: labeled from ordinal 0, no catch-up ever ----
    let labels_a = labels_on(&msgs, chan_a);
    assert_eq!(
        labels_a.len(),
        A_TOTAL,
        "a declared multi-publisher topic carries ONE label per frame: {labels_a:?}"
    );
    assert!(
        labels_a.iter().all(|(_, l)| l.kind == KIND_FRAME_LABEL),
        "a topic armed AT OPEN has no unlabeled prefix, so it owes no CatchUpPrefix: {labels_a:?}"
    );
    let labelled_a = assert_labels_pair_with_their_frames(&msgs, chan_a, &all_ids, "two-topics /a");
    assert_eq!(
        labelled_a,
        (0..A_TOTAL as u64).collect::<Vec<_>>(),
        "the declared tap's ordinals are its OWN 0..{A_TOTAL} run"
    );

    // ---- the OBSERVED tap: ONE catch-up, then its OWN independent run ----
    let labels_b = labels_on(&msgs, chan_b);
    let catch_ups: Vec<&(usize, Label)> = labels_b
        .iter()
        .filter(|(_, l)| l.kind == KIND_CATCH_UP_PREFIX)
        .collect();
    assert_eq!(
        catch_ups.len(),
        1,
        "observed plurality mints EXACTLY ONE CatchUpPrefix for its own channel, found \
         {catch_ups:?} among {labels_b:?}"
    );
    let (catch_pos, catch) = *catch_ups[0];
    assert_eq!(
        catch.ordinal, B_PREFIX,
        "the prefix covers every frame of THIS channel written before arming — a count that came \
         from the other tap's ordinal would read {A_TOTAL}"
    );
    assert_eq!(
        catch.publisher_id, ids_b[&'R'],
        "the prefix belongs to THIS topic's first origin"
    );
    assert_eq!(
        msgs[catch_pos].data.as_slice(),
        expected_record_bytes(KIND_CATCH_UP_PREFIX, chan_b, B_PREFIX, ids_b[&'R']).as_slice(),
        "the CatchUpPrefix must be byte-identical to the hand-built oracle"
    );

    let labelled_b = assert_labels_pair_with_their_frames(&msgs, chan_b, &all_ids, "two-topics /b");
    assert_eq!(
        labelled_b,
        (B_PREFIX..B_TOTAL as u64).collect::<Vec<_>>(),
        "the observed tap's labeled run starts at its OWN prefix length and is gap-free"
    );

    // ---- the DURABLE half: the two rows tell the two different stories ----
    let health = read_record_health(&out);
    let ha = health_of(&health, &topic_a);
    let hb = health_of(&health, &topic_b);

    assert_eq!(ha.producer_labels, A_TOTAL as u64);
    assert!(
        !ha.label_catch_up,
        "the DECLARED tap never owed a catch-up — a marker here is another tap's"
    );
    assert!(ha.multi_publisher, "declared is what armed /a");
    assert_eq!(
        ha.gap_detection_disabled_reason.as_deref(),
        Some("declared_multi_publisher"),
        "a declared multi-publisher topic never armed gap detection in the first place"
    );
    assert_eq!(ha.frames_recorded, A_TOTAL as u64);
    assert_labels_bounded_by_frames(ha, &topic_a);

    assert_eq!(
        hb.producer_labels,
        B_TOTAL as u64 - B_PREFIX,
        "the observed tap counts only the frames from its arming one on"
    );
    assert!(
        hb.label_catch_up,
        "the runtime-armed marker belongs to the tap that OBSERVED plurality"
    );
    assert!(
        !hb.multi_publisher,
        "the graph declared nothing about /b — that is what makes it the observed case"
    );
    assert_eq!(
        hb.gap_detection_disabled_reason.as_deref(),
        Some("observed_multi_publisher"),
        "observing a second writer stands THIS topic's gap accounting down, under its own reason"
    );
    assert_eq!(hb.frames_recorded, B_TOTAL as u64);
    assert_labels_bounded_by_frames(hb, &topic_b);

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ===========================================================================
// ARM 8 — the label channel is RESERVED, so the user walk never yields it
// ===========================================================================

/// `cerulion_bag`'s `UserFrameWalk` — the walk replay drives — must never yield
/// a producer record as a user frame.
///
/// The exclusion is spelled ONE way: `next_user_frame` skips a channel whose
/// topic `starts_with(RESERVED_PREFIX)`, and `RESERVED_PREFIX` is SLASH-LESS
/// (`__cerulion/`). So `FRAME_PRODUCERS_TOPIC` respelled with a leading slash —
/// which reads perfectly well as a topic name, and which every other arm in this
/// file would still pass because they resolve the channel by that same constant
/// — turns the reserved channel into a USER topic. Replay would then see 28-byte
/// records on a channel its graph never declared: a `BagGraphMismatch`, i.e. a
/// bag that refuses to replay because it carries its own attribution.
///
/// Driven through the PRODUCTION walk rather than by comparing the constant to a
/// literal, on the `bag_state_channel_test` precedent — with the ANTI-TAUTOLOGY
/// half in the same body, since "the walk yields no label" is also true of a
/// walk that yields nothing at all.
#[test]
#[serial_test::serial]
fn the_label_channel_is_reserved_so_replay_never_sees_it_as_a_user_topic() {
    const PER_WRITER: u32 = 3;
    const TOTAL: usize = (PER_WRITER as usize) * 2;

    let mgr = make_manager(QUEUE);
    let topic = unique_topic("reserved");
    let out = unique_out("reserved");
    let ready = unique_ready_file("reserved");

    let mut writer_a = multi_publisher_with_provisioning(&mgr, &topic, QUEUE, 4096);
    let mut writer_b = multi_publisher_with_provisioning(&mgr, &topic, QUEUE, 4096);

    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(
        mgr.clone(),
        record_cfg(out.clone(), vec![tap(&topic, true)], ready.clone()),
        shutdown.clone(),
    );
    await_bagd_ready(&ready, "reserved-channel");
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");
    settle();

    for i in 0..PER_WRITER {
        writer_a.publish_raw(&frame('A', i)).expect("publish A");
        writer_b.publish_raw(&frame('B', i)).expect("publish B");
    }
    await_written(&mut status, &topic, TOTAL as u64, "reserved-channel");
    finish(handle, &shutdown, "reserved-channel");

    // PRECONDITION: this bag really does carry producer records, or "the walk
    // yields none" is a claim about an empty channel.
    let msgs = read_bag(&out);
    assert_eq!(
        labels_in_order(&msgs).len(),
        TOTAL,
        "the walk exclusion is only meaningful over a bag that HAS labels"
    );

    let reader = BagReader::open(&out).expect("open bag");
    let channels = reader.channels().expect("channel table");
    assert!(
        channels.iter().any(|c| c.topic == FRAME_PRODUCERS_TOPIC),
        "the reserved channel must be in the table the walk resolves against: {:?}",
        channels.iter().map(|c| &c.topic).collect::<Vec<_>>()
    );
    let by_id: BTreeMap<u16, String> = channels.iter().map(|c| (c.id, c.topic.clone())).collect();

    let mut walk = reader.user_frames().expect("user frame walk");
    let mut user_topics: Vec<String> = Vec::new();
    while let Some((channel_id, _span)) = walk.next_user_frame().expect("walk step") {
        user_topics.push(by_id[&channel_id].clone());
    }

    // THE pin, and its anti-tautology half: the walk yields EXACTLY the data
    // topic's frames — none of the label channel's, and not an empty walk.
    assert_eq!(
        user_topics,
        vec![topic.clone(); TOTAL],
        "the user walk must yield ONLY this recording's {TOTAL} data frames — a producer record \
         reaching it is what makes replay refuse a labeled bag"
    );

    // The SPELLING that makes the exclusion hold, stated where a respelling
    // would be introduced rather than only where it would be felt.
    assert!(
        FRAME_PRODUCERS_TOPIC.starts_with(cerulion_bag::RESERVED_PREFIX),
        "the reserved prefix is SLASH-LESS ('{}'), so the label channel's name must be too — \
         got '{FRAME_PRODUCERS_TOPIC}'",
        cerulion_bag::RESERVED_PREFIX
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ===========================================================================
// ARM 9 — a DEFERRED arming batch keeps its catch-up
// ===========================================================================

/// The `CatchUpPrefix` is minted at the arming transition, TAKEN when a batch is
/// handed off, and must be PUT BACK when that hand-off is deferred.
///
/// A deferred hand-off is not a failure — the drain→writer channel is
/// transiently full, the frames are RETAINED in their taps and retried next
/// cycle, and nothing is lost. But the catch-up travelled WITH that batch, so a
/// defer that dropped it would leave the topic's whole single-writer prefix
/// unattributed FOR THE REST OF THE RECORDING, on nothing worse than a busy
/// writer — and silently, because a missing record is exactly what an
/// unarmed topic looks like.
///
/// Every other arm in this file hands off cleanly, so the restore loop is
/// reachable by none of them: deleting it leaves them all green.
///
/// The stall is engaged only AFTER the prefix is durably counted, so the frames
/// attempted inside the stall window are exactly the fillers and the arming one.
///
/// # Why there is a SECOND, idle tap
///
/// The restore walks `returned.catch_up` and hands each slot back to
/// `self.taps[i]`. With ONE tap in the recording that index is always `0`, so a
/// `self.taps[0]` slip is output-equivalent — MEASURED: it leaves this arm and
/// all nine others green. The decoy tap is named to sort FIRST, so the topic
/// that actually owes a catch-up is never index `0` and a slip hands its debt to
/// a tap that is not labeling and will therefore never write it.
#[test]
#[serial_test::serial]
fn a_deferred_arming_batch_keeps_its_catch_up() {
    /// Frames written before the stall is engaged.
    const PRE_STALL: u32 = 6;
    /// Frames published while stalled, to fill the bounded drain→writer channel
    /// so the arming frame's batch is the one that cannot be handed off.
    const FILLERS: u32 = FILL_THE_CHANNEL;
    /// Everything before the arming frame — what the catch-up must name.
    const PREFIX: u64 = (PRE_STALL + FILLERS) as u64;
    const TAIL: u32 = 2;
    const TOTAL: usize = PREFIX as usize + 1 + TAIL as usize;
    /// Deep enough that the tap's own receive queue can never lap, so the ONLY
    /// mechanism under test is the channel-full DEFER.
    const DEEP: usize = 256;

    /// Frames on the idle DECOY tap, which owes no catch-up ever.
    const DECOY_FRAMES: u32 = 2;

    let mgr = make_manager(QUEUE);
    // `decoy_a` sorts before `deferred`, so the tap that owes the catch-up is
    // never index 0 (see the doc above).
    let topic_decoy = unique_topic("decoy_a");
    let topic = unique_topic("deferred");
    let out = unique_out("deferred");
    let ready = unique_ready_file("deferred");

    let mut writer_decoy = multi_publisher_with_provisioning(&mgr, &topic_decoy, DEEP, 4096);
    let mut writer_a = multi_publisher_with_provisioning(&mgr, &topic, DEEP, 4096);

    let gate = Arc::new(cerulion_bagd::WriterStallGate::default());
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = record_cfg(
        out.clone(),
        vec![tap(&topic_decoy, false), tap(&topic, false)],
        ready.clone(),
    );
    cfg.fault_inject_writer_stall_gate = Some(gate.clone());
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    await_bagd_ready(&ready, "deferred-catch-up");
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");
    settle();

    for i in 0..DECOY_FRAMES {
        writer_decoy.publish_raw(&frame('D', i)).expect("publish D");
    }
    let mut seq = 0u32;
    for _ in 0..PRE_STALL {
        writer_a.publish_raw(&frame('A', seq)).expect("publish A");
        seq += 1;
    }
    await_written(&mut status, &topic, u64::from(PRE_STALL), "deferred prefix");
    await_written(
        &mut status,
        &topic_decoy,
        u64::from(DECOY_FRAMES),
        "deferred decoy",
    );

    // STALL the writer, then fill the drain->writer channel EXACTLY: one filler
    // per successful hand-off, each rendezvoused on the recorder's own written
    // count, so the last one leaves the writer blocked with the channel full and
    // the NEXT batch cannot be handed off at all.
    //
    // One filler per rendezvous rather than a spaced burst, because a burst can
    // be COMPRESSED into a single batch by a loaded runner — the writer then
    // takes that one batch, the channel is EMPTY, and the arming frame is handed
    // off cleanly with nothing to restore.
    gate.engaged.store(true, Ordering::Relaxed);
    for k in 1..=FILLERS {
        writer_a.publish_raw(&frame('A', seq)).expect("publish A");
        seq += 1;
        await_written(
            &mut status,
            &topic,
            u64::from(PRE_STALL + k),
            "deferred channel filler",
        );
    }
    assert!(
        await_condition(RECORDER_JOIN_DEADLINE, || gate
            .entered
            .load(Ordering::Relaxed)),
        "the writer must ENTER the stall, or the channel never fills and nothing defers"
    );

    // The SECOND writer arrives while the channel is full: this frame arms
    // labeling, mints the catch-up, and its batch cannot be handed off.
    let mut writer_b = multi_publisher_with_provisioning(&mgr, &topic, DEEP, 4096);
    let ids: BTreeMap<char, u128> = [
        ('A', writer_a.publisher_id()),
        ('B', writer_b.publisher_id()),
    ]
    .into_iter()
    .collect();
    writer_b.publish_raw(&frame('B', 0)).expect("publish B");
    // Hold the stall until the recorder's OWN loop has run for several flush
    // cycles, so the batch holding the arming frame has certainly been attempted
    // against the full channel and deferred. A wall-clock sleep here would pass
    // on a starved runner that attempted nothing, silently turning this into the
    // ordinary observed-plurality arm.
    await_recorder_span_growth(
        &mut status,
        Duration::from_millis(200),
        "deferred arming batch",
    );
    gate.engaged.store(false, Ordering::Relaxed);

    await_written(&mut status, &topic, PREFIX + 1, "deferred arming");
    for i in 0..TAIL {
        if i % 2 == 0 {
            writer_a.publish_raw(&frame('A', seq)).expect("publish A");
            seq += 1;
        } else {
            writer_b.publish_raw(&frame('B', i + 1)).expect("publish B");
        }
    }
    await_written(&mut status, &topic, TOTAL as u64, "deferred tail");
    let summary = finish(handle, &shutdown, "deferred-catch-up");

    // A defer LOSES NOTHING — that is the contract this arm rides on,
    // and without it the ordinal oracle below would be measuring loss.
    assert_eq!(
        summary.messages,
        TOTAL as u64 + u64::from(DECOY_FRAMES),
        "every frame of both taps recorded"
    );
    assert_eq!(summary.frames_lost, 0, "the deep queue must not lap");
    assert_eq!(summary.dropped_unwritten, 0, "a DEFER is not a drop");

    let health = read_record_health(&out);
    let h = health_of(&health, &topic);
    // PRECONDITION, not the pin: without a real defer this arm is the
    // ordinary observed-plurality arm with extra sleeps.
    assert!(
        h.defer_count > 0,
        "the stall must have forced at least one hand-off to DEFER, or the restore path was never \
         exercised (defer_count = {})",
        h.defer_count
    );

    let msgs = read_bag(&out);
    let channel = data_channel_id(&msgs, &topic);
    let decoy_channel = data_channel_id(&msgs, &topic_decoy);
    assert_ne!(channel, decoy_channel);
    // NOTHING was written naming the decoy: a restore handed to the wrong tap
    // gives the debt to a tap that is not labeling, so it can never be paid.
    assert_every_label_names(&msgs, &[channel], "deferred-catch-up");
    assert_eq!(
        ordinals_by_position(&msgs, channel).len(),
        TOTAL,
        "all {TOTAL} frames must be recorded"
    );

    let records = labels_in_order(&msgs);
    let catch_ups: Vec<&(usize, Label)> = records
        .iter()
        .filter(|(_, l)| l.kind == KIND_CATCH_UP_PREFIX)
        .collect();
    assert_eq!(
        catch_ups.len(),
        1,
        "the catch-up survives the defer EXACTLY ONCE — neither lost nor duplicated by the \
         retry: {records:?}"
    );
    let (catch_pos, catch) = *catch_ups[0];
    assert_eq!(
        catch.ordinal, PREFIX,
        "the restored catch-up still names every frame written before the arming one"
    );
    assert_eq!(catch.publisher_id, ids[&'A']);
    assert_eq!(
        msgs[catch_pos].data.as_slice(),
        expected_record_bytes(KIND_CATCH_UP_PREFIX, channel, PREFIX, ids[&'A']).as_slice(),
        "byte-identical to the hand-built oracle"
    );

    // The label stream is the one an UNDEFERRED run gives: gap-free ordinals
    // from the arming frame on (the hand oracle arm 2 pins for the clean path).
    let labelled = assert_labels_pair_with_their_frames(&msgs, channel, &ids, "deferred-catch-up");
    assert_eq!(
        labelled,
        (PREFIX..TOTAL as u64).collect::<Vec<_>>(),
        "a defer changes WHEN the batch is written, never WHAT it says"
    );

    assert_eq!(h.producer_labels, TOTAL as u64 - PREFIX);
    assert!(
        h.label_catch_up,
        "the runtime-armed marker must survive the defer too"
    );
    assert_labels_bounded_by_frames(h, &topic);

    // The decoy earned NOTHING — it never saw a second writer, and a debt handed
    // to it by a mis-indexed restore is one it could never have owed.
    let hd = health_of(&health, &topic_decoy);
    assert_eq!(hd.producer_labels, 0, "the decoy tap has one writer");
    assert!(!hd.label_catch_up);
    assert_eq!(hd.frames_recorded, u64::from(DECOY_FRAMES));

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ===========================================================================
// ARM 10 — a FORCE-DROPPED arming batch trips the impossible-triple tripwire
// ===========================================================================

/// Arming is ONE-WAY on the DRAIN side, and the catch-up rides a BATCH — so a
/// batch that is dropped rather than deferred takes the catch-up with it while
/// every later frame keeps being labeled.
///
/// The bag that results is internally inconsistent in a way `record_health.json`
/// can state and nothing else can: labels on a topic that is neither DECLARED
/// `multi_publisher` nor carrying the catch-up an observed second writer always
/// mints. `build_record_health` calls that triple IMPOSSIBLE and warns, because
/// the alternative is a manifest that quietly contradicts itself — and this arm
/// is the only place the triple is reachable at all (the production path is
/// defer-not-drop; the drop is the A/B control seam).
///
/// `run_bagd` runs on the TEST thread with the stimulus on a helper — the
/// INVERSE of every other arm here — because `tracing-test` scopes its capture
/// to the test's span and a spawned thread does not inherit it, so with the
/// usual harness the warn assertion would be vacuous.
#[test]
#[serial_test::serial]
#[tracing_test::traced_test]
fn a_force_dropped_arming_batch_trips_the_impossible_triple_tripwire() {
    /// Frames written before the stall is engaged.
    const PRE_STALL: u32 = 6;
    /// Frames published while stalled, to fill the bounded channel.
    const FILLERS: u32 = FILL_THE_CHANNEL;
    /// Frames after the release. The rendezvous below waits for
    /// `PRE_STALL + FILLERS + 1`, which is UNREACHABLE without them: the arming
    /// frame's batch is dropped, so `PRE_STALL + FILLERS` is the most that can be
    /// written between the stall and the release.
    const TAIL: u32 = 6;
    const DEEP: usize = 256;

    let mgr = make_manager(QUEUE);
    let topic = unique_topic("forcedrop");
    let out = unique_out("forcedrop");
    let ready = unique_ready_file("forcedrop");

    // Created HERE, before the recorder starts, and MOVED into the stimulus: the
    // tap is an open-only subscriber built in `Recorder::setup`, so a topic that
    // does not exist yet is a hard refusal rather than a late attach.
    let mut writer_a = multi_publisher_with_provisioning(&mgr, &topic, DEEP, 4096);

    let gate = Arc::new(cerulion_bagd::WriterStallGate::default());
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = record_cfg(out.clone(), vec![tap(&topic, false)], ready.clone());
    cfg.fault_inject_writer_stall_gate = Some(gate.clone());
    // The A/B control: a full channel DROPS the batch instead of
    // retaining it, which is what strands the catch-up.
    cfg.fault_inject_force_drop_on_channel_full = true;

    let mgr_c = mgr.clone();
    let topic_c = topic.clone();
    let ready_c = ready.clone();
    let gate_c = gate.clone();
    let shutdown_c = shutdown.clone();
    let stimulus = std::thread::spawn(move || -> BTreeMap<char, u128> {
        // The recorder runs on the TEST thread here, so a panic below must still
        // stop it or this test HANGS instead of failing (see `ShutdownOnDrop`).
        let _stop = ShutdownOnDrop(shutdown_c);
        await_bagd_ready(&ready_c, "force-drop");
        let mut status = mgr_c
            .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
            .expect("status subscriber");
        settle();

        let mut seq = 0u32;
        for _ in 0..PRE_STALL {
            writer_a.publish_raw(&frame('A', seq)).expect("publish A");
            seq += 1;
        }
        await_written(
            &mut status,
            &topic_c,
            u64::from(PRE_STALL),
            "force-drop prefix",
        );

        // Fill the channel EXACTLY: one filler per successful hand-off, each
        // rendezvoused on the recorder's own written count. A spaced burst can be
        // COMPRESSED into one batch by a loaded runner, which leaves the channel
        // empty and lets the arming frame through cleanly.
        gate_c.engaged.store(true, Ordering::Relaxed);
        for k in 1..=FILLERS {
            writer_a.publish_raw(&frame('A', seq)).expect("publish A");
            seq += 1;
            await_written(
                &mut status,
                &topic_c,
                u64::from(PRE_STALL + k),
                "force-drop channel filler",
            );
        }
        assert!(
            await_condition(RECORDER_JOIN_DEADLINE, || gate_c
                .entered
                .load(Ordering::Relaxed)),
            "the writer must ENTER the stall, or the channel never fills and nothing drops"
        );

        // The arming frame lands on a FULL channel, so its batch — carrying the
        // catch-up it just minted — is dropped outright.
        let mut writer_b = multi_publisher_with_provisioning(&mgr_c, &topic_c, DEEP, 4096);
        let ids: BTreeMap<char, u128> = [
            ('A', writer_a.publisher_id()),
            ('B', writer_b.publisher_id()),
        ]
        .into_iter()
        .collect();
        writer_b.publish_raw(&frame('B', 0)).expect("publish B");
        // Hold the stall until a drop is REPORTED. With `force_drop` a full
        // channel drops rather than retains, so a tap holds nothing between
        // flushes and the batch carrying the arming frame is the one attempted
        // next — releasing on a timer instead would let a slow runner hand that
        // batch off cleanly, which silently turns this into the ordinary
        // observed-plurality arm.
        await_dropped(&mut status, 1, "force-drop arming batch");

        // RELEASE, THEN RENDEZVOUS — never release and publish in one breath.
        //
        // At this instant the writer holds one batch inside the stall and the
        // bounded hand-off channel holds the other, so the channel is FULL. The
        // slot frees only once the released writer returns to its `recv`, and
        // under `force_drop` a flush landing before then DROPS its batch
        // outright — with nothing left to re-drain, because the tail below is
        // the last thing anybody publishes. Publishing straight after the
        // release is therefore a bet that the writer thread is scheduled before
        // the drive loop's next pass, and the drive loop does NOT sleep after a
        // flush that cleared its held frames (`next_pass_sleep`'s progress arm).
        // A loaded runner loses that bet: the whole TAIL is force-dropped and
        // the wait below dies at its deadline having written nothing.
        //
        // `entered` is set once per batch the writer takes, so clearing it
        // BEFORE the release turns "the writer has taken the QUEUED batch off
        // the channel, so there is room again" into an OBSERVED fact. Clearing
        // it here is safe precisely because the writer is parked in the stall:
        // it cannot be taking a batch, so the next store can only come from the
        // batch that frees the slot. Load can only DELAY this wait.
        gate_c.entered.store(false, Ordering::Relaxed);
        gate_c.engaged.store(false, Ordering::Relaxed);
        assert!(
            await_condition(RECORDER_JOIN_DEADLINE, || gate_c
                .entered
                .load(Ordering::Relaxed)),
            "force-drop release: the writer never took another batch off the hand-off channel \
             within {RECORDER_JOIN_DEADLINE:?}, so the channel stayed full and the tail below \
             would have been force-dropped exactly like the arming batch"
        );

        for i in 0..TAIL {
            if i % 2 == 0 {
                writer_a.publish_raw(&frame('A', seq)).expect("publish A");
                seq += 1;
            } else {
                writer_b.publish_raw(&frame('B', i + 1)).expect("publish B");
            }
        }
        // Only the TAIL can carry the count this high (see `TAIL`'s doc).
        await_written(
            &mut status,
            &topic_c,
            u64::from(PRE_STALL + FILLERS + 1),
            "force-drop tail",
        );
        // `_stop` drops HERE and signals the recorder — on this path and on an
        // unwind alike.
        ids
    });

    let summary = run_bagd(mgr.clone(), cfg, shutdown).expect("the recorder must finalize cleanly");
    let ids = stimulus.join().expect("stimulus thread");

    // PRECONDITIONS, not the pin: without a real drop this is the ordinary
    // observed-plurality arm, and every assertion below would be about a
    // recording in which nothing went wrong.
    assert!(
        summary.dropped_unwritten > 0,
        "the force-drop seam must actually drop a batch, or the stranded catch-up never happens"
    );

    let msgs = read_bag(&out);
    let channel = data_channel_id(&msgs, &topic);
    assert_every_label_names(&msgs, &[channel], "force-drop");
    let records = labels_in_order(&msgs);
    let frame_labels = records
        .iter()
        .filter(|(_, l)| l.kind == KIND_FRAME_LABEL)
        .count();
    assert!(
        frame_labels > 0,
        "arming is ONE-WAY on the drain side, so every frame after the dropped batch is still \
         labeled: {records:?}"
    );
    assert_eq!(
        records
            .iter()
            .filter(|(_, l)| l.kind == KIND_CATCH_UP_PREFIX)
            .count(),
        0,
        "the catch-up went with the batch that was dropped — nothing re-mints it: {records:?}"
    );
    // The labels that DID land still name their own frames.
    assert_labels_pair_with_their_frames(&msgs, channel, &ids, "force-drop");

    // THE TRIPLE, on the durable document.
    let health = read_record_health(&out);
    let h = health_of(&health, &topic);
    assert!(
        h.producer_labels > 0,
        "labels were earned: {}",
        h.producer_labels
    );
    assert!(!h.multi_publisher, "the graph declared nothing");
    assert!(
        !h.label_catch_up,
        "the marker is set on the hand-off that carried the catch-up, and that hand-off was \
         dropped — which is exactly the shape build_record_health calls impossible"
    );
    // The AUTHORITATIVE signal, and the two-surfaces-agree pin's bag-side half.
    // Arming happened on the DRAIN side, so the gap stand-down is unaffected by
    // the drop — which is precisely why `bag_cmd::classify_loss` must route on
    // THIS field and not on `label_catch_up`: with only the emission flag to go
    // on, this recording reports a verified `0` for a topic whose gap arithmetic
    // was abandoned six frames in. The classify half is
    // `cerulion_cli_engine::bag_cmd`'s
    // `a_stood_down_topic_with_no_catch_up_record_is_still_unmeasured`, which
    // drives EXACTLY this field triple (it cannot live here: `cerulion_cli_engine`
    // depends on this crate, so importing it back would be a package cycle).
    assert_eq!(
        h.gap_detection_disabled_reason.as_deref(),
        Some("observed_multi_publisher"),
        "the drain-side stand-down is the only surviving evidence that this topic went \
         multi-writer — without it nothing downstream can tell this bag from a clean one"
    );
    assert!(
        health.dropped_unwritten > 0,
        "the durable stamp carries the drop too: {health:?}"
    );
    assert_labels_bounded_by_frames(h, &topic);

    // The LOUD half. A manifest that contradicts itself must say so.
    //
    // Every field is matched as a WHOLE WHITESPACE TOKEN: the
    // message itself names `multi_publisher` and `label_catch_up` in prose, so a
    // substring check would be satisfied by the sentence rather than the data,
    // and `producer_labels=6` is a prefix of `producer_labels=60`.
    let expected: Vec<String> = vec![
        format!("topic={topic}"),
        "multi_publisher=false".to_string(),
        "label_catch_up=false".to_string(),
        format!("producer_labels={}", h.producer_labels),
    ];
    let triple = (h.producer_labels, h.multi_publisher, h.label_catch_up);
    logs_assert(|lines: &[&str]| {
        let heads: Vec<&&str> = lines
            .iter()
            .filter(|l| l.contains("PRODUCER LABEL BOOKKEEPING IMPOSSIBLE"))
            .collect();
        if heads.len() != 1 {
            // Carry the MANIFEST's own triple into the failure: the assertions
            // above have already established it held, so a missing warn means
            // the recorder and its own document disagree — and which half moved
            // is the whole diagnosis.
            return Err(format!(
                "the impossible triple must be announced EXACTLY once (latched per tap), got \
                 {heads:?}. The manifest for this topic reads producer_labels={}, \
                 multi_publisher={}, label_catch_up={} — i.e. the triple the tripwire tests. \
                 All captured lines: {lines:?}",
                triple.0, triple.1, triple.2
            ));
        }
        let head = heads[0];
        if !head.split_whitespace().any(|t| t == "WARN") {
            return Err(format!(
                "a self-contradicting manifest must be LOUD: {head:?}"
            ));
        }
        for field in &expected {
            if !head.split_whitespace().any(|t| t == field) {
                return Err(format!("the warn must carry `{field}`: {head:?}"));
            }
        }
        Ok(())
    });

    // The INVERSE tripwire, which the one above is structurally blind to: it
    // keys on `producer_labels > 0`, so a topic that stood its gap accounting
    // down and then handed over NO catch-up trips this one instead. Both fire on
    // this arm — the recording is short one record either way — and they are
    // latched SEPARATELY so neither open regime can swallow the other's head.
    let topic_field = format!("topic={topic}");
    logs_assert(|lines: &[&str]| {
        let heads: Vec<&&str> = lines
            .iter()
            .filter(|l| l.contains("PRODUCER LABEL CATCH-UP UNACCOUNTED"))
            .collect();
        if heads.len() != 1 {
            return Err(format!(
                "the unaccounted catch-up must be announced EXACTLY once (latched per tap), got \
                 {heads:?}. All captured lines: {lines:?}"
            ));
        }
        let head = heads[0];
        if !head.split_whitespace().any(|t| t == "WARN") {
            return Err(format!(
                "a missing prefix attribution must be LOUD: {head:?}"
            ));
        }
        for field in [topic_field.as_str(), "label_catch_up=false"] {
            if !head.split_whitespace().any(|t| t == field) {
                return Err(format!("the warn must carry `{field}`: {head:?}"));
            }
        }
        Ok(())
    });

    // The ARMING line, and its one NUMBER. `frames_drained_before` is what the
    // drain thread had taken BEFORE the arming frame, and on this arm that is
    // deterministic rather than approximate: every prefix frame is rendezvoused
    // on the recorder's own written count before the next is published, so the
    // drain thread has taken exactly `PRE_STALL + FILLERS` when writer B's first
    // frame arrives. An off-by-one there (dropping the `- 1`, or bumping
    // `drained_total` after the observe instead of before) is invisible to every
    // other assertion in this file.
    let expected_before = format!("frames_drained_before={}", PRE_STALL + FILLERS);
    logs_assert(|lines: &[&str]| {
        let heads: Vec<&&str> = lines
            .iter()
            .filter(|l| l.contains("PRODUCER LABELING ARMED"))
            .collect();
        if heads.len() != 1 {
            return Err(format!(
                "arming is ONE-WAY, so it must be announced exactly once: {heads:?}"
            ));
        }
        let head = heads[0];
        if !head.split_whitespace().any(|t| t == "INFO") {
            return Err(format!("the arming line is an INFO: {head:?}"));
        }
        if !head.split_whitespace().any(|t| t == expected_before) {
            return Err(format!(
                "the arming line must carry `{expected_before}` — the run BEFORE the arming \
                 frame, not including it: {head:?}"
            ));
        }
        Ok(())
    });

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ===========================================================================
// A FLASHBACK CAPTURE carries the same attribution
// ===========================================================================
//
// Each frame's origin is threaded through `StagedFrames::suffix_from` into
// the Flashback window. Stopping there would leave a capture of a shared topic
// UNLABELLED: its `__cerulion/frame_producers` channel would exist and be empty.
// So the capture path MINTS the labels, which includes defining the
// capture bag's ordinal base.
//
// # The ordinal base, and why it is not a free choice
//
// `ProducerRecord` DEFINES a `FrameLabel`'s ordinal as "that frame's 0-based
// index in that channel's stream of frames written to THIS bag". A capture IS a
// bag, so its ordinals start at 0 at its own first frame of the channel. The
// alternative — continuing the recording's numbering — names positions the
// capture does not contain, which is unresolvable for the reader a black box
// exists to serve: somebody holding the capture and not the recording.
//
// `a_capture_numbers_its_own_frames_from_zero_not_from_the_recordings_count` is
// where that is DISCRIMINATING rather than incidental: its window span is short
// enough that the capture provably begins mid-recording, so a base taken from
// the recorder's running count and a base of 0 give different answers on the
// very same frames.
//
// | # | Function |
// |---|---|
// | 11 | `a_capture_of_a_declared_shared_topic_labels_every_frame_it_holds` |
// | 12 | `a_capture_numbers_its_own_frames_from_zero_not_from_the_recordings_count` |
// | 13 | `a_capture_attributes_its_unlabelled_prefix_with_one_catch_up_record` |
// | 14 | `a_capture_of_a_single_writer_topic_writes_no_producer_records` |
// | 15 | `a_capture_of_two_labelled_topics_keeps_each_channels_ordinals_to_itself` |
// | 16 | `a_headerless_frame_in_a_capture_is_labelled_and_consumes_its_ordinal` |
//
// | Mutant | Killed by |
// |---|---|
// | the capture path never mints (origins reach the window and nothing is made of them) | 11 (finds 0 labels), 13, and 12 |
// | ordinals continued from the recorder's running count | 12 (the capture's first ordinal is 0 while the recording's is not) |
// | a label written AFTER its frame | 11 (adjacency, through the shared helper) |
// | the catch-up dropped from the capture | 13 (its leading run is unattributed) |
// | the catch-up's `prefix_len` taken from the recording rather than the capture | 13 (asserted against the capture's OWN unlabelled run) |
// | the catch-up attributed to the ARMING writer rather than the first | 13 (`publisher_id` is A's) |
// | labels minted on a single-writer capture | 14 |
// | the origin dropped between the window and the capture writer | 11 (the `u128` cross-check against the frame's own body) |
// | ONE shared ordinal counter across topics, or any per-topic state read at `[0]` rather than by NAME | 15 ONLY — arms 11-14 each capture a SINGLE topic, where "this channel's ordinal" and "any ordinal" are the same number, so all four stay green under it |
// | a headerless frame's label SKIPPED on the capture path | 16 ONLY — arms 11-15 publish only parseable frames |
// | a headerless frame's label stamped with a wall time rather than the frame's fabricated 0 | 16 |
// | the capture's ordinal advance moved INSIDE the label guard | 16, and also 13 and 15 (their unlabelled prefixes collapse) |

/// A `--record` config that ALSO holds the Flashback window.
///
/// `--record` rather than window-only deliberately: it gives these arms the same
/// `/bagd/status` `per_topic` rendezvous every arm above uses (a window-only
/// recorder writes no continuous bag, so it reports no written count), and it
/// makes the continuous bag available as a FOIL — arm 12 reads both and requires
/// them to number the same frames differently.
fn record_cfg_with_window(
    out: PathBuf,
    taps: Vec<TapSpec>,
    ready: PathBuf,
    dir: &Path,
    window_span: Duration,
) -> BagdConfig {
    let mut cfg = record_cfg(out, taps, ready);
    cfg.flashback = Some(FlashbackSettings {
        window_span,
        window_max_bytes: 64 * 1024 * 1024,
        anchor_max_bytes: 64 * 1024 * 1024,
        anchor_cap_basis: cerulion_core::flashback::CapBasis::Env,
        trace_max_bytes: cerulion_core::flashback::DEFAULT_FLASHBACK_TRACE_MAX_MB * 1024 * 1024,
        dir: dir.to_path_buf(),
        label: "probe".into(),
        caps: cerulion_core::flashback::retention::RetentionCaps::default(),
        policy: TriggerPolicy {
            // The shipped post window is 15 s and has nothing to do with
            // labelling; waiting it out would make these arms a minute long.
            post_window_ns: 300 * 1_000_000,
            ..Default::default()
        },
        posture: TriggerPosture::default(),
        exclude_topics: cerulion_core::flashback::ExcludeTopics::default(),
        window_only: false,
        tap_budget_bytes: cerulion_core::flashback::DEFAULT_FLASHBACK_TAP_BUDGET_MB * 1024 * 1024,
    });
    cfg
}

/// A private directory for a capture to land in.
fn capture_dir(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "{tag}-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos())
            .unwrap_or(0)
    ));
    std::fs::create_dir_all(&dir).expect("capture dir");
    dir
}

/// Every `.mcap` in `dir`, sorted.
fn captures_in(dir: &Path) -> Vec<PathBuf> {
    let mut out: Vec<PathBuf> = std::fs::read_dir(dir)
        .map(|r| {
            r.flatten()
                .map(|e| e.path())
                .filter(|p| p.extension().and_then(|e| e.to_str()) == Some("mcap"))
                .collect()
        })
        .unwrap_or_default();
    out.sort();
    out
}

/// Ask for a capture over the REAL trigger channel and block until the recorder
/// reports it FINISHED, then return the finalized bag's path.
///
/// # ONE drain loop, not two
///
/// `FlashbackRequester::drain_outcomes` CONSUMES the queue, so two sequential
/// `await_condition` loops — one waiting for `Accepted`, then one for
/// `Finished` — can lose the race outright: if the first successful drain
/// happens after the recorder has already published BOTH (the post window here
/// is 300 ms, and the test thread need only be descheduled once), the first
/// loop reads `Finished`, discards it as "not Accepted", and the second loop
/// then waits for a frame that no longer exists and times out. That is a wait
/// load can INVERT rather than merely delay, which this repo's test rules
/// forbid.
///
/// So both verdicts are latched in ONE pass over each drain, and the condition
/// is `finished` alone — `Accepted` necessarily preceded it, and asserting it
/// separately is what created the hazard. Waiting for `Finished` (rather than
/// for the file to exist) is still load-safe and still load-BEARING: it is the
/// verdict that says the bag has been finalized, and reading a file that merely
/// exists would read a bag still being appended to.
fn capture_now(mgr: &Arc<TransportManager>, dir: &Path, what: &str) -> PathBuf {
    let requester = FlashbackRequester::open_on_manager(mgr).expect("requester");
    let request_id = requester
        .request(&CaptureRequest::manual(what))
        .expect("request");
    let mut accepted = false;
    let mut finished = false;
    assert!(
        await_condition(RECORDER_JOIN_DEADLINE, || {
            for frame in requester.drain_outcomes(request_id) {
                match frame.outcome {
                    FlashbackOutcome::Accepted { .. } => accepted = true,
                    FlashbackOutcome::Finished { .. } => finished = true,
                    _ => {}
                }
            }
            finished
        }),
        "{what}: the recorder must ACCEPT the capture request over the trigger channel and report \
         it FINISHED (which is what says the bag is finalized and safe to read). accepted={accepted}"
    );
    let files = captures_in(dir);
    assert_eq!(
        files.len(),
        1,
        "{what}: exactly one capture must have landed in {}, found {files:?}",
        dir.display()
    );
    files.into_iter().next().expect("exactly one, asserted")
}

// ---------------------------------------------------------------------------
// ARM 11 — a declared shared topic's capture labels every frame it holds
// ---------------------------------------------------------------------------

/// Two writers interleave on a declared `multi_publisher_topics:` topic; the
/// FLASHBACK CAPTURE — not the recording — carries a label for every frame it
/// holds, each naming the writer that frame's own body names, each written
/// immediately before it.
///
/// This is the arm a capture path that carries the origins into
/// the window and mints nothing from them cannot pass.
#[test]
#[serial_test::serial]
fn a_capture_of_a_declared_shared_topic_labels_every_frame_it_holds() {
    const PER_WRITER: u32 = 4;
    const TOTAL: usize = (PER_WRITER as usize) * 2;

    let mgr = make_manager(QUEUE);
    let topic = unique_topic("capdecl");
    let out = unique_out("cap_declared");
    let ready = unique_ready_file("cap_declared");
    let dir = capture_dir("cap-declared");

    let mut writer_a = multi_publisher_with_provisioning(&mgr, &topic, QUEUE, 4096);
    let mut writer_b = multi_publisher_with_provisioning(&mgr, &topic, QUEUE, 4096);
    let ids: BTreeMap<char, u128> = [
        ('A', writer_a.publisher_id()),
        ('B', writer_b.publisher_id()),
    ]
    .into_iter()
    .collect();
    assert_ne!(ids[&'A'], ids[&'B'], "two writers, two ids");

    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(
        mgr.clone(),
        record_cfg_with_window(
            out.clone(),
            vec![tap(&topic, true)],
            ready.clone(),
            &dir,
            // Long enough that nothing is evicted: this arm is about the labels,
            // not about the window's reach.
            Duration::from_secs(30),
        ),
        shutdown.clone(),
    );
    await_bagd_ready(&ready, "cap-declared");
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");
    settle();

    for i in 0..PER_WRITER {
        writer_a.publish_raw(&frame('A', i)).expect("publish A");
        writer_b.publish_raw(&frame('B', i)).expect("publish B");
    }
    await_written(&mut status, &topic, TOTAL as u64, "cap-declared");

    let capture = capture_now(&mgr, &dir, "cap-declared");
    let _ = finish(handle, &shutdown, "cap-declared");

    let msgs = read_bag(&capture);
    let channel = data_channel_id(&msgs, &topic);
    let frames: Vec<&Msg> = msgs.iter().filter(|m| m.channel_id == channel).collect();
    assert_eq!(
        frames.len(),
        TOTAL,
        "the window held every frame, so the capture must carry all {TOTAL}"
    );
    assert_every_label_names(&msgs, &[channel], "cap-declared");

    // The headline: every frame is labelled, the labels are ADJACENT to their
    // frames, and each `u128` matches the writer the frame's own BODY names — an
    // identity the recorder never supplied.
    let labelled = assert_labels_pair_with_their_frames(&msgs, channel, &ids, "cap-declared");
    assert_eq!(
        labelled,
        (0..TOTAL as u64).collect::<Vec<u64>>(),
        "a DECLARED shared topic is labelled from its first frame, so the capture's labelled \
         ordinals must be exactly 0..{TOTAL}"
    );

    // A declared tap owes no catch-up: it was armed at open, so there is no
    // unlabelled run ahead of anything.
    assert!(
        labels_on(&msgs, channel)
            .iter()
            .all(|(_, l)| l.kind == KIND_FRAME_LABEL),
        "a DECLARED tap has no unlabelled prefix, so its capture must carry no CatchUpPrefix: {:?}",
        labels_on(&msgs, channel)
    );

    // BYTE oracle on one record, built by hand rather than by `encode`.
    let (pos, first) = labels_on(&msgs, channel)
        .into_iter()
        .next()
        .expect("at least one label, asserted above");
    let tag = writer_tag(&msgs[pos + 1].data);
    assert_eq!(
        msgs[pos].data.as_slice(),
        expected_record_bytes(KIND_FRAME_LABEL, channel, 0, ids[&tag]).as_slice(),
        "the capture's first producer record must be the 28 bytes a FrameLabel for ordinal 0 \
         is, decoded {first:?}"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
    let _ = std::fs::remove_dir_all(&dir);
}

// ---------------------------------------------------------------------------
// ARM 12 — the ordinal BASE is the capture's own first frame
// ---------------------------------------------------------------------------

/// A capture that begins MID-RECORDING numbers its frames from 0, not from the
/// count the recorder had already written.
///
/// # How the precondition is made a FACT rather than a hope
///
/// The window span is short, and the first burst is published, RENDEZVOUSED on
/// the recorder's own written count, and then left to age for longer than that
/// span before the second burst goes out. The capture's floor is
/// `capture_time - window_span`, and the capture happens after the second burst,
/// so the first burst is older than the floor BY CONSTRUCTION. Load can only
/// make it older still — it can delay this arm, never invert it (the
/// load-robustness rule).
///
/// The arm then asserts the precondition it relies on (the capture holds FEWER
/// frames than the recording) before making its claim, so a run in which the
/// eviction did not happen fails naming that rather than passing vacuously.
///
/// # The OTHER precondition, and why the budget is seconds
///
/// The evicted side is load-monotone. The SURVIVING side is not: burst 2 has to
/// still be inside the window when the recorder freezes the capture's floor, and
/// between the rendezvous on burst 2 and that instant sit a trigger-channel
/// round trip and a drive-loop pass. A stall longer than the window there makes
/// the capture hold nothing.
///
/// That direction fails LOUD rather than green (the `cap_frames > 0` assertion
/// names it), so it can never fabricate a pass — but a 400 ms budget was thin on
/// exactly the runner class this repo has documented for timer coalescing, so
/// the pair is scaled to 3 s / 9 s. The RATIO is what the arm is about; the
/// absolute numbers only have to keep both sides off the runner's noise floor.
/// Cost: this one serial arm spends ~9 s.
#[test]
#[serial_test::serial]
fn a_capture_numbers_its_own_frames_from_zero_not_from_the_recordings_count() {
    /// Frames per burst, per writer.
    const BURST: u32 = 3;
    /// Both writers publish each burst, so a burst is this many frames.
    const PER_BURST: u64 = (BURST as u64) * 2;
    /// The window's reach. Short (relative to the run), because this arm is
    /// ABOUT eviction — but not SO short that the surviving burst's own budget
    /// becomes a wall load can trip. See the note on both preconditions below.
    const WINDOW: Duration = Duration::from_secs(3);
    /// Longer than the window, so the first burst is provably outside it.
    const AGE_OUT: Duration = Duration::from_secs(9);

    let mgr = make_manager(QUEUE);
    let topic = unique_topic("capbase");
    let out = unique_out("cap_base");
    let ready = unique_ready_file("cap_base");
    let dir = capture_dir("cap-base");

    let mut writer_a = multi_publisher_with_provisioning(&mgr, &topic, QUEUE, 4096);
    let mut writer_b = multi_publisher_with_provisioning(&mgr, &topic, QUEUE, 4096);
    let ids: BTreeMap<char, u128> = [
        ('A', writer_a.publisher_id()),
        ('B', writer_b.publisher_id()),
    ]
    .into_iter()
    .collect();

    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(
        mgr.clone(),
        record_cfg_with_window(
            out.clone(),
            vec![tap(&topic, true)],
            ready.clone(),
            &dir,
            WINDOW,
        ),
        shutdown.clone(),
    );
    await_bagd_ready(&ready, "cap-base");
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");
    settle();

    // BURST 1 — destined to fall out of the window.
    for i in 0..BURST {
        writer_a.publish_raw(&frame('A', i)).expect("publish A");
        writer_b.publish_raw(&frame('B', i)).expect("publish B");
    }
    await_written(&mut status, &topic, PER_BURST, "cap-base burst 1");
    // Not a rendezvous: the frames above are already drained and staged (the
    // line before proves it). This is the AGE the window is about to measure,
    // and load can only make it longer.
    std::thread::sleep(AGE_OUT);

    // BURST 2 — inside the window.
    for i in BURST..(BURST * 2) {
        writer_a.publish_raw(&frame('A', i)).expect("publish A");
        writer_b.publish_raw(&frame('B', i)).expect("publish B");
    }
    await_written(&mut status, &topic, PER_BURST * 2, "cap-base burst 2");

    let capture = capture_now(&mgr, &dir, "cap-base");
    let summary = finish(handle, &shutdown, "cap-base");
    assert_eq!(
        summary.messages,
        PER_BURST * 2,
        "the RECORDING holds every frame — only the window evicts"
    );

    // The RECORDING: ordinals 0..12, the foil.
    let rec_msgs = read_bag(&out);
    let rec_channel = data_channel_id(&rec_msgs, &topic);
    let rec_labelled =
        assert_labels_pair_with_their_frames(&rec_msgs, rec_channel, &ids, "cap-base recording");
    assert_eq!(
        rec_labelled,
        (0..PER_BURST * 2).collect::<Vec<u64>>(),
        "the continuous bag numbers its own stream 0..{}",
        PER_BURST * 2
    );

    // The CAPTURE: a strict suffix of it, numbered from 0 again.
    let cap_msgs = read_bag(&capture);
    let cap_channel = data_channel_id(&cap_msgs, &topic);
    let cap_frames = cap_msgs
        .iter()
        .filter(|m| m.channel_id == cap_channel)
        .count() as u64;
    assert!(
        cap_frames > 0,
        "cap-base: the capture must hold SOMETHING, or this arm proves nothing"
    );
    assert!(
        cap_frames < PER_BURST * 2,
        "cap-base PRECONDITION: the capture must begin MID-recording for the ordinal base to be \
         discriminating, but it holds all {cap_frames} of the recording's {} frames — the \
         {AGE_OUT:?} age-out did not evict burst 1 past the {WINDOW:?} window",
        PER_BURST * 2
    );

    // …and it is the RECENT suffix, not merely "fewer frames":
    // a count check alone is satisfied by a capture holding burst ONE — the
    // frames the window was supposed to have EVICTED — which would be the exact
    // inverse of what the window is for and would still number 0..N. The frame
    // BODIES say which burst they came from, so the claim is checkable.
    let cap_bodies: Vec<String> = cap_msgs
        .iter()
        .filter(|m| m.channel_id == cap_channel)
        .map(|m| String::from_utf8_lossy(&m.data[WireHeader::SIZE..]).into_owned())
        .collect();
    let rec_bodies: Vec<String> = rec_msgs
        .iter()
        .filter(|m| m.channel_id == rec_channel)
        .map(|m| String::from_utf8_lossy(&m.data[WireHeader::SIZE..]).into_owned())
        .collect();
    assert_eq!(
        cap_bodies,
        rec_bodies[rec_bodies.len() - cap_bodies.len()..].to_vec(),
        "the capture must hold the RECENT SUFFIX of the recording, frame for frame — a window \
         that kept the evicted burst instead would still be 'fewer frames, numbered from 0'"
    );
    // The surviving frames are burst TWO's — seq >= BURST for each writer, by
    // construction of the publish loop above.
    for body in &cap_bodies {
        let seq: u32 = body
            .rsplit('-')
            .next()
            .and_then(|n| n.parse().ok())
            .unwrap_or_else(|| panic!("unrecognised frame body {body:?}"));
        assert!(
            seq >= BURST,
            "every surviving frame must come from burst TWO (seq >= {BURST}), got {body:?}"
        );
    }

    assert_every_label_names(&cap_msgs, &[cap_channel], "cap-base capture");
    let cap_labelled =
        assert_labels_pair_with_their_frames(&cap_msgs, cap_channel, &ids, "cap-base capture");
    assert_eq!(
        cap_labelled,
        (0..cap_frames).collect::<Vec<u64>>(),
        "THE ORDINAL BASE: a capture's labels name positions in ITS OWN stream, so they must be \
         exactly 0..{cap_frames} — continuing the recorder's count would start at {}",
        PER_BURST * 2 - cap_frames
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
    let _ = std::fs::remove_dir_all(&dir);
}

// ---------------------------------------------------------------------------
// ARM 13 — the capture's own unlabelled prefix gets its own catch-up
// ---------------------------------------------------------------------------

/// A capture of a topic whose plurality was OBSERVED carries ONE `CatchUpPrefix`
/// covering the run of frames it holds from BEFORE the arming frame, attributed
/// to the topic's FIRST writer.
///
/// Those frames are staged `Unlabeled` and their publisher id was never stored
/// (that is the point of the enum — an unlabelled frame owes no label, and
/// paying 16 bytes a frame on every single-writer topic is the cost it avoids),
/// so the capture cannot recover it from the frames. It recovers it from the
/// tap's FIRST ORIGIN, which is exact: labelling arms on the first frame that did
/// NOT carry it.
#[test]
#[serial_test::serial]
fn a_capture_attributes_its_unlabelled_prefix_with_one_catch_up_record() {
    /// Frames writer A publishes alone, before B ever appears.
    const PREFIX: u32 = 5;
    /// Frames after the arming frame.
    const TAIL: u32 = 2;
    const TOTAL: u64 = PREFIX as u64 + 1 + TAIL as u64;

    let mgr = make_manager(QUEUE);
    let topic = unique_topic("capcatch");
    let out = unique_out("cap_catchup");
    let ready = unique_ready_file("cap_catchup");
    let dir = capture_dir("cap-catchup");

    let mut writer_a = multi_publisher_with_provisioning(&mgr, &topic, QUEUE, 4096);

    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(
        mgr.clone(),
        record_cfg_with_window(
            out.clone(),
            // NOT declared: arming must come from OBSERVATION, which is what
            // creates an unlabelled prefix in the first place.
            vec![tap(&topic, false)],
            ready.clone(),
            &dir,
            Duration::from_secs(30),
        ),
        shutdown.clone(),
    );
    await_bagd_ready(&ready, "cap-catchup");
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");
    settle();

    for i in 0..PREFIX {
        writer_a.publish_raw(&frame('A', i)).expect("publish A");
    }
    // Rendezvoused so the arming frame's ordinal is unambiguous: A's whole run
    // is drained and staged before B exists.
    await_written(&mut status, &topic, PREFIX as u64, "cap-catchup prefix");

    let mut writer_b = multi_publisher_with_provisioning(&mgr, &topic, QUEUE, 4096);
    let ids: BTreeMap<char, u128> = [
        ('A', writer_a.publisher_id()),
        ('B', writer_b.publisher_id()),
    ]
    .into_iter()
    .collect();
    writer_b.publish_raw(&frame('B', 0)).expect("publish B");
    await_written(
        &mut status,
        &topic,
        PREFIX as u64 + 1,
        "cap-catchup arming frame",
    );
    for i in 0..TAIL {
        writer_a
            .publish_raw(&frame('A', PREFIX + i))
            .expect("publish A tail");
    }
    await_written(&mut status, &topic, TOTAL, "cap-catchup tail");

    let capture = capture_now(&mgr, &dir, "cap-catchup");
    let _ = finish(handle, &shutdown, "cap-catchup");

    let msgs = read_bag(&capture);
    let channel = data_channel_id(&msgs, &topic);
    let frames = msgs.iter().filter(|m| m.channel_id == channel).count() as u64;
    assert_eq!(
        frames, TOTAL,
        "the 30 s window held everything, so the capture carries all {TOTAL} frames"
    );
    assert_every_label_names(&msgs, &[channel], "cap-catchup");

    let records = labels_on(&msgs, channel);
    let catch_ups: Vec<&(usize, Label)> = records
        .iter()
        .filter(|(_, l)| l.kind == KIND_CATCH_UP_PREFIX)
        .collect();
    assert_eq!(
        catch_ups.len(),
        1,
        "a capture with an unlabelled prefix owes EXACTLY ONE catch-up, found {} in {records:?}",
        catch_ups.len()
    );
    let (catch_pos, catch) = *catch_ups[0];
    assert_eq!(
        catch.ordinal, PREFIX as u64,
        "the prefix covers the capture's OWN ordinals [0, {PREFIX}) — the frames IT holds from \
         before the arming frame, not the recording's count of them"
    );
    assert_eq!(
        catch.publisher_id, ids[&'A'],
        "the prefix belongs to the topic's FIRST origin, never to the writer whose arrival armed \
         labelling"
    );
    assert_eq!(
        msgs[catch_pos].data.as_slice(),
        expected_record_bytes(KIND_CATCH_UP_PREFIX, channel, PREFIX as u64, ids[&'A']).as_slice(),
        "the catch-up must be the 28 bytes it is, decoded {catch:?}"
    );

    // …and it comes BEFORE the first frame label, so a reader walking the file
    // has the prefix attributed before it meets a labelled frame.
    let first_label_pos = records
        .iter()
        .find(|(_, l)| l.kind == KIND_FRAME_LABEL)
        .map(|(p, _)| *p)
        .expect("the arming frame and its tail are labelled");
    assert!(
        catch_pos < first_label_pos,
        "the catch-up must precede the first FrameLabel (file positions {catch_pos} vs \
         {first_label_pos})"
    );

    // Every LABELLED frame is the arming frame or later, adjacent to its label,
    // and attributed to the writer its own body names.
    let labelled = assert_labels_pair_with_their_frames(&msgs, channel, &ids, "cap-catchup");
    assert_eq!(
        labelled,
        (PREFIX as u64..TOTAL).collect::<Vec<u64>>(),
        "labelling arms AT the arming frame and is one-way, so ordinals {PREFIX}..{TOTAL} carry \
         labels and the prefix carries none"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
    let _ = std::fs::remove_dir_all(&dir);
}

// ---------------------------------------------------------------------------
// ARM 14 — the single-writer control
// ---------------------------------------------------------------------------

/// A capture of an ordinary single-writer topic writes ZERO producer records.
///
/// The anti-tautology half of arms 11–13: without it, a capture path that
/// labelled EVERYTHING would pass all three, and every ordinary robot's black
/// box would carry one 28-byte record per frame for an attribution nothing is
/// ambiguous about.
#[test]
#[serial_test::serial]
fn a_capture_of_a_single_writer_topic_writes_no_producer_records() {
    const FRAMES: u32 = 6;

    let mgr = make_manager(QUEUE);
    let topic = unique_topic("capsingle");
    let out = unique_out("cap_single");
    let ready = unique_ready_file("cap_single");
    let dir = capture_dir("cap-single");

    let mut writer_a = multi_publisher_with_provisioning(&mgr, &topic, QUEUE, 4096);

    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(
        mgr.clone(),
        record_cfg_with_window(
            out.clone(),
            vec![tap(&topic, false)],
            ready.clone(),
            &dir,
            Duration::from_secs(30),
        ),
        shutdown.clone(),
    );
    await_bagd_ready(&ready, "cap-single");
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");
    settle();

    for i in 0..FRAMES {
        writer_a.publish_raw(&frame('A', i)).expect("publish A");
    }
    await_written(&mut status, &topic, FRAMES as u64, "cap-single");

    let capture = capture_now(&mgr, &dir, "cap-single");
    let _ = finish(handle, &shutdown, "cap-single");

    let msgs = read_bag(&capture);
    let channel = data_channel_id(&msgs, &topic);
    assert_eq!(
        msgs.iter().filter(|m| m.channel_id == channel).count(),
        FRAMES as usize,
        "the capture holds every frame"
    );
    assert!(
        labels_in_order(&msgs).is_empty(),
        "a single-writer capture must write ZERO producer records, found {:?}",
        labels_in_order(&msgs)
    );
    // …and the reserved CHANNEL is still registered, exactly as in a recording:
    // `BagWriter::create` registers it unconditionally, so its presence is not
    // evidence of attribution and its absence would break every reader.
    assert!(
        msgs.iter().any(|m| m.topic == FRAME_PRODUCERS_TOPIC)
            || read_bag_channels(&capture)
                .iter()
                .any(|t| t == FRAME_PRODUCERS_TOPIC),
        "the reserved channel is registered in EVERY bag, capture included"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
    let _ = std::fs::remove_dir_all(&dir);
}

/// Every channel TOPIC declared by a bag, read through `cerulion_bag`'s own
/// reader (the message stream cannot see a channel nothing was published on).
fn read_bag_channels(path: &Path) -> Vec<String> {
    let reader = BagReader::open(path).expect("open bag");
    reader
        .channels()
        .expect("channel table")
        .into_iter()
        .map(|c| c.topic)
        .collect()
}

// ---------------------------------------------------------------------------
// ARM 15 — two labelled topics in ONE capture keep their own ordinals
// ---------------------------------------------------------------------------

/// A capture holding TWO labelled topics — one DECLARED, one OBSERVED —
/// interleaves their records on the ONE reserved channel, and each topic's
/// ordinals, catch-up and attribution stay its own.
///
/// # Why this is not covered by arms 11-14
///
/// Every one of them captures a SINGLE topic, and `write_capture` keys FOUR
/// independent things by topic: the ordinal counter, the catch-up-owed latch,
/// the data channel it resolves, and the first-origin lookup. With one topic in
/// the bag, "this topic's value" and "the only value" are the same answer, so
/// each of these passes all four earlier arms:
///
/// * ONE shared ordinal counter — topic two's labels name frames it does not have;
/// * ONE shared catch-up latch — only the FIRST owing topic gets a `CatchUpPrefix`,
///   and a second observed-plurality topic's leading run is silently unattributed;
/// * the data channel resolved once from `batches[0]` — every record names topic
///   one's channel;
/// * `first_origins.values().next()` — topic two's catch-up is attributed to
///   topic one's writer.
///
/// This is the capture-side twin of ARM 7 and the same `[0]`-instead-of-`[idx]`
/// class. The MIX is what makes it total: a declared tap (labelled from ordinal
/// 0, owing nothing) beside an observed one (an unlabelled prefix, owing exactly
/// one catch-up) puts the two latches in DIFFERENT states at the same time, so a
/// shared one cannot be right for both. Frame counts differ, and the four
/// publishers are asserted distinct, so no cross-check can pass by coincidence.
#[test]
#[serial_test::serial]
fn a_capture_of_two_labelled_topics_keeps_each_channels_ordinals_to_itself() {
    /// Frames per writer on the DECLARED topic (so 6 frames, labelled from 0).
    const ONE_PER_WRITER: u32 = 3;
    const ONE_TOTAL: u64 = (ONE_PER_WRITER as u64) * 2;
    /// The OBSERVED topic's single-writer prefix — deliberately a DIFFERENT
    /// count, which is what makes a shared ordinal counter observable.
    const TWO_PREFIX: u32 = 4;
    /// Frames after its arming frame.
    const TWO_TAIL: u32 = 2;
    const TWO_TOTAL: u64 = TWO_PREFIX as u64 + 1 + TWO_TAIL as u64;

    let mgr = make_manager(QUEUE);
    let declared = unique_topic("captwo-decl");
    let observed = unique_topic("captwo-obs");
    let out = unique_out("cap_two");
    let ready = unique_ready_file("cap_two");
    let dir = capture_dir("cap-two");

    let mut decl_a = multi_publisher_with_provisioning(&mgr, &declared, QUEUE, 4096);
    let mut decl_b = multi_publisher_with_provisioning(&mgr, &declared, QUEUE, 4096);
    let mut obs_a = multi_publisher_with_provisioning(&mgr, &observed, QUEUE, 4096);
    let ids_decl: BTreeMap<char, u128> =
        [('A', decl_a.publisher_id()), ('B', decl_b.publisher_id())]
            .into_iter()
            .collect();

    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(
        mgr.clone(),
        record_cfg_with_window(
            out.clone(),
            vec![tap(&declared, true), tap(&observed, false)],
            ready.clone(),
            &dir,
            Duration::from_secs(30),
        ),
        shutdown.clone(),
    );
    await_bagd_ready(&ready, "cap-two");
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");
    settle();

    // The DECLARED topic: two interleaved writers, labelled from its ordinal 0.
    for i in 0..ONE_PER_WRITER {
        decl_a.publish_raw(&frame('A', i)).expect("publish decl A");
        decl_b.publish_raw(&frame('B', i)).expect("publish decl B");
    }
    // The OBSERVED topic's single-writer PREFIX, rendezvoused so its arming
    // frame's ordinal is unambiguous.
    for i in 0..TWO_PREFIX {
        obs_a.publish_raw(&frame('A', i)).expect("publish obs A");
    }
    await_written(&mut status, &declared, ONE_TOTAL, "cap-two declared");
    await_written(
        &mut status,
        &observed,
        TWO_PREFIX as u64,
        "cap-two observed prefix",
    );

    // …then its SECOND writer appears and arms labelling.
    let mut obs_b = multi_publisher_with_provisioning(&mgr, &observed, QUEUE, 4096);
    let ids_obs: BTreeMap<char, u128> = [('A', obs_a.publisher_id()), ('B', obs_b.publisher_id())]
        .into_iter()
        .collect();
    assert_eq!(
        [ids_decl[&'A'], ids_decl[&'B'], ids_obs[&'A'], ids_obs[&'B'],]
            .iter()
            .collect::<std::collections::BTreeSet<_>>()
            .len(),
        4,
        "four live publishers must carry four distinct iceoryx2 ids, or the cross-checks below \
         cannot tell one topic's writers from the other's"
    );
    obs_b.publish_raw(&frame('B', 0)).expect("publish obs B");
    await_written(
        &mut status,
        &observed,
        TWO_PREFIX as u64 + 1,
        "cap-two observed arming",
    );
    for i in 0..TWO_TAIL {
        obs_a
            .publish_raw(&frame('A', TWO_PREFIX + i))
            .expect("publish obs tail");
    }
    await_written(&mut status, &observed, TWO_TOTAL, "cap-two observed tail");

    let capture = capture_now(&mgr, &dir, "cap-two");
    let _ = finish(handle, &shutdown, "cap-two");

    let msgs = read_bag(&capture);
    let chan_decl = data_channel_id(&msgs, &declared);
    let chan_obs = data_channel_id(&msgs, &observed);
    assert_ne!(chan_decl, chan_obs, "two topics must map to two channels");

    // THE COMPLEMENT: nothing was written naming a channel neither tap owns.
    // (It is NOT where a "resolve the channel once from `batches[0]`" slip lands
    // — this accepts EITHER channel, so that slip passes here and is killed
    // below, by the observed topic's empty label set and by the adjacency
    // helper. Stated because this is the assertion a future reader would delete
    // first, believing it redundant.)
    assert_every_label_names(&msgs, &[chan_decl, chan_obs], "cap-two");

    // The DECLARED topic: ordinals 0..ITS OWN count, and NO catch-up.
    let labelled_decl =
        assert_labels_pair_with_their_frames(&msgs, chan_decl, &ids_decl, "cap-two declared");
    assert_eq!(
        labelled_decl,
        (0..ONE_TOTAL).collect::<Vec<u64>>(),
        "the declared topic's ordinals must be exactly 0..{ONE_TOTAL} over its OWN frames"
    );
    assert!(
        labels_on(&msgs, chan_decl)
            .iter()
            .all(|(_, l)| l.kind == KIND_FRAME_LABEL),
        "a DECLARED tap has no unlabelled prefix, so it owes no catch-up: {:?}",
        labels_on(&msgs, chan_decl)
    );

    // The OBSERVED topic: its OWN ordinals, and its OWN catch-up — the half a
    // single shared latch or a `values().next()` lookup gets wrong.
    let labelled_obs =
        assert_labels_pair_with_their_frames(&msgs, chan_obs, &ids_obs, "cap-two observed");
    assert_eq!(
        labelled_obs,
        (TWO_PREFIX as u64..TWO_TOTAL).collect::<Vec<u64>>(),
        "the observed topic's labels must run {TWO_PREFIX}..{TWO_TOTAL} in ITS OWN numbering — a \
         shared counter would continue from the declared topic's {ONE_TOTAL}"
    );
    let obs_catch_ups: Vec<(usize, Label)> = labels_on(&msgs, chan_obs)
        .into_iter()
        .filter(|(_, l)| l.kind == KIND_CATCH_UP_PREFIX)
        .collect();
    assert_eq!(
        obs_catch_ups.len(),
        1,
        "the observed topic owes EXACTLY ONE catch-up even with a declared sibling in the same \
         capture — a SHARED catch-up latch would have been spent by the declared tap"
    );
    let (_, obs_catch) = obs_catch_ups[0];
    assert_eq!(
        obs_catch.ordinal, TWO_PREFIX as u64,
        "the catch-up covers the OBSERVED topic's own leading run"
    );
    assert_eq!(
        obs_catch.publisher_id, ids_obs[&'A'],
        "…attributed to the OBSERVED topic's first writer, never to the declared sibling's"
    );
    assert!(
        !labels_on(&msgs, chan_decl)
            .iter()
            .any(|(_, l)| l.kind == KIND_CATCH_UP_PREFIX),
        "…and the declared topic must not have acquired one"
    );

    // ONE sequence for the reserved channel across BOTH topics, gap-free from 0
    // — `write_capture` claims this ("exactly as the continuous writer keeps
    // it") and no capture arm read `sequence` at all before this. Arm 15 is the
    // only place it is checkable: with one topic, a per-topic counter is
    // indistinguishable from a shared one. A per-topic counter duplicates values
    // here; a frozen counter makes them all 0.
    let seqs: Vec<u32> = labels_in_order(&msgs)
        .iter()
        .map(|(pos, _)| msgs[*pos].sequence)
        .collect();
    assert_eq!(
        seqs,
        (0..seqs.len() as u32).collect::<Vec<u32>>(),
        "the reserved channel carries ONE gap-free sequence across every topic's records"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
    let _ = std::fs::remove_dir_all(&dir);
}

// ---------------------------------------------------------------------------
// ARM 16 — a headerless frame in a capture is labelled and consumes its ordinal
// ---------------------------------------------------------------------------

/// A frame whose wire header will not parse is still WRITTEN, still LABELLED,
/// and still consumes its ordinal — in a CAPTURE, not just in a recording.
///
/// # Why this needs its own arm
///
/// `headerless_frame_on_an_armed_topic_is_labeled_and_consumes_its_ordinal`
/// (ARM 6) covers the CONTINUOUS writer. `write_capture` re-implements those
/// decisions independently, and TWO of them are pinned on the capture path by
/// nothing else: the `(0, 0)` fabrication with its label stamped to match, and
/// the label mint keyed on the SAMPLE's origin rather than on the header. (The
/// third — the ordinal advance sitting OUTSIDE the label guard — IS already
/// pinned, by arms 13 and 15, whose unlabelled prefixes collapse if it moves
/// inside. It is asserted here too because this arm is where a HEADERLESS frame
/// exercises it.)
///
/// A frame's producer is sample METADATA: iceoryx2 reports it whether or not the
/// bytes parse. So a headerless frame on an armed topic is exactly as
/// attributable as any other, and skipping its label would put an unlabelled
/// frame in the middle of a labelled run — which a reader can only read as an
/// unattributed prefix. Skipping its ORDINAL would be worse: every later label
/// in that channel would name the wrong frame.
///
/// The oracle is written by hand rather than through
/// `assert_labels_pair_with_their_frames`, because that helper reads the writer
/// tag out of the frame BODY and a garbage frame has no parseable body.
#[test]
#[serial_test::serial]
fn a_headerless_frame_in_a_capture_is_labelled_and_consumes_its_ordinal() {
    /// A payload too short to hold a wire header — the same shape ARM 6 uses.
    const GARBAGE: [u8; 10] = [0xBA; 10];

    let mgr = make_manager(QUEUE);
    let topic = unique_topic("caphdrless");
    let out = unique_out("cap_headerless");
    let ready = unique_ready_file("cap_headerless");
    let dir = capture_dir("cap-headerless");

    let mut writer_a = multi_publisher_with_provisioning(&mgr, &topic, QUEUE, 4096);
    let id_a = writer_a.publisher_id();

    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(
        mgr.clone(),
        record_cfg_with_window(
            out.clone(),
            // DECLARED, so every frame is labelled from ordinal 0 and the
            // headerless one is squarely inside a labelled run.
            vec![tap(&topic, true)],
            ready.clone(),
            &dir,
            Duration::from_secs(30),
        ),
        shutdown.clone(),
    );
    await_bagd_ready(&ready, "cap-headerless");
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");
    settle();

    // ordinal 0: a good frame. ordinal 1: GARBAGE. ordinal 2: a good frame.
    writer_a.publish_raw(&frame('A', 0)).expect("publish 0");
    writer_a.publish_raw(&GARBAGE).expect("publish garbage");
    writer_a.publish_raw(&frame('A', 2)).expect("publish 2");
    await_written(&mut status, &topic, 3, "cap-headerless");

    let capture = capture_now(&mgr, &dir, "cap-headerless");
    let _ = finish(handle, &shutdown, "cap-headerless");

    let msgs = read_bag(&capture);
    let channel = data_channel_id(&msgs, &topic);
    let frames: Vec<&Msg> = msgs.iter().filter(|m| m.channel_id == channel).collect();
    assert_eq!(frames.len(), 3, "the capture holds all three frames");
    assert_eq!(
        frames[1].data.as_slice(),
        &GARBAGE[..],
        "the garbage frame is preserved WHOLE — a black box that dropped the one malformed frame \
         around an incident would be discarding the evidence"
    );
    assert_eq!(
        frames[1].log_time, 0,
        "…with a FABRICATED stamp, since nothing trustworthy was on its wire"
    );

    // Every ordinal 0,1,2 is labelled — the headerless one included — and each
    // label sits immediately before its own frame.
    let labels = labels_on(&msgs, channel);
    assert_eq!(
        labels.iter().map(|(_, l)| l.ordinal).collect::<Vec<u64>>(),
        vec![0, 1, 2],
        "a headerless frame CONSUMES its ordinal and still earns a label: {labels:?}"
    );
    assert!(
        labels.iter().all(|(_, l)| l.publisher_id == id_a),
        "…attributed to the publisher that committed it, which is sample metadata and is known \
         even when the frame's own bytes are not: {labels:?}"
    );
    let ordinals = ordinals_by_position(&msgs, channel);
    for (pos, label) in &labels {
        assert_eq!(
            ordinals.get(&(pos + 1)).copied(),
            Some(label.ordinal),
            "the label at file position {pos} must sit immediately before ordinal {}",
            label.ordinal
        );
    }
    // The headerless frame's own label carries the SAME fabricated stamp, so the
    // two time-align in the bag rather than the label floating at a real time.
    let (headerless_label_pos, _) = labels
        .iter()
        .find(|(_, l)| l.ordinal == 1)
        .expect("ordinal 1 is labelled, asserted above");
    assert_eq!(
        msgs[*headerless_label_pos].log_time, 0,
        "the headerless frame's label is stamped with the frame's own fabricated 0"
    );
    assert_eq!(
        msgs[*headerless_label_pos].data.as_slice(),
        expected_record_bytes(KIND_FRAME_LABEL, channel, 1, id_a).as_slice(),
        "…and is the 28 bytes a FrameLabel for ordinal 1 is"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
    let _ = std::fs::remove_dir_all(&dir);
}
