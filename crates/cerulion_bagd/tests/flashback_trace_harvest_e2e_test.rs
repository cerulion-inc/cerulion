// SPDX-License-Identifier: AGPL-3.0-only
//! End to end: a WINDOW-ONLY recorder DRAINS this run's
//! scheduler-trace ring, so its captures carry a trace and can be re-executed.
//!
//! Real `TraceRingOwner` and `StateRingOwner` producers, a real window-only
//! recorder (no continuous bag anywhere in this file), a real trigger channel,
//! the finished MCAP read back. Isolated per-test SHM roots + unique ring tags,
//! so the file is parallel-safe.
//!
//! # The twin of `flashback_trace_e2e_test.rs`, one mode over
//!
//! That file's module docs say why every arm in it runs `--record`: the trace
//! ring's one consumer was the WRITER THREAD, and a window-only recorder was
//! handed no `--ring` at all. This file is the other half of that sentence.
//! The window-only recorder now has its OWN reader (a dedicated drain thread,
//! `crate::trace_drain`), so `window_only: true` is the mode under test
//! here rather than a detail, and every arm would be green against the shipped
//! `--record` path while the window-only one read nothing.
//!
//! # What these arms prove that the unit tests cannot
//!
//! `trace_drain`'s own oracles are pure state-machine arms: the hole
//! bookkeeping, the merge rule, the attach verdict. Every one of them stays
//! green if nothing SPAWNS the thread, if the rings are never moved onto it, if
//! its records never reach the retention, or if the numbers it publishes never
//! reach a manifest. Those are the inert-shipping shapes, and only a run that
//! pushes real records onto a real POSIX-SHM ring and then READS THE BAG can see
//! them.
//!
//! # Injected constants, and only these
//!
//! The post window is injected at ~300 ms (the shipped 15 s would make this a
//! minute-long suite for a property that has nothing to do with the number). The
//! lap arms inject a TINY ring capacity, because they are ABOUT a producer
//! outrunning a reader and the shipped 2^20 records is 95 seconds of a 1 kHz
//! graph. Every other constant is the shipped one, and every deadline is a
//! generous liveness ceiling in seconds rather than a wall stated in units of the
//! thing under test.

#![cfg(unix)]

mod common;

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_bagd::{run_bagd, BagdConfig, BagdSummary, FlashbackSettings, TapSpec};
use cerulion_core::flashback::channel::{FlashbackOutcome, FlashbackRequester};
use cerulion_core::flashback::retention::RetentionCaps;
use cerulion_core::flashback::switch::TriggerPosture;
use cerulion_core::flashback::trigger::{CaptureRequest, TriggerPolicy};
use cerulion_core::state_ring::{
    encode_record, StateRecordHeader, StateRingOwner, StateRingProducer, RECORD_KIND_FINAL,
    STATE_RECORD_PAYLOAD, STATE_RECORD_SIZE,
};
use cerulion_core::trace_ring::{
    TraceRingConsumer, TraceRingOwner, TraceRingProducer, TraceRingRecord, RECORD_TYPE_FIRE,
    RECORD_TYPE_STEP_BOUNDARY, TRACE_RECORD_SIZE,
};
use cerulion_core::transport::subscriber::DataOnlySubscriber;
use cerulion_core::TransportManager;

use common::{
    await_bagd_ready, await_condition, build_frame, join_bagd, make_manager, publisher, unique_out,
    unique_ready_file, unique_ring_tag, unique_topic,
};

/// Shipped everywhere except the post window — see the module docs.
const POST_WINDOW_MS: u64 = 300;
/// Generous liveness ceilings. Load can only delay these, never invert them.
const DEADLINE: Duration = Duration::from_secs(30);
/// An arbitrary, stable schema hash for the hand-built data frames.
const HASH: u64 = 0x0C1E_0C1E_0C1E_1002;
/// The run every state record in this file carries.
const RUN: u64 = 0x0000_C1E5_0000_1002;
/// Far above what the healthy arms push, so no arm is measuring a lap it did not
/// mean.
const RING_RECORDS: u32 = 256;
/// The lap arms' ring — small enough that a burst this test can push in a tight
/// loop really does outrun a reader waking every few milliseconds.
const TINY_RING_RECORDS: u32 = 16;
/// The step the healthy arm's embedded checkpoint is taken at.
const ANCHOR_STEP: u64 = 40;
/// The nodes the rings declare, in manifest order.
const NODES: [&str; 2] = ["ticker", "relay"];

fn temp_dir(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "harvest-{tag}-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos())
            .unwrap_or(0)
    ));
    std::fs::create_dir_all(&dir).expect("temp dir");
    dir
}

/// WINDOW-ONLY settings. `window_only: true` is THE mode under test — see the
/// module docs — and every arm asserts it was real by requiring the recorder to
/// have written no continuous bag.
fn settings(dir: &Path) -> FlashbackSettings {
    FlashbackSettings {
        // Long, deliberately: no arm in this file is about EVICTION. The lap arms
        // get their two verdicts from the capture's own TRIM (which keeps records
        // past the anchor step) rather than from records ageing out, so nothing
        // here depends on a wall clock beyond the post window.
        window_span: Duration::from_secs(30),
        window_max_bytes: 64 * 1024 * 1024,
        anchor_max_bytes: 64 * 1024 * 1024,
        anchor_cap_basis: cerulion_core::flashback::CapBasis::Env,
        trace_max_bytes: 64 * 1024 * 1024,
        dir: dir.to_path_buf(),
        label: "trace-harvest".into(),
        caps: RetentionCaps::default(),
        policy: TriggerPolicy {
            post_window_ns: POST_WINDOW_MS * 1_000_000,
            ..Default::default()
        },
        posture: TriggerPosture::default(),
        exclude_topics: cerulion_core::flashback::ExcludeTopics::default(),
        window_only: true,
        tap_budget_bytes: cerulion_core::flashback::DEFAULT_FLASHBACK_TAP_BUDGET_MB * 1024 * 1024,
    }
}

// ===========================================================================
// Hand-built records — the ORACLE side
// ===========================================================================

fn boundary(step: u64) -> TraceRingRecord {
    TraceRingRecord {
        step,
        // A distinct, recognisable gating-clock value per step, so a wrong step's
        // target cannot pass for the right one.
        fire_time_ns: 1_000_000 + step * 4_000_000,
        duration_ns: 0,
        node_idx: 0,
        global_level: 0,
        record_type: RECORD_TYPE_STEP_BOUNDARY,
        reserved: 0,
    }
}

fn fire(step: u64, node_idx: u32) -> TraceRingRecord {
    TraceRingRecord {
        step,
        fire_time_ns: 1_000_000 + step * 4_000_000,
        duration_ns: 700 + u64::from(node_idx),
        node_idx,
        global_level: node_idx,
        record_type: RECORD_TYPE_FIRE,
        reserved: 0,
    }
}

/// One step's records, in the order the scheduler pushes them: the boundary at
/// `begin_step`, then that step's fires.
fn step_records(step: u64) -> Vec<TraceRingRecord> {
    vec![boundary(step), fire(step, 0), fire(step, 1)]
}

/// A trace spanning `ANCHOR_STEP` and three steps past it.
fn trace_across_the_anchor() -> Vec<TraceRingRecord> {
    ((ANCHOR_STEP - 2)..=(ANCHOR_STEP + 3))
        .flat_map(step_records)
        .collect()
}

/// The records at or after `boundary(ANCHOR_STEP + 1)` — what the capture MUST
/// carry, stated independently of the code that trims.
fn expected_kept() -> Vec<TraceRingRecord> {
    trace_across_the_anchor()
        .into_iter()
        .filter(|r| r.step > ANCHOR_STEP)
        .collect()
}

/// One node's COMPLETE anchor at `step`, as its single final record.
fn anchor_record(step: u64, node_idx: u32, fill: u8) -> Vec<u8> {
    let payload = vec![fill; 24];
    encode_record(
        &StateRecordHeader {
            run_id: RUN,
            step,
            node_idx,
            part: 0,
            kind: RECORD_KIND_FINAL,
            len: payload.len() as u32,
        },
        &payload,
    )
    .to_vec()
}

// The anchor fixture writes 24 bytes into the record payload — hold that at
// compile time (the repo's const-assert idiom).
const _: () = assert!(24 <= STATE_RECORD_PAYLOAD);

/// A COMPLETE anchor at `step` for both declared nodes.
fn full_anchor(step: u64) -> Vec<Vec<u8>> {
    vec![anchor_record(step, 0, 0xA1), anchor_record(step, 1, 0xB2)]
}

/// A graph that PARSES **and VALIDATES** — what `run_replay` actually requires
/// (its gate 4). A single producer node, so there is no input reference to
/// resolve.
const GRAPH_YAML: &[u8] = b"name: harvest-demo\nnodes:\n  - id: probe\n    type: probe_node\n    outputs:\n      - name: out\n        schema: geometry_msgs/Vector3\n";

// ===========================================================================
// Reading the finished bag
// ===========================================================================

fn captures(dir: &Path) -> Vec<PathBuf> {
    let mut out: Vec<PathBuf> = std::fs::read_dir(dir)
        .expect("read the capture dir")
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().and_then(|e| e.to_str()) == Some("mcap"))
        .collect();
    out.sort();
    out
}

/// Every `__cerulion/scheduler_trace` message in the bag, in bag order.
fn trace_payloads(bag: &Path) -> Vec<Vec<u8>> {
    let reader = cerulion_bag::BagReader::open(bag).expect("open the capture");
    let (msgs, _) = reader.recover_messages().expect("recover");
    msgs.into_iter()
        .filter(|m| m.topic == cerulion_bag::SCHEDULER_TRACE_TOPIC)
        .map(|m| m.data)
        .collect()
}

/// Those payloads DECODED, so an arm can talk about steps rather than bytes.
fn trace_records(bag: &Path) -> Vec<TraceRingRecord> {
    trace_payloads(bag)
        .iter()
        .map(|p| {
            let record: [u8; TRACE_RECORD_SIZE as usize] = p
                .as_slice()
                .try_into()
                .expect("a recorded trace message is exactly one record wide");
            TraceRingRecord::from_bytes(&record)
        })
        .collect()
}

fn flashback_manifest(bag: &Path) -> serde_json::Value {
    let reader = cerulion_bag::BagReader::open(bag).expect("open the capture");
    let att = reader
        .attachment("__cerulion/flashback.json")
        .expect("read the attachment")
        .expect("the capture manifest is present");
    serde_json::from_slice(&att.data).expect("the manifest must be valid JSON")
}

/// Scheduler-trace records the manifest says this capture CARRIES, read out of
/// the handoff's `trace` verdict — the one place the manifest states it.
fn carried_records(m: &serde_json::Value) -> u64 {
    let verdict = m["handoff"]["trace"].as_str().expect("a trace verdict");
    verdict
        .strip_prefix("carried: ")
        .and_then(|rest| rest.split(' ').next())
        .and_then(|n| n.parse().ok())
        .unwrap_or(0)
}

/// The capture's resim VERDICT.
///
/// It lives under `anchor`, not at the top level, because the manifest states it
/// beside the checkpoint the resume would start from — one object answering "can
/// this be re-executed, and from where".
fn resimmable(m: &serde_json::Value) -> Option<bool> {
    m["anchor"]["resimmable"].as_bool()
}

/// The sentence a REFUSED capture carries.
fn refusal(m: &serde_json::Value) -> String {
    m["anchor"]["resimmable_reason"]
        .as_str()
        .unwrap_or_else(|| panic!("a refused capture states its reason: {m}"))
        .to_string()
}

/// The two step numbers a `TraceLapped` refusal names, parsed out of its own
/// sentence.
///
/// Parsed rather than matched against an expected literal because WHICH step the
/// drain last admitted before a lap is a race by construction — that is what a
/// lap IS — so an arm that pinned one number would be pinning the scheduler, not
/// the code. What the arms assert on these is the SHAPE: both real, ordered, and
/// the near one at or past what the arm actually pushed.
fn hole_edges(reason: &str) -> (u64, u64) {
    let rest = reason
        .split("between step ")
        .nth(1)
        .unwrap_or_else(|| panic!("the lap refusal names its near edge: {reason}"));
    let mut parts = rest.split(" and step ");
    let near: u64 = parts
        .next()
        .and_then(|s| s.trim().split(' ').next())
        .and_then(|s| s.parse().ok())
        .unwrap_or_else(|| panic!("the near edge is a number: {reason}"));
    let far: u64 = parts
        .next()
        .and_then(|s| s.trim().split(',').next())
        .and_then(|s| s.trim().split(' ').next())
        .and_then(|s| s.parse().ok())
        .unwrap_or_else(|| panic!("the far edge is a number: {reason}"));
    (near, far)
}

fn handoff_str(m: &serde_json::Value, key: &str) -> String {
    m["handoff"][key]
        .as_str()
        .unwrap_or_else(|| panic!("handoff.{key} must be a string: {m}"))
        .to_string()
}

fn handoff_u64(m: &serde_json::Value, key: &str) -> u64 {
    m["handoff"][key]
        .as_u64()
        .unwrap_or_else(|| panic!("handoff.{key} must be a number: {m}"))
}

// ===========================================================================
// The live status plane — the rendezvous
// ===========================================================================

/// The drain's own numbers, off `/bagd/status`.
///
/// This is the ONLY in-process view of a trace reader's progress that is not
/// itself a second reader, and that is a property of the ring rather than an
/// inconvenience: a trace ring is `FailLoud`, so a consumer's cursor is LOCAL and
/// `free_records()` answers `None` by construction (`shm_ring`). The state ring's
/// arms rendezvous on the producer's view of its consumer cursor; there is no
/// such view here, so the arms wait on the recorder's own published counters
/// instead — the same numbers an operator reads.
#[derive(Debug, Clone, Default)]
struct DrainStatus {
    ring_records: u64,
    laps: u64,
    attach: Option<String>,
    trace_records: u64,
}

fn drain_status(sub: &mut DataOnlySubscriber) -> Vec<DrainStatus> {
    // Chunked at the DEFAULT borrow budget: an owned sample reads zero-copy out
    // of SHM and holds a borrow until dropped, so a larger `max` is a
    // guaranteed `ExceedsMaxBorrows` on a loaded runner.
    const STATUS_TAP_BORROW_BUDGET: usize = 2;
    let mut out = Vec::new();
    loop {
        let mut batch = Vec::new();
        let drained = sub
            .drain_owned(STATUS_TAP_BORROW_BUDGET, &mut batch)
            .expect("drain status");
        for frame in batch {
            let payload = frame.payload();
            if payload.len() <= cerulion_core::wire::WireHeader::SIZE {
                continue;
            }
            let body = &payload[cerulion_core::wire::WireHeader::SIZE..];
            let Ok(v) = serde_json::from_slice::<serde_json::Value>(body) else {
                continue;
            };
            let Some(fb) = v.get("flashback") else {
                continue;
            };
            out.push(DrainStatus {
                ring_records: fb["trace_ring_records"].as_u64().unwrap_or(0),
                laps: fb["trace_laps"].as_u64().unwrap_or(0),
                attach: fb["trace_attach"].as_str().map(|s| s.to_string()),
                trace_records: fb["trace_records"].as_u64().unwrap_or(0),
            });
        }
        if drained == 0 {
            break;
        }
    }
    out
}

/// The LATEST status this poll saw, folded so an arm never has to reason about
/// which frame in a batch it read.
///
/// A running maximum rather than "the last frame", because the counters are
/// monotone and the status publisher is best-effort: a dropped frame must not be
/// able to make a satisfied condition unsatisfied again.
struct StatusWatch {
    sub: DataOnlySubscriber,
    seen: DrainStatus,
}

impl StatusWatch {
    fn open(mgr: &TransportManager) -> Self {
        Self {
            sub: mgr
                .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
                .expect("status subscriber"),
            seen: DrainStatus::default(),
        }
    }

    fn poll(&mut self) -> DrainStatus {
        for s in drain_status(&mut self.sub) {
            self.seen.ring_records = self.seen.ring_records.max(s.ring_records);
            self.seen.laps = self.seen.laps.max(s.laps);
            self.seen.trace_records = self.seen.trace_records.max(s.trace_records);
            if s.attach.is_some() {
                self.seen.attach = s.attach;
            }
        }
        self.seen.clone()
    }

    /// Wait until the drain has admitted at least `n` records.
    fn await_records(&mut self, n: u64, what: &str) {
        assert!(
            await_condition(DEADLINE, || self.poll().ring_records >= n),
            "{what}: the drain must admit at least {n} record(s); last saw {:?}",
            self.seen
        );
    }
}

// ===========================================================================
// The harness
// ===========================================================================

struct Harness {
    dir: PathBuf,
    ready: PathBuf,
}

impl Harness {
    fn cleanup(&self) {
        std::fs::remove_dir_all(&self.dir).ok();
        std::fs::remove_file(&self.ready).ok();
    }
}

fn push_trace(producer: &mut TraceRingProducer, records: &[TraceRingRecord]) {
    for r in records {
        producer.push(r);
    }
}

fn push_state(producer: &mut StateRingProducer, records: &[Vec<u8>]) {
    for record in records {
        let mut buf = [0u8; STATE_RECORD_SIZE as usize];
        buf.copy_from_slice(record);
        producer.push_record(&buf);
    }
}

/// Everything an arm needs to drive one window-only recorder.
struct Run {
    mgr: Arc<TransportManager>,
    harness: Harness,
    shutdown: Arc<AtomicBool>,
    join: std::thread::JoinHandle<Result<BagdSummary, cerulion_bagd::BagdError>>,
    status: StatusWatch,
    trace: TraceRingProducer,
    state: StateRingProducer,
    #[allow(dead_code)]
    trace_owner: TraceRingOwner,
    #[allow(dead_code)]
    state_owner: StateRingOwner,
    trace_ring_name: String,
    publisher: cerulion_core::transport::publisher::CerulionPublisher,
    seq: u32,
    /// Carried for `join_bagd`, which names the recorder in its timeout panic.
    tag: String,
}

impl Run {
    /// Stand up a window-only recorder over a real trace ring and a real state
    /// ring.
    ///
    /// `pre_push` runs BEFORE the recorder is spawned, which is how the
    /// late-attach arm gets a ring that has already lapped by the time anybody
    /// opens it.
    fn start(tag: &str, ring_records: u32, pre_push: impl FnOnce(&mut TraceRingProducer)) -> Self {
        Self::start_with_rank(tag, ring_records, 0, pre_push)
    }

    /// [`start`](Self::start) on the ATTACH path, with `extra_rings` DECLARED
    /// beyond the one this harness really creates.
    ///
    /// The partial-trace arm needs a recorder that was handed a
    /// ring it cannot read. `attached_mid_run` is what makes that a DEGRADE
    /// rather than a fatal setup error (a `graph run --record` recorder created
    /// its own rings moments earlier, so a failure there is an internal
    /// invariant violation and stays fatal), and it is the mode a
    /// vanished ring actually arises in: the owning run may be exiting, and a
    /// ring's SHM name is unlinked when its owner drops.
    ///
    /// A name no ring was ever created under is a faithful fixture for that: the
    /// open fails for the reason a vanished ring's does.
    fn start_attached(tag: &str, extra_rings: &[String]) -> Self {
        Self::start_inner(tag, RING_RECORDS, 0, true, extra_rings, |_| {})
    }

    /// [`start`](Self::start) with an explicit ring RANK: the rank-stamp arm needs a
    /// nonzero one, and rank 0 is the value an unstamped drain is invisible at.
    fn start_with_rank(
        tag: &str,
        ring_records: u32,
        rank: u32,
        pre_push: impl FnOnce(&mut TraceRingProducer),
    ) -> Self {
        Self::start_inner(tag, ring_records, rank, false, &[], pre_push)
    }

    fn start_inner(
        tag: &str,
        ring_records: u32,
        rank: u32,
        attached_mid_run: bool,
        extra_rings: &[String],
        pre_push: impl FnOnce(&mut TraceRingProducer),
    ) -> Self {
        let mgr = make_manager(64);
        let topic = unique_topic("/fbth/probe");
        let dir = temp_dir(tag);
        let ready = unique_ready_file(&format!("fbth_{tag}"));

        let mut cfg = BagdConfig::new(
            unique_out(&format!("fbth_{tag}")),
            vec![TapSpec::attach(&topic)],
        );
        cfg.flush_interval = Duration::from_millis(20);
        cfg.schema_wait = Duration::from_millis(200);
        // The arms rendezvous on the recorder's own published counters — see
        // `DrainStatus` for why there is no producer-side view to wait on.
        cfg.status_period = Some(Duration::from_millis(20));
        cfg.discover_live = false;
        cfg.ready_file = Some(ready.clone());
        cfg.attachments = vec![("graph.yaml".to_string(), GRAPH_YAML.to_vec())];
        cfg.flashback = Some(settings(&dir));

        // The tap is OPEN-ONLY, so the producer must exist before
        // `Recorder::setup`.
        let publisher = publisher(&mgr, &topic, 256);

        let state_tag = unique_ring_tag(&format!("st{tag}"));
        let mut state_owner = StateRingOwner::create(&state_tag, RING_RECORDS, 0, RUN, &NODES)
            .expect("create the state ring");
        cfg.state_rings = vec![state_owner.name().to_string()];
        // MINTED ONCE and held: a ring is SPSC on the producer side, so a second
        // `producer()` answers `None`.
        let state = state_owner.producer().expect("the single state producer");

        let trace_tag = unique_ring_tag(&format!("tr{tag}"));
        let mut trace_owner = TraceRingOwner::create(&trace_tag, ring_records, rank, &NODES)
            .expect("create the trace ring");
        let trace_ring_name = trace_owner.name().to_string();
        cfg.rings = std::iter::once(trace_ring_name.clone())
            .chain(extra_rings.iter().cloned())
            .collect();
        cfg.attached_mid_run = attached_mid_run;
        let mut trace = trace_owner.producer().expect("the single trace producer");

        pre_push(&mut trace);

        let shutdown = Arc::new(AtomicBool::new(false));
        let rec_mgr = Arc::clone(&mgr);
        let rec_shutdown = Arc::clone(&shutdown);
        let join = std::thread::spawn(move || run_bagd(rec_mgr, cfg, rec_shutdown));
        await_bagd_ready(&ready, tag);

        let status = StatusWatch::open(&mgr);
        Self {
            mgr,
            harness: Harness { dir, ready },
            shutdown,
            join,
            status,
            trace,
            state,
            trace_owner,
            state_owner,
            trace_ring_name,
            publisher,
            seq: 0,
            tag: tag.to_string(),
        }
    }

    /// Publish a few data frames, so the window holds something and the capture
    /// is an ordinary one rather than a frames-empty corner.
    fn publish_frames(&mut self, n: u32) {
        for _ in 0..n {
            self.publisher
                .publish_raw(&build_frame(
                    HASH,
                    self.seq,
                    1_000 + u64::from(self.seq),
                    b"pre",
                ))
                .expect("publish");
            self.seq += 1;
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    /// Trigger a manual capture and wait for it to FINISH.
    fn capture(&self, why: &'static str) {
        let requester = FlashbackRequester::open_on_manager(&self.mgr).expect("requester");
        let request_id = requester
            .request(&CaptureRequest::manual(why))
            .expect("request");
        let mut finished = false;
        assert!(
            await_condition(DEADLINE, || {
                for frame in requester.drain_outcomes(request_id) {
                    if matches!(frame.outcome, FlashbackOutcome::Finished { .. }) {
                        finished = true;
                    }
                }
                finished
            }),
            "the recorder must report the '{why}' capture FINISHED"
        );
    }

    /// Shut the recorder down and return its summary plus the captures it wrote.
    fn finish(self) -> (BagdSummary, Vec<PathBuf>, Harness) {
        // SIGNAL, then join. `join_bagd` WAITS and reports the flag in its
        // timeout message; it does not set it, so dropping this line would leave
        // the recorder running and turn every arm into a timeout.
        self.shutdown.store(true, Ordering::Relaxed);
        // BOUNDED (`common::join_bagd`), never a bare `JoinHandle::join()`: a
        // recorder that wedges must fail this arm loudly instead of hanging the
        // suite until CI's own timeout kills it with no attribution.
        let summary = join_bagd(self.join, &self.shutdown, &self.tag).expect("recorder");
        assert!(
            summary.bag_paths.is_empty(),
            "a WINDOW-ONLY recorder writes no continuous bag — the mode has to be REAL, or these \
             arms are driving the `--record` path this file exists to be the twin of"
        );
        let bags = captures(&self.harness.dir);
        (summary, bags, self.harness)
    }
}

// ===========================================================================
// THE ACCEPTANCE ARM
// ===========================================================================

/// **THE HEADLINE**: a window-only recorder handed this run's trace ring drains
/// it, and its capture carries the trace TRIMMED to its own anchor — byte for
/// byte the records the ring was handed — with `resimmable: true`.
///
/// Four independent claims, each against a HAND oracle rather than against the
/// code's own output:
///
/// 1. the BYTES: every carried payload equals `expected_kept()`, the records
///    past the anchor step, stated by this file;
/// 2. the VERDICT: `resimmable: true`, which was unreachable on this mode before
///    this drain existed (a window-only capture always read
///    `HANDOFF_TRACE_NONE_NO_RINGS`);
/// 3. the ATTACH ARM: `from_start`, because the recorder read from record 0 —
///    the property that makes step-0 captures resimmable with no anchor at all;
/// 4. the ACCOUNTING: the drain's lifetime record count reaches BOTH the capture
///    manifest and the recorder's summary under the same name the `--record`
///    path reports it, and no lap was observed.
///
/// Deleting `rec.spawn_trace_drain(start)` from the drive loop
/// leaves every unit oracle green and fails claim 1 here with an empty trace.
#[test]
fn a_window_only_recorder_drains_its_trace_ring_and_its_capture_carries_it() {
    let mut run = Run::start("headline", RING_RECORDS, |_| {});
    let pushed = trace_across_the_anchor();
    push_trace(&mut run.trace, &pushed);
    let anchor = full_anchor(ANCHOR_STEP);
    push_state(&mut run.state, &anchor);
    run.publish_frames(6);

    // The drain must have READ the ring before the capture is triggered — a
    // capture that fired first would be trace-less for a reason that has nothing
    // to do with the code under test. Waited on as a CONDITION under a liveness
    // ceiling load can delay but not invert.
    run.status
        .await_records(pushed.len() as u64, "the headline arm");
    // …and the anchor must be OLDER than the deadline this capture derives
    // (`trigger − post_window`), or `select` correctly refuses it and the arm
    // measures the harness. A sleep is the right instrument: what has to become
    // true is that WALL TIME has passed, and it is a LOWER bound, so load can
    // only push the anchor further from the boundary.
    std::thread::sleep(Duration::from_millis(POST_WINDOW_MS * 2));

    run.capture("an operator saw the wobble");
    let (summary, bags, harness) = run.finish();
    assert_eq!(bags.len(), 1, "exactly one capture, got {bags:?}");
    let bag = &bags[0];

    // (1) THE BYTES.
    let oracle = expected_kept();
    let carried = trace_payloads(bag);
    assert_eq!(
        carried.len(),
        oracle.len(),
        "the capture must carry every record past the anchor step and no other"
    );
    for (i, (payload, expected)) in carried.iter().zip(&oracle).enumerate() {
        assert_eq!(
            payload.as_slice(),
            expected.as_bytes().as_slice(),
            "record {i} must be byte-identical to the one the ring was handed"
        );
    }

    let m = flashback_manifest(bag);
    // (2) THE VERDICT.
    assert_eq!(
        resimmable(&m),
        Some(true),
        "a window-only capture carrying a trimmed trace and a complete anchor is RESIMMABLE — \
         this is the verdict the mid-run attach makes reachable on this mode: {m}"
    );
    assert_eq!(carried_records(&m), oracle.len() as u64, "{m}");

    // (3) THE ATTACH ARM.
    let attach = handoff_str(&m, "trace_attach");
    assert!(
        attach.starts_with("from_start:"),
        "the recorder opened at record 0 and must say so: {attach}"
    );

    // (4) THE ACCOUNTING. Lifetime figures, so they count what the DRAIN read
    // rather than what this capture kept — the pair is the diagnosis.
    assert_eq!(
        handoff_u64(&m, "trace_ring_records"),
        pushed.len() as u64,
        "the manifest reports every record the drain admitted: {m}"
    );
    assert_eq!(handoff_u64(&m, "trace_laps"), 0, "no lap: {m}");
    assert_eq!(
        handoff_u64(&m, "trace_rings_configured"),
        1,
        "one ring was handed over: {m}"
    );
    assert_eq!(
        summary.ring_records,
        pushed.len() as u64,
        "the window-only summary reports the SAME quantity the `--record` path reports under \
         this name — reporting 0 would hide whether the drain ran at all"
    );

    harness.cleanup();
}

/// Records committed during shutdown reach the final
/// capture, because the drain is stopped and JOINED before the capture is
/// resolved.
///
/// The drain's loop runs one more pass after it sees the stop flag, exactly so
/// the tail is collected. But a pass reachable only through the
/// handle's `Drop` fires when the `Recorder` is dropped, AFTER `finalize`
/// has resolved the outstanding capture and built the summary — and the drive
/// loop's exit condition never observes the drain at all. The tail would land in
/// a retention nobody reads again, and the LAST capture of a run (the one an
/// incident is most likely to be in) would be short by whatever the graph
/// committed while it was stopping.
///
/// Driven by pushing the tail with NO rendezvous, then triggering the capture
/// and shutting down. The oracle is the TAIL records specifically — asserting
/// "some trace" would pass on a build that carried only the earlier records.
///
/// # Scope — this arm does not catch a wrong join order
///
/// It is a LIVENESS arm: it proves the tail reaches the final capture, not
/// WHICH mechanism put it there. `Run::capture` waits for `Finished`, so the
/// capture has already closed and selected its trace before `finish()` sets the
/// shutdown flag — and at a 5 ms drain tick against a 300 ms post window the
/// periodic pass has collected the tail long before that. So a build whose join
/// runs AFTER `resolve_flashback_at_shutdown` passes this arm too (MEASURED:
/// that build left the whole e2e file green).
///
/// Tightening it would make it a FLAKE: making the tail
/// reachable only through the join would mean pushing it inside the window
/// between the drain's last periodic pass and the recorder observing shutdown —
/// a window neither this test nor the recorder can bound, so the arm would
/// depend on losing a race rather than on the code being right.
///
/// What the join guarantees is an ORDER, and the order is pinned where it can be:
/// `cerulion_bagd::tests::the_trace_drain_is_finished_before_the_shutdown_capture_is_resolved`
/// (structural, over a comment-stripped view of `finalize`), with its sibling
/// pinning that the shutdown INSTANT is sampled after the join.
#[test]
fn records_committed_during_shutdown_reach_the_final_capture() {
    let mut run = Run::start("shutdowntail", RING_RECORDS, |_| {});

    // The steps the capture would carry even WITHOUT the join — pushed and
    // rendezvoused on, so the arm cannot pass by them being late.
    let early = trace_across_the_anchor();
    push_trace(&mut run.trace, &early);
    push_state(&mut run.state, &full_anchor(ANCHOR_STEP));
    run.publish_frames(4);
    run.status
        .await_records(early.len() as u64, "the shutdown-tail arm's early records");
    std::thread::sleep(Duration::from_millis(POST_WINDOW_MS * 2));

    // THE TAIL: pushed with NO rendezvous, immediately before the capture and
    // the shutdown. Whether the periodic drain happens to catch it is a race the
    // fix removes — the finalize-time join is what makes it deterministic.
    let tail_start = ANCHOR_STEP + 500;
    let tail: Vec<TraceRingRecord> = (tail_start..=(tail_start + 2))
        .flat_map(step_records)
        .collect();
    push_trace(&mut run.trace, &tail);

    run.capture("with a tail still on the ring");
    let (summary, bags, harness) = run.finish();
    assert_eq!(bags.len(), 1, "exactly one capture, got {bags:?}");

    // THE PIN: every tail record is in the capture. A capture that closes
    // before the drain's final pass is missing these.
    let carried = trace_records(&bags[0]);
    let carried_steps: Vec<u64> = carried.iter().map(|r| r.step).collect();
    for r in &tail {
        assert!(
            carried
                .iter()
                .any(|c| c.step == r.step && c.record_type == r.record_type),
            "a record committed during shutdown (step {}) must be in the FINAL \
             capture — the drain is joined before the capture resolves. Carried: \
             {carried_steps:?}",
            r.step
        );
    }
    // …and the recorder's own accounting counts them too.
    assert_eq!(
        summary.ring_records,
        (early.len() + tail.len()) as u64,
        "the summary counts the tail the final pass collected"
    );

    harness.cleanup();
}

/// A window-only capture stamps each record with its ring's
/// HEADER RANK, so a nonzero-rank ring's capture decodes against its own
/// manifest instead of reading `ForeignRank`.
///
/// A record's on-ring `reserved` carries the discard bit and NOT the rank — the
/// rank lives in the ring header, and the continuous writer co-stamps it at
/// drain time (`WriterCore::write_batch`: `rank | (on_ring & TRACE_DISCARD_BIT)`)
/// so a bag's records are self-describing. A window-only drain that admits them
/// UNSTAMPED makes every record claim rank 0. On rank 0 that is invisible; on any
/// other rank the capture's own `trace_manifest_rank<N>.json` is for rank N while
/// its records say 0, `bag play --resim` refuses at the rank gate, and the
/// capture cannot be resimmed at all.
///
/// LIVE the moment a window-only recorder is handed one ring per rank —
/// the same reachability argument as the cross-ring hole, and the same
/// reason it must hold.
#[test]
fn a_nonzero_rank_rings_capture_carries_that_rank_on_every_record() {
    const RANK: u32 = 3;
    let mut run = Run::start_with_rank("rankstamp", RING_RECORDS, RANK, |_| {});
    let pushed = trace_across_the_anchor();
    push_trace(&mut run.trace, &pushed);
    push_state(&mut run.state, &full_anchor(ANCHOR_STEP));
    run.publish_frames(4);
    run.status
        .await_records(pushed.len() as u64, "the rank-stamp arm");
    std::thread::sleep(Duration::from_millis(POST_WINDOW_MS * 2));

    run.capture("off a nonzero-rank ring");
    let (_summary, bags, harness) = run.finish();
    assert_eq!(bags.len(), 1, "exactly one capture, got {bags:?}");

    // (1) EVERY carried record reports the ring's rank. The producer pushed
    // `reserved: 0` throughout, so an unstamped drain yields 0 here — which is
    // exactly the failure this arm pins, and on rank 0 would be unobservable.
    let carried = trace_records(&bags[0]);
    assert!(!carried.is_empty(), "the capture must carry a trace");
    for (i, r) in carried.iter().enumerate() {
        assert_eq!(
            r.reserved, RANK,
            "record {i} must carry the ring's header rank {RANK}, not the on-ring \
             0 — an unstamped record decodes as ForeignRank against this \
             capture's own manifest"
        );
        assert_eq!(
            r.rank(),
            RANK,
            "…and it must read back through the masked accessor resim uses"
        );
    }

    // (2) …and the records RESOLVE against the rank-3 manifest, which is what
    // the stamp buys. A lone rank-3 ring is legitimately refused — its worker
    // roster is `[3]`, so ranks 0..2 have no manifest — but the refusal must be
    // that ROSTER gap and NOT a record-level rejection.
    //
    // That distinction is the sharp end of this arm. Unstamped, every record
    // claims rank 0, `rank_node_counts` gives rank 0 a count of zero, and the
    // record walk refuses at the FIRST record with an out-of-range `node_idx` —
    // a different gap, reached earlier, naming a record instead of a roster. So
    // the reason discriminates the stamp even though both states refuse.
    let m = flashback_manifest(&bags[0]);
    let reason = refusal(&m);
    assert!(
        reason.contains("worker trace manifests are not contiguous"),
        "the only thing wrong with a lone rank-3 capture is its roster: {reason}"
    );
    assert!(
        !reason.contains("refuses a trace on that record"),
        "no RECORD-level rejection — the records resolved against their own \
         rank, which is exactly what the stamp is for: {reason}"
    );

    harness.cleanup();
}

/// A ring that had ALREADY LAPPED when the recorder first read it attaches at
/// the LIVE cursor and says so — and that is an ATTACH, not a hole.
///
/// The distinction is the whole arm. A reader that was lapped before its first
/// read has no near side to have lost, so there is nothing a capture can straddle
/// and nothing to refuse; what it does have is a trace that does not reach step
/// 0, which a reader must not be allowed to assume from a record count. So the
/// manifest reports `at_live` with the gate's own numbers, and `trace_laps` stays
/// 0.
///
/// The pre-lap is REAL: `TINY_RING_RECORDS` capacity, four times that many
/// records pushed before the recorder exists.
#[test]
fn a_ring_that_lapped_before_the_recorder_read_it_attaches_at_live_and_says_so() {
    let overflow: Vec<TraceRingRecord> = (0..u64::from(TINY_RING_RECORDS) * 4)
        .map(|s| boundary(s + 1))
        .collect();
    let mut run = Run::start("lateattach", TINY_RING_RECORDS, |p| {
        push_trace(p, &overflow);
    });

    // The re-attach happens on the drain's FIRST pass, and it must be observed
    // before the post-attach records are pushed — otherwise those records are
    // skipped by the live cursor too and the arm's oracle is a race.
    assert!(
        await_condition(DEADLINE, || run
            .status
            .poll()
            .attach
            .is_some_and(|a| a.starts_with("at_live:"))),
        "the drain must report the LIVE attach arm; last saw {:?}",
        run.status.seen
    );
    assert_eq!(
        run.status.poll().laps,
        0,
        "a reader lapped before its FIRST read lost no near side, so this is an ATTACH and not \
         a hole: {:?}",
        run.status.seen
    );

    // Now the ordinary shape: an anchor, and steps past it.
    let anchor_step = 500;
    let after: Vec<TraceRingRecord> = ((anchor_step - 1)..=(anchor_step + 2))
        .flat_map(step_records)
        .collect();
    push_trace(&mut run.trace, &after);
    push_state(&mut run.state, &full_anchor(anchor_step));
    run.publish_frames(4);
    run.status
        .await_records(after.len() as u64, "the late-attach arm");
    std::thread::sleep(Duration::from_millis(POST_WINDOW_MS * 2));

    run.capture("after a late attach");
    let (_summary, bags, harness) = run.finish();
    assert_eq!(bags.len(), 1, "exactly one capture, got {bags:?}");
    let m = flashback_manifest(&bags[0]);

    let attach = handoff_str(&m, "trace_attach");
    assert!(attach.starts_with("at_live:"), "{attach}");
    assert!(
        attach.contains("already lapped"),
        "the cause is named, not just the arm: {attach}"
    );
    assert!(
        attach.contains("leading record(s) discarded"),
        "the head-step gate's own coverage is reported: {attach}"
    );
    assert_eq!(
        handoff_u64(&m, "trace_laps"),
        0,
        "a pre-lapped FIRST read is an attach, never a hole: {m}"
    );
    // The carried trace is compared against the FULL oracle, exactly as the
    // headline arm does — `>= 1 post-anchor record` would pass on a capture that
    // carried a single record, or the wrong ones, or them out of order, which is
    // most of what could go wrong on a path that just re-attached mid-stream.
    //
    // The oracle is the post-attach records past the anchor step, stated by this
    // arm rather than read back out of the code that trimmed them.
    let expected: Vec<TraceRingRecord> = after
        .iter()
        .copied()
        .filter(|r| r.step > anchor_step)
        .collect();
    assert!(
        !expected.is_empty(),
        "the arm's own oracle must be non-empty, or it asserts nothing"
    );
    let carried = trace_records(&bags[0]);
    assert_eq!(
        carried, expected,
        "the capture carries exactly the post-attach records past the anchor, in \
         order — and nothing the live cursor skipped: {m}"
    );

    harness.cleanup();
}

/// A declared trace ring the recorder cannot read
/// makes its captures NON-RESIMMABLE — and they still carry the trace it did
/// read.
///
/// `Recorder::setup` warns and CONTINUES past a declared ring it cannot open on
/// the attach path, deliberately: one vanished ring must cost its trace, not the
/// frames of every topic in the bag. The verdict then judged only the records
/// and identities that SURVIVED, so a capture whose recorder read one of a run's
/// two ranks published `resimmable: true` over a trace that LOOKS whole — every
/// record well formed, the boundaries in order, the one manifest it carries
/// contiguous — while an entire rank's fires were simply absent. Nothing a bag
/// reader can compute distinguishes that from a run whose other nodes never
/// fired, which is why the recorder has to say so.
///
/// The vanished ring is created and then DROPPED, which unlinks its SHM name —
/// the exact way one arises in production (the owning run exits while a recorder
/// is attaching), rather than a name that was never plausible.
///
/// TWO runs, one difference. The CONTROL is the whole arm: without it this
/// passes against a judge that refuses every attached recorder's capture.
#[test]
fn a_declared_ring_the_recorder_cannot_read_refuses_the_capture_but_keeps_the_trace() {
    // The ring that VANISHED: created so the name is a real one, dropped so
    // nothing can open it.
    let vanished = {
        let owner = TraceRingOwner::create(&unique_ring_tag("gone"), 16, 1, &NODES)
            .expect("create the ring that is about to vanish");
        owner.name().to_string()
    };

    for (tag, extra, want_resimmable) in [
        ("partialctl", Vec::new(), true),
        ("partial", vec![vanished.clone()], false),
    ] {
        let mut run = Run::start_attached(tag, &extra);
        let pushed = trace_across_the_anchor();
        push_trace(&mut run.trace, &pushed);
        push_state(&mut run.state, &full_anchor(ANCHOR_STEP));
        run.publish_frames(4);
        run.status.await_records(pushed.len() as u64, tag);
        // The anchor must be older than the deadline this capture derives — see
        // the headline arm for why a sleep is the right instrument here.
        std::thread::sleep(Duration::from_millis(POST_WINDOW_MS * 2));

        run.capture("a ring this recorder could not read");
        let (summary, bags, harness) = run.finish();
        assert_eq!(bags.len(), 1, "{tag}: exactly one capture, got {bags:?}");
        let m = flashback_manifest(&bags[0]);

        // BOTH runs carry the trace they DID read: a refusal is about resim, not
        // about evidence.
        assert!(
            carried_records(&m) > 0,
            "{tag}: the capture must still carry the trace this recorder read: {m}"
        );
        assert_eq!(
            resimmable(&m),
            Some(want_resimmable),
            "{tag}: a capture missing a whole ring's records must not be stamped resimmable: {m}"
        );
        if want_resimmable {
            assert!(
                summary.rings_unavailable.is_empty(),
                "{tag}: CONTROL: every declared ring was readable, or this arm proves nothing"
            );
        } else {
            assert_eq!(
                summary.rings_unavailable.len(),
                1,
                "{tag}: the vanished ring is reported as unavailable: {:?}",
                summary.rings_unavailable
            );
            assert_eq!(
                handoff_u64(&m, "trace_rings_configured"),
                2,
                "{tag}: the handoff states what this recorder was HANDED: {m}"
            );
            let why = refusal(&m);
            assert!(
                why.contains("2 scheduler-trace ring(s)") && why.contains("no reader for 1"),
                "{tag}: the refusal states the SHORTFALL: {why}"
            );
            assert!(
                why.contains("in the bag and readable"),
                "{tag}: …and that the capture is still evidence: {why}"
            );
        }
        harness.cleanup();
    }
}

/// **THE LAP ARM**: a hole refuses only the capture that CARRIES it. A later
/// capture of the same run is resimmable.
///
/// This is "a hole is refused, a run is not" applied to a recorder stall, and it is the reason
/// `trace_drain` re-opens at the live cursor instead of RETIRING the ring the way
/// the state-ring harvest does. Two captures, one run, one hole:
///
/// * capture A resumes from an anchor BEFORE the hole, so its trimmed trace holds
///   records on both sides of it — refused, with the hole's own edges named;
/// * capture B resumes from an anchor AFTER the re-attach, so its trimmed trace
///   holds only the far side — resimmable.
///
/// The difference between them is the TRIM, which is deterministic, so neither
/// verdict depends on a wall clock or on eviction.
///
/// Making a lap RETIRE the ring (the `harvest_anchors` rule)
/// fails capture B — the recorder would read nothing after the hole, so B carries
/// no trace at all and reports `NoTrace`.
#[test]
fn a_lap_refuses_only_the_capture_that_carries_it_and_a_later_one_is_resimmable() {
    let mut run = Run::start("lap", TINY_RING_RECORDS, |_| {});

    // PHASE 1 — the near side. An anchor at `NEAR_ANCHOR`, and one step past it
    // that capture A's trim will keep.
    const NEAR_ANCHOR: u64 = 11;
    /// The last step of the near side — the earliest the hole's near edge can be,
    /// since the drain admitted this step before the burst began.
    const NEAR_LAST: u64 = NEAR_ANCHOR + 1;
    let near: Vec<TraceRingRecord> = (NEAR_ANCHOR..=NEAR_LAST).flat_map(step_records).collect();
    push_trace(&mut run.trace, &near);
    push_state(&mut run.state, &full_anchor(NEAR_ANCHOR));
    run.publish_frames(4);
    run.status
        .await_records(near.len() as u64, "the lap arm's near side");

    // PHASE 2 — outrun the reader. A tight burst into a 16-record ring, repeated
    // until the recorder REPORTS the lap. Bounded by the same liveness ceiling
    // as everything else: this is a condition, not a sleep, and a slow runner
    // makes it easier rather than harder.
    let mut burst_step = 100u64;
    assert!(
        await_condition(DEADLINE, || {
            for _ in 0..64 {
                run.trace.push(&boundary(burst_step));
                burst_step += 1;
            }
            run.status.poll().laps >= 1
        }),
        "a burst into a {TINY_RING_RECORDS}-record ring must lap the reader; last saw {:?}",
        run.status.seen
    );

    // PHASE 3 — the far side, past the hole.
    //
    // Pushed IN THE WAIT, one step per poll, rather than once up front: when the
    // burst loop exits there may still be an OBSERVED-BUT-UNHANDLED overrun in
    // flight, and the re-attach that handles it sets the cursor to the write
    // cursor AT THAT INSTANT — so a far side pushed before it is jumped over and
    // this arm waits forever for records the drain will never see. (MEASURED, at
    // both the unit and the e2e level; the primitive re-attaches and reads a tail
    // correctly, so it is the harness racing the code.) Three records per poll
    // into a `TINY_RING_RECORDS` ring cannot lap a drain on its own cadence, so
    // the recovery loop cannot cause the condition it is recovering from.
    let before_far = run.status.poll().ring_records;
    let far_start = burst_step + 100;
    let mut far_step = far_start;
    assert!(
        await_condition(DEADLINE, || {
            push_trace(&mut run.trace, &step_records(far_step));
            far_step += 1;
            run.status.poll().ring_records > before_far
        }),
        "the re-attached drain must read the far side; last saw {:?}",
        run.status.seen
    );
    run.publish_frames(4);
    std::thread::sleep(Duration::from_millis(POST_WINDOW_MS * 2));

    // CAPTURE A — resumes from the near anchor, so it carries both sides.
    run.capture("straddling the hole");

    // PHASE 4 — a NEW anchor past the hole, and steps past THAT. Above every
    // step the recovery loop above reached, so capture B's trim keeps only what
    // this phase pushes.
    let later_anchor = far_step + 500;
    let later: Vec<TraceRingRecord> = (later_anchor..=(later_anchor + 2))
        .flat_map(step_records)
        .collect();
    let before_later = run.status.poll().ring_records;
    push_trace(&mut run.trace, &later);
    push_state(&mut run.state, &full_anchor(later_anchor));
    run.publish_frames(4);
    run.status.await_records(
        before_later + later.len() as u64,
        "the lap arm's later window",
    );
    std::thread::sleep(Duration::from_millis(POST_WINDOW_MS * 2));

    // CAPTURE B — resumes past the re-attach, so it carries only the far side.
    run.capture("after the re-attach");

    let (_summary, bags, harness) = run.finish();
    assert_eq!(bags.len(), 2, "two captures, got {bags:?}");
    let a = flashback_manifest(&bags[0]);
    let b = flashback_manifest(&bags[1]);

    // A: refused, naming the hole.
    assert_eq!(
        resimmable(&a),
        Some(false),
        "the capture that carries the hole is REFUSED: {a}"
    );
    let reason = refusal(&a);
    assert!(
        reason.contains("HOLE"),
        "the refusal must be the LAP one, not a downstream symptom of it: {reason}"
    );
    // The hole's EDGES, parsed rather than string-matched against one expected
    // number. Which step the drain last admitted before the lap is not something
    // this test may pin: the burst that forces the lap is a race by construction
    // (that is what a lap IS), so the reader may or may not have admitted some of
    // it first. What IS invariant is the SHAPE — a closed hole, both edges real,
    // the near one at or after the last step of the near side this arm pushed,
    // and ordered. A message that fabricated an edge, left one unknown, or
    // reported them backwards fails here; one that names a legitimately different
    // burst step does not, because that is the truth about the run.
    let (near_edge, far_edge) = hole_edges(&reason);
    assert!(
        near_edge >= NEAR_LAST,
        "the near edge is a step the recorder really admitted before the hole, and it had \
         admitted at least step {NEAR_LAST}: {reason}"
    );
    assert!(
        far_edge > near_edge,
        "a hole runs forward: {near_edge} -> {far_edge} in {reason}"
    );
    assert!(
        !reason.contains("step ?"),
        "the hole was CLOSED by the far side, so neither edge is unknown: {reason}"
    );
    assert!(
        reason.contains("Later captures"),
        "the refusal scopes the damage to THIS capture: {reason}"
    );
    assert!(handoff_u64(&a, "trace_laps") >= 1, "{a}");

    // B: the SAME run, the SAME hole, resimmable.
    assert_eq!(
        resimmable(&b),
        Some(true),
        "a capture whose window begins after the re-attach is RESIMMABLE — one recorder stall \
         must not condemn the rest of a run (project rule): {b}"
    );
    assert!(
        handoff_u64(&b, "trace_laps") >= 1,
        "…and it still reports the run's lap count, which is the accurate pair: the run had a \
         stall, this bag does not contain it: {b}"
    );
    let b_steps: Vec<u64> = trace_records(&bags[1]).iter().map(|r| r.step).collect();
    assert!(
        b_steps.iter().all(|s| *s > later_anchor),
        "capture B carries only the far side: {b_steps:?}"
    );

    harness.cleanup();
}

/// The recorder's drain and a SECOND, independent reader each see the WHOLE
/// stream: one producer, N independent `FailLoud` readers.
///
/// The property under test is the one the mid-run attach rests on: a run's
/// standing window recorder drains its trace ring while a mid-run
/// `cerulion bag record --run` attaches to the SAME ring, and neither is a party
/// to the other. `cerulion_core`'s `shm_ring_test.rs` pins it between two hand-driven
/// consumers; this arm
/// re-runs it with a REAL LIVE RECORDER as one of the two readers, which is the
/// shape that ships.
///
/// Both halves are asserted against the same hand oracle, so this cannot pass by
/// one reader getting everything and the other nothing.
#[test]
fn the_live_recorder_and_a_second_reader_each_see_the_whole_stream() {
    let mut run = Run::start("twoconsumers", RING_RECORDS, |_| {});
    // The second reader, opened at record 0 while the recorder's drain is
    // already running.
    let mut second =
        TraceRingConsumer::open(&run.trace_ring_name).expect("a second reader of the same ring");

    let pushed = trace_across_the_anchor();
    push_trace(&mut run.trace, &pushed);
    push_state(&mut run.state, &full_anchor(ANCHOR_STEP));
    run.publish_frames(4);
    run.status
        .await_records(pushed.len() as u64, "the two-consumer arm");

    // READER 2 — the whole stream, from record 0, unaffected by the recorder
    // having drained the same records.
    let mut mine: Vec<TraceRingRecord> = Vec::new();
    assert!(
        await_condition(DEADLINE, || {
            second.drain(&mut mine).expect("the second reader drains");
            mine.len() >= pushed.len()
        }),
        "the second reader must see the whole stream; saw {} of {}",
        mine.len(),
        pushed.len()
    );
    assert_eq!(
        mine, pushed,
        "the second reader sees every record the ring was handed, in order — the recorder's own \
         drain neither consumed them nor advanced this reader's cursor"
    );

    std::thread::sleep(Duration::from_millis(POST_WINDOW_MS * 2));
    run.capture("with a second reader attached");
    let (summary, bags, harness) = run.finish();
    assert_eq!(bags.len(), 1, "exactly one capture, got {bags:?}");

    // READER 1 — the recorder, against the SAME oracle.
    let oracle = expected_kept();
    let carried = trace_payloads(&bags[0]);
    assert_eq!(
        carried.len(),
        oracle.len(),
        "the RECORDER's own read is complete too — a second reader must cost it nothing"
    );
    for (payload, expected) in carried.iter().zip(&oracle) {
        assert_eq!(payload.as_slice(), expected.as_bytes().as_slice());
    }
    assert_eq!(summary.ring_records, pushed.len() as u64);

    harness.cleanup();
}
