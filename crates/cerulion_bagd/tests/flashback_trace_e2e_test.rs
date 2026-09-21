// SPDX-License-Identifier: AGPL-3.0-only
//! End to end: a FLASHBACK CAPTURE carries the SCHEDULER TRACE
//! a resume is driven by — trimmed to its own anchor — and says whether
//! it can be resumed.
//!
//! Real `TraceRingOwner` and `StateRingOwner` producers, a real recorder in
//! `--record` mode, a real trigger channel, the finished MCAP read back.
//! Isolated per-test SHM roots + unique ring tags, so the file is parallel-safe.
//!
//! # Why every arm here runs `--record` mode
//!
//! On a `--record` run THIS RECORDER's consumer of the trace ring is the WRITER
//! THREAD (`WriterCore::write_batch` drains it, writes the continuous bag from
//! the same span, then commits) and it feeds the retention from inside that one
//! drain. So `window_only: false` is the mode under test here rather than a
//! detail, and the traceless shape gets its own arm precisely because it must
//! stay ORDINARY.
//!
//! It is not the ONLY mode that feeds a retention: the WINDOW-ONLY
//! recorder has its own reader (`crate::trace_drain`, a dedicated drain
//! thread), so a run's standing recorder carries a trace too. That is the
//! twin file `flashback_trace_harvest_e2e_test.rs`, and the two are deliberately
//! apart: the mode is what each one is about, and an arm that drove both would
//! be pinning neither.
//!
//! # What these arms prove that the unit tests cannot
//!
//! `trace_window.rs` is oracle-tested in isolation, and every one of those arms
//! stays green if nothing WIRES it: the writer thread could stop feeding the
//! retention, the trim could be handed the wrong step, the trimmed records could
//! be dropped between `select_trace` and the `CaptureJob`, and the verdict could
//! be computed and then never rendered. Those are the inert-shipping shapes, and
//! only a run that pushes real records onto a real ring and then READS THE BAG
//! can see them.
//!
//! It is also the only place the two halves meet: the retention holds RAW records
//! so a capture's `__cerulion/scheduler_trace` bytes are identical to a
//! recording's — a claim about BYTES that only a byte comparison against
//! hand-built records can settle.
//!
//! # Injected constants, and only these
//!
//! The post window is injected at ~300 ms (the shipped 15 s would make this a
//! minute-long suite for a property that has nothing to do with the number). One
//! arm injects the trace retention's BYTE CEILING because it is ABOUT that
//! ceiling. Every other constant is the shipped one, and every deadline is a
//! generous liveness ceiling in seconds rather than a wall stated in units of the
//! thing under test.
//!
//! # EVERY arm rendezvous on the READY-FILE before it publishes anything
//!
//! `run_bagd` attaches its data-only TAPS and opens its flashback RESPONDER
//! inside `Recorder::setup`, so anything published before setup returns is at
//! risk: a tap requests no late-joiner history, and iceoryx2 keeps none for a
//! control subscriber that attaches later. A loaded runner widens that window;
//! nothing narrows it.
//!
//! MEASURED rather than argued, under concurrent build load with
//! the test process demoted to macOS background QoS (`taskpolicy -b`): the
//! spawn→ready window runs **130-1014 ms (p50 232 ms)** over 40 samples, while
//! `capture_inner_full` publishes its whole pre-window inside 80 ms. So without
//! the rendezvous EVERY pre-window frame is published before the tap exists,
//! and the capture comes back carrying **7 of 8** frames on 8 of 8 runs — against
//! **8 of 8** with the rendezvous in place, on 6 of 6.
//!
//! The untreated shape is a harness that spawns the recorder and immediately
//! pushes ring records and publishes eight frames. Its symptom is a NO-ANCHOR
//! arm failing on a loaded machine, on the assertion "a capture with no anchor
//! still carries its frames". The sibling flashback files rendezvous the same
//! way for the same reason.
//!
//! The NO-ANCHOR arms are the exposed ones, and the reason is structural rather
//! than a matter of margins: the state-ring drain rendezvous below is guarded by
//! `if !state.is_empty()`, and those arms pass `&[]`. So that guard gives them NO
//! rendezvous at all — neither before the frames nor before the capture request.
//!
//! `cfg.ready_file` + [`await_bagd_ready`] closes both: it is bagd's own
//! taps-ready launch handshake, not a test hook, and it is a CONDITION load can
//! delay but not invert. Every cross-thread join goes through [`join_bagd`] for
//! the same reason a wedge must fail rather than hang.
//!
//! The `sleep`s that REMAIN are producer PACING inside the publish loop, spacing
//! frames so the window has a temporal span. None is a rendezvous: the tap
//! provably exists by then, so load can only spread a window WIDER.

#![cfg(unix)]

mod common;

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_bagd::{run_bagd, BagdConfig, FlashbackSettings, TapSpec};
use cerulion_core::flashback::channel::{FlashbackOutcome, FlashbackRequester};
use cerulion_core::flashback::retention::RetentionCaps;
use cerulion_core::flashback::switch::TriggerPosture;
use cerulion_core::flashback::trigger::{CaptureRequest, TriggerPolicy};
use cerulion_core::state_ring::{
    encode_record, StateRecordHeader, StateRingOwner, RECORD_KIND_FINAL, STATE_RECORD_PAYLOAD,
    STATE_RECORD_SIZE,
};
use cerulion_core::trace_ring::{
    TraceRingProducer, TraceRingRecord, RECORD_TYPE_DEPARTURE, RECORD_TYPE_FIRE,
    RECORD_TYPE_STEP_BOUNDARY, TRACE_RECORD_SIZE,
};

use common::{
    await_bagd_ready, await_condition, build_frame, join_bagd, make_manager, publisher, unique_out,
    unique_ready_file, unique_ring_tag, unique_topic,
};

/// Shipped everywhere except the post window — see the module docs.
const POST_WINDOW_MS: u64 = 300;

/// How many PRE-WINDOW frames `capture_inner_full` publishes, and therefore how
/// many a capture must carry.
///
/// It is a NAMED constant so the publish loop and the oracle read ONE number.
/// The window is 30 s / 64 MiB against eight ~35-byte frames published inside
/// 80 ms, so nothing legitimate can trim them — a short count is LOSS.
const PRE_WINDOW_FRAMES: u32 = 8;
/// Generous liveness ceilings. Load can only delay these, never invert them.
const DEADLINE: Duration = Duration::from_secs(20);
/// An arbitrary, stable schema hash for the hand-built data frames.
const HASH: u64 = 0x0C1E_0C1E_0C1E_1260;
/// The run every state record in this file carries.
const RUN: u64 = 0x0000_C1E5_0000_1260;
/// Far above what any arm pushes, so no arm is measuring a lap it did not mean.
const RING_RECORDS: u32 = 256;
/// The step the embedded checkpoint is taken at.
const ANCHOR_STEP: u64 = 40;
/// The nodes both rings declare, in manifest order.
const NODES: [&str; 2] = ["ticker", "relay"];
/// The armed plane's cadence, for the arms that stand one up.
const ARM_CADENCE_STEPS: u64 = 100;
/// Its onset — deliberately NOT 0, so a reader that fabricated a default could
/// not pass for one that read the word.
const ARM_FIRST_ANCHOR_STEP: u64 = 37;

fn temp_dir(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "trace-{tag}-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos())
            .unwrap_or(0)
    ));
    std::fs::create_dir_all(&dir).expect("temp dir");
    dir
}

fn settings(dir: &Path, trace_max_bytes: u64) -> FlashbackSettings {
    FlashbackSettings {
        window_span: Duration::from_secs(30),
        window_max_bytes: 64 * 1024 * 1024,
        anchor_max_bytes: 64 * 1024 * 1024,
        // A STATED ceiling: these arms are about the
        // TRACE retention, and an anchor reserve free to move would change what
        // the frame window holds for a reason no arm here is asking about.
        anchor_cap_basis: cerulion_core::flashback::CapBasis::Env,
        trace_max_bytes,
        dir: dir.to_path_buf(),
        label: "trace-demo".into(),
        caps: RetentionCaps::default(),
        policy: TriggerPolicy {
            post_window_ns: POST_WINDOW_MS * 1_000_000,
            ..Default::default()
        },
        posture: TriggerPosture::default(),
        // THE mode under test — see the module docs. A window-only recorder is
        // handed no trace ring at all, so it could not drive any of this.
        // These arms are not about the exclude lever.
        exclude_topics: cerulion_core::flashback::ExcludeTopics::default(),

        window_only: false,
        // Inert here — a `--record`-shaped recorder's
        // taps are ceiling-deep whatever the budget says.
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

/// A DEPARTURE (fault) boundary — a peer worker lost mid-run.
///
/// `node_idx` carries the departed process RANK on this record kind, not a
/// manifest index.
fn departure(step: u64) -> TraceRingRecord {
    TraceRingRecord {
        step,
        fire_time_ns: 1_000_000 + step * 4_000_000,
        duration_ns: 0,
        node_idx: 1,
        global_level: 0,
        record_type: RECORD_TYPE_DEPARTURE,
        reserved: 0,
    }
}

/// A trace spanning the anchor step and three steps past it.
///
/// Written by hand, in the order the scheduler pushes them (a boundary at
/// `begin_step`, then that step's fires), so the trim's answer can be compared
/// against a set this file states rather than against the trim's own output.
fn trace_across_the_anchor() -> Vec<TraceRingRecord> {
    let mut out = Vec::new();
    for step in (ANCHOR_STEP - 2)..=(ANCHOR_STEP + 3) {
        out.push(boundary(step));
        out.push(fire(step, 0));
        out.push(fire(step, 1));
    }
    out
}

/// The records at or after `boundary(ANCHOR_STEP + 1)` — what the capture MUST
/// carry, stated independently of the code that trims.
fn expected_kept() -> Vec<TraceRingRecord> {
    trace_across_the_anchor()
        .into_iter()
        .filter(|r| r.step > ANCHOR_STEP)
        .collect()
}

/// One node's COMPLETE anchor at [`ANCHOR_STEP`], as its single final record.
fn anchor_record(node_idx: u32, fill: u8) -> Vec<u8> {
    let payload = vec![fill; 24];
    encode_record(
        &StateRecordHeader {
            run_id: RUN,
            step: ANCHOR_STEP,
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
// compile time (the repo's const-assert idiom; clippy 1.97 refuses a runtime
// assert on a constant).
const _: () = assert!(24 <= STATE_RECORD_PAYLOAD);

fn full_anchor() -> Vec<Vec<u8>> {
    vec![anchor_record(0, 0xA1), anchor_record(1, 0xB2)]
}

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

/// The per-rank `__cerulion/trace_manifest_rank<N>.json` attachments.
///
/// `bag play --resim` REFUSES a bag whose trace records name a rank it has no
/// manifest for, so a capture that carried a trace and no manifests would
/// advertise `resimmable: true` and then be rejected at exit 2 — the
/// confident-false class this vocabulary exists to prevent.
fn trace_manifests(bag: &Path) -> Vec<(String, serde_json::Value)> {
    let reader = cerulion_bag::BagReader::open(bag).expect("open the capture");
    let mut out = Vec::new();
    for rank in 0..4u32 {
        let name = format!("__cerulion/trace_manifest_rank{rank}.json");
        if let Ok(Some(att)) = reader.attachment(&name) {
            out.push((
                name,
                serde_json::from_slice(&att.data).expect("a manifest is valid JSON"),
            ));
        }
    }
    out
}

fn flashback_manifest(bag: &Path) -> serde_json::Value {
    let reader = cerulion_bag::BagReader::open(bag).expect("open the capture");
    let att = reader
        .attachment("__cerulion/flashback.json")
        .expect("read the attachment")
        .expect("the capture manifest is present");
    serde_json::from_slice(&att.data).expect("the manifest must be valid JSON")
}

// ===========================================================================
// The harness
// ===========================================================================

struct Harness {
    dir: PathBuf,
    bag: PathBuf,
    /// The topic the arm's producer published on — carried because
    /// `common::unique_topic` namespaces its argument (`/bagd_test/<base>/…`),
    /// so an arm cannot reconstruct it from the base it passed in.
    topic: String,
    /// The continuous bag a `--record` recorder also wrote.
    continuous: Option<PathBuf>,
}

impl Harness {
    fn cleanup(&self) {
        std::fs::remove_dir_all(&self.dir).ok();
        // The CONTINUOUS bag is NOT under `self.dir` — `common::unique_out`
        // returns a path directly in `std::env::temp_dir()` — so removing the
        // directory leaves it behind. On a `--record` arm that is a full
        // recording per run, accumulating in the system temp directory for as
        // long as the machine keeps it.
        if let Some(path) = &self.continuous {
            std::fs::remove_file(path).ok();
        }
    }
}

fn push_trace(producer: &mut TraceRingProducer, records: &[TraceRingRecord]) {
    for r in records {
        producer.push(r);
    }
}

/// Drive one capture on a `--record` recorder holding BOTH rings.
///
/// `trace` may be empty, and `with_trace_ring` may be `false` — the traceless and
/// ring-less shapes are arms, not failures.
/// The `graph.yaml` a real recorder is handed.
///
/// `graph_cmd` always passes `--attach graph.yaml:<path>` on the
/// `graph run --record` path, and `bag play --resim` REFUSES a bag without it
/// (its gate 4), so a harness that omitted it was modelling a recorder no
/// `graph run` produces — and every arm below would have been asserting a
/// verdict about the wrong bag. Its CONTENT is never parsed here (the capture
/// is not replayed in this file); only its presence is judged.
/// A graph that PARSES **and VALIDATES** — what `run_replay` actually requires.
///
/// `nodes: []` is NOT enough: it parses and FAILS `validate_graph`. Every
/// "resimmable" arm in this file would then run against a graph `bag play
/// --resim` refuses (`replay_cmd.rs`), and would pass only if
/// the verdict's own predicate stopped at `parse_graph` — a harness
/// modelling a bag no `graph
/// run --record` produces.
///
/// A single producer node, so there is no input reference to resolve.
const GRAPH_YAML: &[u8] = b"name: trace-demo\nnodes:\n  - id: probe\n    type: probe_node\n    outputs:\n      - name: out\n        schema: geometry_msgs/Vector3\n";

/// A graph that PARSES and fails `validate_graph`: `sink`'s input names a
/// producer no node in this graph declares, and the name is relative so it is
/// not the external-source exemption either.
///
/// The discriminator for the REPLAY_VERDICT thread: a predicate that
/// stops at `parse_graph` accepts this, and `run_replay` refuses it at
/// `BagInvalidAttachment` (`replay_cmd.rs:579`).
const GRAPH_YAML_PARSES_BUT_INVALID: &[u8] =
    b"name: trace-demo\nnodes:\n  - id: sink\n    type: sink_node\n    inputs:\n      - name: inp\n        source: ghost/out\n";

fn capture(
    tag: &str,
    trace: &[TraceRingRecord],
    state: &[Vec<u8>],
    trace_max_bytes: u64,
    with_trace_ring: bool,
) -> Harness {
    capture_inner(
        tag,
        trace,
        state,
        trace_max_bytes,
        with_trace_ring,
        Some(GRAPH_YAML),
        false,
    )
}

/// A capture whose recorder DECLARES several state
/// rings — the ordinary multi-process shape, and the one a verdict judged
/// against a hardcoded ring count of 0 gets wrong.
fn capture_with_extra_state_rings(
    tag: &str,
    trace: &[TraceRingRecord],
    state: &[Vec<u8>],
    extra_state_rings: usize,
) -> Harness {
    capture_inner_full(
        tag,
        trace,
        state,
        64 * 1024 * 1024,
        true,
        Some(GRAPH_YAML),
        false,
        extra_state_rings,
        false,
    )
}

fn capture_inner(
    tag: &str,
    trace: &[TraceRingRecord],
    state: &[Vec<u8>],
    trace_max_bytes: u64,
    with_trace_ring: bool,
    graph_attachment: Option<&[u8]>,
    with_armed_plane: bool,
) -> Harness {
    capture_inner_full(
        tag,
        trace,
        state,
        trace_max_bytes,
        with_trace_ring,
        graph_attachment,
        with_armed_plane,
        0,
        false,
    )
}

/// A capture whose recorder held ONE worker ring PLUS the
/// supervisor's DEPARTURE ring — which, since every multi-process run
/// provisions rings, is the SMALLEST deployment there is rather than a corner.
fn capture_with_departure_ring(tag: &str, trace: &[TraceRingRecord], state: &[Vec<u8>]) -> Harness {
    capture_inner_full(
        tag,
        trace,
        state,
        64 * 1024 * 1024,
        true,
        Some(GRAPH_YAML),
        false,
        0,
        true,
    )
}

#[allow(clippy::too_many_arguments)]
fn capture_inner_full(
    tag: &str,
    trace: &[TraceRingRecord],
    state: &[Vec<u8>],
    trace_max_bytes: u64,
    with_trace_ring: bool,
    graph_attachment: Option<&[u8]>,
    with_armed_plane: bool,
    extra_state_rings: usize,
    with_departure_ring: bool,
) -> Harness {
    let mgr = make_manager(64);
    let topic = unique_topic("/fbt/probe");
    let dir = temp_dir(tag);

    let mut cfg = BagdConfig::new(
        unique_out(&format!("fbt_{tag}")),
        vec![TapSpec::attach(&topic)],
    );
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    cfg.discover_live = false;
    // The launch handshake: `run_bagd` writes this the moment
    // `Recorder::setup` returns — which is where the data-only TAPS attach and
    // where the flashback RESPONDER's control subscriber is created.
    let ready = unique_ready_file(&format!("fbt_{tag}"));
    cfg.ready_file = Some(ready.clone());
    if let Some(graph) = graph_attachment {
        cfg.attachments = vec![("graph.yaml".to_string(), graph.to_vec())];
    }
    // A REAL armed capture plane, when the arm asks for one. The recorder ARMS
    // NOTHING (the graph owns the word); it only OBSERVES it, so
    // this stands in for the graph. Held to the end of this function, which
    // outlives the capture: `observe_arm_word` opens it `unowned` and snapshots,
    // but a word unlinked before the recorder looked would simply not be found.
    let _arm_word = with_armed_plane.then(|| {
        let tag = unique_ring_tag(&format!("arm{tag}"));
        let arm = cerulion_core::state_arm::MappedStateArm::create_owned(&tag)
            .expect("create the capture plane's arm word");
        arm.arm(ARM_CADENCE_STEPS, ARM_FIRST_ANCHOR_STEP);
        cfg.state_ring_discovery_tag = Some(tag);
        arm
    });
    cfg.flashback = Some(settings(&dir, trace_max_bytes));

    // The tap is OPEN-ONLY, so the producer must exist before `Recorder::setup`.
    let mut pub_ = publisher(&mgr, &topic, 256);

    // The STATE ring — always present, so the capture has an anchor to trim to.
    let state_tag = unique_ring_tag(&format!("st{tag}"));
    let mut state_owner = StateRingOwner::create(&state_tag, RING_RECORDS, 0, RUN, &NODES)
        .expect("create the state ring");
    cfg.state_rings = vec![state_owner.name().to_string()];
    // REAL extra rings, created and held to the end of
    // this function exactly as the primary one is. Declaring a name with no ring
    // behind it would exercise the recorder's degraded-open path instead of the
    // multi-ring one, which is a different arm.
    let _extra_state_owners: Vec<StateRingOwner> = (0..extra_state_rings)
        .map(|i| {
            let extra_tag = unique_ring_tag(&format!("st{tag}x{i}"));
            let owner = StateRingOwner::create(&extra_tag, RING_RECORDS, 0, RUN, &NODES)
                .expect("create an extra state ring");
            cfg.state_rings.push(owner.name().to_string());
            owner
        })
        .collect();
    // MINTED ONCE and held: a ring is SPSC, so a second `producer()` answers
    // `None`. It is also the RENDEZVOUS below — the producer is the only handle
    // in this process that can see the consumer's cursor without becoming a
    // second consumer.
    let mut state_producer = state_owner.producer().expect("the single state producer");

    // The TRACE ring — optional, because ring ABSENCE is one of the arms.
    let mut trace_owner = if with_trace_ring {
        let trace_tag = unique_ring_tag(&format!("tr{tag}"));
        let owner =
            cerulion_core::trace_ring::TraceRingOwner::create(&trace_tag, RING_RECORDS, 0, &NODES)
                .expect("create the trace ring");
        cfg.rings = vec![owner.name().to_string()];
        Some(owner)
    } else {
        None
    };
    // The supervisor's DEPARTURE ring: header rank
    // `DEPARTURE_RING_RANK`, EMPTY node manifest (a departure record's
    // `node_idx` carries a worker RANK, not a manifest index). Created and HELD
    // to the end of this function exactly as the worker ring is: declaring a
    // name with no ring behind it would exercise the degraded-open path, which
    // is a different arm.
    let _departure_owner = with_departure_ring.then(|| {
        let dep_tag = unique_ring_tag(&format!("dep{tag}"));
        let owner = cerulion_core::trace_ring::TraceRingOwner::create(
            &dep_tag,
            1024,
            cerulion_core::trace_ring::DEPARTURE_RING_RANK,
            &[],
        )
        .expect("create the departure ring");
        cfg.rings.push(owner.name().to_string());
        owner
    });
    let mut trace_producer = trace_owner
        .as_mut()
        .map(|o| o.producer().expect("the single trace producer"));

    let shutdown = Arc::new(AtomicBool::new(false));
    let rec_mgr = Arc::clone(&mgr);
    let rec_shutdown = Arc::clone(&shutdown);
    let handle = std::thread::spawn(move || run_bagd(rec_mgr, cfg, rec_shutdown));
    // RENDEZVOUS RULE 1 — nothing below may run before the recorder's ready-file
    // appears. See the module docs; this is the handshake, and
    // the arms it protects are the ones with no drain to wait on.
    await_bagd_ready(&ready, &format!("the '{tag}' flashback-trace capture"));

    // THE PRE-WINDOW, in the order the drain observes it: the trace FIRST, so
    // the pass that consumes the state ring has already consumed the trace one
    // (`write_batch` drains the trace rings at Phase A and the state rings
    // after) — which is what makes the state rendezvous below sound for BOTH.
    if let Some(p) = trace_producer.as_mut() {
        push_trace(p, trace);
    }
    for record in state {
        // The mint is the CALLER's because a ring is SPSC and `producer()` is
        // mint-once — a helper that minted its own would work exactly once and
        // then silently push nothing.
        let mut buf = [0u8; STATE_RECORD_SIZE as usize];
        buf.copy_from_slice(record);
        state_producer.push_record(&buf);
    }
    for seq in 0..PRE_WINDOW_FRAMES {
        pub_.publish_raw(&build_frame(HASH, seq, 1_000 + u64::from(seq), b"pre"))
            .expect("publish");
        std::thread::sleep(Duration::from_millis(10));
    }

    // The recorder must have DRAINED the rings before the capture is triggered —
    // a capture that fired first would be trace-less for a reason that has
    // nothing to do with the code under test, and every arm below would be
    // measuring the harness. Waited on as a CONDITION (the records leaving the
    // ring, read off the producer's view of the consumer cursor), never as a
    // sleep, under a liveness ceiling load can delay but not invert.
    if !state.is_empty() {
        assert!(
            await_condition(DEADLINE, || {
                state_producer.free_records() == Some(u64::from(RING_RECORDS))
            }),
            "the recorder must drain the state ring — nothing else in this process consumes it"
        );
    }

    let requester = FlashbackRequester::open_on_manager(&mgr).expect("requester");
    let request_id = requester
        .request(&CaptureRequest::manual("an operator saw the wobble"))
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
        "the recorder must report the capture FINISHED"
    );

    shutdown.store(true, Ordering::Relaxed);
    // BOUNDED join: a recorder that never notices its shutdown flag would HANG
    // the whole binary, burning the CI job timeout with no attributable red.
    let summary = join_bagd(
        handle,
        &shutdown,
        &format!("the '{tag}' flashback-trace capture"),
    )
    .expect("recorder");
    std::fs::remove_file(&ready).ok();
    let continuous = summary.bag_paths.first().cloned();
    assert!(
        continuous.is_some(),
        "a `--record` recorder writes a continuous bag — the mode has to be REAL, or the \
         writer-thread feed under test is not the thing being driven"
    );

    let bags = captures(&dir);
    assert_eq!(bags.len(), 1, "exactly one capture, got {bags:?}");
    let bag = bags[0].clone();
    Harness {
        dir,
        bag,
        topic,
        continuous,
    }
}

/// Scheduler-trace records the manifest says this capture CARRIES.
///
/// Read out of the handoff's `trace` verdict, which is the ONE
/// place the manifest states this number — there is no top-level
/// `trace_records` key. A cause string that is not a
/// carried count (either of the two `none:` causes) reads 0, which is what those
/// causes mean.
fn carried_records(m: &serde_json::Value) -> u64 {
    let verdict = m["handoff"]["trace"].as_str().expect("a trace verdict");
    verdict
        .strip_prefix("carried: ")
        .and_then(|rest| rest.split(' ').next())
        .and_then(|n| n.parse().ok())
        .unwrap_or(0)
}

// ===========================================================================
// THE ACCEPTANCE ARM
// ===========================================================================

/// THE HEADLINE: a capture on a `--record` run carries a trace TRIMMED to its
/// own anchor, byte-identical to the records the ring was handed, and its
/// manifest says `resimmable: true` with a MEASURED count.
///
/// Three independent claims, each against a HAND oracle rather than against the
/// code's own output:
///
/// 1. the kept SET is exactly `[boundary(S+1), end]` — `expected_kept()`;
/// 2. the kept BYTES are the ring's own 40-byte wire form, so a resume reads a
///    capture through the path it already reads a recording through;
/// 3. `resolve_resume`'s own derivation (`first_boundary.step − 1`) lands back on
///    the anchor step the capture embedded, which is the whole point of the trim.
#[test]
fn a_capture_carries_its_trace_trimmed_to_its_anchor_and_reads_resimmable() {
    let h = capture(
        "headline",
        &trace_across_the_anchor(),
        &full_anchor(),
        64 * 1024 * 1024,
        true,
    );

    let payloads = trace_payloads(&h.bag);
    let oracle = expected_kept();
    assert_eq!(
        payloads.len(),
        oracle.len(),
        "the capture must carry exactly the records at or after boundary({}) — got {} of {}",
        ANCHOR_STEP + 1,
        payloads.len(),
        oracle.len()
    );

    // (2) BYTE identity against the hand-built wire form.
    for (i, (got, want)) in payloads.iter().zip(oracle.iter()).enumerate() {
        assert_eq!(
            got.len(),
            TRACE_RECORD_SIZE as usize,
            "record {i} is not one 40-byte trace record"
        );
        assert_eq!(
            got.as_slice(),
            want.as_bytes().as_slice(),
            "record {i} is not byte-identical to what the ring was handed"
        );
    }

    // (1)+(3) the decoded shape, and resim's own derivation.
    let decoded: Vec<TraceRingRecord> = payloads
        .iter()
        .map(|p| TraceRingRecord::from_bytes(p.as_slice().try_into().expect("40 bytes")))
        .collect();
    assert!(
        decoded.iter().all(|r| r.step > ANCHOR_STEP),
        "nothing at or before the anchor step may survive the trim: {:?}",
        decoded.iter().map(|r| r.step).collect::<Vec<_>>()
    );
    let first_boundary = decoded
        .iter()
        .find(|r| r.record_type == RECORD_TYPE_STEP_BOUNDARY)
        .expect("the trimmed trace must begin with a step boundary");
    assert_eq!(
        first_boundary.step,
        ANCHOR_STEP + 1,
        "the FIRST boundary is what `resolve_resume` reads"
    );
    assert_eq!(
        first_boundary.step - 1,
        ANCHOR_STEP,
        "…and its derivation must land back on the anchor this capture embedded"
    );

    // The manifest's own account of it.
    let m = flashback_manifest(&h.bag);
    assert_eq!(
        m["anchor"]["embedded"],
        serde_json::json!(true),
        "precondition: the capture really carries state"
    );
    assert_eq!(m["anchor"]["step"], serde_json::json!(ANCHOR_STEP));
    assert_eq!(
        m["handoff"]["trace"],
        serde_json::json!(format!(
            "carried: {} scheduler-trace record(s)",
            oracle.len()
        )),
        "the count is MEASURED, and it is the number of records the bag really holds — \
         stated in the handoff's own vocabulary, the one place this manifest says it: {m}"
    );
    // The RECORDER's own drained count, on the mode whose
    // reader is the WRITER THREAD.
    //
    // This reported a flat `0` for the whole `--record` mode — beside the
    // `carried: N` verdict two assertions up, i.e. one manifest contradicting
    // itself about one run — because the field read only the window-only drain,
    // which does not exist here (`spawn_trace_drain` returns unless
    // `window_only()`). Bounded on BOTH sides rather than pinned to an exact
    // value: the writer stores its live counter immediately after the batch that
    // banks the retention, so an exact equality would be a race, while
    // "at least what the bag carries, at most what the ring was handed" is
    // load-proof and still fails a `0`, a fabricated number, and a double count.
    let drained = m["handoff"]["trace_ring_records"]
        .as_u64()
        .expect("the handoff states how many records this recorder drained");
    assert!(
        drained >= oracle.len() as u64 && drained <= trace_across_the_anchor().len() as u64,
        "a `--record` recorder's reader is its WRITER THREAD: it drained at least the {} \
         record(s) this bag carries and at most the {} it was handed, got {drained}: {m}",
        oracle.len(),
        trace_across_the_anchor().len()
    );
    assert_eq!(
        m["anchor"]["resimmable"],
        serde_json::json!(true),
        "trace + complete anchor + one ring + no departure = resimmable: {m}"
    );
    let reason = m["anchor"]["resimmable_reason"]
        .as_str()
        .expect("a verdict states its reason");
    assert!(
        reason.contains("--resim"),
        "the positive reason tells the operator what to RUN: {reason}"
    );
    // `target(S−1)`, recovered from the boundary the trim discards — the value
    // `replay_engine` says no arithmetic on its side can compute.
    assert_eq!(
        m["anchor_target_ns"],
        serde_json::json!(boundary(ANCHOR_STEP).fire_time_ns),
        "the capture must carry the ANCHOR step's own target, not the resume step's: {m}"
    );
    // The range's UPPER endpoint, measured over the records this bag
    // really holds. A capture's frames are staged by the DRIVE LOOP while its
    // trace is banked by the WRITER THREAD, so the frames can run past the last
    // boundary the bag carries; `bag play --resim` reads this to know how far a
    // resume covers, and without it those tail frames were refused as corrupt.
    //
    // The oracle is the LAST boundary of the trim's own expected output, derived
    // from `expected_kept()` rather than restated — so a change to the fixture
    // moves both together, and the assertion cannot drift into agreeing with a
    // hardcoded number.
    let last_kept_boundary = decoded
        .iter()
        .filter(|r| r.record_type == RECORD_TYPE_STEP_BOUNDARY)
        .map(|r| r.fire_time_ns)
        .next_back()
        .expect("the trimmed trace carries at least one boundary");
    assert_eq!(
        m["anchor"]["resim_covered_through_ns"],
        serde_json::json!(last_kept_boundary),
        "the covered range ends at the LAST boundary the bag carries: {m}"
    );
    // …and it is NOT the anchor target. Both are gating-clock instants of one
    // recording — the range's two endpoints — so only asserting them apart can
    // tell a correct renderer from one that spliced the wrong endpoint.
    assert_ne!(
        m["anchor"]["resim_covered_through_ns"], m["anchor_target_ns"],
        "the range's two endpoints must be two different measurements: {m}"
    );
    assert!(
        reason.contains(&format!("{last_kept_boundary} ns")),
        "the operator-facing sentence names the same instant the field does: {reason}"
    );

    // THE MANIFESTS. A trace whose records name rank 0 is unreplayable without
    // `trace_manifest_rank0.json`: `load_rank_tables` refuses a bag with zero
    // manifests outright, and bounds-checks every FIRE record's `node_idx`
    // against its OWN rank's table. So a capture that carried the trace and not
    // these would read `resimmable: true` and then be refused at exit 2.
    let manifests = trace_manifests(&h.bag);
    assert_eq!(
        manifests.len(),
        1,
        "one ring, one manifest — got {:?}",
        manifests.iter().map(|(n, _)| n).collect::<Vec<_>>()
    );
    let (name, manifest) = &manifests[0];
    assert_eq!(name, "__cerulion/trace_manifest_rank0.json");
    assert_eq!(manifest["rank"], serde_json::json!(0));
    assert_eq!(
        manifest["node_ids"],
        serde_json::json!(NODES),
        "the manifest must name the ring's OWN nodes, so a FIRE record's \
         node_idx resolves: {manifest}"
    );
    // …and it is BYTE-IDENTICAL to the one the recording beside it carries,
    // which is the property one shared renderer buys: a capture and its
    // recording cannot describe the same ring differently.
    let continuous_manifests = trace_manifests(&h.continuous.clone().expect("a continuous bag"));
    assert_eq!(
        manifests, continuous_manifests,
        "the capture and the recording must describe the ring identically"
    );

    // The continuous bag still holds the WHOLE trace — the capture's trim takes
    // nothing away from the recording beside it.
    let continuous = h.continuous.clone().expect("a continuous bag");
    assert_eq!(
        trace_payloads(&continuous).len(),
        trace_across_the_anchor().len(),
        "the recording keeps every record; only the CAPTURE is trimmed"
    );

    h.cleanup();
}

// ===========================================================================
// The NEGATIVE twins
// ===========================================================================

/// A capture with a healthy trace and a complete anchor but NO `graph.yaml` is
/// NOT resimmable, and says which attachment is missing.
///
/// `bag play --resim` requires that attachment (`run_replay` gate 4,
/// `BagMissingAttachment` -> exit 2). A verdict that does not consult it
/// publishes a bag resim refuses outright as `resimmable: true` — the
/// confident-false class the shared judge exists to prevent.
///
/// REACHABLE, and not hypothetically: `cerulion bagd` takes `--attach` and
/// `--ring` INDEPENDENTLY, so a standalone recorder with trace rings and no
/// attachments is an ordinary invocation. A harness whose every arm runs
/// against a recorder with no `graph.yaml` cannot see the gap, which is why
/// the other arms here attach one.
///
/// The stimulus is the HEADLINE arm's, minus the attachment, so the ONLY
/// difference between `resimmable: true` there and `false` here is the graph.
#[test]
fn a_capture_with_no_graph_attachment_reads_not_resimmable_naming_the_missing_graph() {
    let h = capture_inner(
        "nograph",
        &trace_across_the_anchor(),
        &full_anchor(),
        64 * 1024 * 1024,
        true,
        None,
        false,
    );
    let m = flashback_manifest(&h.bag);
    assert_eq!(
        m["anchor"]["resimmable"],
        serde_json::json!(false),
        "a bag `bag play --resim` refuses at its graph gate must not claim resimmable: {m}"
    );
    let reason = m["anchor"]["resimmable_reason"]
        .as_str()
        .unwrap_or_default()
        .to_string();
    assert!(
        reason.contains("graph.yaml"),
        "the reason must NAME the attachment, so an operator knows what to add: {reason}"
    );
    // The trace and the anchor really are healthy — without this the arm would
    // also pass on a capture that failed for one of the other five reasons.
    assert_eq!(
        m["anchor"]["embedded"],
        serde_json::json!(true),
        "the anchor is present, so `graph.yaml` is the ONLY thing missing: {m}"
    );
    assert!(
        !trace_payloads(&h.bag).is_empty(),
        "the trace is present too: {m}"
    );
    h.cleanup();
}

/// The two fixture graphs are what their names say — pinned once, so an edit to
/// either cannot silently turn a positive arm into a negative one (or the
/// reverse), which is exactly how the `nodes: []` graph went unnoticed.
#[test]
fn the_fixture_graphs_have_the_validity_their_arms_depend_on() {
    let parse = |b: &[u8]| {
        cerulion_core::graph::parse_graph(std::str::from_utf8(b).expect("utf-8"))
            .expect("both fixture graphs must PARSE — that is what makes them discriminating")
    };
    assert!(
        cerulion_core::graph::validate_graph(&parse(GRAPH_YAML)).is_ok(),
        "the POSITIVE fixture must validate, or every resimmable arm here is testing a graph \
         `run_replay` refuses"
    );
    assert!(
        cerulion_core::graph::validate_graph(&parse(GRAPH_YAML_PARSES_BUT_INVALID)).is_err(),
        "the NEGATIVE fixture must fail validation, or its arm tests the same thing as the \
         parse case"
    );
}

/// **A graph that PARSES but fails VALIDATION is not a usable graph, and the
/// verdict must say so.**
///
/// The sibling arm above covers a graph that is ABSENT. This one covers a graph
/// that is present and unusable — the case a presence check and a parse check
/// both wave through. `run_replay` runs UTF-8 -> parse -> `validate_graph`
/// (`replay_cmd.rs:553-583`), each failing as `BagInvalidAttachment` -> exit 2,
/// so a capture whose predicate stopped at `parse_graph` published
/// `resimmable: true` against a bag replay refuses before loading anything.
///
/// The stimulus is the HEADLINE arm's, with ONLY the graph bytes swapped, so the
/// difference between `true` there and `false` here is validation and nothing
/// else — which is also why the harness carries the BYTES rather than a bool: a
/// bool cannot express "present and unusable" at all.
#[test]
fn a_capture_whose_graph_parses_but_fails_validation_reads_not_resimmable() {
    let h = capture_inner(
        "badgraph",
        &trace_across_the_anchor(),
        &full_anchor(),
        64 * 1024 * 1024,
        true,
        Some(GRAPH_YAML_PARSES_BUT_INVALID),
        false,
    );
    let m = flashback_manifest(&h.bag);
    assert_eq!(
        m["anchor"]["resimmable"],
        serde_json::json!(false),
        "a graph `run_replay` refuses at its validation step is not a usable graph: {m}"
    );
    let reason = m["anchor"]["resimmable_reason"]
        .as_str()
        .unwrap_or_default()
        .to_string();
    assert!(
        reason.contains("graph.yaml"),
        "the reason must NAME the attachment, as the absent-graph arm does: {reason}"
    );
    // The trace and the anchor are healthy, so the graph is the ONLY fault —
    // without this the arm would also pass on a capture refused for one of the
    // other reasons.
    assert_eq!(
        m["anchor"]["embedded"],
        serde_json::json!(true),
        "the anchor is present: {m}"
    );
    assert!(
        !trace_payloads(&h.bag).is_empty(),
        "the trace is present too: {m}"
    );
    // The fixture's own precondition, so a graph that stopped parsing would fail
    // as a harness failure rather than passing this arm for the wrong reason.
    let yaml = std::str::from_utf8(GRAPH_YAML_PARSES_BUT_INVALID).expect("utf-8");
    let cfg = cerulion_core::graph::parse_graph(yaml).expect("the fixture graph must PARSE");
    assert!(
        cerulion_core::graph::validate_graph(&cfg).is_err(),
        "…and must fail VALIDATION, or this arm tests the same thing as the parse case"
    );
    h.cleanup();
}

/// **A trace record `bag play --resim` refuses makes the capture NOT
/// resimmable: record TYPES.**
///
/// The judge counted records and derived nodes but never asked whether the
/// records were ones the replay gate ACCEPTS. `run_replay` walks every trace
/// record and refuses a reserved/unknown `record_type` outright
/// (`replay_cmd.rs` 628-722, `TraceRecordUnsupported` -> exit 2), so a trace
/// written by a NEWER Cerulion — the shape this arm models, `record_type` 7 —
/// was stamped `resimmable: true` and then refused. That is the confident-false
/// class, and it is the likeliest of the four to be met in the field: it is what
/// a forward-compatible writer produces.
///
/// The bad record sits PAST the anchor so it survives the trim and really
/// reaches the bag; the rest of the trace is the healthy fixture, so the record
/// kind is the only difference from the resimmable headline.
#[test]
fn a_capture_carrying_an_unsupported_trace_record_type_reads_not_resimmable() {
    let mut trace = trace_across_the_anchor();
    let mut alien = fire(ANCHOR_STEP + 2, 0);
    alien.record_type = 7;
    trace.push(alien);

    let h = capture("aliankind", &trace, &full_anchor(), 64 * 1024 * 1024, true);
    let m = flashback_manifest(&h.bag);
    assert_eq!(
        m["anchor"]["resimmable"],
        serde_json::json!(false),
        "a trace the gate refuses on a record kind cannot be resimmable: {m}"
    );
    let reason = m["anchor"]["resimmable_reason"]
        .as_str()
        .unwrap_or_default()
        .to_string();
    assert!(
        reason.contains("record_type 7"),
        "the reason must name the KIND, so an operator knows to upgrade rather than \
         re-record: {reason}"
    );
    // The trace and the anchor are otherwise healthy — without this the arm
    // would also pass on a capture refused for one of the other reasons.
    assert_eq!(
        m["anchor"]["embedded"],
        serde_json::json!(true),
        "the anchor is present: {m}"
    );
    assert!(
        !trace_payloads(&h.bag).is_empty(),
        "and the trace really reached the bag: {m}"
    );
    h.cleanup();
}

/// …and the same rule for a FIRE record whose `node_idx` is past its rank's
/// manifest.
///
/// `run_replay` bounds-checks every FIRE `node_idx` against that record's OWN
/// rank table and refuses with `BagInvalidAttachment` (exit 2). The fixture ring
/// declares [`NODES`] (2 ids), so index 2 is the first out-of-range one — a
/// corrupt or mis-demuxed record, and the fault that would otherwise send the
/// replay to a node that does not exist.
///
/// Separate from the record-kind arm deliberately: they are different gate arms
/// with different remedies (upgrade vs re-record), and one arm covering both
/// would pass with either check deleted.
#[test]
fn a_capture_carrying_a_fire_record_past_its_manifest_reads_not_resimmable() {
    let mut trace = trace_across_the_anchor();
    // NODES.len() is the first index the manifest does NOT list.
    trace.push(fire(ANCHOR_STEP + 2, NODES.len() as u32));

    let h = capture("badnodeidx", &trace, &full_anchor(), 64 * 1024 * 1024, true);
    let m = flashback_manifest(&h.bag);
    assert_eq!(
        m["anchor"]["resimmable"],
        serde_json::json!(false),
        "a FIRE record naming a node the manifest does not list cannot be resimmable: {m}"
    );
    let reason = m["anchor"]["resimmable_reason"]
        .as_str()
        .unwrap_or_default()
        .to_string();
    assert!(
        reason.contains(&format!("node_idx {}", NODES.len())),
        "the reason must name the offending index: {reason}"
    );
    assert!(
        reason.contains(&format!("only {} node id(s)", NODES.len())),
        "…and what the manifest actually lists, which is what makes it actionable: {reason}"
    );
    assert_eq!(
        m["anchor"]["embedded"],
        serde_json::json!(true),
        "the anchor is present: {m}"
    );
    h.cleanup();
}

/// A capture with NO anchor still carries the ARMED PLANE, so a replay of it
/// applies the catch-up clamp the recording ran under.
///
/// `state_coverage.json` must NOT hang off the anchor, because it carries TWO
/// independent things: the `node_idx -> node id` table (meaningless without an
/// anchor) and `armed.first_anchor_step`, which is where
/// `replay_state::read_state_arm` reads the catch-up clamp onset from. Tying
/// them together means a capture whose retention holds no checkpoint writes no
/// manifest at all — so a from-start replay of it runs UNCLAMPED, firing
/// `Period` catch-up bursts the recording never ran and reporting them as a
/// divergence the candidate did not cause.
///
/// The stimulus is a real armed plane and an EMPTY state ring, which is the
/// shape: the plane is always-on by design, so a capture taken before the
/// first anchor is due has an arm and no checkpoint.
#[test]
fn a_capture_with_no_anchor_still_carries_the_armed_plane_so_a_replay_can_clamp() {
    let h = capture_inner(
        "armnoanchor",
        &trace_across_the_anchor(),
        &[],
        64 * 1024 * 1024,
        true,
        Some(GRAPH_YAML),
        true,
    );
    let m = flashback_manifest(&h.bag);
    assert_eq!(
        m["anchor"]["embedded"],
        serde_json::json!(false),
        "PRECONDITION: this arm is about the NO-ANCHOR shape: {m}"
    );

    let reader = cerulion_bag::BagReader::open(&h.bag).expect("open the capture");
    let att = reader
        .attachment("__cerulion/state_coverage.json")
        .expect("read the coverage attachment")
        .expect(
            "a capture with an armed plane must carry state_coverage.json even with NO anchor —              it is the only place a replay can read the catch-up clamp's onset from",
        );
    let coverage: serde_json::Value =
        serde_json::from_slice(&att.data).expect("the coverage manifest is valid JSON");
    assert_eq!(
        coverage["armed"]["first_anchor_step"],
        serde_json::json!(ARM_FIRST_ANCHOR_STEP),
        "the ONSET must survive verbatim — a fabricated default would clamp from the wrong step: \
         {coverage}"
    );
    assert_eq!(
        coverage["armed"]["cadence_steps"],
        serde_json::json!(ARM_CADENCE_STEPS),
        "…and so must the cadence it was published with: {coverage}"
    );
    // NOTHING is claimed about a checkpoint that does not exist. Without this
    // the arm would also pass on a manifest that invented node coverage.
    assert!(
        coverage["nodes"].as_object().is_none_or(|n| n.is_empty()),
        "a capture with no anchor must claim no node coverage: {coverage}"
    );
    h.cleanup();
}

/// A DEPARTURE-carrying capture never claims resimmable, and names fault replay.
///
/// Until fault replay lands, such a bag is readable and not resumable, and the
/// manifest must say which.
#[test]
fn a_departure_carrying_capture_reads_not_resimmable_naming_fault_replay() {
    let mut trace = trace_across_the_anchor();
    // A peer died at a step INSIDE the kept window, so the trim carries it.
    trace.push(departure(ANCHOR_STEP + 2));
    let h = capture("departure", &trace, &full_anchor(), 64 * 1024 * 1024, true);

    let m = flashback_manifest(&h.bag);
    assert_eq!(
        m["anchor"]["embedded"],
        serde_json::json!(true),
        "the state is still there — this is a readable bag, not a broken one"
    );
    assert!(carried_records(&m) > 0, "…and so is the trace: {m}");
    assert_eq!(
        m["anchor"]["resimmable"],
        serde_json::json!(false),
        "a fault-degraded recording is not resumable: {m}"
    );
    let reason = m["anchor"]["resimmable_reason"]
        .as_str()
        .expect("a refusal states its reason");
    assert!(
        reason.contains("fault-degraded recording is not supported yet"),
        "naming fault replay: {reason}"
    );
    assert!(
        reason.contains("readable"),
        "…and saying the evidence is still there: {reason}"
    );

    h.cleanup();
}

/// **On a trace carrying BOTH, the gap a capture names is the FIRST refusing
/// record's, not a fixed precedence between the two kinds.**
///
/// `bag play --resim` walks the trace ONCE and refuses per record: the departure
/// gate, then the four record-level faults. So a bag whose alien record sits
/// BEFORE its departure is refused at the alien record with
/// `TraceRecordUnsupported`, while the capture judged an aggregate departure
/// COUNT first and advertised fault replay — two different named gaps for one
/// bag, which is the drift this shared verdict exists to prevent. An operator
/// reading "wait for fault replay" would be waiting for a feature that
/// was never what stopped them.
///
/// Both orders are driven from the SAME pair of records, so an implementation
/// that always names one kind fails whichever direction it got wrong, and each
/// arm asserts the OTHER gap is ABSENT rather than only that its own is present.
#[test]
fn the_gap_a_capture_names_is_the_first_refusing_records_in_file_order() {
    // ARM 1 — the fault comes FIRST. Both records sit past the anchor so the
    // trim carries them, and they are pushed in file order.
    let mut fault_first = trace_across_the_anchor();
    let mut alien = fire(ANCHOR_STEP + 2, 0);
    alien.record_type = 7;
    fault_first.push(alien);
    fault_first.push(departure(ANCHOR_STEP + 3));

    let h = capture(
        "faultfirst",
        &fault_first,
        &full_anchor(),
        64 * 1024 * 1024,
        true,
    );
    let m = flashback_manifest(&h.bag);
    assert_eq!(
        m["anchor"]["resimmable"],
        serde_json::json!(false),
        "a trace the gate refuses cannot be resimmable whichever record it refuses at: {m}"
    );
    let reason = m["anchor"]["resimmable_reason"]
        .as_str()
        .expect("a refusal states its reason")
        .to_string();
    assert!(
        reason.contains("record_type 7"),
        "the gate refuses at the ALIEN record, which comes first — so that is the gap: {reason}"
    );
    assert!(
        !reason.contains("fault-degraded"),
        "…and NOT fault replay, which is a later record's refusal: {reason}"
    );
    h.cleanup();

    // ARM 2 — the SAME two records, swapped. Now the departure comes first, so
    // fault replay IS the gap; without this half, "name the fault" would pass a
    // capture that never reports a departure at all.
    let mut departure_first = trace_across_the_anchor();
    departure_first.push(departure(ANCHOR_STEP + 2));
    let mut later_alien = fire(ANCHOR_STEP + 3, 0);
    later_alien.record_type = 7;
    departure_first.push(later_alien);

    let h2 = capture(
        "depfirst",
        &departure_first,
        &full_anchor(),
        64 * 1024 * 1024,
        true,
    );
    let m2 = flashback_manifest(&h2.bag);
    assert_eq!(
        m2["anchor"]["resimmable"],
        serde_json::json!(false),
        "still not resimmable: {m2}"
    );
    let reason2 = m2["anchor"]["resimmable_reason"]
        .as_str()
        .expect("a refusal states its reason")
        .to_string();
    assert!(
        reason2.contains("fault-degraded recording is not supported yet"),
        "the departure comes first, so fault replay is the gap: {reason2}"
    );
    assert!(
        !reason2.contains("record_type 7"),
        "…and NOT the later alien record: {reason2}"
    );
    h2.cleanup();
}

/// NO ANCHOR IN WINDOW: the capture is accurate rather than optimistic.
///
/// The state ring is present but pushes NOTHING, so there is no checkpoint to
/// resume from. The trace is still carried (a black box keeps its evidence) and
/// the verdict is `false`.
#[test]
fn a_capture_with_no_anchor_in_window_reads_not_resimmable_honestly() {
    let h = capture(
        "noanchor",
        &trace_across_the_anchor(),
        &[],
        64 * 1024 * 1024,
        true,
    );

    let m = flashback_manifest(&h.bag);
    assert_eq!(m["anchor"]["embedded"], serde_json::json!(false));
    assert_eq!(
        m["anchor"]["reason"],
        serde_json::json!("no_anchor_retained"),
        "a reader that cannot tell WHICH absence this is cannot act on either"
    );
    assert_eq!(
        m["anchor"]["resimmable"],
        serde_json::json!(false),
        "state is half of a resume, and this bag has none: {m}"
    );
    // The evidence is STILL in the bag — this is the frames-only degradation,
    // and it is still a black box.
    let reader = cerulion_bag::BagReader::open(&h.bag).expect("open");
    let (msgs, _) = reader.recover_messages().expect("recover");
    // IDENTITIES, not a count — and the difference is load-bearing twice over.
    //
    // `any` (a weaker oracle) is satisfied by a capture that lost frames off
    // its HEAD, which is exactly what a tap attached late produces, so it reports
    // a healthy black box over a truncated one. Measured under load: without the
    // rendezvous 7 of 8 carried on 8 of 8 runs; with it, 8 of 8.
    //
    // A COUNT closes that hole and leaves another: eight messages is not the same
    // claim as THESE eight. A recorder that dropped one frame and duplicated
    // another still counts eight, and this arm would report every frame carried
    // while one of them was gone. So the oracle is the SEQUENCE IDENTITIES in bag
    // order — which pins count, order and uniqueness in one comparison — and each
    // frame is additionally required to be BYTE-IDENTICAL to what was published.
    //
    // Nothing legitimate can trim or reorder these (see `PRE_WINDOW_FRAMES`), so
    // any deviation is loss, duplication or corruption, and this arm now says
    // WHICH.
    let carried: Vec<&cerulion_bag::BagMessage> =
        msgs.iter().filter(|m| m.topic == h.topic).collect();
    let got_seqs: Vec<u32> = carried
        .iter()
        .map(|m| {
            cerulion_core::wire::WireHeader::read_from_buf(&m.data)
                .unwrap_or_else(|| panic!("a carried frame has no parseable wire header"))
                .sequence
        })
        .collect();
    let want_seqs: Vec<u32> = (0..PRE_WINDOW_FRAMES).collect();
    assert_eq!(
        got_seqs, want_seqs,
        "a capture with no anchor still carries its frames — ALL of them, each exactly once, in \
         order. A SHORT run means frames were published into a tap that did not exist yet (the \
         recorder's ready-file rendezvous); a run of the right LENGTH that is not this one means \
         the capture dropped a frame and repeated another, which no window trim can produce"
    );
    for (m, seq) in carried.iter().zip(&want_seqs) {
        assert_eq!(
            m.data,
            build_frame(HASH, *seq, 1_000 + u64::from(*seq), b"pre"),
            "the carried frame for wire sequence {seq} is not byte-identical to what was published"
        );
    }
    assert!(
        carried_records(&m) > 0,
        "…and its trace: a black box does not discard evidence it cannot resume from: {m}"
    );

    h.cleanup();
}

/// A NO-ANCHOR capture is still judged against the ring count its own manifest
/// declares.
///
/// The no-anchor arm handed the verdict a hardcoded `0` while writing coverage
/// carrying `state_rings.len()`, so the capture's two answers disagreed on the
/// ordinary multi-process shape: the bag said `rings_declared: 2` and the judge
/// was told `0`, skipped the `MultiRingAmbiguous` arm, and reported an
/// ANCHOR-shaped reason — against a replay that refuses at `read_bag_anchors` on
/// the very count in the bag.
///
/// Two REAL state rings, neither pushed, so the anchor is genuinely absent and
/// the arm under test is the one that runs.
#[test]
fn a_no_anchor_capture_is_judged_against_the_ring_count_its_manifest_declares() {
    let h = capture_with_extra_state_rings("noanchormulti", &trace_across_the_anchor(), &[], 1);

    let m = flashback_manifest(&h.bag);
    // PRECONDITIONS: this really is the no-anchor arm, and the manifest really
    // does declare more than one ring — without both, the assertion below could
    // pass for reasons unrelated to the ring count.
    assert_eq!(
        m["anchor"]["embedded"],
        serde_json::json!(false),
        "PRECONDITION: the no-anchor arm must be the one under test: {m}"
    );
    // The ring COUNT is not in this document — `rings_declared` rides
    // `state_coverage.json`, which is written only when an armed plane was
    // observed, and this harness stands one up for no arm. What can be pinned
    // here is what the recorder was CONFIGURED with (two REAL rings, created by
    // `capture_with_extra_state_rings`) and what the verdict then says about
    // them, which is the whole of what this arm pins.
    assert_eq!(
        m["handoff"]["trace_rings_configured"],
        serde_json::json!(1),
        "PRECONDITION: exactly one TRACE ring, so the node map is resolvable and \
         the verdict cannot be reporting the node-map gap instead: {m}"
    );
    assert_eq!(m["anchor"]["resimmable"], serde_json::json!(false));

    let reason = m["anchor"]["resimmable_reason"]
        .as_str()
        .expect("a refusal states its reason");
    assert!(
        reason.contains("state rings") && reason.contains("no rank"),
        "the verdict must report the RING ambiguity the bag declares — the gap resim reaches \
         first — rather than an anchor-shaped reason: {reason}"
    );

    h.cleanup();
}

/// RING ABSENCE is ORDINARY: a run holding NO trace ring still produces a
/// frames-only capture that reads `resimmable: false`, never a capture FAILURE.
///
/// This is the arm that stops the new machinery from turning a missing ring into
/// an error. It is also the shape a plain `graph run` had before rings shipped
/// by default.
#[test]
fn a_run_with_no_trace_ring_still_captures_and_says_it_is_not_resimmable() {
    let h = capture("noring", &[], &full_anchor(), 64 * 1024 * 1024, false);

    let m = flashback_manifest(&h.bag);
    assert_eq!(carried_records(&m), 0, "no ring, no trace: {m}");
    // …and the manifest says WHICH absence it is. The two causes must never be
    // spelled as each other, so this arm pins the NO-RINGS one by the fact that
    // separates them.
    assert_eq!(
        m["handoff"]["trace_rings_configured"],
        serde_json::json!(0),
        "the no-ring cause is the one with no rings: {m}"
    );
    assert!(
        m["handoff"]["trace"]
            .as_str()
            .expect("a cause")
            .contains("handed no trace ring"),
        "and it is reported as such rather than as its sibling: {m}"
    );
    assert!(
        trace_payloads(&h.bag).is_empty(),
        "…which the bag itself agrees with"
    );
    assert_eq!(
        m["anchor"]["embedded"],
        serde_json::json!(true),
        "the STATE half still landed — ring absence costs the trace, not the anchor"
    );
    assert_eq!(
        m["anchor"]["resimmable"],
        serde_json::json!(false),
        "state without a trace is not resumable: {m}"
    );
    let reason = m["anchor"]["resimmable_reason"]
        .as_str()
        .expect("a refusal states its reason");
    // The actionable half is WHERE the run stated what it did about
    // its rings, not a tracker id — rings ship by default now, so a run that has
    // none said so, and its `run.json` is where it said it.
    assert!(
        reason.contains("run.json"),
        "naming where the run stated what it did about its rings, so the reason is \
         actionable: {reason}"
    );
    // No ring, no manifest — the ANTI-TAUTOLOGY half of the headline's
    // presence assertion, which would otherwise be satisfied by a recorder that
    // always wrote one.
    assert!(
        trace_manifests(&h.bag).is_empty(),
        "a run with no trace ring has no rank to describe"
    );

    // The FRAMES are there — this is a black box, just not a resumable one.
    let reader = cerulion_bag::BagReader::open(&h.bag).expect("open");
    let (msgs, _) = reader.recover_messages().expect("recover");
    assert!(
        msgs.iter().any(|m| m.topic == h.topic),
        "a capture with no trace ring still carries its frames"
    );

    h.cleanup();
}

/// A trace whose window holds only steps AT OR BEFORE the anchor is trimmed to
/// NOTHING, and the capture says so rather than shipping a trace that begins
/// mid-step.
///
/// The adversarial shape the memo names: a trigger that lands before any boundary
/// past the anchor.
#[test]
fn a_trace_with_no_boundary_past_the_anchor_is_trimmed_away_and_reads_not_resimmable() {
    // Everything at or BELOW the anchor step.
    let trace: Vec<TraceRingRecord> = trace_across_the_anchor()
        .into_iter()
        .filter(|r| r.step <= ANCHOR_STEP)
        .collect();
    assert!(!trace.is_empty(), "the arm must really push records");

    let h = capture("noboundary", &trace, &full_anchor(), 64 * 1024 * 1024, true);

    let m = flashback_manifest(&h.bag);
    assert_eq!(
        carried_records(&m),
        0,
        "nothing after the anchor survives the trim: {m}"
    );
    // …which the bag itself agrees with, the same second half the no-ring arm
    // carries: without it these arms pin only what the MANIFEST says, and a
    // capture that wrote trace records while reporting none would pass.
    assert!(
        trace_payloads(&h.bag).is_empty(),
        "the bag carries no trace channel either"
    );
    // The OTHER cause: rings were handed over and the capture still carries
    // nothing. Pinned apart from the no-ring arm above, which is the distinction
    // this closed vocabulary exists to keep.
    assert!(
        m["handoff"]["trace_rings_configured"]
            .as_u64()
            .expect("a count")
            > 0,
        "precondition: this recorder really was handed a ring: {m}"
    );
    let cause = m["handoff"]["trace"].as_str().expect("a cause");
    assert!(
        !cause.contains("handed no trace ring"),
        "a ring WAS handed over, so the cause is not the no-ring one: {m}"
    );
    // And it is the TRIM-specific cause, not the
    // generic one. This arm's stimulus IS a trim that discarded every retained
    // record, so before the three-way split it was reported under a sentence
    // asserting "either the trim … or the … ceiling" — an exhaustive-sounding
    // disjunction that named the right cause only by luck. Naming it exactly is
    // the difference between an operator widening a window and raising a
    // ceiling that was never the problem.
    assert!(
        cause.contains("belonged to a step before this capture's anchor"),
        "the trim discarded every record, so the cause must SAY so rather than \
         offer a disjunction: {m}"
    );
    assert_eq!(
        m["anchor"]["resimmable"],
        serde_json::json!(false),
        "a trace trimmed to nothing cannot drive a resume: {m}"
    );
    // …and `target(S−1)` was STILL recovered from the boundary the trim
    // discarded, which is the one thing that survives an empty trim.
    assert_eq!(
        m["anchor_target_ns"],
        serde_json::json!(boundary(ANCHOR_STEP).fire_time_ns),
        "{m}"
    );

    h.cleanup();
}

/// The BYTE CEILING: past it the oldest trace records go, and a capture that
/// loses its own resume boundary reports itself NOT resimmable rather than
/// shipping a trace that begins in the middle of a step.
///
/// The ceiling is injected because this arm is ABOUT the ceiling. It is set below
/// one batch, so nothing can be retained at all.
#[test]
fn a_trace_that_does_not_fit_the_ceiling_degrades_to_frames_and_state_only() {
    let h = capture(
        "ceiling",
        &trace_across_the_anchor(),
        &full_anchor(),
        // Below one record, so the retention can hold nothing.
        1,
        true,
    );

    let m = flashback_manifest(&h.bag);
    assert_eq!(
        carried_records(&m),
        0,
        "past the ceiling a capture carries no trace — the degradation, not a silent cost: {m}"
    );
    // …which the bag itself agrees with, the same second half the no-ring arm
    // carries: without it these arms pin only what the MANIFEST says, and a
    // capture that wrote trace records while reporting none would pass.
    assert!(
        trace_payloads(&h.bag).is_empty(),
        "the bag carries no trace channel either"
    );
    assert_eq!(
        m["anchor"]["embedded"],
        serde_json::json!(true),
        "the anchor is unaffected: the two retentions have their own ceilings"
    );
    assert_eq!(m["anchor"]["resimmable"], serde_json::json!(false), "{m}");

    h.cleanup();
}

/// DETERMINISM: two captures of the identical stimulus carry byte-identical
/// traces (Principle #7 where it applies — the records are the recording's own
/// bytes, so nothing about the run's timing may enter them).
///
/// Compared against the HAND oracle as well, so this is not two runs of one
/// closure agreeing with each other.
#[test]
fn two_captures_of_one_stimulus_carry_byte_identical_traces() {
    let a = capture(
        "det_a",
        &trace_across_the_anchor(),
        &full_anchor(),
        64 * 1024 * 1024,
        true,
    );
    let b = capture(
        "det_b",
        &trace_across_the_anchor(),
        &full_anchor(),
        64 * 1024 * 1024,
        true,
    );

    let pa = trace_payloads(&a.bag);
    let pb = trace_payloads(&b.bag);
    assert_eq!(pa, pb, "two captures of one stimulus must agree");

    let oracle: Vec<Vec<u8>> = expected_kept()
        .iter()
        .map(|r| r.as_bytes().to_vec())
        .collect();
    assert_eq!(pa, oracle, "…and both must equal the hand oracle");

    a.cleanup();
    b.cleanup();
}

/// **A ONE-WORKER-GROUP capture resolves its node map, departure ring and all,
/// and the RESOLUTION is real, not a relaxed count.**
///
/// The judge asked "did EXACTLY ONE trace ring supply the node table?" over
/// every ring the recorder held, departure ring included. A departure ring
/// carries an EMPTY node manifest by construction — a departure record's
/// `node_idx` is a worker RANK, not a manifest index — so it can never be the
/// ring a `node_idx` resolves through, and counting it made the smallest
/// possible deployment refuse `AmbiguousNodeMap` for a ring that names no node.
/// Tolerable while only `--record` held rings; the DEFAULT verdict on a one-node
/// graph once every multi-process run does.
///
/// BOTH halves of the lockstep are in this one body, because relaxing the COUNT
/// while leaving `trim_node_ids` empty turns a conservative refusal into a
/// VACUOUS acceptance — and that defect produces the SAME `resimmable: true` the
/// correct code does:
///
/// * the FULL-anchor arm must be resimmable (the count really was relaxed), and
/// * the PARTIAL-anchor twin must be REFUSED FOR THE ANCHOR, which is only
///   reachable if the node map genuinely resolved: `judge_resimmable` compares
///   the anchor's facts against the executed-node set, and an EMPTY set (the
///   vacuous shape) has nothing to be incomplete against, so it would read
///   resimmable too.
#[test]
fn a_one_group_capture_resolves_its_node_map_despite_the_departure_ring() {
    let trace = trace_across_the_anchor();

    let full = capture_with_departure_ring("deprngfull", &trace, &full_anchor());
    let m = flashback_manifest(&full.bag);
    assert_eq!(
        m["anchor"]["resimmable"],
        serde_json::json!(true),
        "a one-worker-group capture is resimmable — the departure ring names no node, so it \
         cannot make the node map ambiguous: {m}"
    );

    // The lockstep half. Exactly ONE of the two nodes is anchored, so a
    // RESOLVED node map has something the anchor is missing; an empty one does
    // not, and would read resimmable.
    let partial = capture_with_departure_ring("deprngpart", &trace, &[anchor_record(0, 0xA1)]);
    let mp = flashback_manifest(&partial.bag);
    assert_eq!(
        mp["anchor"]["resimmable"],
        serde_json::json!(false),
        "…and the map really RESOLVED: with one of two nodes anchored the anchor is \
         incomplete, which a judge holding an EMPTY executed-node set could not tell: {mp}"
    );
    let reason = mp["anchor"]["resimmable_reason"]
        .as_str()
        .expect("a refusal states its reason");
    assert!(
        !reason.contains("SEVERAL trace rings"),
        "…and it is refused for the ANCHOR, not for an ambiguity the departure ring never \
         created: {reason}"
    );
}
