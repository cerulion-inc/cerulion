// SPDX-License-Identifier: AGPL-3.0-only
//! **The capture-scoped coverage manifest.**
//!
//! A Flashback capture was UN-RESIMMABLE whenever any topic outside the run's
//! graph was live on the default iceoryx2 namespace. The window recorder records
//! what is LIVE (its `--topics-json` is empty ON PURPOSE: live discovery's argument
//! applied to a black box), a capture carried no `record_coverage.json`, and so
//! `replay_engine::classify_topics` had no evidence that such a topic was
//! recorded deliberately: it fell through to the bag/graph arm and REFUSED the
//! whole file as "corrupt or hand-edited". One stranger topic — a co-tenant
//! graph, an orphaned `graph run-worker`, a leaked service — refused the black
//! box.
//!
//! The fix is a capture-scoped manifest, so replay's EXISTING
//! `unmodelled` escape applies unchanged. This file is the manifest's own
//! oracle, over the REAL recorder, the REAL trigger channel and the bag on disk.
//!
//! # What these arms prove that the CLI acceptance test cannot
//!
//! `cerulion_cli/tests/plain_run_resim_e2e_test.rs` drives the whole loop and
//! asserts the OUTCOME (exit 0, one `unmodelled` warn). That is the claim that
//! matters, and it is structurally blind to what the manifest SAYS: a manifest
//! that marked every channel `discovered` and one that marked them correctly
//! produce the same exit code there. These arms read the document back through
//! the SAME type and the SAME reader `--record` uses, and check each field
//! against a hand-written expectation.
//!
//! # The claims a capture may NOT make, and the control that proves it
//!
//! [`a_capture_makes_no_head_claim_even_when_its_recorder_was_armed_before_producers`]
//! is the sharp one. It runs a NON-window-only recorder — one that writes a
//! continuous bag AND holds a window — configured exactly as `graph run --record`
//! configures one: `armed_before_producers = true`, a DECLARED tap, and a stream
//! whose first wire sequence is nonzero. That recorder's CONTINUOUS manifest
//! therefore carries a proven head loss, and its CAPTURE's manifest must carry
//! none, because a rolling window BEGINS at its floor by construction.
//!
//! Two manifests, one recorder, one run: the continuous one is the
//! anti-tautology control (it proves the apparatus can produce the claim at all),
//! and the capture one is the pin. A `build_capture_coverage` that copied
//! `build_record_coverage` fails it on both fields.
//!
//! That is not pedantry about a marker. `replay_engine::collect_topic_losses`
//! feeds `prefix_lost` straight into `refuse_lossy_reexecuted_topics`, so a
//! capture stamping a real head count would refuse its own resim — the exact
//! defect this fix exists to close, restored through a different door.
//!
//! Isolated per-test SHM roots (the `common` harness), hand-built wire frames,
//! hand-written oracles. Parallel-safe: no `#[serial]`, no shared namespace.
//! Every deadline is a generous LIVENESS ceiling in seconds rather than a wall
//! stated in units of the thing under test, and every arm
//! rendezvous on bagd's own ready-file handshake before publishing anything —
//! see `flashback_e2e_test`'s module docs for why that is not optional.

#![cfg(unix)]

mod common;

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_bagd::{
    run_bagd, BagdConfig, FlashbackSettings, RecordCoverage, ReplayGrade, RunBinding, SchemaSource,
    TapSource, TapSpec, DISCOVERY_RESCAN_INTERVAL, RECORD_COVERAGE_ATTACHMENT,
    RECORD_COVERAGE_VERSION,
};
use cerulion_core::flashback::channel::{FlashbackOutcome, FlashbackRequester};
use cerulion_core::flashback::retention::RetentionCaps;
use cerulion_core::flashback::switch::TriggerPosture;
use cerulion_core::flashback::trigger::{CaptureRequest, TriggerPolicy};
use cerulion_core::transport::TransportManager;

use common::{
    await_bagd_ready, await_condition, build_frame, join_bagd, make_manager, publisher, unique_out,
    unique_ready_file, unique_ring_tag, unique_topic,
};

/// An arbitrary, stable schema hash for the hand-built frames.
const HASH: u64 = 0x1420_1420_1420_1420;
/// Shipped everywhere except the post window — see `flashback_e2e_test`.
const POST_WINDOW_MS: u64 = 300;
/// Generous liveness ceiling for every capture-side condition wait.
const CAPTURE_DEADLINE: Duration = Duration::from_secs(60);
/// A liveness ceiling for [`await_rescan_tap`], deliberately stated in seconds
/// rather than in units of [`DISCOVERY_RESCAN_INTERVAL`].
const TAP_ATTACH_DEADLINE: Duration = Duration::from_secs(20);
/// How often [`await_rescan_tap`] asks.
const TAP_ATTACH_POLL: Duration = Duration::from_millis(5);
/// How long the run-binding arm's recorder runs BEFORE
/// its window has anything in it.
///
/// FIXTURE PACING, never an oracle. It exists to separate two quantities that
/// otherwise coincide: the recorder's LIFETIME and the span of the window its
/// capture holds. With frames published the instant the recorder arms those are
/// the same number to within a few milliseconds, and no assertion can tell a
/// window-scoped `unheard_for_ms` from a lifetime-scoped one. The recorder drains
/// nothing here — a tap with no frames pushes no batch, `harvest_window` skips an
/// empty one — so the window's oldest surviving batch lands a full
/// `RECORDER_ALIVE_BEFORE_WINDOW` after the drive loop began.
///
/// LOAD-SAFE in the direction that matters: a sleep only ever overshoots, so load
/// WIDENS the separation the arm's precondition requires.
const RECORDER_ALIVE_BEFORE_WINDOW: Duration = Duration::from_secs(2);
/// The margin by which the recorder's lifetime must exceed the window it holds
/// for the run-binding arm's oracle to discriminate.
///
/// A quarter of [`RECORDER_ALIVE_BEFORE_WINDOW`], so the precondition has 4x
/// headroom on a healthy desk and load can only add to it. NOT a timing
/// assertion: it is checked against two numbers the CAPTURE ITSELF reports, on
/// one clock, so nothing here is a wall.
const LIFETIME_SEPARATION_MARGIN_MS: u64 = 500;

fn temp_dir(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "{tag}-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos())
            .unwrap_or(0)
    ));
    std::fs::create_dir_all(&dir).expect("temp dir");
    dir
}

/// Shipped Flashback settings apart from the injected post window.
fn settings(dir: &Path, window_only: bool) -> FlashbackSettings {
    FlashbackSettings {
        window_span: Duration::from_secs(30),
        window_max_bytes: 64 * 1024 * 1024,
        anchor_max_bytes: 64 * 1024 * 1024,
        anchor_cap_basis: cerulion_core::flashback::CapBasis::Env,
        trace_max_bytes: cerulion_core::flashback::DEFAULT_FLASHBACK_TRACE_MAX_MB * 1024 * 1024,
        dir: dir.to_path_buf(),
        label: "probe".into(),
        caps: RetentionCaps::default(),
        policy: TriggerPolicy {
            post_window_ns: POST_WINDOW_MS * 1_000_000,
            ..Default::default()
        },
        posture: TriggerPosture::default(),
        exclude_topics: cerulion_core::flashback::ExcludeTopics::default(),
        window_only,
        tap_budget_bytes: cerulion_core::flashback::DEFAULT_FLASHBACK_TAP_BUDGET_MB * 1024 * 1024,
    }
}

/// Every `.mcap` in `dir`, sorted.
fn captures(dir: &Path) -> Vec<PathBuf> {
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

/// Block until bagd's discovery RESCAN has ATTACHED a tap to `topic`.
///
/// A rendezvous on the STATE the next publish depends on, never a sleep: a
/// data-only tap requests no late-joiner history, so a frame committed before
/// the tap attaches lands in no queue at all. Same helper, same reasoning, as
/// `discovery_e2e_test::await_rescan_tap` — copied rather than shared because
/// each integration test binary compiles its own `mod common`, and hoisting a
/// second harness into it for one caller buys nothing.
fn await_rescan_tap(mgr: &TransportManager, topic: &str) {
    let start = Instant::now();
    loop {
        if mgr.topic_subscriber_count(topic) >= 1 {
            return;
        }
        assert!(
            start.elapsed() < TAP_ATTACH_DEADLINE,
            "bagd's discovery rescan never attached a tap to '{topic}' after {:?} (the rescan \
             cadence is {DISCOVERY_RESCAN_INTERVAL:?}). Every frame published from here would \
             land in no queue at all, so this is a FAILURE of the recorder or of this harness — \
             not a coverage bug in the assertions below",
            start.elapsed()
        );
        std::thread::sleep(TAP_ATTACH_POLL);
    }
}

/// Read a bag's `record_coverage.json` back through the SAME type and reader
/// `--record` bags are read with.
///
/// Deliberately NOT a hand-rolled JSON walk: the whole manifest claim is that a
/// capture carries the SAME document a recording does, so a test that parsed it
/// some other way would not be checking that claim.
fn coverage_of(bag: &Path) -> RecordCoverage {
    let reader = cerulion_bag::BagReader::open(bag).expect("open the bag");
    let att = reader
        .attachment(RECORD_COVERAGE_ATTACHMENT)
        .expect("attachment lookup")
        .unwrap_or_else(|| {
            panic!(
                "{} carries no `{RECORD_COVERAGE_ATTACHMENT}` — a capture without one is refused \
                 by `bag play --resim` the moment it holds a topic the run's graph does not \
                 model",
                bag.display()
            )
        });
    serde_json::from_slice::<RecordCoverage>(&att.data).unwrap_or_else(|e| {
        panic!(
            "the capture's coverage manifest must parse as the SAME `RecordCoverage` a \
             recording's does: {e}\n{}",
            String::from_utf8_lossy(&att.data)
        )
    })
}

/// A bag's `record_coverage.json` as RAW BYTES.
///
/// The typed reader above cannot see the one thing `skip_serializing_if` claims:
/// `#[serde(default)]` makes an ABSENT key and a present `false` decode
/// identically, so a byte-identity claim can only be checked on the bytes.
fn coverage_bytes(bag: &Path) -> String {
    let reader = cerulion_bag::BagReader::open(bag).expect("open the bag");
    let att = reader
        .attachment(RECORD_COVERAGE_ATTACHMENT)
        .expect("attachment lookup")
        .expect("the coverage manifest");
    String::from_utf8(att.data).expect("the manifest is UTF-8 JSON")
}

/// A bag's `flashback.json`, parsed loosely — the capture's OWN manifest, used
/// here only to cross-check a number `record_coverage.json` also carries.
fn flashback_manifest(bag: &Path) -> serde_json::Value {
    let reader = cerulion_bag::BagReader::open(bag).expect("open the bag");
    let att = reader
        .attachment(cerulion_bagd::FLASHBACK_ATTACHMENT)
        .expect("attachment lookup")
        .expect("a capture carries its own manifest");
    serde_json::from_slice(&att.data).expect("the capture manifest is valid JSON")
}

/// A `u64` field of the capture's own `flashback.json`.
///
/// `null` is a real value in that document (`achieved_from_ns` is `null` when a
/// capture carries no frames), so a missing-or-null field PANICS rather than
/// defaulting — a 0 substituted here would silently satisfy the arithmetic the
/// callers do with it.
fn manifest_u64(manifest: &serde_json::Value, key: &str) -> u64 {
    manifest[key]
        .as_u64()
        .unwrap_or_else(|| panic!("`{key}` must be a number in the capture manifest:\n{manifest}"))
}

/// How many messages a bag holds on `topic`.
fn messages_on(bag: &Path, topic: &str) -> u64 {
    let reader = cerulion_bag::BagReader::open(bag).expect("open the bag");
    let (msgs, completeness) = reader.recover_messages().expect("recover_messages");
    assert!(
        completeness.is_finalized(),
        "{} is not finalized: {completeness:?}",
        bag.display()
    );
    msgs.iter().filter(|m| m.topic == topic).count() as u64
}

/// Drive one manual capture to `Finished` and return its path.
fn capture_now(
    mgr: &TransportManager,
    dir: &Path,
    reason: &str,
    mut keep_publishing: impl FnMut(),
) -> PathBuf {
    let requester = FlashbackRequester::open_on_manager(mgr).expect("requester");
    let request_id = requester
        .request(&CaptureRequest::manual(reason))
        .expect("request");
    assert!(
        await_condition(CAPTURE_DEADLINE, || {
            keep_publishing();
            requester
                .drain_outcomes(request_id)
                .iter()
                .any(|f| matches!(f.outcome, FlashbackOutcome::Finished { .. }))
        }),
        "the recorder must report the capture FINISHED"
    );
    let bags = captures(dir);
    assert_eq!(bags.len(), 1, "exactly one capture, got {bags:?}");
    bags.into_iter().next().expect("one capture")
}

/// **THE CLASSIFICATION ARM.** A capture holds a DECLARED tap and a co-tenant
/// the recorder DISCOVERED, and its manifest says which is which.
///
/// This is the field the manifest exists for: `classify_topics` skips a recorded
/// topic the graph models nowhere ONLY when the bag's own manifest marks it
/// `source: discovered`. A capture that marked everything `declared` is refused
/// exactly as a capture with no manifest at all is.
///
/// The source is the TAP's own, never re-derived from the embedded `graph.yaml`:
/// `TapSource::Declared` MEANS "the caller named it", and on the always-on window
/// recorder — whose `--topics-json` is empty — nobody did. So this arm drives
/// BOTH values off ONE recorder, which is the only way to see that the field is
/// read from the tap rather than defaulted: a manifest hardcoding either value
/// fails on the other topic.
#[test]
fn a_capture_says_which_of_its_topics_the_recorder_was_told_about_and_which_it_found() {
    let mgr = make_manager(64);
    let declared = unique_topic("/cov/declared");
    let stranger = unique_topic("/cov/stranger");
    let dir = temp_dir("classify");
    let mut cfg = BagdConfig::new(unique_out("classify"), vec![TapSpec::attach(&declared)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    // The co-tenant half: this recorder ENUMERATES, exactly as the always-on
    // window recorder does. The settle window is zeroed so the arm is about
    // classification rather than about the settle cap.
    cfg.discover_live = true;
    cfg.discovery_settle = Duration::ZERO;
    cfg.flashback = Some(settings(&dir, true));
    // A REAL declared trace ring, whose only job here is to make the
    // cross-artifact assertion below NON-VACUOUS: with no ring declared,
    // `rings_declared` and `trace_rings_configured` are both 0 and their
    // equality is true of any implementation (MEASURED — a `rings_declared: 0`
    // implementation would pass the arm before this ring existed). The owner is held for
    // the whole arm so the ring stays open.
    let ring_tag = unique_ring_tag("cov");
    let _ring = cerulion_core::trace_ring::TraceRingOwner::create(&ring_tag, 64, 0, &[])
        .expect("the declared trace ring");
    cfg.rings = vec![_ring.name().to_string()];
    let ready = unique_ready_file("classify");
    cfg.ready_file = Some(ready.clone());

    // The declared tap is OPEN-ONLY, so its producer must exist before setup.
    let mut declared_pub = publisher(&mgr, &declared, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let rec_mgr = Arc::clone(&mgr);
    let rec_shutdown = Arc::clone(&shutdown);
    let handle = std::thread::spawn(move || run_bagd(rec_mgr, cfg, rec_shutdown));
    await_bagd_ready(&ready, "the classification arm");

    // The CO-TENANT: a live producer nobody told this recorder about, created
    // after it armed — the shape a second graph on the machine produces.
    let mut stranger_pub = publisher(&mgr, &stranger, 256);
    await_rescan_tap(&mgr, &stranger);

    for seq in 0..8u32 {
        declared_pub
            .publish_raw(&build_frame(HASH, seq, 1_000 + u64::from(seq), b"own"))
            .expect("publish declared");
        stranger_pub
            .publish_raw(&build_frame(HASH, seq, 1_000 + u64::from(seq), b"other"))
            .expect("publish stranger");
        std::thread::sleep(Duration::from_millis(10));
    }

    let bag = capture_now(&mgr, &dir, "a co-tenant is live", || {});
    shutdown.store(true, Ordering::Relaxed);
    join_bagd(handle, &shutdown, "the classification arm").expect("recorder");

    let coverage = coverage_of(&bag);

    // THE MARKER, read first because it qualifies every count below.
    assert!(
        coverage.window_capture,
        "a capture's manifest must declare itself a WINDOW capture, or its frame counts read as \
         a recorder's lifetime account and its absent head-loss markers read as a proof of no \
         head loss: {coverage:?}"
    );
    assert_eq!(
        coverage.version, RECORD_COVERAGE_VERSION,
        "the SAME document version a recording writes — one schema, one reader"
    );

    // THE HEADLINE: two topics, two sources, read off the taps.
    assert_eq!(
        coverage.tapped.get(&declared).map(|t| t.source),
        Some(TapSource::Declared),
        "the topic the CALLER named must be `declared`: {coverage:?}"
    );
    assert_eq!(
        coverage.tapped.get(&stranger).map(|t| t.source),
        Some(TapSource::Discovered),
        "the co-tenant this recorder FOUND must be `discovered` — that mark is the whole of the \
         evidence `replay_engine::classify_topics` accepts for skipping a topic the graph does \
         not model, and without it `bag play --resim` refuses the capture: {coverage:?}"
    );
    assert_eq!(
        coverage.tapped.len(),
        2,
        "the manifest describes exactly the capture's channel set: {coverage:?}"
    );

    // The enumeration facts are the RECORDER's, and the capture's channel set IS
    // that recorder's tap set — so they describe this bag as truly as they
    // describe a continuous one.
    assert!(coverage.discovery_requested, "{coverage:?}");
    assert!(
        coverage.enumerated,
        "enumeration RAN and succeeded here — it is what tapped the co-tenant: {coverage:?}"
    );

    // …and the frame counts are the WINDOW's, cross-checked against the bag.
    for topic in [&declared, &stranger] {
        let in_bag = messages_on(&bag, topic);
        assert!(in_bag > 0, "{topic} must have frames in the capture");
        assert_eq!(
            coverage.tapped.get(topic).map(|t| t.frames_recorded),
            Some(in_bag),
            "the manifest's count for {topic} must be what the BAG holds: {coverage:?}"
        );
    }

    // THE LADDER'S BY-PRODUCTS, and they are only checkable HERE.
    //
    // `capture_channel_set` runs the schema ladder at CAPTURE time and now
    // carries its two outputs out rather than dropping them — the fn's own doc
    // says recomputing them is "how the bag's channels and the bag's manifest
    // come to describe the same channel differently". This recorder is
    // WINDOW-ONLY, so it never created a continuous bag and its own
    // `schema_sources` / `schema_descriptors` are EMPTY: a manifest built from
    // those (exactly what the refactor exists to prevent) reports
    // `schema_source: None` on every row and `replay_grade: None`. Both are
    // wrong in a specific, readable way: `None` is documented as "an older
    // bag, genuinely UNKNOWN provenance", which a bag written today is not.
    for topic in [&declared, &stranger] {
        assert_eq!(
            coverage
                .tapped
                .get(topic)
                .and_then(|t| t.schema_source.clone()),
            Some(SchemaSource::Unresolved),
            "{topic} carries hand-built frames under a hash no corpus knows, so the ladder's \
             correct answer is Unresolved — never `None`, which claims this bag predates the \
             ladder: {coverage:?}"
        );
    }
    assert_eq!(
        coverage.replay_grade,
        Some(ReplayGrade::Observability),
        "no channel here is describable, so the capture's OWN descriptors grade it \
         Observability. `None` means the grade was read off the continuous bag's descriptors, \
         which a window-only recorder does not have: {coverage:?}"
    );

    // The TRACE count is one number in two artifacts of the same bag — the fn
    // doc's "cannot disagree" claim, asserted rather than asserted-in-prose.
    // NON-VACUOUS because this recorder really declares a ring: the equality
    // below is 1 == 1, and a manifest that stopped reading `cfg.rings` reports 0
    // against the capture's own 1.
    assert_eq!(
        coverage.rings_declared, 1,
        "this recorder was DECLARED one trace ring, and the manifest must say so — a 0 here \
         makes the cross-artifact equality below compare zero with zero: {coverage:?}"
    );
    let flashback = flashback_manifest(&bag);
    assert_eq!(
        flashback["handoff"]["trace_rings_configured"].as_u64(),
        Some(coverage.rings_declared as u64),
        "`record_coverage.json`'s rings_declared and `flashback.json`'s \
         trace_rings_configured are the same `cfg.rings.len()`: {coverage:?}\n{flashback}"
    );

    // The UNTAPPED ledger is carried, and on this arm it must be EMPTY: both
    // live producers were tapped. A ledger that leaked a gap onto a capture
    // would render `coverage: INCOMPLETE` in `bag info` for a bag that holds
    // everything the recorder saw.
    assert!(
        coverage.untapped.is_empty(),
        "every live producer was tapped, so the capture inherits an empty ledger: {coverage:?}"
    );
    assert_eq!(coverage.gap_count(), 0, "{coverage:?}");

    std::fs::remove_dir_all(&dir).ok();
    std::fs::remove_file(&ready).ok();
}

/// **THE NO-FABRICATION ARM**, with its own anti-tautology control.
///
/// One recorder, configured as `graph run --record` configures one
/// (`armed_before_producers`, a DECLARED tap, a stream whose first wire sequence
/// is nonzero), writes TWO manifests: its continuous bag's and its capture's.
///
/// The CONTINUOUS one must carry the proven head loss — that is the control, and
/// without it every assertion below would pass against a recorder that simply
/// never measures head loss at all. The CAPTURE's must carry none of it: a
/// rolling window begins at its floor by construction, so a head count there
/// would be both false and actively harmful (`collect_topic_losses` feeds
/// `prefix_lost` into `refuse_lossy_reexecuted_topics`, which would refuse the
/// capture's own resim).
///
/// The same arm pins the INGRESS-BUILD block's absence: it answers "why did this
/// bag's channel set close when it did?", and a capture's channel set is resolved
/// at capture time rather than frozen at creation.
///
/// It says NOTHING about the run binding. Asserting `run_binding` is absent on a
/// capture would be wrong: `watch_failed` and `never_heard` are recorder-WATCHER
/// facts, as true of a capture as of a recording, and this fixture configures no
/// binding at all, so the assertion could not fail under any implementation while
/// its message would document the opposite of the shipped design. A BOUND
/// recorder's capture
/// carries the block; that is
/// [`a_capture_names_the_run_it_belongs_to_and_escalates_on_a_watcher_that_heard_nothing`].
#[test]
fn a_capture_makes_no_head_claim_even_when_its_recorder_was_armed_before_producers() {
    let mgr = make_manager(64);
    let topic = unique_topic("/cov/head");
    let dir = temp_dir("head");
    let out = unique_out("head");
    let mut cfg = BagdConfig::new(out, vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    // The guarantee `graph run --record` makes: this recorder armed
    // before its producers committed anything.
    cfg.armed_before_producers = true;
    // NOT window-only: this recorder writes a continuous bag TOO, and that bag's
    // manifest is the control.
    cfg.flashback = Some(settings(&dir, false));
    let ready = unique_ready_file("head");
    cfg.ready_file = Some(ready.clone());

    let mut pub_ = publisher(&mgr, &topic, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let rec_mgr = Arc::clone(&mgr);
    let rec_shutdown = Arc::clone(&shutdown);
    let handle = std::thread::spawn(move || run_bagd(rec_mgr, cfg, rec_shutdown));
    await_bagd_ready(&ready, "the no-head-claim arm");

    // The stream's first WIRE SEQUENCE is nonzero, which under the guarantee
    // above is the prefix-loss proof that frames are missing from this topic's head.
    const FIRST_SEQ: u32 = 5;
    for seq in FIRST_SEQ..FIRST_SEQ + 10 {
        pub_.publish_raw(&build_frame(HASH, seq, 1_000 + u64::from(seq), b"head"))
            .expect("publish");
        std::thread::sleep(Duration::from_millis(10));
    }

    let bag = capture_now(&mgr, &dir, "the head question", || {});
    shutdown.store(true, Ordering::Relaxed);
    let summary = join_bagd(handle, &shutdown, "the no-head-claim arm").expect("recorder");

    // THE CONTROL: the continuous bag's manifest DOES make the claim.
    let continuous = summary
        .bag_paths
        .first()
        .expect("a non-window-only recorder writes a continuous bag");
    let recording = coverage_of(continuous);
    assert!(
        !recording.window_capture,
        "a continuous recording's manifest must NOT claim to be a window capture: {recording:?}"
    );
    assert!(
        recording.armed_before_producers,
        "ANTI-TAUTOLOGY: the recorder really was told its producers had committed nothing, so \
         the continuous manifest must carry the guarantee — without this the capture's \
         assertions below would pass against a build that never records it: {recording:?}"
    );
    assert_eq!(
        recording.tapped.get(&topic).and_then(|t| t.prefix_lost),
        Some(u64::from(FIRST_SEQ)),
        "ANTI-TAUTOLOGY: under that guarantee a first wire sequence of {FIRST_SEQ} PROVES \
         {FIRST_SEQ} frames are missing from this stream's head, and the continuous manifest \
         must say so: {recording:?}"
    );

    // THE PIN: the capture's manifest makes neither claim.
    let coverage = coverage_of(&bag);
    assert!(coverage.window_capture, "{coverage:?}");
    assert!(
        !coverage.armed_before_producers,
        "a rolling window begins at its FLOOR on every topic, so a capture makes no head claim \
         at all — carrying the recorder's arm-ordering guarantee would make its absent \
         `prefix_lost` read as a PROOF of no head loss: {coverage:?}"
    );
    assert_eq!(
        coverage.tapped.get(&topic).and_then(|t| t.prefix_lost),
        None,
        "a capture must stamp NO head loss. `replay_engine::collect_topic_losses` feeds this \
         straight into `refuse_lossy_reexecuted_topics`, so a real count here would refuse the \
         resim of every capture whose window does not start at the stream's first frame — i.e. \
         all of them, which is the very defect this fix closes: {coverage:?}"
    );

    assert!(
        coverage.ingress_build.is_none(),
        "a capture states no ingress-build block: it answers why a bag's channel set FROZE at \
         creation, and a capture's channel set is resolved at capture time: {coverage:?}"
    );
    assert_eq!(
        coverage.window_capture_contradiction(),
        None,
        "…and the manifest the shipped recorder writes trips no contradiction: {coverage:?}"
    );

    // THE BYTE ORACLE for the additive claim. `#[serde(default)]` makes an
    // ABSENT key and a present `false` decode identically, so every typed
    // assertion above is blind to `skip_serializing_if` — dropping it would
    // change every `--record` manifest's bytes and pass the whole suite. This
    // arm has BOTH documents from ONE recorder, which is the only place the two
    // halves can be compared.
    let recording_bytes = coverage_bytes(continuous);
    assert!(
        !recording_bytes.contains("window_capture"),
        "a `--record` manifest must stay BYTE-IDENTICAL to one from before the flag existed — the flag is \
         skipped at its `false` default, which is what makes it additive with no \
         RECORD_COVERAGE_VERSION bump:\n{recording_bytes}"
    );
    let capture_bytes = coverage_bytes(&bag);
    assert!(
        capture_bytes.contains("\"window_capture\":true"),
        "…and a capture's manifest must carry it, or nothing distinguishes the two documents \
         on the wire:\n{capture_bytes}"
    );

    std::fs::remove_dir_all(&dir).ok();
    std::fs::remove_file(&ready).ok();
}

/// **THE WINDOW-SCOPE ARM.** A capture's per-topic frame count is what the
/// WINDOW holds, never what its recorder drained.
///
/// The two numbers are equal on an ordinary capture, which is why this arm makes
/// them differ: the byte ceiling is injected a few frames wide and the inflow
/// continues through the post window, so most of what the tap drained is evicted
/// before the capture is written. A manifest reporting the recorder's own
/// lifetime count then over-reports by exactly the evicted frames — a bag
/// claiming to hold data it does not.
///
/// The precondition (the ceiling really bit) is asserted separately, so a
/// starved runner fails as a starved runner rather than inverting the verdict.
#[test]
fn a_captures_frame_count_is_what_the_window_holds_not_what_the_recorder_drained() {
    let mgr = make_manager(64);
    let topic = unique_topic("/cov/window");
    let dir = temp_dir("window");
    let mut cfg = BagdConfig::new(unique_out("window"), vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    let mut s = settings(&dir, true);
    // The ONE injected constant: a ceiling a few frames wide.
    const CAP_BYTES: u64 = 8 * 1024;
    const PAYLOAD: usize = 2 * 1024;
    s.window_max_bytes = CAP_BYTES;
    cfg.flashback = Some(s);
    let ready = unique_ready_file("window");
    cfg.ready_file = Some(ready.clone());

    let mut pub_ = publisher(&mgr, &topic, PAYLOAD as u32 + 64);
    let payload = vec![0x14u8; PAYLOAD];

    let shutdown = Arc::new(AtomicBool::new(false));
    let rec_mgr = Arc::clone(&mgr);
    let rec_shutdown = Arc::clone(&shutdown);
    let handle = std::thread::spawn(move || run_bagd(rec_mgr, cfg, rec_shutdown));
    await_bagd_ready(&ready, "the window-scope arm");

    // A pre-window far larger than the ceiling, so most of it is already gone.
    let mut seq = 0u32;
    for _ in 0..30 {
        pub_.publish_raw(&build_frame(HASH, seq, 1_000 + u64::from(seq), &payload))
            .expect("publish");
        seq += 1;
        std::thread::sleep(Duration::from_millis(10));
    }
    let published_before_request = seq;

    // …and the inflow CONTINUES through the post window, which is what evicts
    // the lead-up out from under the capture.
    let bag = capture_now(&mgr, &dir, "the window is bounded by bytes", || {
        pub_.publish_raw(&build_frame(HASH, seq, 1_000 + u64::from(seq), &payload))
            .expect("publish");
        seq += 1;
        std::thread::sleep(Duration::from_millis(10));
    });
    shutdown.store(true, Ordering::Relaxed);
    join_bagd(handle, &shutdown, "the window-scope arm").expect("recorder");

    let in_bag = messages_on(&bag, &topic);
    // PRECONDITION, not the verdict: the ceiling must really have bitten, or the
    // two numbers below coincide and the arm proves nothing. The floor is the
    // other half — a ceiling that evicted EVERYTHING would make the equality
    // below `Some(0) == Some(0)`, which is true of any implementation.
    assert!(
        in_bag > 0,
        "PRECONDITION: the capture must still hold frames — a ceiling that evicted the whole \
         window makes the count assertion below compare zero with zero"
    );
    assert!(
        in_bag < u64::from(published_before_request),
        "PRECONDITION: the {CAP_BYTES}-byte ceiling must have evicted frames — the capture holds \
         {in_bag} of the {published_before_request} published before the request alone. A \
         non-biting ceiling makes this arm vacuous, so it fails here rather than passing quietly"
    );

    let coverage = coverage_of(&bag);
    assert!(coverage.window_capture, "{coverage:?}");
    assert_eq!(
        coverage.tapped.get(&topic).map(|t| t.frames_recorded),
        Some(in_bag),
        "the manifest must report what THIS WINDOW holds ({in_bag}), not what the recorder \
         drained — a bag that claims frames it does not carry is the silent dishonesty the \
         coverage manifest exists to make impossible: {coverage:?}"
    );

    std::fs::remove_dir_all(&dir).ok();
    std::fs::remove_file(&ready).ok();
}

/// **THE RUN-BINDING ARM.** A capture NAMES the run it belongs to, and escalates
/// on a watcher that heard nothing — exactly as its sibling recording does.
///
/// The first cut of this feature left `run_binding: None` on a capture, reasoning
/// that its one measured number (`unattributed_frames`) is counted over the
/// CONTINUOUS bag's message stream. That was true of one of
/// `RunBindingCoverage::is_ambiguous`'s three terms and was applied to all three,
/// and it cost two things an operator opening a black box needs:
///
/// 1. `watch_failed` / `never_heard` are facts about the RECORDER'S WATCHER, as
///    true of the capture as of the recording, and both escalate
///    `is_incomplete()`. Dropping the block made a capture read CLEAN where the
///    sibling bag from the SAME recorder reads INCOMPLETE.
/// 2. The RUN ID — the question a black box exists to answer. No other
///    attachment carries it: `flashback.json` describes the window, its causes
///    and its resumability, and names no run.
///
/// This recorder is bound to a run that never announces, so the watcher opens
/// and hears nothing — the `never_heard` state, reachable with no registry at
/// all. The `unattributed_frames` rule for that state is pinned too: with no
/// instant at which the run was provably alive there is nothing to count FROM,
/// so the correct answer is 0 and `never_heard` carries the doubt. Counting the
/// whole window instead would report one fact twice.
#[test]
fn a_capture_names_the_run_it_belongs_to_and_escalates_on_a_watcher_that_heard_nothing() {
    const RUN_ID: u128 = 0x0000_1420_0000_1420_0000_1420_0000_1420;
    let mgr = make_manager(64);
    let topic = unique_topic("/cov/bound");
    let dir = temp_dir("bound");
    let mut cfg = BagdConfig::new(unique_out("bound"), vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    cfg.run_binding = Some(RunBinding {
        run_id: RUN_ID,
        vanish_grace: None,
    });
    cfg.flashback = Some(settings(&dir, true));
    let ready = unique_ready_file("bound");
    cfg.ready_file = Some(ready.clone());

    let mut pub_ = publisher(&mgr, &topic, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let rec_mgr = Arc::clone(&mgr);
    let rec_shutdown = Arc::clone(&shutdown);
    let handle = std::thread::spawn(move || run_bagd(rec_mgr, cfg, rec_shutdown));
    await_bagd_ready(&ready, "the run-binding arm");

    // The recorder runs with an EMPTY window first, so its LIFETIME and the span
    // of the window it ends up holding are provably different numbers — see
    // `RECORDER_ALIVE_BEFORE_WINDOW`. Without this the two coincide and the
    // `unheard_for_ms` oracle below cannot discriminate.
    std::thread::sleep(RECORDER_ALIVE_BEFORE_WINDOW);

    for seq in 0..8u32 {
        pub_.publish_raw(&build_frame(HASH, seq, 1_000 + u64::from(seq), b"bound"))
            .expect("publish");
        std::thread::sleep(Duration::from_millis(10));
    }

    let bag = capture_now(&mgr, &dir, "which run is this", || {});
    shutdown.store(true, Ordering::Relaxed);
    join_bagd(handle, &shutdown, "the run-binding arm").expect("recorder");

    let coverage = coverage_of(&bag);
    let binding = coverage
        .run_binding
        .as_ref()
        .expect("a capture taken by a BOUND recorder must name its run — no other attachment does");
    assert_eq!(
        binding.run_id, RUN_ID,
        "…and it must be THAT run: {coverage:?}"
    );
    assert!(
        binding.never_heard,
        "this run never announced, so the watcher opened and heard nothing — the state that \
         makes the bag's boundary say nothing about the run: {coverage:?}"
    );
    assert!(
        !binding.watch_failed,
        "the watcher itself opened fine; only the run was silent: {coverage:?}"
    );
    // THE QUANTITY. With no instant at which the run was provably alive there is
    // nothing to count FROM, and 0 would then be a positive claim of "nothing
    // unattributable" over a bag full of it — so the whole window is
    // unattributable, which is exactly what the CONTINUOUS recorder reports on
    // this path (`run_messages_at_last_heard` starts at 0, so its difference is
    // every message). Cross-checked against the BAG rather than against a
    // constant, so a manifest that reported some other number fails here.
    let in_bag = messages_on(&bag, &topic);
    assert!(
        in_bag > 0,
        "the capture must hold frames for this to mean anything"
    );
    assert_eq!(
        binding.unattributed_frames, in_bag,
        "a run that was NEVER HEARD leaves the whole window unattributable, and the count must \
         be the WINDOW's — a count of 0 makes `bag info` render \"none of its 0 \
         frame(s)\" over a bag full of them: {coverage:?}"
    );
    // THE SPAN, against the capture's OWN sibling manifest rather than a wall.
    //
    // `> 0` alone would verify NOTHING: the value it exists to
    // reject — the recorder's LIFETIME, which `unheard_for_ms()` falls back to
    // when the run was never heard — is also `> 0`, so a revert would pass.
    // The two are separated by construction (the
    // recorder ran with an empty window for `RECORDER_ALIVE_BEFORE_WINDOW`) and
    // told apart EXACTLY.
    //
    // Exactly, because both numbers are the same subtraction on the same clock:
    // `flashback.json`'s `achieved_span_ms` is `(ended_ns - achieved_from_ns) /
    // 1e6`, and the manifest's `ended_ns` IS the `now_ns` the coverage manifest's
    // span is measured to. So a window-scoped `unheard_for_ms` must EQUAL it —
    // no tolerance, nothing for load to move.
    let flashback = flashback_manifest(&bag);
    let achieved_from_ns = manifest_u64(&flashback, "achieved_from_ns");
    let ended_ns = manifest_u64(&flashback, "ended_ns");
    let achieved_span_ms = manifest_u64(&flashback, "achieved_span_ms");
    // PRECONDITION: the recorder's lifetime is what a revert would report, and it
    // is ns-since-drive-start exactly as `ended_ns` is — so if the fixture failed
    // to separate them, this arm must fail HERE, as a fixture failure, rather
    // than passing silently.
    let lifetime_ms = ended_ns / 1_000_000;
    assert!(
        achieved_from_ns > 0,
        "PRECONDITION: the window must start AFTER the recorder did, or its span and the \
         recorder's lifetime are the same number and the oracle below is blind:\n{flashback}"
    );
    assert!(
        lifetime_ms > achieved_span_ms + LIFETIME_SEPARATION_MARGIN_MS,
        "PRECONDITION: the recorder's lifetime ({lifetime_ms} ms) must exceed the window it \
         holds ({achieved_span_ms} ms) by more than {LIFETIME_SEPARATION_MARGIN_MS} ms, or a \
         manifest reporting the LIFETIME would satisfy the assertion below by coincidence. The \
         recorder was given {RECORDER_ALIVE_BEFORE_WINDOW:?} of empty running to guarantee \
         it:\n{flashback}"
    );
    assert_eq!(
        binding.unheard_for_ms, achieved_span_ms,
        "the SPAN those frames were committed in is the WINDOW's, and it must equal the span \
         this capture's own `flashback.json` reports. The recorder's `unheard_for_ms()` falls \
         back to `drive_start`'s elapsed when the run was never heard — {lifetime_ms} ms here — \
         which `bag info` would print in ONE sentence beside a {achieved_span_ms} ms window's \
         frame count: {coverage:?}\n{flashback}"
    );
    assert!(
        coverage.is_incomplete(),
        "THE ESCALATION: a bag bound to a run it never heard cannot claim coverage, and a \
         capture must say so exactly as its sibling recording does: {coverage:?}"
    );

    std::fs::remove_dir_all(&dir).ok();
    std::fs::remove_file(&ready).ok();
}
