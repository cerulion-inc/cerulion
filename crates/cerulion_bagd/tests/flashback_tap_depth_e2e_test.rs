// SPDX-License-Identifier: AGPL-3.0-only
//! The tap-depth mode gate, end to end over real
//! iceoryx2.
//!
//! Isolated per-test SHM roots (the `common` harness), hand-built wire frames,
//! hand-written oracles. Parallel-safe: no `#[serial]`, no shared namespace, no
//! process-global env.
//!
//! # Why the gate needs a test at all, and why BOTH arms live in one body
//!
//! `open_tap` serves BOTH of `bagd`'s recorders out of one binary. Switching
//! `create_data_only_subscriber` to a budgeted one
//! WITHOUT a gate would be a silent flag-day on the replay-grade `--record` loss
//! boundary — a 64 KiB-slice topic's kHz absorbance going from its provisioned
//! ceiling to 63 frames, with nothing failing and nothing said.
//!
//! An arm that only checked the window-only side could not see that. So the
//! headline drives BOTH recorders against IDENTICALLY PROVISIONED topics and
//! asserts the two depths in one body: the `--record` tap at the topic's own
//! ceiling, the window-only tap at the budget. That pins the EDGE — a gate that
//! always answers `Ceiling` and a gate that always answers `Budgeted` each fail
//! it, and each fails it on a different assertion.
//!
//! # The oracle is a NUMBER, hand-computed, not a re-derivation
//!
//! The budgeted depth is asserted as `63` rather than as
//! `flashback_tap_buffer_depth(..)`, because calling the rule to check the rule
//! is a self-compare. 63 is `4 MiB / (64 KiB + 48)` floored — and the arm ALSO
//! asserts it is strictly under the NOMINAL `4 MiB / 64 KiB = 64`, which is the
//! discriminator for the real-vs-nominal divisor through the production path.
//!
//! # The budget is set on the SETTINGS, never through the environment
//!
//! `FlashbackSettings::tap_budget_bytes` is a public field, so every arm here
//! dials it directly and the file stays parallel-safe. The env WIRING — that
//! `bagd_cli_run` reads `CERULION_FLASHBACK_TAP_BUDGET_MB` into that field at
//! all — is a different claim, covered by the structural walk at the bottom;
//! the parser itself is oracle-tested in `cerulion_core::flashback`.

#![cfg(unix)]

mod common;

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_bagd::{run_bagd, BagdConfig, BagdSummary, FlashbackSettings, TapSpec};
use cerulion_core::flashback::retention::RetentionCaps;
use cerulion_core::flashback::switch::TriggerPosture;
use cerulion_core::flashback::trigger::TriggerPolicy;

use common::{
    await_bagd_ready, build_frame, code_only, join_bagd, make_manager, publisher_with_provisioning,
    unique_out, unique_ready_file, unique_topic,
};

/// The slice every topic in this file is provisioned with.
///
/// 64 KiB is chosen so the budget arithmetic lands JUST under a power of two:
/// `4 MiB / 65_584` is 63 while `4 MiB / 65_536` is 64, so the real-slot divisor
/// and the nominal one give measurably different answers. A slice where they
/// agreed would make every assertion below pass under either implementation.
const SLICE: u32 = 64 * 1024;

/// The per-topic budget these arms dial. 4 MiB, not the shipped 64 MiB, purely
/// so the resulting depth (63) sits comfortably under a modest service ceiling
/// — the RULE is the same at any budget and its own boundaries are oracle-tested
/// in `cerulion_core::transport`.
const BUDGET: u64 = 4 * 1024 * 1024;

/// The depth the budget buys at [`SLICE`], HAND-COMPUTED:
/// `4 * 1024 * 1024 / (64 * 1024 + 48)` = `4_194_304 / 65_584` = 63.
const BUDGETED_DEPTH: u64 = 63;

/// What a NOMINAL divisor would have answered — `4 MiB / 64 KiB`. Asserted
/// against so the real-slot rule is pinned by a difference, not by a literal
/// somebody could re-bless.
const NOMINAL_DEPTH: u64 = 64;

/// The topics' provisioned service ceiling. Above the budgeted depth on purpose:
/// if it were below, the ceiling would clamp both modes to the same number and
/// the gate would be unobservable.
const CEILING: usize = 256;

/// The borrow budget every topic here is provisioned with. Beyond the arming
/// floor of 1 and irrelevant to what is under test; stated so the two modes'
/// topics are provisioned IDENTICALLY and the only difference is the recorder.
const BORROW: usize = 4;

fn temp_dir(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "tap-{tag}-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos())
            .unwrap_or(0)
    ));
    std::fs::create_dir_all(&dir).expect("temp dir");
    dir
}

fn settings(dir: &Path, window_only: bool, tap_budget_bytes: u64) -> FlashbackSettings {
    FlashbackSettings {
        window_span: Duration::from_secs(30),
        window_max_bytes: 64 * 1024 * 1024,
        anchor_max_bytes: 64 * 1024 * 1024,
        // A stated ceiling, and no exclusions: these arms
        // are about TAP DEPTH, and a reserve free to re-split the plane would
        // move their budgets under them.
        anchor_cap_basis: cerulion_core::flashback::CapBasis::Env,
        exclude_topics: cerulion_core::flashback::ExcludeTopics::default(),
        trace_max_bytes: 64 * 1024 * 1024,
        dir: dir.to_path_buf(),
        label: "tapdepth".into(),
        caps: RetentionCaps::default(),
        policy: TriggerPolicy::default(),
        posture: TriggerPosture::default(),
        window_only,
        tap_budget_bytes,
    }
}

/// Run one recorder over one already-provisioned topic and return its summary.
///
/// `publish` frames are sent AFTER the taps-ready handshake, which is the
/// rendezvous every arm in this crate owes a recorder it spawns: a data-only tap
/// requests no late-joiner history, so anything published before `setup` returns
/// is not late, it is gone.
///
/// The recorder is then held open until its OWN status feed reports it has
/// observed a frame, so no arm below is racing the drive loop. The predicate is
/// a disjunction because the two modes report progress in different fields — a
/// window-only recorder writes no bag, so `messages` stays 0 for it forever and
/// its evidence is the pinned figure; a `--record` recorder prices no slot, so
/// its pinned figure stays 0 forever and its evidence is `messages`.
fn run_one(
    mgr: &Arc<cerulion_core::TransportManager>,
    topic: &str,
    tag: &str,
    fb: Option<FlashbackSettings>,
    publish: impl FnOnce(&mut cerulion_core::transport::publisher::CerulionPublisher),
) -> BagdSummary {
    let out = unique_out(tag);
    let ready = unique_ready_file(tag);
    let mut cfg = BagdConfig::new(out, vec![TapSpec::attach(topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = Some(Duration::from_millis(20));
    cfg.ready_file = Some(ready.clone());
    cfg.flashback = fb;

    // The tap is OPEN-ONLY and never creates a service, so the producer must
    // exist first — and for the BUDGETED arm it must exist for a second reason:
    // the slice is read off the live publisher's dynamic config, so a tap opened
    // with no publisher attached cannot price the budget and falls back to the
    // ceiling. Every arm here asserts a budgeted depth, so every arm depends on
    // that publisher being up, which is why it is created before the spawn
    // rather than after the handshake.
    let mut pubr = publisher_with_provisioning(mgr, topic, BORROW, CEILING, SLICE);

    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = {
        let m = Arc::clone(mgr);
        let c = cfg.clone();
        let s = Arc::clone(&shutdown);
        std::thread::spawn(move || run_bagd(m, c, s))
    };
    await_bagd_ready(&ready, tag);
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");
    publish(&mut pubr);
    assert!(
        common::await_condition(Duration::from_secs(30), || {
            drain_status(&mut status)
                .into_iter()
                .any(|s| s.pinned > 0 || s.messages > 0)
        }),
        "{tag}: the recorder must observe the published frame before this arm reads its \
         numbers — otherwise it is racing the drive loop"
    );
    shutdown.store(true, Ordering::Relaxed);
    join_bagd(handle, &shutdown, tag).expect("recorder")
}

/// The depth the recorder reports for `topic` — the mode gate's observable.
///
/// Also checks the INVARIANT that makes the pinned figure safe to publish at
/// all: it may never exceed `depth × slot`, the physical maximum this tap could
/// have held. That is what the occupancy estimate's cap buys, and it is asserted
/// wherever a depth is read rather than in one arm, because the shape that
/// violates it — frames arriving mid-drain, so a pass takes more than the queue
/// ever held at once — is not one a test can schedule on demand. SCOPE:
/// removing the cap therefore does NOT reliably fail this suite; what this
/// guarantees is that a violation can never pass unnoticed on a run where it
/// does occur.
fn depth_of(summary: &BagdSummary, topic: &str) -> u64 {
    let row = summary
        .record_health
        .topics
        .get(topic)
        .unwrap_or_else(|| panic!("no health row for {topic}"));
    let depth = row
        .tap_buffer_depth
        .expect("the recorder stamps a depth on every tap it opens");
    if let Some(pinned) = row.shm_pinned_bytes {
        assert!(
            pinned <= depth * SLOT,
            "{topic}: the pinned figure ({pinned}) must never exceed what the tap \
             could physically hold ({depth} slots x {SLOT} bytes) — an estimate \
             that over-reports past the hardware maximum is not a bound"
        );
    }
    depth
}

/// THE HEADLINE: one binary, two recorders, two contracts.
///
/// Both topics are provisioned IDENTICALLY — same slice, same ceiling, same
/// borrow budget — so the only variable is which recorder opened the tap. A gate
/// that always answers `Ceiling` fails the window-only assertion; one that always
/// answers `Budgeted` fails the `--record` assertion; and the nominal-divisor
/// variant fails the 63-vs-64 assertion while both mode assertions still pass,
/// which is why all three are here rather than one.
#[test]
fn a_record_tap_stays_ceiling_deep_while_a_window_only_tap_takes_its_budget() {
    let mgr = make_manager(16);
    let dir = temp_dir("gate");

    // (1) THE ALWAYS-ON RECORDER: window-only, budgeted.
    let win_topic = unique_topic("gate_window");
    let win = run_one(
        &mgr,
        &win_topic,
        "gate_window",
        Some(settings(&dir, true, BUDGET)),
        |p| {
            p.publish_raw(&build_frame(0xC133, 0, 1_000, b"w"))
                .expect("publish");
        },
    );

    // (2) THE EXPLICIT `--record` RECORDER, same provisioning, no budget applied.
    let rec_topic = unique_topic("gate_record");
    let rec = run_one(
        &mgr,
        &rec_topic,
        "gate_record",
        Some(settings(&dir, false, BUDGET)),
        |p| {
            p.publish_raw(&build_frame(0xC133, 0, 1_000, b"r"))
                .expect("publish");
        },
    );

    assert_eq!(
        depth_of(&rec, &rec_topic),
        CEILING as u64,
        "an explicit `--record` tap must stay CEILING-DEEP — that is the \
         replay-grade loss boundary sized from a measured drain stall, \
         and R1's whole point is that the budget must not reach it"
    );
    assert_eq!(
        depth_of(&win, &win_topic),
        BUDGETED_DEPTH,
        "a window-only tap must take the byte budget: {BUDGET} / (64 KiB + 48 B \
         of iceoryx2 sample header) = {BUDGETED_DEPTH}"
    );
    // THE REAL-vs-NOMINAL DISCRIMINATOR, through the production path. A divisor
    // that used the nominal slice would answer 64 here and both mode assertions
    // above would still pass.
    assert!(
        depth_of(&win, &win_topic) < NOMINAL_DEPTH,
        "the divisor must be the REAL slot ({BUDGETED_DEPTH}), not the nominal \
         slice ({NOMINAL_DEPTH}) — got {}",
        depth_of(&win, &win_topic)
    );
    // …and the two modes really did differ, stated as the comparison rather than
    // inferred from two literals.
    assert!(
        depth_of(&win, &win_topic) < depth_of(&rec, &rec_topic),
        "the gate must produce DIFFERENT depths for the two recorders"
    );

    // THE PAIRING: a `--record` recorder prices nothing, and says so.
    // Without `unpriced_taps` the zero below reads as "this recorder pinned
    // nothing", which is a claim nobody measured.
    assert_eq!(
        rec.shm_pinned_bytes, 0,
        "a ceiling-deep tap never asks what a slot costs"
    );
    assert_eq!(
        rec.unpriced_taps, 1,
        "…and the count is what makes that zero readable as UNMEASURED"
    );
    assert_eq!(
        rec.record_health
            .topics
            .get(&rec_topic)
            .and_then(|h| h.shm_pinned_bytes),
        None,
        "the per-topic figure is absent, never a fabricated 0"
    );

    // The window-only recorder DID price itself: every tap is accounted for.
    assert_eq!(
        win.unpriced_taps, 0,
        "a budgeted tap opened against a live publisher can always price a slot"
    );
    assert!(
        win.record_health
            .topics
            .get(&win_topic)
            .and_then(|h| h.shm_pinned_bytes)
            .is_some(),
        "…so its per-topic pinned figure is a number, not UNKNOWN"
    );
}

/// The FLOOR: a budget that cannot buy one slot still leaves a tap that can
/// receive.
///
/// Two slots, not one and not zero — a one-deep queue holds nothing between
/// drains, and a zero-deep one cannot be created at all. Driven at a budget
/// SMALLER than a single slot, which is the only way to reach the floor without
/// provisioning a multi-megabyte-slice topic.
#[test]
fn a_budget_too_small_for_one_slot_still_leaves_a_usable_tap() {
    let mgr = make_manager(16);
    let dir = temp_dir("floor");
    let topic = unique_topic("floor");

    // One slot at this slice costs 65_584 bytes; the budget is 4 096.
    let summary = run_one(
        &mgr,
        &topic,
        "floor",
        Some(settings(&dir, true, 4096)),
        |p| {
            p.publish_raw(&build_frame(0xC133, 0, 1_000, b"f"))
                .expect("publish");
        },
    );

    assert_eq!(
        depth_of(&summary, &topic),
        2,
        "the floor is TWO — a budget that buys zero slots must not produce a tap \
         that can never receive"
    );
    // And the tap really worked: the floor is a usable queue, not a formality.
    //
    // The evidence is the PINNED figure, not `frames_recorded`: that counter is
    // write-side (frames put into a continuous bag) and a window-only recorder
    // writes no bag, so it is 0 on this path by design. A nonzero pinned figure
    // can only come from a drain that actually yielded a frame.
    assert_eq!(
        summary
            .record_health
            .topics
            .get(&topic)
            .and_then(|h| h.shm_pinned_bytes),
        Some(SLOT),
        "a floored tap still drains — one frame, one slot"
    );
}

// ===========================================================================
// Priced-ness, and the sticky price
// ===========================================================================

/// A budgeted tap opened while its topic has NO publisher is UNPRICED —
/// and it HEALS the moment one shows up.
///
/// `prices_occupancy` records the MODE. Priced-ness is a different fact: the
/// slice is a per-PUBLISHER property, so a budgeted tap whose service carries no
/// publisher cannot know what a slot costs. Conflating them reports such a tap
/// as measured, contributing `0` to `shm_pinned_bytes` with `unpriced_taps` also
/// `0` — an unmeasured tap presented as measured AND free, which is the one
/// reading the whole unpriced/pinned pairing exists to prevent.
///
/// The service is kept alive with no publisher the way it happens in practice: a
/// producer that has exited while something else still holds the topic open. A
/// test-side data-only tap is that holder, so the recorder's own `.open()`
/// succeeds against a service with zero publishers.
///
/// BOTH halves in one body: the unpriced reading and the heal, over one
/// recorder. Split apart, the heal could be satisfied by a tap that was priced
/// all along, and the unpriced arm by a tap that is unpriced forever.
#[test]
fn a_budgeted_tap_opened_without_a_publisher_is_unpriced_and_heals_when_one_joins() {
    let mgr = make_manager(16);
    let dir = temp_dir("unpriced");
    let topic = unique_topic("unpriced");
    let out = unique_out("unpriced");
    let ready = unique_ready_file("unpriced");

    // Create the service, then let its ONLY publisher go — while a test-side tap
    // keeps the service itself alive.
    let keepalive = {
        let pubr = publisher_with_provisioning(&mgr, &topic, BORROW, SHAPE_CEILING, WIDE_SLICE);
        let tap = mgr
            .create_data_only_subscriber(&topic)
            .expect("keepalive tap");
        drop(pubr);
        tap
    };
    assert_eq!(
        keepalive.live_max_slice_len(),
        None,
        "the fixture must really present a publisher-less service, or this arm \
         proves nothing"
    );

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = BagdConfig::new(out, vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = Some(Duration::from_millis(20));
    cfg.ready_file = Some(ready.clone());
    cfg.flashback = Some(settings(&dir, true, SHAPE_BUDGET));

    let handle = {
        let m = Arc::clone(&mgr);
        let c = cfg.clone();
        let s = Arc::clone(&shutdown);
        std::thread::spawn(move || run_bagd(m, c, s))
    };
    await_bagd_ready(&ready, "the unpriced arm");
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");

    // (1) UNPRICED: the tap exists, prices nothing, and SAYS it prices nothing.
    let mut seen: Vec<(u64, u64)> = Vec::new();
    let reported = common::await_condition(Duration::from_secs(30), || {
        for s in drain_status(&mut status) {
            seen.push((s.pinned, s.unpriced));
        }
        !seen.is_empty()
    });
    assert!(reported, "the recorder must publish status");
    for (pinned, unpriced) in &seen {
        assert_eq!(
            (*pinned, *unpriced),
            (0, 1),
            "a budgeted tap with no publisher must read UNPRICED (0 bytes over 1 \
             unpriced tap), never measured-and-free (0 over 0); readings: {seen:?}"
        );
    }

    // (2) HEALS: a publisher joins and the very next drain prices the tap.
    let mut pubr = publisher_with_provisioning(&mgr, &topic, BORROW, SHAPE_CEILING, WIDE_SLICE);
    let body = vec![0xAB; WIDE_SLICE as usize - cerulion_core::wire::WireHeader::SIZE];
    let mut seq = 0u32;
    let healed = common::await_condition(Duration::from_secs(30), || {
        pubr.publish_raw(&build_frame(0xC133, seq, 1_000 + u64::from(seq), &body))
            .expect("publish");
        seq += 1;
        drain_status(&mut status)
            .into_iter()
            .any(|s| s.unpriced == 0 && s.pinned > 0)
    });
    assert!(
        healed,
        "an unpriced tap must HEAL — the drain reads the price every pass, so the \
         first successful read makes the tap priced"
    );

    shutdown.store(true, Ordering::Relaxed);
    let summary = join_bagd(handle, &shutdown, "the unpriced arm").expect("recorder");
    drop(keepalive);
    assert_eq!(summary.unpriced_taps, 0, "it ended priced");
    assert!(
        summary
            .record_health
            .topics
            .get(&topic)
            .and_then(|h| h.shm_pinned_bytes)
            .is_some_and(|p| p > 0),
        "…and the per-topic figure is a number, not the UNKNOWN it started as"
    );
}

/// A wide publisher that DETACHES before the drain still has its frames
/// priced at the WIDE slot.
///
/// The live price is read at pass ENTRY, which narrows the race but cannot close
/// it — a publisher can detach while frames it committed sit in the queue, and
/// per-sample slot pricing is not observable at all (the drain sees payload
/// LENGTHS, not slot CAPACITIES). So the price is STICKY-MAX for the tap's life:
/// it can OVERSTATE after a permanent detach, which is the declared direction of
/// an upper bound, and it can never UNDERCOUNT, which is the failure this arm
/// exists to exclude.
///
/// The race is made deterministic rather than raced for: the drain gate holds
/// the recorder off while the wide publisher publishes AND detaches, so by the
/// time any drain runs the live answer is the narrow survivor and only the
/// sticky max can produce the right number.
#[test]
fn a_wide_publisher_that_detaches_before_the_drain_still_prices_its_frames_wide() {
    let mgr = make_manager(16);
    let dir = temp_dir("sticky");
    let topic = unique_topic("sticky");
    let out = unique_out("sticky");
    let ready = unique_ready_file("sticky");

    // The NARROW publisher creates the service and SURVIVES the whole run.
    let _narrow = narrow_publisher(&mgr, &topic);
    // The WIDE one is attached before the recorder arms, so the tap's sticky
    // price starts at the wide slot.
    let mut wide = wide_publisher(&mgr, &topic);

    let gate = Arc::new(AtomicBool::new(false));
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = BagdConfig::new(out, vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = Some(Duration::from_millis(20));
    cfg.ready_file = Some(ready.clone());
    cfg.flashback = Some(settings(&dir, true, SHAPE_BUDGET));
    cfg.fault_inject_tap_drain_gate = Some(gate.clone());

    let handle = {
        let m = Arc::clone(&mgr);
        let c = cfg.clone();
        let s = Arc::clone(&shutdown);
        std::thread::spawn(move || run_bagd(m, c, s))
    };
    await_bagd_ready(&ready, "the sticky arm");
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");

    // The wide publisher commits its frames and LEAVES, all while the recorder
    // is held off its taps.
    const BURST: u32 = 8;
    let body = vec![0xAB; WIDE_SLICE as usize - cerulion_core::wire::WireHeader::SIZE];
    for seq in 0..BURST {
        wide.publish_raw(&build_frame(0xC133, seq, 1_000 + u64::from(seq), &body))
            .expect("publish");
    }
    drop(wide);

    // The live answer is now the NARROW survivor — asserted, so the arm cannot
    // pass by the wide publisher lingering.
    let probe = mgr
        .create_data_only_subscriber(&topic)
        .expect("width probe");
    assert!(
        common::await_condition(Duration::from_secs(30), || {
            probe.live_max_slice_len() == Some(NARROW_SLICE as usize)
        }),
        "the wide publisher must be GONE before the drain, or the sticky max is \
         not what is under test; live width is {:?}",
        probe.live_max_slice_len()
    );
    gate.store(true, Ordering::Relaxed);

    let expected = WIDE_SLOT * u64::from(BURST);
    let live_would_say = NARROW_SLOT * u64::from(BURST);
    let mut readings: Vec<u64> = Vec::new();
    let reached = common::await_condition(Duration::from_secs(30), || {
        for s in drain_status(&mut status) {
            readings.push(s.pinned);
        }
        readings.iter().any(|p| *p > 0)
    });
    assert!(reached, "the burst must be drained; readings: {readings:?}");
    assert!(
        readings.iter().all(|p| *p == 0 || *p == expected),
        "frames committed while a WIDE publisher was attached must be priced at \
         the wide slot ({WIDE_SLOT} B x {BURST} = {expected}); a post-drain live \
         read sees only the narrow survivor and would say {live_would_say}. \
         readings: {readings:?}"
    );

    shutdown.store(true, Ordering::Relaxed);
    let summary = join_bagd(handle, &shutdown, "the sticky arm").expect("recorder");
    assert_eq!(
        summary.shm_pinned_bytes, expected,
        "the durable figure agrees, and the sticky price never fell back to the \
         narrow survivor's {NARROW_SLOT} B"
    );
}

/// The gate reaches the RESCAN site too — the one that opens most of an
/// always-on recorder's taps.
///
/// `open_tap` has TWO call sites, and the second is not a corner: the always-on
/// shape declares NO topics at all (`Recorder::setup` explicitly permits an
/// empty tap set when discovery is on and a plane exists) and lets discovery
/// find the producers, because a static declaration was measured naming
/// four of a bridged robot's ~75 topics. A gate wired only at arm time would therefore bound
/// almost nothing on the very recorder it exists for, and every other arm in
/// this file — all of which declare their taps — would stay green.
///
/// So this one declares nothing and asserts the DISCOVERED tap is budgeted.
#[test]
fn a_discovered_tap_on_a_window_only_recorder_is_budgeted_too() {
    let mgr = make_manager(16);
    let dir = temp_dir("disc");
    let topic = unique_topic("disc");
    let ready = unique_ready_file("disc");

    // The producer must exist before the scan can find it.
    let mut pubr = publisher_with_provisioning(&mgr, &topic, BORROW, CEILING, SLICE);

    // THE ALWAYS-ON SHAPE: no declared taps, discovery on, a plane present.
    let mut cfg = BagdConfig::new(unique_out("disc"), Vec::new());
    cfg.discover_live = true;
    cfg.discovery_settle = Duration::ZERO;
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = Some(Duration::from_millis(20));
    cfg.ready_file = Some(ready.clone());
    cfg.flashback = Some(settings(&dir, true, BUDGET));

    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = {
        let m = Arc::clone(&mgr);
        let c = cfg.clone();
        let s = Arc::clone(&shutdown);
        std::thread::spawn(move || run_bagd(m, c, s))
    };
    await_bagd_ready(&ready, "the discovery arm");

    // Keep publishing until the recorder's own numbers show it observed a frame
    // on the discovered tap — the rescan runs on the drive loop, so a single
    // publish before it attaches would land in no queue.
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");
    let mut seq = 0u32;
    assert!(
        common::await_condition(Duration::from_secs(30), || {
            pubr.publish_raw(&build_frame(0xC133, seq, 1_000 + u64::from(seq), b"d"))
                .expect("publish");
            seq += 1;
            drain_status(&mut status).into_iter().any(|s| s.pinned > 0)
        }),
        "the rescan must discover this topic and drain it"
    );

    shutdown.store(true, Ordering::Relaxed);
    let summary = join_bagd(handle, &shutdown, "the discovery arm").expect("recorder");

    assert_eq!(
        depth_of(&summary, &topic),
        BUDGETED_DEPTH,
        "a DISCOVERED tap must take the same byte budget an arm-time one does — \
         the gate is resolved once per recorder and handed to both `open_tap` \
         sites"
    );
    assert_eq!(
        summary.unpriced_taps, 0,
        "a discovered tap opened against a live publisher prices its own slot"
    );
}

/// One slot at [`SLICE`]: 64 KiB of payload plus iceoryx2's 48-byte
/// publish-subscribe sample header, already a multiple of the header's 8-byte
/// alignment. Written out rather than called from the production rule, so this
/// file's oracles are arithmetic it states itself. The header was 40 bytes
/// before iceoryx2 0.10 added `payload_offset`; `transport::tap_depth_tests`
/// reads the size off the type and is where that number is pinned.
const SLOT: u64 = 64 * 1024 + 48;

/// `shm_pinned_bytes` is a HIGH-WATER that does NOT recede when the queues
/// drain — asserted LIVE off `/bagd/status`, which is also the surface an
/// operator reads it on.
///
/// The occupancy is forced deterministically rather than raced for: the drain
/// gate holds the recorder off its taps while a burst accumulates, then opens,
/// so the first drain takes the whole burst.
///
/// The RECEDING half is what the status feed buys. A recorder publishes a status
/// frame every `status_period`, so after the burst is drained the arm keeps
/// reading frames while the queues sit EMPTY — many passes' worth — and requires
/// the figure to hold. An instantaneous reading would be back at 0 on the very
/// next frame, which is what a healthy robot's queues hold almost all the time,
/// and no end-of-run assertion could tell the two apart.
#[test]
fn the_pinned_figure_is_a_high_water_that_does_not_recede_when_the_queues_drain() {
    let mgr = make_manager(16);
    let dir = temp_dir("hw");
    let topic = unique_topic("hw");
    let out = unique_out("hw");
    let ready = unique_ready_file("hw");

    const BURST: u32 = 8;
    let mut pubr = publisher_with_provisioning(&mgr, &topic, BORROW, CEILING, SLICE);

    let gate = Arc::new(AtomicBool::new(false));
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = BagdConfig::new(out, vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = Some(Duration::from_millis(20));
    cfg.flashback = Some(settings(&dir, true, BUDGET));
    cfg.ready_file = Some(ready.clone());
    cfg.fault_inject_tap_drain_gate = Some(gate.clone());

    let handle = {
        let m = Arc::clone(&mgr);
        let c = cfg.clone();
        let s = Arc::clone(&shutdown);
        std::thread::spawn(move || run_bagd(m, c, s))
    };
    await_bagd_ready(&ready, "the high-water arm");
    // The recorder created its status service inside `setup`, so this attaches
    // AFTER the handshake — and before the burst, so no frame of the sequence
    // under test can be missed.
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");

    // The tap is ARMED and draining NOTHING, so the burst accumulates in the
    // topic's queue. BURST is far under the tap's budgeted depth (63), so none
    // of it is reclaimed and the occupancy really does reach BURST.
    for seq in 0..BURST {
        pubr.publish_raw(&build_frame(0xC133, seq, 1_000 + u64::from(seq), b"hw"))
            .expect("publish");
    }
    gate.store(true, Ordering::Relaxed);

    // (1) The figure CLIMBS to the burst's worth of slots. A condition with a
    //     generous liveness ceiling — load can delay it, never invert it.
    let expected = SLOT * u64::from(BURST);
    let mut readings: Vec<u64> = Vec::new();
    let reached = common::await_condition(Duration::from_secs(30), || {
        for s in drain_status(&mut status) {
            readings.push(s.pinned);
        }
        readings.last().copied() == Some(expected)
    });
    assert!(
        reached,
        "the status feed must report the peak occupancy ({BURST} slots of {SLOT} \
         bytes = {expected}); readings so far: {readings:?}"
    );

    // (2) …and HOLDS while the queues sit empty. `STEADY_FRAMES` further status
    //     frames, every one of which an instantaneous figure would report as 0.
    const STEADY_FRAMES: usize = 5;
    let before = readings.len();
    let held = common::await_condition(Duration::from_secs(30), || {
        for s in drain_status(&mut status) {
            readings.push(s.pinned);
        }
        readings.len() >= before + STEADY_FRAMES
    });
    assert!(
        held,
        "the recorder must keep publishing status while it idles; got {} frames",
        readings.len()
    );
    for (i, v) in readings.iter().enumerate().skip(before) {
        assert_eq!(
            *v, expected,
            "reading {i} came back at {v} after the queues drained — the figure is \
             a HIGH-WATER, not an instantaneous occupancy (all readings: {readings:?})"
        );
    }

    // (3) THE DISCRIMINATOR the idle stretch cannot supply.
    //
    // An idle pass writes nothing at all (the bump is guarded on a nonzero
    // drain), so steps 1-2 above are equally satisfied by a figure that merely
    // remembers the LAST NON-EMPTY drain — MEASURED: that figure passes them.
    // What separates the two is a SMALLER drain landing afterwards, which is
    // also the ordinary shape of a robot that bursts once and then trickles.
    //
    // One more frame, and the proof it was DRAINED is the window's own byte
    // count growing (`frames_recorded` cannot serve: it is write-side, and this
    // recorder writes no bag).
    let window_before = last_window_bytes(&mut status, &mut readings)
        .expect("the status feed reports the window's held bytes");
    pubr.publish_raw(&build_frame(0xC133, BURST, 9_999, b"trickle"))
        .expect("publish");
    let mut after: Vec<u64> = Vec::new();
    let grew = common::await_condition(Duration::from_secs(30), || {
        for s in drain_status(&mut status) {
            after.push(s.pinned);
            if s.window.is_some_and(|w| w > window_before) {
                return true;
            }
        }
        false
    });
    assert!(
        grew,
        "the trickle frame must reach the window (held bytes must grow past \
         {window_before}) — otherwise this step proves nothing"
    );
    let low = SLOT; // what a one-frame drain alone would price.
    assert!(
        after.iter().all(|v| *v == expected),
        "after a ONE-frame drain the figure must still read the PEAK {expected}, \
         not the latest {low} — a value that tracks the last non-empty drain is \
         not a high-water (readings: {after:?})"
    );

    shutdown.store(true, Ordering::Relaxed);
    let summary = join_bagd(handle, &shutdown, "the high-water arm").expect("recorder");

    let row = summary
        .record_health
        .topics
        .get(&topic)
        .expect("a health row");
    // The durable per-topic figure agrees with the live one — and it is also the
    // proof the WHOLE burst was drained in one pass, since `SLOT * BURST` is
    // reachable only from an eight-frame drain. (`frames_recorded` cannot say
    // this: it is write-side, and a window-only recorder writes no bag.)
    assert_eq!(
        row.shm_pinned_bytes,
        Some(expected),
        "the durable per-topic figure agrees with the live one, and pins that all \
         {BURST} frames were taken in one drain"
    );
    assert_eq!(
        summary.shm_pinned_bytes, expected,
        "the run total is the sum over its taps, and there is one"
    );
    assert_eq!(summary.unpriced_taps, 0);
}

/// One `/bagd/status` frame, reduced to the three numbers this file reads.
#[derive(Debug, Clone, Copy)]
struct Status {
    /// `shm_pinned_bytes` — the figure under test.
    pinned: u64,
    /// `messages` — the WRITE-side count, the only progress signal a `--record`
    /// recorder has here (a window-only one leaves it at 0 forever).
    messages: u64,
    /// `flashback.window_bytes` — how much the rolling window holds. `None` on a
    /// recorder with no plane. Used as the "a frame really was drained" signal
    /// on a window-only run, where no write-side counter moves.
    window: Option<u64>,
    /// `shm_unpriced_taps` — taps that could not price one slot. Read beside
    /// `pinned` because a `0` there means "nothing pinned" only when this is
    /// also `0`; otherwise it means "nothing measured".
    unpriced: u64,
}

/// Block until the next status frame and report the window's held bytes,
/// draining any pinned readings into `readings` on the way so the caller's
/// sequence stays complete.
fn last_window_bytes(
    sub: &mut cerulion_core::transport::subscriber::DataOnlySubscriber,
    readings: &mut Vec<u64>,
) -> Option<u64> {
    let mut found = None;
    common::await_condition(Duration::from_secs(30), || {
        for s in drain_status(sub) {
            readings.push(s.pinned);
            if let Some(w) = s.window {
                found = Some(w);
            }
        }
        found.is_some()
    });
    found
}

/// Every reading in the status frames currently queued — while also pinning the
/// two LABEL fields that ride beside the pinned figure.
///
/// The scope label is asserted on every frame rather than once: it is what stops
/// a reader comparing this number against machine-wide SHM or against the pool's
/// apparent reservation, and a label that appeared on some frames and not others
/// would be worse than none.
fn drain_status(sub: &mut cerulion_core::transport::subscriber::DataOnlySubscriber) -> Vec<Status> {
    let mut out = Vec::new();
    // Drain in chunks of the DEFAULT borrow budget (2) and let each chunk's
    // `OwnedInboundSample`s DROP before asking for more: an owned sample reads
    // zero-copy out of SHM, so it HOLDS a borrow until dropped, and
    // `drain_owned`'s own contract says "pass `max` <= remaining budget or
    // iceoryx2 fails with `ExceedsMaxBorrows`". The status tap is
    // default-provisioned (`subscriber_max_borrowed_samples = 2`), so the old
    // `drain_owned(64, ..)` demanded up to 64 concurrent borrows — fine on a
    // quiet desk that never queued more than 2 status frames between polls,
    // and a guaranteed `ReceiveError::ExceedsMaxBorrows` on a loaded CI runner
    // whose backlog crossed the budget (macOS shard 2, run 32391503296).
    const STATUS_TAP_BORROW_BUDGET: usize = 2;
    loop {
        let mut batch = Vec::new();
        let drained = sub
            .drain_owned(STATUS_TAP_BORROW_BUDGET, &mut batch)
            .expect("drain status");
        parse_status_batch(batch, &mut out);
        if drained == 0 {
            break;
        }
    }
    out
}

/// Parse one drained batch into `out`, CONSUMING the batch — the samples (and
/// the SHM borrows they hold) drop at the end of this call, which is what
/// makes the chunked loop above budget-safe.
fn parse_status_batch(
    batch: Vec<cerulion_core::transport::subscriber::OwnedInboundSample>,
    out: &mut Vec<Status>,
) {
    for frame in batch {
        let payload = frame.payload();
        if payload.len() <= cerulion_core::wire::WireHeader::SIZE {
            continue;
        }
        let body = &payload[cerulion_core::wire::WireHeader::SIZE..];
        let v: serde_json::Value = match serde_json::from_slice(body) {
            Ok(v) => v,
            Err(_) => continue,
        };
        assert_eq!(
            v.get("shm_pinned_scope").and_then(|s| s.as_str()),
            Some("bagd_local_taps"),
            "every status frame must LABEL the pinned figure's scope — it is \
             neither machine-wide SHM nor the apparent reservation"
        );
        let unpriced = v.get("shm_unpriced_taps").and_then(|n| n.as_u64()).expect(
            "every status frame must carry the unpriced count beside the \
                 pinned figure, or a 0 reads as 'nothing pinned' when it means \
                 'nothing measured'",
        );
        let pinned = v
            .get("shm_pinned_bytes")
            .and_then(|n| n.as_u64())
            .expect("every status frame carries the pinned figure");
        let messages = v
            .get("messages")
            .and_then(|n| n.as_u64())
            .expect("every status frame carries its message count");
        let window = v
            .get("flashback")
            .and_then(|f| f.get("window_bytes"))
            .and_then(|n| n.as_u64());
        out.push(Status {
            pinned,
            messages,
            window,
            unpriced,
        });
    }
}

// ===========================================================================
// The live slot price
// ===========================================================================
//
// The shapes below are exact, because their numbers
// are the ones the arms assert. A tap opened while only a NARROW
// publisher exists is sized against the narrow slot; a WIDE publisher then
// joins, and a cached price leaves both the budget and the accounting quoting
// the width that no longer governs.

/// The NARROW publisher's slice — the one the tap is sized against.
const NARROW_SLICE: u32 = 64;
/// The WIDE publisher's slice — the one that joins afterwards.
const WIDE_SLICE: u32 = 4096;
/// One slot at [`WIDE_SLICE`]: `align(48 + 4096, 8)` = 4144.
const WIDE_SLOT: u64 = 4144;
/// One slot at [`NARROW_SLICE`]: `align(48 + 64, 8)` = 112. Never an expected
/// value — it is what a CACHED price quotes, and the headline arm
/// asserts the figure is NOT this.
const NARROW_SLOT: u64 = 112;
/// The budget of this shape: 64 KiB, the shipped ingress number over
/// 1024 so a modest ceiling is reachable.
const SHAPE_BUDGET: u64 = 64 * 1024;
/// The service ceiling of this shape, which is what the budgeted depth clamps to
/// (64 KiB / 112 = 585 slots, far past any sane ceiling).
const SHAPE_CEILING: usize = 16;

/// Open a topic whose FIRST publisher is narrow, so the service ceiling and the
/// tap's sizing both come from the narrow width.
fn narrow_publisher(
    mgr: &Arc<cerulion_core::TransportManager>,
    topic: &str,
) -> cerulion_core::transport::publisher::CerulionPublisher {
    publisher_with_provisioning(mgr, topic, BORROW, SHAPE_CEILING, NARROW_SLICE)
}

/// Attach a SECOND, WIDER publisher to a topic that already exists. `max_slice_len`
/// is a per-PUBLISHER property, so this genuinely widens what one queued chunk of
/// the topic costs without touching the service's static config.
fn wide_publisher(
    mgr: &Arc<cerulion_core::TransportManager>,
    topic: &str,
) -> cerulion_core::transport::publisher::CerulionPublisher {
    publisher_with_provisioning(mgr, topic, BORROW, SHAPE_CEILING, WIDE_SLICE)
}

/// THE HEADLINE: a wider publisher joining after the tap opened must move
/// the pinned figure, because the price of a queued chunk is not fixed.
///
/// The shape is exact, and its numbers are the oracle: the tap opens at depth 16
/// against a 112-byte slot; a 4096-byte publisher joins; 16 of ITS samples
/// occupy `16 x 4144` = 66,304 B — OVER the 64 KiB budget — while a cached
/// price reports `16 x 112` = 1,792 B.
///
/// Both numbers are asserted: the equality says the live price is used, and the
/// explicit inequality against 1,664 names the defect, so a future reader can
/// see which reading is being refused rather than trusting that some other
/// number would have failed too.
#[test]
fn a_wider_publisher_joining_after_the_tap_reprices_the_pinned_figure() {
    let mgr = make_manager(16);
    let dir = temp_dir("reprice");
    let topic = unique_topic("reprice");
    let out = unique_out("reprice");
    let ready = unique_ready_file("reprice");

    // The tap is sized while ONLY the narrow publisher exists.
    let _narrow = narrow_publisher(&mgr, &topic);

    let gate = Arc::new(AtomicBool::new(false));
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = BagdConfig::new(out, vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = Some(Duration::from_millis(20));
    cfg.ready_file = Some(ready.clone());
    cfg.flashback = Some(settings(&dir, true, SHAPE_BUDGET));
    cfg.fault_inject_tap_drain_gate = Some(gate.clone());

    let handle = {
        let m = Arc::clone(&mgr);
        let c = cfg.clone();
        let s = Arc::clone(&shutdown);
        std::thread::spawn(move || run_bagd(m, c, s))
    };
    await_bagd_ready(&ready, "the reprice arm");
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");

    // …and the WIDE publisher joins only now, after the depth is already fixed.
    let mut wide = wide_publisher(&mgr, &topic);

    // Exactly the tap's depth, so one drain takes them all and the queue is
    // never over-full — the estimate's clamp is not what is under test here.
    let burst = SHAPE_CEILING as u32;
    let body = vec![0xAB; WIDE_SLICE as usize - cerulion_core::wire::WireHeader::SIZE];
    for seq in 0..burst {
        wide.publish_raw(&build_frame(0xC133, seq, 1_000 + u64::from(seq), &body))
            .expect("publish");
    }
    gate.store(true, Ordering::Relaxed);

    let expected = WIDE_SLOT * u64::from(burst);
    let cached_would_say = NARROW_SLOT * u64::from(burst);
    let mut readings: Vec<u64> = Vec::new();
    let reached = common::await_condition(Duration::from_secs(30), || {
        for s in drain_status(&mut status) {
            readings.push(s.pinned);
        }
        readings.last().copied() == Some(expected)
    });
    assert!(
        reached,
        "the pinned figure must be priced at the LIVE slot ({WIDE_SLOT} B), so \
         {burst} wide samples read {expected}; a price cached when the tap was \
         sized would read {cached_would_say}. readings: {readings:?}"
    );
    assert!(
        !readings.contains(&cached_would_say),
        "the figure must never quote the width the tap was SIZED against \
         ({cached_would_say}); readings: {readings:?}"
    );

    shutdown.store(true, Ordering::Relaxed);
    let summary = join_bagd(handle, &shutdown, "the reprice arm").expect("recorder");
    assert_eq!(
        summary.shm_pinned_bytes, expected,
        "the durable figure agrees with the live one"
    );
    assert_eq!(
        summary
            .record_health
            .topics
            .get(&topic)
            .and_then(|h| h.tap_buffer_depth),
        Some(SHAPE_CEILING as u64),
        "the depth is unchanged — it cannot be, once the queue exists"
    );
}

/// THE REPORT: a tap the joined width has put over its budget SAYS SO,
/// once per regime, naming what an operator needs to act.
///
/// `self.taps` is only ever pushed and indexed — there is no remove or replace
/// path — and a recreate is not a refactor away: it would drop queued frames
/// with no accounting (against the never-evict rule) while the flush path indexes taps
/// POSITIONALLY against channel ids fixed at bag creation. So the tap keeps its
/// depth and the overrun is reported. This arm is what stops that report being
/// deletable in silence.
///
/// INVERSE HARNESS — `run_bagd` on the TEST thread, stimulus on a helper —
/// because `tracing-test` scopes its capture to the test's own span and the
/// recorder's warn would otherwise be invisible (the
/// `discovery_e2e_test::an_incomplete_recording_says_so_on_its_terminal_line`
/// precedent).
#[test]
#[tracing_test::traced_test]
fn a_tap_the_joined_width_put_over_its_budget_says_so_once() {
    let mgr = make_manager(16);
    let dir = temp_dir("overrun");
    let topic = unique_topic("overrun");
    let out = unique_out("overrun");
    let ready = unique_ready_file("overrun");

    let _narrow = narrow_publisher(&mgr, &topic);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = BagdConfig::new(out, vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    cfg.ready_file = Some(ready.clone());
    cfg.flashback = Some(settings(&dir, true, SHAPE_BUDGET));

    let stim_mgr = Arc::clone(&mgr);
    let stim_ready = ready.clone();
    let stim_topic = topic.clone();
    let stim_shutdown = Arc::clone(&shutdown);
    let stimulus = std::thread::spawn(move || {
        await_bagd_ready(&stim_ready, "the overrun arm");
        let mut wide = wide_publisher(&stim_mgr, &stim_topic);
        let body = vec![0xAB; WIDE_SLICE as usize - cerulion_core::wire::WireHeader::SIZE];
        // Several drain passes' worth, so a report that fired PER PASS rather
        // than per regime would be visible as a count.
        let mut seq = 0u32;
        let saw_warn = common::await_condition(Duration::from_secs(30), || {
            for _ in 0..4 {
                let _ = wide.publish_raw(&build_frame(0xC133, seq, 1_000 + u64::from(seq), &body));
                seq += 1;
            }
            std::thread::sleep(Duration::from_millis(20));
            logs_contain("OVER ITS BUDGET")
        });
        assert!(
            saw_warn,
            "the recorder must report the overrun; it published {seq} wide frames"
        );
        stim_shutdown.store(true, Ordering::Relaxed);
    });

    let summary = run_bagd(mgr.clone(), cfg, shutdown.clone()).expect("clean finalize");
    stimulus.join().expect("stimulus");

    // The capacity the tap can now pin, and the budget it was sized to.
    let capacity = WIDE_SLOT * SHAPE_CEILING as u64;
    assert!(
        capacity > SHAPE_BUDGET,
        "the fixture must really be over budget: {capacity} vs {SHAPE_BUDGET}"
    );
    logs_assert(|lines: &[&str]| {
        let hits: Vec<&&str> = lines
            .iter()
            .filter(|l| l.contains("OVER ITS BUDGET"))
            .collect();
        if hits.len() != 1 {
            return Err(format!(
                "the overrun must be reported ONCE per regime, not per pass; got {} lines: {hits:?}",
                hits.len()
            ));
        }
        let line = hits[0];
        for needle in [
            "WARN",
            &format!("budget_bytes={SHAPE_BUDGET}"),
            &format!("widest_slot_bytes={WIDE_SLOT}"),
            &format!("capacity_bytes={capacity}"),
            &format!("overrun_bytes={}", capacity - SHAPE_BUDGET),
            "CERULION_FLASHBACK_TAP_BUDGET_MB",
        ] {
            if !line.contains(needle) {
                return Err(format!("the report must carry {needle:?}; got: {line}"));
            }
        }
        if !line.contains(&topic) {
            return Err(format!("the report must name the topic; got: {line}"));
        }
        Ok(())
    });

    // …and the recording is UNAFFECTED: the tap keeps its depth and keeps
    // recording, which is the whole reason this is a report rather than a
    // refusal.
    // The over-budget state is DERIVED onto the durable
    // verdict row, not only logged — which is the whole point (the
    // depth cannot be changed, so the fact must outlive the one line that
    // reports it). The unit and renderer arms hand-construct an `OverBudget`,
    // so this is the only place the DERIVATION runs, and an implementation that
    // always answered `None` passes both of those.
    //
    // Dropping the `budget_bytes > 0` guard is not caught here, for
    // two reasons: this tap is budgeted, so
    // the guard is satisfied either way — and dropping it has no production
    // effect at all, because a ceiling-deep tap is never PRICED
    // (`TapDepthMode::Ceiling => None` at open; both drain-site folds are gated
    // on `prices_occupancy`), so the enclosing `and_then` short-circuits before
    // the guard is consulted. The ceiling-deep NEGATIVE is asserted by
    // `a_ceiling_deep_tap_claims_no_budget_occupancy_at_all` below.
    let row = summary
        .record_health
        .topics
        .get(&topic)
        .and_then(|h| h.absorbance)
        .expect("the over-budget tap must carry a verdict row");
    let ob = row
        .over_budget
        .expect("…and that row must carry the over-budget clause");
    assert_eq!(ob.budget_bytes, SHAPE_BUDGET);
    assert_eq!(ob.widest_slot_bytes, WIDE_SLOT);
    assert_eq!(ob.capacity_bytes, capacity);
    assert_eq!(
        row.depth_mode(),
        "budgeted",
        "a window-only tap is budgeted, and that is what decides its remedy"
    );

    assert!(
        summary
            .record_health
            .topics
            .get(&topic)
            .and_then(|h| h.shm_pinned_bytes)
            .is_some_and(|p| p > 0),
        "the over-budget tap must still be draining"
    );
}

/// THE MID-DRAIN BOUND: frames arriving DURING a drain may inflate the estimate
/// toward the ceiling, and must never carry it past.
///
/// This is the property that makes an UPPER BOUND true rather than merely
/// convenient. A pass drains until the queue is empty, so a producer publishing
/// throughout is taken in the same pass and the batch can exceed anything ever
/// simultaneously queued, which is why no inference from
/// "the queue went empty" can substitute for a measurement. What CAN be
/// guaranteed is the ceiling: `depth x slot` is the most the queue can pin,
/// slots being released back to the pool as the tap drains.
///
/// An INVARIANT, not a threshold — load can change how much is drained per pass
/// and can only make the arm exercise the mid-drain case harder.
#[test]
fn mid_drain_arrivals_never_carry_the_estimate_past_the_pin_ceiling() {
    let mgr = make_manager(16);
    let dir = temp_dir("bound");
    let topic = unique_topic("bound");
    let out = unique_out("bound");
    let ready = unique_ready_file("bound");

    let mut pubr = publisher_with_provisioning(&mgr, &topic, BORROW, SHAPE_CEILING, WIDE_SLICE);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = BagdConfig::new(out, vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = Some(Duration::from_millis(20));
    cfg.ready_file = Some(ready.clone());
    cfg.flashback = Some(settings(&dir, true, SHAPE_BUDGET));

    let handle = {
        let m = Arc::clone(&mgr);
        let c = cfg.clone();
        let s = Arc::clone(&shutdown);
        std::thread::spawn(move || run_bagd(m, c, s))
    };
    await_bagd_ready(&ready, "the bound arm");
    let mut status = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");

    // Publish CONTINUOUSLY while the recorder drains, which is the shape that
    // puts arrivals inside a drain pass.
    let body = vec![0xAB; WIDE_SLICE as usize - cerulion_core::wire::WireHeader::SIZE];
    let ceiling = WIDE_SLOT * SHAPE_CEILING as u64;
    let mut seq = 0u32;
    let mut readings: Vec<u64> = Vec::new();
    let flowed = common::await_condition(Duration::from_secs(30), || {
        for _ in 0..8 {
            let _ = pubr.publish_raw(&build_frame(0xC133, seq, 1_000 + u64::from(seq), &body));
            seq += 1;
        }
        for s in drain_status(&mut status) {
            readings.push(s.pinned);
        }
        readings.iter().any(|p| *p > 0) && readings.len() >= 5
    });
    assert!(
        flowed,
        "the tap must be draining a flowing producer; readings: {readings:?}"
    );
    for (i, p) in readings.iter().enumerate() {
        assert!(
            *p <= ceiling,
            "reading {i} = {p} exceeds the pin ceiling ({SHAPE_CEILING} slots x \
             {WIDE_SLOT} B = {ceiling}) — an estimate that can exceed the \
             hardware maximum is not a bound (readings: {readings:?})"
        );
    }

    shutdown.store(true, Ordering::Relaxed);
    let summary = join_bagd(handle, &shutdown, "the bound arm").expect("recorder");
    assert!(
        summary.shm_pinned_bytes <= ceiling,
        "the durable figure obeys the same ceiling: {} vs {ceiling}",
        summary.shm_pinned_bytes
    );
    assert!(
        summary.shm_pinned_bytes > 0,
        "…and is not vacuously satisfied by never having drained anything"
    );
}

/// STRUCTURAL: the recorder really reads `CERULION_FLASHBACK_TAP_BUDGET_MB` into
/// the settings it hands its taps.
///
/// Behaviourally unreachable from this file: the env is read inside
/// `bagd_cli_run`, which builds a `BagdConfig` from argv and then RUNS a whole
/// recorder, and every arm above dials `tap_budget_bytes` directly so the suite
/// stays parallel-safe. So the wiring — the one place this feature could ship
/// INERT, with the parser oracle-tested and nothing calling it — is walked over
/// the crate's own source instead.
///
/// The parser itself is pinned in `cerulion_core::flashback`
/// (`the_tap_budget_resolves_through_the_shared_parser`), and the gate that
/// consumes the field in `tap_depth_gate_tests`.
#[test]
fn the_recorder_reads_the_tap_budget_from_the_environment() {
    let src = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/src/lib.rs"))
        .expect("bagd's own source");
    let code = code_only(&src);

    for needle in [
        // The knob NAME, from the shared constant rather than a second literal.
        "FLASHBACK_TAP_BUDGET_MB_ENV",
        // Through the SHARED parser, so a `0` and an unparseable value behave
        // here exactly as they do for every other Flashback ceiling.
        "resolve_tap_budget_bytes",
        // …and into the field the gate reads.
        "tap_budget_bytes,",
    ] {
        assert!(
            code.contains(needle),
            "the recorder must wire the tap budget: `{needle}` is absent from a \
             comment-stripped view of cerulion_bagd/src/lib.rs"
        );
    }

    // ANTI-TAUTOLOGY: the stripper must not have eaten the file. Without this,
    // a `code_only` that returned an empty string would make every assertion
    // above fail loudly — but one that returned the WHOLE file including
    // comments would make them pass on prose alone, which is the failure this
    // guards.
    assert!(
        code.contains("fn tap_depth_mode"),
        "the stripped view must still contain real code"
    );
    let doc_phrase = "THE mode gate, as a pure function of the";
    assert!(
        src.contains(doc_phrase),
        "the doc-comment phrase this arm keys on must exist in the raw source, or the \
         negative below proves nothing"
    );
    assert!(
        !code.contains(doc_phrase),
        "…and must NOT contain the doc comments, or the needles above could be \
         satisfied by prose"
    );
}

/// THE NEGATIVE HALF of the over-budget row: a CEILING-DEEP (`--record`) tap
/// claims no budget occupancy at all — neither over it, nor unpriced.
///
/// The positive arm above drives a budgeted tap, so on its own it cannot tell an
/// implementation that derives the clause correctly from one that stamps it on
/// everything. This is the other side, and it also pins the two facts the
/// remedy branch keys on: such a tap is `"ceiling"`, so the fix it names is the
/// topic's own `subscriber_buffer_size` rather than a recorder budget that does
/// not apply to it.
#[test]
fn a_ceiling_deep_tap_claims_no_budget_occupancy_at_all() {
    let mgr = make_manager(16);
    let topic = unique_topic("ceilingrow");
    // `fb: None` — no Flashback plane at all, which is the `--record` shape and
    // the one that opens a tap at the topic's own ceiling.
    let summary = run_one(&mgr, &topic, "ceilingrow", None, |pubr| {
        let body = vec![0xCD; 128];
        for seq in 0..8u32 {
            let _ = pubr.publish_raw(&build_frame(0xC133, seq, 1_000 + u64::from(seq), &body));
        }
    });

    let row = summary
        .record_health
        .topics
        .get(&topic)
        .and_then(|h| h.absorbance)
        .expect("every tap carries a verdict row, budgeted or not");
    assert!(
        row.over_budget.is_none(),
        "a tap with no budget cannot be OVER one: {row:?}"
    );
    // …and the absence is a PROVEN absence rather than an unmeasured one. These
    // are different facts and the row keeps them apart: `budget_unpriced` is
    // what a budgeted tap nobody could price reports, and a ceiling-deep tap is
    // not that either.
    assert!(
        !row.budget_unpriced,
        "a ceiling-deep tap has no budget to be unpriced against: {row:?}"
    );
    assert_eq!(row.budget_bytes, None, "{row:?}");
    assert_eq!(
        row.depth_mode(),
        "ceiling",
        "which is what decides the remedy this row names: {row:?}"
    );
    assert!(
        row.remedy().contains("subscriber_buffer_size"),
        "a ceiling-deep tap's fix is the TOPIC's own depth — no recorder knob moves \
         it, so naming the tap budget here would be wrong every time: {}",
        row.remedy()
    );
    // ANTI-TAUTOLOGY: the row is real, not a default. This tap really was opened
    // at the topic's provisioned ceiling.
    assert_eq!(
        row.tap_buffer_depth, CEILING as u64,
        "the row must price the depth this tap actually got: {row:?}"
    );
}
