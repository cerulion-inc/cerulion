// SPDX-License-Identifier: AGPL-3.0-only
//! The single-process scheduler → trace-ring hook.
//!
//! These pin the [`Scheduler::set_trace_ring_producer`] hook END-TO-END over a
//! REAL POSIX-SHM [`TraceRingOwner`]/producer/consumer (no iceoryx2 — the trace
//! ring is a standalone SPSC POSIX-SHM ring), against HAND oracles (never a
//! two-run self-compare):
//!
//! - `completeness`: with an in-memory `--trace-limit` SMALLER than the total
//!   fire count, the ring receives ALL fire records while the in-memory trace
//!   is capped to the newest `limit` (the ring push is UNGATED by
//!   the cap, so the bag's trace is complete).
//! - `node_idx_manifest_consistency`: every fire record's `node_idx` resolves
//!   (through the ring MANIFEST decoded by the consumer) to the node that
//!   actually fired, matched against a hand-known 2-node distinct-period fire
//!   sequence (record `node_idx` ↔ bagged manifest agree).
//! - `recording_on_vs_off_trace_is_byte_identical`: the recording firewall — the
//!   in-memory trace is BYTE-IDENTICAL whether or not a ring is installed (the
//!   ring is a pure side-effect that never perturbs what fires), AND both equal
//!   a hand oracle (so it is not a pure two-run tautology).
//!
//! # SHM-touching — run in the shared-memory test window
//!
//! Each test mints a real `TraceRingOwner` (POSIX `shm_open`). Ring names are
//! `hook_<label>_<pid>`-scoped and the owner unlinks on drop, so the
//! file is parallel-safe, even on a machine whose POSIX SHM namespace is
//! shared with other test runs. `#[serial]` is
//! kept as belt-and-suspenders for any future in-binary sibling.

#![cfg(unix)]

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::scheduler::{NodeConfig, Scheduler, TraceEntry, TriggerPolicy};
use cerulion_core::trace_ring::{
    TraceRingConsumer, TraceRingOwner, TraceRingRecord, RECORD_TYPE_FIRE, RECORD_TYPE_STEP_BOUNDARY,
};
use serial_test::serial;

/// A `Period` policy with unlimited catch-up (all interval boundaries fire).
fn period(interval_ms: u64) -> TriggerPolicy {
    TriggerPolicy::Period {
        interval: Duration::from_millis(interval_ms),
        max_catchup: None,
    }
}

/// A no-op node callback (the trace/ring content is driven by the SCHEDULE, not
/// the tick body).
fn noop() -> Box<dyn FnMut() + Send> {
    Box::new(|| {})
}

/// A unique, pid-scoped ring tag so parallel/repeat runs never collide.
fn ring_tag(label: &str) -> String {
    format!("hook_{}_{}", label, std::process::id())
}

/// Build a fresh `VirtualClock` scheduler.
fn fresh_scheduler() -> Scheduler {
    Scheduler::with_virtual_clock(Arc::new(VirtualClock::new()))
}

/// Drain every record currently on the ring named `name` into a fresh Vec.
fn drain_ring(name: &str) -> Vec<TraceRingRecord> {
    let mut consumer = TraceRingConsumer::open(name).expect("open trace ring consumer");
    let mut out = Vec::new();
    consumer
        .drain(&mut out)
        .expect("drain trace ring (no overrun)");
    out
}

/// One 40-byte FIRE record with the frozen field map (record_type =
/// FIRE, reserved = 0, duration_ns = 0 with B-dur recording off).
fn fire_record(step: u64, fire_time_ns: u64, node_idx: u32, global_level: u32) -> TraceRingRecord {
    TraceRingRecord {
        step,
        fire_time_ns,
        duration_ns: 0,
        node_idx,
        global_level,
        record_type: RECORD_TYPE_FIRE,
        reserved: 0,
    }
}

/// One 40-byte STEP-BOUNDARY record (the contract addendum): pushed at
/// `begin_step` BEFORE any of the step's fires, `fire_time_ns` = the step's
/// ADVANCED clock, `node_idx`/`global_level`/`duration_ns` = 0.
fn boundary_record(step: u64, advanced_ns: u64) -> TraceRingRecord {
    TraceRingRecord {
        step,
        fire_time_ns: advanced_ns,
        duration_ns: 0,
        node_idx: 0,
        global_level: 0,
        record_type: RECORD_TYPE_STEP_BOUNDARY,
        reserved: 0,
    }
}

// ---------------------------------------------------------------------------
// (a) Completeness past the in-memory cap (the ungated ring push)
// ---------------------------------------------------------------------------

/// The RING receives ALL fire records even when the in-memory `--trace-limit`
/// is SMALLER than the total fire count: a single `period_ms=10` node stepped
/// 10× (one fire/step) with an in-memory cap of 3 leaves the in-memory trace at
/// the NEWEST 3 fires, while the ring carries the full 10-record sequence (hand
/// oracle). Reverting the "ring push FIRST, ungated by the cap" order would
/// truncate the ring to the cap and fail this.
#[test]
#[serial]
fn ring_receives_all_fires_when_in_memory_trace_is_capped() {
    const STEPS: u64 = 10;
    const CAP: usize = 3;
    let tag = ring_tag("completeness");

    let mut ring_owner = TraceRingOwner::create(&tag, 64, 0, &["n0"]).expect("create ring");
    let ring_name = ring_owner.name().to_string();
    let producer = ring_owner.producer().expect("mint producer");

    let mut scheduler = fresh_scheduler();
    scheduler.set_trace_limit(CAP);
    scheduler.set_trace_ring_producer(producer, &["n0".to_string()]);
    scheduler
        .add_node(NodeConfig {
            id: "n0".to_string(),
            policy: period(10),
            callback: noop(),
        })
        .unwrap();

    for _ in 0..STEPS {
        scheduler.step_ms(10);
    }

    // In-memory trace is capped to the NEWEST 3 (steps 7,8,9; fire_times
    // 80/90/100 ms).
    let in_memory: Vec<TraceEntry> = scheduler.trace().to_vec();
    assert_eq!(in_memory.len(), CAP, "in-memory trace capped to the limit");
    let in_memory_oracle: Vec<TraceEntry> = (7..STEPS)
        .map(|k| TraceEntry {
            node_id: Arc::from("n0"),
            step: k,
            fire_time_ns: (k + 1) * 10_000_000,
            global_level: 0,
            duration_ns: 0,
            discarded: false,
        })
        .collect();
    assert_eq!(
        in_memory, in_memory_oracle,
        "in-memory trace keeps the NEWEST {CAP} fires"
    );

    // The RING carries ALL 10 fires — complete regardless of the in-memory
    // cap — each step led by its StepBoundary record (contract addendum:
    // begin_step pushes the boundary BEFORE the step's fires).
    let records = drain_ring(&ring_name);
    let ring_oracle: Vec<TraceRingRecord> = (0..STEPS)
        .flat_map(|k| {
            [
                boundary_record(k, (k + 1) * 10_000_000),
                fire_record(k, (k + 1) * 10_000_000, 0, 0),
            ]
        })
        .collect();
    assert_eq!(
        records, ring_oracle,
        "the ring receives the FULL boundary+fire sequence, ungated by the in-memory cap"
    );

    drop(ring_owner);
}

// ---------------------------------------------------------------------------
// (b) node_idx ↔ manifest consistency
// ---------------------------------------------------------------------------

/// Every fire record's `node_idx` resolves (through the MANIFEST the consumer
/// decodes from the ring) to the node that actually fired. A 2-node
/// distinct-period graph (`alpha`=period5, `beta`=period10) run 2 steps yields
/// a hand-known fire sequence; the resolved (node-id, fire_time) sequence is
/// asserted against that oracle AND against the scheduler's own in-memory trace
/// 1:1 — proving the manifest table order the ring was created with matches the
/// scheduler's node order.
#[test]
#[serial]
fn record_node_idx_resolves_through_manifest_to_the_firing_node() {
    let tag = ring_tag("manifest");
    let node_ids = ["alpha", "beta"];

    let mut ring_owner = TraceRingOwner::create(&tag, 64, 0, &node_ids).expect("create ring");
    let ring_name = ring_owner.name().to_string();
    let producer = ring_owner.producer().expect("mint producer");

    let mut scheduler = fresh_scheduler();
    scheduler.set_trace_ring_producer(
        producer,
        &node_ids.iter().map(|s| s.to_string()).collect::<Vec<_>>(),
    );
    scheduler
        .add_node(NodeConfig {
            id: "alpha".to_string(),
            policy: period(5),
            callback: noop(),
        })
        .unwrap();
    scheduler
        .add_node(NodeConfig {
            id: "beta".to_string(),
            policy: period(10),
            callback: noop(),
        })
        .unwrap();

    scheduler.step_ms(10); // step 0: alpha@5, alpha@10, beta@10
    scheduler.step_ms(10); // step 1: alpha@15, alpha@20, beta@20

    // Hand oracle of the fire sequence: (node-id, step, fire_time_ns).
    let oracle: [(&str, u64, u64); 6] = [
        ("alpha", 0, 5_000_000),
        ("alpha", 0, 10_000_000),
        ("beta", 0, 10_000_000),
        ("alpha", 1, 15_000_000),
        ("alpha", 1, 20_000_000),
        ("beta", 1, 20_000_000),
    ];

    // ONE consumer (the ring is strictly SPSC): it decodes the MANIFEST from
    // the on-ring bytes (the real bagd path — NOT the owner's in-memory copy)
    // and drains the records. `node_ids()` is the manifest decoded at `open`.
    let mut consumer = TraceRingConsumer::open(&ring_name).expect("open consumer");
    let manifest: Vec<String> = consumer.node_ids().to_vec();
    assert_eq!(
        manifest,
        vec!["alpha".to_string(), "beta".to_string()],
        "the on-ring manifest table order == node insertion order"
    );
    let mut records = Vec::new();
    consumer.drain(&mut records).expect("drain (no overrun)");

    // Contract addendum: each step is led by its StepBoundary; the FIRE
    // records follow. Split the stream and pin the boundary half by hand.
    let (boundaries, fires): (Vec<&TraceRingRecord>, Vec<&TraceRingRecord>) = records
        .iter()
        .partition(|r| r.record_type == RECORD_TYPE_STEP_BOUNDARY);
    assert_eq!(
        boundaries,
        vec![
            &boundary_record(0, 10_000_000),
            &boundary_record(1, 20_000_000)
        ],
        "one boundary per step at the step's advanced clock"
    );
    assert_eq!(fires.len(), oracle.len(), "one FIRE record per fire");

    // Resolve node_idx → node id via the manifest and compare to the oracle.
    let resolved: Vec<(String, u64, u64)> = fires
        .iter()
        .map(|r| {
            assert_eq!(r.record_type, RECORD_TYPE_FIRE, "fires partitioned above");
            assert_eq!(r.global_level, 0, "flat scheduler → global_level 0");
            let name = manifest
                .get(r.node_idx as usize)
                .unwrap_or_else(|| panic!("node_idx {} out of manifest range", r.node_idx))
                .clone();
            (name, r.step, r.fire_time_ns)
        })
        .collect();
    let expect: Vec<(String, u64, u64)> = oracle
        .iter()
        .map(|(n, s, t)| (n.to_string(), *s, *t))
        .collect();
    assert_eq!(
        resolved, expect,
        "each record's node_idx resolves to the node that actually fired"
    );

    // And the resolved sequence equals the scheduler's own in-memory trace 1:1.
    let in_memory: Vec<(String, u64, u64)> = scheduler
        .trace()
        .iter()
        .map(|e| (e.node_id.to_string(), e.step, e.fire_time_ns))
        .collect();
    assert_eq!(
        resolved, in_memory,
        "ring records match the in-memory trace 1:1 (same nodes, same order)"
    );

    drop(ring_owner);
}

// ---------------------------------------------------------------------------
// (c) Recording-ON vs recording-OFF fire-sequence byte-identity (firewall)
// ---------------------------------------------------------------------------

/// The in-memory trace is BYTE-IDENTICAL whether or not a trace-ring hook is
/// installed — the hook is a pure side-effect that never changes what fires or
/// in what order (the recording firewall). To avoid a pure two-run tautology the
/// trace is ALSO asserted against a hand oracle, and the ON run's ring is shown
/// to carry the matching records.
#[test]
#[serial]
fn recording_on_vs_off_trace_is_byte_identical() {
    // Build + run a 2-node graph; optionally install a ring. Returns the trace.
    fn run(with_ring: Option<&str>) -> (Vec<TraceEntry>, Option<Vec<TraceRingRecord>>) {
        let node_ids = ["src", "aux"];
        let mut owner = with_ring
            .map(|tag| TraceRingOwner::create(tag, 64, 0, &node_ids).expect("create ring"));
        let mut scheduler = fresh_scheduler();
        if let Some(owner) = owner.as_mut() {
            let producer = owner.producer().expect("mint producer");
            scheduler.set_trace_ring_producer(
                producer,
                &node_ids.iter().map(|s| s.to_string()).collect::<Vec<_>>(),
            );
        }
        scheduler
            .add_node(NodeConfig {
                id: "src".to_string(),
                policy: period(10),
                callback: noop(),
            })
            .unwrap();
        scheduler
            .add_node(NodeConfig {
                id: "aux".to_string(),
                policy: period(20),
                callback: noop(),
            })
            .unwrap();
        for _ in 0..3 {
            scheduler.step_ms(10);
        }
        let trace = scheduler.trace().to_vec();
        let records = owner.as_ref().map(|o| drain_ring(o.name()));
        // Keep the owner alive until after the drain, then unlink.
        drop(owner);
        (trace, records)
    }

    let (off_trace, off_records) = run(None);
    assert!(off_records.is_none(), "no ring installed on the OFF run");
    let (on_trace, on_records) = run(Some(&ring_tag("firewall")));

    // Hand oracle: src(p10) fires each step; aux(p20) fires at 20ms only.
    // step 0 (→10ms): src@10.  step 1 (→20ms): src@20, aux@20.  step 2 (→30ms): src@30.
    let oracle = |node: &str, step: u64, t: u64| TraceEntry {
        node_id: Arc::from(node),
        step,
        fire_time_ns: t,
        global_level: 0,
        duration_ns: 0,
        discarded: false,
    };
    let trace_oracle = vec![
        oracle("src", 0, 10_000_000),
        oracle("src", 1, 20_000_000),
        oracle("aux", 1, 20_000_000),
        oracle("src", 2, 30_000_000),
    ];
    assert_eq!(off_trace, trace_oracle, "OFF trace matches the hand oracle");
    assert_eq!(on_trace, trace_oracle, "ON trace matches the hand oracle");
    // The firewall: the two in-memory traces are byte-identical.
    assert_eq!(
        on_trace, off_trace,
        "the trace-ring hook must not perturb the fire sequence (firewall)"
    );

    // And the ON run's ring carries the matching records (idx 0=src, 1=aux),
    // each step led by its StepBoundary (contract addendum). The boundaries
    // are RING-ONLY — the in-memory trace oracle above is UNCHANGED, which is
    // exactly the firewall statement.
    let records = on_records.expect("ring installed on the ON run");
    let ring_oracle = vec![
        boundary_record(0, 10_000_000),
        fire_record(0, 10_000_000, 0, 0),
        boundary_record(1, 20_000_000),
        fire_record(1, 20_000_000, 0, 0),
        fire_record(1, 20_000_000, 1, 0),
        boundary_record(2, 30_000_000),
        fire_record(2, 30_000_000, 0, 0),
    ];
    assert_eq!(
        records, ring_oracle,
        "ring mirrors the boundary+fire sequence"
    );
}

// ---------------------------------------------------------------------------
// (d) Manifest/scheduler desync: unmapped-node fires
// ---------------------------------------------------------------------------

/// A producer handed a node-ids table MISSING nodes (a manifest/scheduler
/// desync): the missing nodes' fires are SKIPPED on the ring (exactly those —
/// mapped nodes unaffected, hand oracle), `trace_ring_unmapped_count()` equals
/// the skip count, the warn fires ONCE PER DISTINCT missing node (not once
/// globally, not per fire), and the in-memory trace is untouched. A
/// `debug_assert!(false)` on this path would abort debug runs and make it
/// untestable; the counter is the stronger contract.
#[test]
#[serial]
#[tracing_test::traced_test]
fn unmapped_node_fires_are_skipped_counted_and_warned_per_node() {
    const STEPS: u64 = 4;
    let tag = ring_tag("unmapped");

    // Ring manifest AND the handed table cover ONLY "alpha" — "beta" and
    // "gamma" are the desync.
    let mut ring_owner = TraceRingOwner::create(&tag, 64, 0, &["alpha"]).expect("create ring");
    let ring_name = ring_owner.name().to_string();
    let producer = ring_owner.producer().expect("mint producer");

    let mut scheduler = fresh_scheduler();
    scheduler.set_trace_ring_producer(producer, &["alpha".to_string()]);
    for id in ["alpha", "beta", "gamma"] {
        scheduler
            .add_node(NodeConfig {
                id: id.to_string(),
                policy: period(10),
                callback: noop(),
            })
            .unwrap();
    }

    for _ in 0..STEPS {
        scheduler.step_ms(10);
    }

    // The ring omits EXACTLY the unmapped nodes: alpha's 4 fires complete,
    // each step still led by its StepBoundary (the boundary carries no node
    // id — a desync never suppresses it).
    let records = drain_ring(&ring_name);
    let ring_oracle: Vec<TraceRingRecord> = (0..STEPS)
        .flat_map(|k| {
            [
                boundary_record(k, (k + 1) * 10_000_000),
                fire_record(k, (k + 1) * 10_000_000, 0, 0),
            ]
        })
        .collect();
    assert_eq!(
        records, ring_oracle,
        "mapped node (alpha) records are complete; unmapped fires are skipped"
    );

    // The counter equals the skip count: beta 4 + gamma 4.
    assert_eq!(
        scheduler.trace_ring_unmapped_count(),
        2 * STEPS,
        "unmapped_dropped counts every skipped fire record"
    );

    // The in-memory trace is UNTOUCHED by the desync: all 3 nodes, all steps.
    assert_eq!(
        scheduler.trace().len(),
        (3 * STEPS) as usize,
        "the in-memory trace records every fire regardless of the ring desync"
    );

    // Warn once PER DISTINCT missing node: exactly 2 warns, one naming beta,
    // one naming gamma, none for alpha.
    logs_assert(|lines: &[&str]| {
        let marker = "missing from the recording manifest";
        let total = lines.iter().filter(|l| l.contains(marker)).count();
        let beta = lines
            .iter()
            .filter(|l| l.contains(marker) && l.contains("beta"))
            .count();
        let gamma = lines
            .iter()
            .filter(|l| l.contains(marker) && l.contains("gamma"))
            .count();
        let alpha = lines
            .iter()
            .filter(|l| l.contains(marker) && l.contains("alpha"))
            .count();
        if total == 2 && beta == 1 && gamma == 1 && alpha == 0 {
            Ok(())
        } else {
            Err(format!(
                "expected exactly one warn per distinct missing node \
                 (beta=1, gamma=1, alpha=0, total=2); got total={total}, \
                 beta={beta}, gamma={gamma}, alpha={alpha}"
            ))
        }
    });

    drop(ring_owner);
}

// ---------------------------------------------------------------------------
// (e) Install-once / before-step-0 guards
// ---------------------------------------------------------------------------

/// A SECOND `set_trace_ring_producer` is refused loudly: the FIRST hook keeps
/// recording (ring A carries the full fire sequence), the second ring stays
/// empty, and the refusal is `tracing::error!`-logged.
#[test]
#[serial]
#[tracing_test::traced_test]
fn second_install_is_refused_and_first_hook_kept() {
    const STEPS: u64 = 3;
    let tag_a = ring_tag("guard_a");
    let tag_b = ring_tag("guard_b");

    let mut owner_a = TraceRingOwner::create(&tag_a, 64, 0, &["n0"]).expect("create ring A");
    let name_a = owner_a.name().to_string();
    let mut owner_b = TraceRingOwner::create(&tag_b, 64, 0, &["n0"]).expect("create ring B");
    let name_b = owner_b.name().to_string();

    let mut scheduler = fresh_scheduler();
    scheduler.set_trace_ring_producer(owner_a.producer().expect("mint A"), &["n0".to_string()]);
    // The offending second install — must be REFUSED, keeping hook A.
    scheduler.set_trace_ring_producer(owner_b.producer().expect("mint B"), &["n0".to_string()]);

    scheduler
        .add_node(NodeConfig {
            id: "n0".to_string(),
            policy: period(10),
            callback: noop(),
        })
        .unwrap();
    for _ in 0..STEPS {
        scheduler.step_ms(10);
    }

    let records_a = drain_ring(&name_a);
    let oracle_a: Vec<TraceRingRecord> = (0..STEPS)
        .flat_map(|k| {
            [
                boundary_record(k, (k + 1) * 10_000_000),
                fire_record(k, (k + 1) * 10_000_000, 0, 0),
            ]
        })
        .collect();
    assert_eq!(
        records_a, oracle_a,
        "the FIRST hook keeps recording (ring A)"
    );
    assert!(
        drain_ring(&name_b).is_empty(),
        "the refused second producer must never receive a record (ring B empty)"
    );
    assert!(
        logs_contain("trace ring producer already installed"),
        "the re-install refusal must be error!-logged naming the contract"
    );

    drop(owner_a);
    drop(owner_b);
}

/// An install AFTER step 0 is refused loudly: the ring stays empty (the bag
/// would silently miss every earlier fire) and the refusal is
/// `tracing::error!`-logged naming the before-step-0 contract.
#[test]
#[serial]
#[tracing_test::traced_test]
fn install_after_step_zero_is_refused() {
    let tag = ring_tag("guard_late");
    let mut owner = TraceRingOwner::create(&tag, 64, 0, &["n0"]).expect("create ring");
    let name = owner.name().to_string();

    let mut scheduler = fresh_scheduler();
    scheduler
        .add_node(NodeConfig {
            id: "n0".to_string(),
            policy: period(10),
            callback: noop(),
        })
        .unwrap();
    // Step 0 runs BEFORE any install.
    scheduler.step_ms(10);

    // The late install — must be REFUSED.
    scheduler.set_trace_ring_producer(owner.producer().expect("mint"), &["n0".to_string()]);
    for _ in 0..3 {
        scheduler.step_ms(10);
    }

    assert!(
        drain_ring(&name).is_empty(),
        "a post-step-0 install must be refused — the ring stays empty"
    );
    assert!(
        logs_contain("trace ring producer installed AFTER step 0"),
        "the late-install refusal must be error!-logged naming the contract"
    );

    drop(owner);
}

// ---------------------------------------------------------------------------
// (f) Contract addendum: one StepBoundary per step BEGUN — no-fire
// steps included
// ---------------------------------------------------------------------------

/// A `period 50` node stepped at 10 ms fires only on every 5th step — yet
/// EVERY step must push its StepBoundary (the addendum FORBIDS eliding empty
/// steps: the boundary is the only record a no-fire step produces, and replay
/// needs the full clock progression). 5 steps → 5 boundaries + 1 fire, in
/// push order (each boundary precedes its step's fires), against the full
/// hand oracle. The boundaries are RING-ONLY: the in-memory trace holds just
/// the single fire.
#[test]
#[serial]
fn no_fire_steps_still_push_one_boundary_each() {
    const STEPS: u64 = 5;
    let tag = ring_tag("nofire");

    let mut ring_owner = TraceRingOwner::create(&tag, 64, 0, &["slow"]).expect("create ring");
    let ring_name = ring_owner.name().to_string();
    let producer = ring_owner.producer().expect("mint producer");

    let mut scheduler = fresh_scheduler();
    scheduler.set_trace_ring_producer(producer, &["slow".to_string()]);
    scheduler
        .add_node(NodeConfig {
            id: "slow".to_string(),
            policy: period(50),
            callback: noop(),
        })
        .unwrap();

    for _ in 0..STEPS {
        scheduler.step_ms(10);
    }

    // Hand oracle: boundaries at 10,20,30,40,50 ms (steps 0..4); the single
    // fire lands on step 4 at 50 ms, AFTER that step's boundary.
    let records = drain_ring(&ring_name);
    let oracle = vec![
        boundary_record(0, 10_000_000),
        boundary_record(1, 20_000_000),
        boundary_record(2, 30_000_000),
        boundary_record(3, 40_000_000),
        boundary_record(4, 50_000_000),
        fire_record(4, 50_000_000, 0, 0),
    ];
    assert_eq!(
        records, oracle,
        "every step begun pushes exactly one boundary — no-fire steps included"
    );

    // Ring-only: the in-memory trace carries just the one fire.
    assert_eq!(
        scheduler.trace().len(),
        1,
        "boundaries never enter the in-memory trace"
    );

    drop(ring_owner);
}

// ---------------------------------------------------------------------------
// (e) The discard marker (record side) — fire_node_into's discard-
//     signal delta drives entry.discarded, and push_fire folds it into the
//     ring record's `reserved` as TRACE_DISCARD_BIT.
// ---------------------------------------------------------------------------

/// A callback that bumps a shared discard signal on the k-th fire it runs (0-
/// based), simulating an `OutputProxy` pre-commit discard on exactly that fire.
fn discard_on_fire(
    signal: Arc<std::sync::atomic::AtomicU32>,
    discard_fire: u64,
) -> Box<dyn FnMut() + Send> {
    let mut n: u64 = 0;
    Box::new(move || {
        if n == discard_fire {
            signal.fetch_add(1, std::sync::atomic::Ordering::Release);
        }
        n += 1;
    })
}

/// Record-side pin: a node whose fire at step 2 discards its
/// output (the shared discard signal is bumped inside the tick) produces a ring
/// FIRE record with `is_discarded() == true` — and ONLY that fire. Every other
/// step's FIRE record decodes `rank() == 0` and `is_discarded() == false`.
///
/// The full record chain under test: the callback bumps the per-node discard
/// signal → `fire_node_into` reads the pre/post-callback DELTA → stamps
/// `TraceEntry::discarded` → `push_fire` folds it into `reserved` as
/// `TRACE_DISCARD_BIT`. Reverting EITHER `push_fire`'s bit write (`reserved: if
/// discarded { TRACE_DISCARD_BIT } else { 0 }` → `reserved: 0`) OR
/// `fire_node_into`'s delta read (`discarded` → `false`) makes the step-2 record
/// decode `is_discarded() == false` and FAILS this test.
#[test]
#[serial]
fn discard_signal_delta_marks_the_fire_record_via_push_fire() {
    use cerulion_core::trace_ring::TRACE_DISCARD_BIT;
    const STEPS: u64 = 5;
    const DISCARD_FIRE: u64 = 2; // the node's 3rd fire = step 2.
    let tag = ring_tag("discard_marker");

    let mut ring_owner = TraceRingOwner::create(&tag, 64, 0, &["n0"]).expect("create ring");
    let ring_name = ring_owner.name().to_string();
    let producer = ring_owner.producer().expect("mint producer");

    let signal = Arc::new(std::sync::atomic::AtomicU32::new(0));
    let suppress = Arc::new(std::sync::atomic::AtomicBool::new(false));

    let mut scheduler = fresh_scheduler();
    scheduler.set_trace_ring_producer(producer, &["n0".to_string()]);
    scheduler
        .add_node(NodeConfig {
            id: "n0".to_string(),
            policy: period(10),
            callback: discard_on_fire(Arc::clone(&signal), DISCARD_FIRE),
        })
        .unwrap();
    // Share the SAME signal the callback bumps with the ScheduledNode so
    // fire_node_into's delta sees it (the production runtime does this via
    // set_node_discard_signals with the publisher-shared Arc).
    scheduler
        .set_node_discard_signals_for_test("n0", Arc::clone(&signal), suppress)
        .unwrap();

    for _ in 0..STEPS {
        scheduler.step_ms(10);
    }

    let records = drain_ring(&ring_name);
    // Hand oracle: every step pushes a boundary then a FIRE; step 2's FIRE is
    // the ONLY discard-marked record. rank() is 0 everywhere (single-process).
    let fires: Vec<&TraceRingRecord> = records
        .iter()
        .filter(|r| r.record_type == RECORD_TYPE_FIRE)
        .collect();
    assert_eq!(fires.len(), STEPS as usize, "one FIRE per step");
    for (step, f) in fires.iter().enumerate() {
        assert_eq!(f.step, step as u64, "fire step order");
        assert_eq!(f.rank(), 0, "single-process rank is 0 (bit masked)");
        let want_discarded = step as u64 == DISCARD_FIRE;
        assert_eq!(
            f.is_discarded(),
            want_discarded,
            "step {step}: is_discarded() must be {want_discarded}"
        );
        assert_eq!(
            f.reserved,
            if want_discarded { TRACE_DISCARD_BIT } else { 0 },
            "step {step}: reserved is exactly the bit-or-zero (rank 0)"
        );
    }
    // Boundary records never carry the marker.
    for b in records
        .iter()
        .filter(|r| r.record_type == RECORD_TYPE_STEP_BOUNDARY)
    {
        assert!(!b.is_discarded(), "boundary records are never discarded");
        assert_eq!(b.reserved, 0);
    }

    drop(ring_owner);
}

/// Control (anti-tautology): with the SAME apparatus but the callback NEVER
/// bumping the signal (`discard_fire` past the step count), NO fire record is
/// marked. Proves the marker tracks the signal delta, not merely "recording is
/// on".
#[test]
#[serial]
fn no_discard_signal_leaves_every_fire_unmarked() {
    const STEPS: u64 = 4;
    let tag = ring_tag("discard_control");

    let mut ring_owner = TraceRingOwner::create(&tag, 64, 0, &["n0"]).expect("create ring");
    let ring_name = ring_owner.name().to_string();
    let producer = ring_owner.producer().expect("mint producer");

    let signal = Arc::new(std::sync::atomic::AtomicU32::new(0));
    let suppress = Arc::new(std::sync::atomic::AtomicBool::new(false));

    let mut scheduler = fresh_scheduler();
    scheduler.set_trace_ring_producer(producer, &["n0".to_string()]);
    scheduler
        .add_node(NodeConfig {
            id: "n0".to_string(),
            policy: period(10),
            callback: discard_on_fire(Arc::clone(&signal), 999), // never fires
        })
        .unwrap();
    scheduler
        .set_node_discard_signals_for_test("n0", signal, suppress)
        .unwrap();

    for _ in 0..STEPS {
        scheduler.step_ms(10);
    }

    let records = drain_ring(&ring_name);
    assert!(
        records
            .iter()
            .filter(|r| r.record_type == RECORD_TYPE_FIRE)
            .all(|r| !r.is_discarded() && r.reserved == 0),
        "no discard signal ⇒ no marked fire record"
    );

    drop(ring_owner);
}
