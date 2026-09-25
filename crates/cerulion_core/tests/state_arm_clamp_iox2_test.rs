// SPDX-License-Identifier: AGPL-3.0-only
//! The catch-up clamp end to end: a real `MappedStateArm` attached to a real
//! `GraphRuntime` clamps a `Period` node's catch-up burst.
//!
//! This is the only place the whole chain runs: a recorder ARMS a real
//! `MAP_SHARED` POSIX-SHM word → `GraphRuntime::attach_state_arm` installs it on
//! the scheduler → `begin_step` reads its onset through the production
//! `CatchupArm` impl → `decide_node` caps the burst. The in-crate wiring tests
//! (`scheduler::catchup_clamp_wiring_tests`) inject a FAKE arm, so they are
//! structurally blind to two things this file pins: that the runtime hands the
//! arm to the scheduler at all, and that the production `impl CatchupArm for
//! MappedStateArm` reports what `arm()` published.
//!
//! Whole file `#![cfg(unix)]` — the arm word is POSIX SHM.
//!
//! Per-test SHM root (`build_for_test`) + pid/nanos-unique arm tags ⇒
//! PARALLEL-SAFE; no `#[serial]`, no `--test-threads=1`.
#![cfg(unix)]

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::scheduler::catchup_clamp::ARMED_MAX_CATCHUP_DEFAULT;
use cerulion_core::state_arm::MappedStateArm;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

/// 10 ms period against a 100 ms step = 10 intervals of catch-up in ONE step,
/// comfortably past the clamp so the two answers cannot be confused.
const BURST_STEP: Duration = Duration::from_millis(100);
const BURST_FIRES: u64 = 10;

/// A `Period` node that declares no `max_catchup`, the shape the arm clamps
/// (an explicit declaration is never overridden).
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct Ticker {
    #[output]
    out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl Ticker {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

fn unique_tag(what: &str) -> String {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    format!("clamp_{what}_{}_{nanos}", std::process::id())
}

fn ticker_graph() -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "clamp".to_string(),
        prefix: "clamp".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "ticker".to_string(),
            node_type: "ticker".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("ticker".to_string(), Box::new(TickerEntry::new()));
    (config, factories)
}

fn build() -> GraphRuntime {
    let (config, factories) = ticker_graph();
    GraphRuntime::build_for_test(config, factories, Arc::new(VirtualClock::new()), 8)
        .expect("build the ticker graph")
}

/// THE end-to-end arm: a real armed word clamps a real runtime's burst, and an
/// un-armed run of the SAME graph does not.
///
/// Both halves in ONE body against hand oracles — the clamp only means anything
/// as a difference, and an "armed fires 4" assertion alone is satisfied by a
/// graph that could only ever fire 4.
#[test]
fn a_real_armed_word_clamps_a_real_runtimes_period_burst() {
    // Un-armed control FIRST, so the oracle for "unclamped" is measured on this
    // machine rather than assumed.
    let mut unarmed = build();
    unarmed.step(BURST_STEP);
    assert_eq!(
        unarmed.node_handle("ticker").unwrap().fire_count(),
        BURST_FIRES,
        "an un-armed run must fire the whole catch-up burst"
    );
    assert!(
        unarmed.state_arm().is_none(),
        "nothing attached an arm to the control run"
    );
    assert!(
        unarmed.state_arm_fork_sweep().is_none(),
        "an un-armed run takes no fork-exclusion sweep"
    );

    // ARMED: a real MAP_SHARED word, armed from step 0 exactly as a recorder
    // attaching at the run's start would.
    let arm = Arc::new(MappedStateArm::create_owned(&unique_tag("burst")).expect("create the arm"));
    arm.arm(0, 0);
    let mut armed = build();
    armed.attach_state_arm(Arc::clone(&arm));
    armed.step(BURST_STEP);
    assert_eq!(
        armed.node_handle("ticker").unwrap().fire_count(),
        u64::from(ARMED_MAX_CATCHUP_DEFAULT),
        "an armed run must clamp the burst to {ARMED_MAX_CATCHUP_DEFAULT}"
    );
}

/// The AGREED ONSET, driven by a real word: an arm whose first anchor is due at
/// step 2 leaves steps 0 and 1 unclamped.
///
/// This is the multi-process agreement property at the production seam — the
/// step number comes off the shared word, so every rank flips at the same step
/// whatever wall instant it noticed the arm at. It also proves the production
/// `CatchupArm` impl really reads `first_anchor_step` rather than defaulting it:
/// an impl that reported 0 would clamp from step 0 and fail here.
#[test]
fn a_real_arm_clamps_only_from_the_step_its_word_published() {
    const ONSET: u64 = 2;
    let arm = Arc::new(MappedStateArm::create_owned(&unique_tag("onset")).expect("create the arm"));
    arm.arm(0, ONSET);
    let mut runtime = build();
    runtime.attach_state_arm(Arc::clone(&arm));

    let mut per_step = Vec::new();
    let mut seen = 0u64;
    for _ in 0..4 {
        runtime.step(BURST_STEP);
        let total = runtime.node_handle("ticker").unwrap().fire_count();
        per_step.push(total - seen);
        seen = total;
    }
    let clamp = u64::from(ARMED_MAX_CATCHUP_DEFAULT);
    assert_eq!(
        per_step,
        vec![BURST_FIRES, BURST_FIRES, clamp, clamp],
        "steps before the word's own first_anchor_step must be unclamped"
    );
}

/// A recorder DETACHING hands the robot its behaviour straight back — driven by
/// a real `disarm()` on the shared word between two steps.
#[test]
fn disarming_the_real_word_restores_the_full_burst_on_the_next_step() {
    let arm =
        Arc::new(MappedStateArm::create_owned(&unique_tag("disarm")).expect("create the arm"));
    arm.arm(0, 0);
    let mut runtime = build();
    runtime.attach_state_arm(Arc::clone(&arm));

    runtime.step(BURST_STEP);
    let clamped = runtime.node_handle("ticker").unwrap().fire_count();
    assert_eq!(clamped, u64::from(ARMED_MAX_CATCHUP_DEFAULT));

    arm.disarm();
    runtime.step(BURST_STEP);
    assert_eq!(
        runtime.node_handle("ticker").unwrap().fire_count() - clamped,
        BURST_FIRES,
        "a disarmed word must restore the full catch-up burst"
    );
}

/// ARMING takes the fork-exclusion sweep, and the sweep is READABLE.
///
/// A sweep that excluded nothing looks exactly like one that excluded everything
/// from outside the process, so the tally is a Principle #3 observable rather
/// than a log line.
///
/// # Why this drives a BATCH rather than asserting `attempts() > 0`
///
/// The ledger is PROCESS-WIDE and monotone, so `attempts() > 0` is satisfied by
/// any sibling test's mapping and would still pass with the `exclude_at_birth`
/// call DELETED from `MappedStateArm::create_owned` — it pins the ledger's
/// existence, not the arm word's own birth hook, which is the regression it
/// claims to cover.
///
/// So this creates `PROBE_ARMS` arm words of its own and requires the ledger to
/// have grown by at least that many. A concurrent sibling can only ADD (nothing
/// ever decrements), so the bound is safe in a parallel binary; if the birth
/// hook is removed, this test's OWN contribution is zero and a
/// sibling would have to create `PROBE_ARMS` mappings inside the same window to
/// hide it. The count is deliberately well above the one-or-two mappings any
/// sibling makes per test.
///
/// Residual, stated rather than implied: the ledger buckets by OUTCOME, not by
/// [`cerulion_core::state_carrier::ForkExcludedMapping`], so no assertion here
/// can name the arm word as the contributor — only that this many attempts
/// really happened.
#[test]
fn arming_records_a_readable_fork_exclusion_sweep() {
    use cerulion_core::state_carrier::sweep_birth_exclusions_at_arm;

    /// Enough that a concurrent sibling cannot plausibly account for the delta.
    const PROBE_ARMS: u32 = 64;

    let before = sweep_birth_exclusions_at_arm();
    let probes: Vec<MappedStateArm> = (0..PROBE_ARMS)
        .map(|i| {
            MappedStateArm::create_owned(&unique_tag(&format!("sweepprobe{i}")))
                .expect("create a probe arm word")
        })
        .collect();
    let after_probes = sweep_birth_exclusions_at_arm();
    assert!(
        after_probes.attempts() >= before.attempts() + PROBE_ARMS,
        "creating {PROBE_ARMS} arm words must fold {PROBE_ARMS} birth exclusions into the \
         ledger — the arm word's own constructor hook is what this pins: \
         {before:?} -> {after_probes:?}"
    );
    drop(probes);

    let arm = Arc::new(MappedStateArm::create_owned(&unique_tag("sweep")).expect("create the arm"));
    arm.arm(0, 0);
    let mut runtime = build();
    assert!(runtime.state_arm_fork_sweep().is_none(), "not armed yet");

    // Read the ledger IMMEDIATELY before the attach, so the recorded sweep can
    // be required to be at least this — a sweep read at BUILD time, or a
    // hardcoded one, cannot satisfy it.
    let at_attach = sweep_birth_exclusions_at_arm();
    runtime.attach_state_arm(Arc::clone(&arm));
    let sweep = runtime
        .state_arm_fork_sweep()
        .expect("arming must record a sweep");
    assert!(
        sweep.attempts() >= at_attach.attempts(),
        "the arm-time sweep must READ the ledger at attach time, not a stale or \
         fabricated snapshot: {at_attach:?} -> {sweep:?}"
    );
    assert!(
        runtime.state_arm().is_some(),
        "the runtime must hold the arm the carrier reads back"
    );
}
