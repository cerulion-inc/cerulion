// SPDX-License-Identifier: AGPL-3.0-only
//! The `Period` catch-up CLAMP pinned at all THREE
//! `decide_node` call sites, plus the once-per-step read discipline.
//!
//! These live IN-CRATE rather than in `tests/scheduler_test.rs` because two of
//! the three sites (`decide_fires`, `evaluate_nodes_fused`) are the level
//! executor's `pub(crate)` seams — an integration test can only reach the flat
//! `step` path, so dropping the threading at either level site
//! would ship green. The PURE derivation is oracle-tested in
//! [`super::catchup_clamp`]; what is pinned HERE is that the derived value
//! REACHES each decide.

use super::catchup_clamp::{ArmOnset, CatchupArm, ARMED_MAX_CATCHUP_DEFAULT};
use super::*;
use std::sync::atomic::AtomicUsize;

const INTERVAL: Duration = Duration::from_millis(10);
/// 10 intervals of catch-up in ONE step — comfortably past the clamp, so the
/// clamped and unclamped answers cannot be mistaken for each other.
const BURST_STEP: Duration = Duration::from_millis(100);
const BURST_FIRES: u64 = 10;

/// A scriptable arm that COUNTS its readings.
///
/// The count is the whole point: no real `MappedStateArm` can report how many
/// times it was asked, and "read once per step" is a claim about exactly that
/// number. `onset` sits behind a lock so a test can arm/disarm BETWEEN steps
/// and observe the effect on the next one.
struct CountingArm {
    onset: std::sync::Mutex<ArmOnset>,
    reads: AtomicUsize,
}

impl CountingArm {
    fn armed_at(first_anchor_step: u64) -> Arc<Self> {
        Arc::new(Self {
            onset: std::sync::Mutex::new(ArmOnset {
                armed: true,
                first_anchor_step,
            }),
            reads: AtomicUsize::new(0),
        })
    }

    fn reads(&self) -> usize {
        self.reads.load(Ordering::Relaxed)
    }

    fn disarm(&self) {
        self.onset.lock().unwrap().armed = false;
    }
}

impl CatchupArm for CountingArm {
    fn onset(&self) -> ArmOnset {
        self.reads.fetch_add(1, Ordering::Relaxed);
        *self.onset.lock().unwrap()
    }
}

fn scheduler() -> Scheduler {
    Scheduler::with_virtual_clock(Arc::new(VirtualClock::new()))
}

fn add_period_capped(scheduler: &mut Scheduler, id: &str, max_catchup: Option<u32>) {
    scheduler
        .add_node(NodeConfig {
            id: id.to_string(),
            policy: TriggerPolicy::Period {
                interval: INTERVAL,
                max_catchup,
            },
            callback: Box::new(|| {}),
        })
        .unwrap();
}

fn fires(scheduler: &Scheduler, id: &str) -> u64 {
    scheduler.node_handle(id).unwrap().fire_count()
}

/// THE headline, on the FLAT `Scheduler::step` path: one 100 ms step against a
/// 10 ms `Period` node that declared no `max_catchup`.
///
/// Both readings are asserted in ONE body against hand oracles, because the
/// clamp only means anything as a DIFFERENCE: un-armed the burst is all 10
/// intervals (today's shipped behaviour, byte-unchanged), armed it is exactly
/// `ARMED_MAX_CATCHUP_DEFAULT`.
#[test]
fn an_armed_runs_undeclared_period_node_fires_the_clamp_not_the_whole_burst() {
    let mut unarmed = scheduler();
    add_period_capped(&mut unarmed, "cam", None);
    unarmed.step(BURST_STEP);
    assert_eq!(
        unarmed.catchup_cap_override(),
        None,
        "an un-armed run must derive NO override"
    );
    assert_eq!(
        fires(&unarmed, "cam"),
        BURST_FIRES,
        "the un-armed burst must be the full catch-up (the shipped default)"
    );

    let mut armed = scheduler();
    add_period_capped(&mut armed, "cam", None);
    armed.attach_catchup_arm(CountingArm::armed_at(0));
    armed.step(BURST_STEP);
    assert_eq!(
        armed.catchup_cap_override(),
        Some(ARMED_MAX_CATCHUP_DEFAULT),
        "an armed run past its onset must derive the clamp"
    );
    assert_eq!(
        fires(&armed, "cam"),
        u64::from(ARMED_MAX_CATCHUP_DEFAULT),
        "the armed burst must be clamped to {ARMED_MAX_CATCHUP_DEFAULT}"
    );
}

/// The clamp caps a BURST; it does not slow a node down. Ten ORDINARY steps of
/// exactly one interval each fire ten times armed and un-armed alike — without
/// this arm, "the clamp works" is equally satisfied by a change that throttles
/// every armed robot's `Period` nodes to four fires per second.
#[test]
fn the_clamp_does_not_change_an_armed_nodes_ordinary_cadence() {
    let mut s = scheduler();
    add_period_capped(&mut s, "cam", None);
    s.attach_catchup_arm(CountingArm::armed_at(0));
    for _ in 0..10 {
        s.step(INTERVAL);
    }
    assert_eq!(
        fires(&s, "cam"),
        10,
        "one fire per ordinary step, clamped or not"
    );
}

/// An EXPLICIT `max_catchup` outranks the clamp — in BOTH directions, so an
/// attached recorder can neither shorten nor lengthen a declared burst. Driven
/// through the real decide, not the pure precedence function.
#[test]
fn an_explicitly_declared_max_catchup_is_not_overridden_by_the_arm() {
    // BELOW the clamp: the node is stricter than the clamp and stays so.
    let mut strict = scheduler();
    add_period_capped(&mut strict, "strict", Some(2));
    strict.attach_catchup_arm(CountingArm::armed_at(0));
    strict.step(BURST_STEP);
    assert_eq!(
        fires(&strict, "strict"),
        2,
        "a declared 2 must stay 2 while armed"
    );

    // ABOVE the clamp: the node asked for a long burst and gets it.
    let mut loose = scheduler();
    add_period_capped(&mut loose, "loose", Some(8));
    loose.attach_catchup_arm(CountingArm::armed_at(0));
    loose.step(BURST_STEP);
    assert_eq!(
        fires(&loose, "loose"),
        8,
        "a declared 8 must stay 8 while armed"
    );
}

/// The ONSET gate, driven step by step: an arm whose first anchor is due at step
/// 2 leaves steps 0 and 1 completely unclamped and clamps from step 2 on. This
/// is the multi-process agreement property — every rank flips at the SAME step
/// number, whatever wall instant it noticed the arm at.
#[test]
fn the_clamp_engages_at_the_agreed_onset_step_and_not_before() {
    let mut s = scheduler();
    add_period_capped(&mut s, "cam", None);
    s.attach_catchup_arm(CountingArm::armed_at(2));

    let mut per_step = Vec::new();
    let mut seen = 0u64;
    for _ in 0..4 {
        s.step(BURST_STEP);
        let total = fires(&s, "cam");
        per_step.push(total - seen);
        seen = total;
    }
    let clamp = u64::from(ARMED_MAX_CATCHUP_DEFAULT);
    assert_eq!(
        per_step,
        vec![BURST_FIRES, BURST_FIRES, clamp, clamp],
        "steps 0-1 precede the agreed onset and must be unclamped; 2 onward clamped"
    );
}

/// A DISARM is honoured on the next step — a recorder detaching must hand the
/// robot its declared behaviour straight back, and the onset gate must not keep
/// a stale arm alive.
#[test]
fn a_disarm_between_steps_restores_the_full_burst_on_the_next_step() {
    let mut s = scheduler();
    add_period_capped(&mut s, "cam", None);
    let arm = CountingArm::armed_at(0);
    s.attach_catchup_arm(Arc::clone(&arm) as Arc<dyn CatchupArm>);

    s.step(BURST_STEP);
    let clamped = fires(&s, "cam");
    assert_eq!(clamped, u64::from(ARMED_MAX_CATCHUP_DEFAULT));

    arm.disarm();
    s.step(BURST_STEP);
    assert_eq!(
        fires(&s, "cam") - clamped,
        BURST_FIRES,
        "a disarmed run must be back to the full catch-up burst"
    );
    assert_eq!(s.catchup_cap_override(), None);
}

/// THE READ DISCIPLINE: the arm is read EXACTLY ONCE per step, whatever the node
/// count.
///
/// Three nodes make a per-node read distinguishable from a per-step one (a
/// per-node implementation reports 9 after 3 steps, not 3), and the COUNT is
/// what makes the claim checkable at all — the derived value is identical
/// either way on a quiescent word. It is a correctness rule, not a saving: a
/// recorder can flip the word at any instant, and a per-node read lets that flip
/// land BETWEEN two nodes of one step, giving them different caps.
#[test]
fn the_arm_is_read_exactly_once_per_step_whatever_the_node_count() {
    let mut s = scheduler();
    for id in ["a", "b", "c"] {
        add_period_capped(&mut s, id, None);
    }
    let arm = CountingArm::armed_at(0);
    s.attach_catchup_arm(Arc::clone(&arm) as Arc<dyn CatchupArm>);

    assert_eq!(arm.reads(), 0, "attaching must not read the word");
    for expected in 1..=3usize {
        s.step(BURST_STEP);
        assert_eq!(
            arm.reads(),
            expected,
            "exactly one read per step across 3 nodes"
        );
    }
}

/// SITE 2 — the level executor's `decide_fires`, reached by `GraphRuntime` and
/// never by the flat `step`, so dropping the threading here passes
/// every arm above.
///
/// The oracle is the DECISION's own `fire_count`, read off the very buffer the
/// runtime ticks from.
#[test]
fn the_clamp_reaches_the_level_executors_decide_fires() {
    fn decided(arm: Option<Arc<dyn CatchupArm>>) -> u32 {
        let mut s = scheduler();
        add_period_capped(&mut s, "cam", None);
        if let Some(arm) = arm {
            s.attach_catchup_arm(arm);
        }
        let now = s.begin_step(BURST_STEP);
        s.decide_fires(&["cam".to_string()], now);
        let decisions = s.take_decisions();
        let count = match decisions.first().map(|d| &d.kind) {
            Some(FireKind::Period { fire_count, .. }) => *fire_count,
            other => panic!("expected a Period decision, got {other:?}"),
        };
        s.return_decisions(decisions);
        count
    }
    assert_eq!(
        decided(None),
        BURST_FIRES as u32,
        "un-armed: the whole burst is decided"
    );
    assert_eq!(
        decided(Some(CountingArm::armed_at(0))),
        ARMED_MAX_CATCHUP_DEFAULT,
        "armed: `decide_fires` must decide the CLAMPED burst"
    );
}

/// SITE 3 — the level executor's fused block path (`evaluate_nodes_fused`), the
/// third and last `decide_node` call site. Same reason as site 2: nothing else
/// in the repo reaches it with a clamp in play.
#[test]
fn the_clamp_reaches_the_level_executors_fused_block_path() {
    fn fired(arm: Option<Arc<dyn CatchupArm>>) -> u64 {
        let mut s = scheduler();
        add_period_capped(&mut s, "cam", None);
        if let Some(arm) = arm {
            s.attach_catchup_arm(arm);
        }
        let now = s.begin_step(BURST_STEP);
        let step = s.current_step();
        s.evaluate_nodes_fused(&["cam".to_string()], now, 0, step, &mut |_| {});
        fires(&s, "cam")
    }
    assert_eq!(fired(None), BURST_FIRES, "un-armed: the whole burst fires");
    assert_eq!(
        fired(Some(CountingArm::armed_at(0))),
        u64::from(ARMED_MAX_CATCHUP_DEFAULT),
        "armed: the fused block path must fire the CLAMPED burst"
    );
}

/// The FIREWALL as its own arm: a run with NO arm attached derives no override
/// on ANY step, so nothing about its decide path can depend on the clamp. Every
/// "un-armed" half above rests on this, and it is cheap to state where a reader
/// will look for it.
#[test]
fn an_unarmed_run_derives_no_override_on_any_step() {
    let mut s = scheduler();
    add_period_capped(&mut s, "cam", None);
    for _ in 0..5 {
        s.step(BURST_STEP);
        assert_eq!(s.catchup_cap_override(), None);
    }
    assert_eq!(fires(&s, "cam"), 5 * BURST_FIRES);
}
