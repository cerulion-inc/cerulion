// SPDX-License-Identifier: AGPL-3.0-only
//! `Scheduler::set_wedge_page`: the SLOT-BINDING seam, and the two
//! directions a supervisor/worker plan desync can point.
//!
//! The binding is the rule applied to this alarm: the SUPERVISOR decides
//! which slot means which node, because it is the process that reads the page and
//! names the offender. So a desync here does not crash — it makes the alarm name
//! the WRONG node, or no node at all — which is a wrong answer from the feature
//! whose entire job is to name the right one.
//!
//! Both directions must be LOUD, and they are not symmetric in how they fail:
//!
//! - the map names a node this worker does NOT hold (the map is AHEAD): that slot
//!   simply never moves, so the supervisor at least sees something it can report;
//! - the map OMITS a node this worker DOES hold (the map is BEHIND): the node
//!   never marks at all, so there is no unmoving slot to notice and the omission
//!   is INVISIBLE from the observing side. This is the direction that goes silent
//!   if nobody looks for it, and it is the one this file exists for.
//!
//! `#[traced_test]` installs a GLOBAL subscriber, so this is its own binary
//! (`rmw_schema_mismatch_test`'s reason). No transport, no SHM — the marker is a
//! counting double, which is the point: this seam is about WHICH NODE gets bound
//! to WHICH SLOT, not about the page.
//!
//! ```bash
//! cargo test -p cerulion_core --test wedge_slot_binding_test
//! ```

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::scheduler::{NodeConfig, Scheduler, TriggerPolicy, WedgeMarker};

/// A counting [`WedgeMarker`] — records every `(slot, kind)` mark in order.
///
/// A DOUBLE rather than a real page, because what is under test is the slot each
/// node was bound to. A real `MappedWedgePage` would answer the same question
/// through a pair of counters and drag POSIX SHM into a test about an
/// `IndexMap` lookup.
#[derive(Default)]
struct SpyMarker {
    marks: Mutex<Vec<(usize, &'static str)>>,
    steps: AtomicU64,
}

impl SpyMarker {
    fn entered_slots(&self) -> Vec<usize> {
        self.marks
            .lock()
            .expect("spy")
            .iter()
            .filter(|(_, k)| *k == "enter")
            .map(|(s, _)| *s)
            .collect()
    }
}

impl WedgeMarker for SpyMarker {
    fn enter(&self, slot: usize) {
        self.marks.lock().expect("spy").push((slot, "enter"));
    }
    fn exit(&self, slot: usize) {
        self.marks.lock().expect("spy").push((slot, "exit"));
    }
    fn advance_step(&self) {
        self.steps.fetch_add(1, Ordering::Relaxed);
    }
}

/// A scheduler holding `ids`, each on a 10 ms period so one `step_ms(10)` fires
/// every one of them exactly once.
fn scheduler_with(ids: &[&str]) -> Scheduler {
    let mut s = Scheduler::with_virtual_clock(Arc::new(VirtualClock::new()));
    for id in ids {
        s.add_node(NodeConfig {
            id: (*id).to_string(),
            policy: TriggerPolicy::Period {
                interval: Duration::from_millis(10),
                max_catchup: None,
            },
            callback: Box::new(|| {}),
        })
        .expect("add_node");
    }
    s
}

fn slots(pairs: &[(&str, usize)]) -> indexmap::IndexMap<String, usize> {
    pairs
        .iter()
        .map(|(id, slot)| ((*id).to_string(), *slot))
        .collect()
}

/// The HAPPY path, and the anti-tautology control for every arm below: a map
/// that names exactly this worker's nodes binds them to exactly the supervisor's
/// slots, marks them on the fire path, and says nothing.
#[test]
#[tracing_test::traced_test]
fn a_matching_slot_map_binds_every_node_to_the_supervisors_slot_and_is_silent() {
    let mut s = scheduler_with(&["scan", "plan", "drive"]);
    let spy = Arc::new(SpyMarker::default());

    // Deliberately NOT the declaration order: the supervisor's ordering is the
    // contract, and a binding that quietly re-derived it from `nodes` would pass
    // an identity map while renaming every wedged node.
    assert!(
        s.set_wedge_page(
            spy.clone(),
            &slots(&[("scan", 2), ("plan", 0), ("drive", 1)])
        ),
        "a clean install must report success"
    );
    s.step_ms(10);

    assert_eq!(
        spy.entered_slots(),
        vec![2, 0, 1],
        "each node must mark the slot the SUPERVISOR gave it, in fire order — a \
         re-derived ordering would show [0, 1, 2] here and the supervisor would \
         then name the wrong node for every wedge"
    );
    assert!(
        !logs_contain("does not hold") && !logs_contain("OMITS"),
        "a matching map must produce no desync report at all — without this arm, \
         every 'is loud' assertion below is satisfied by a reporter that fires \
         unconditionally"
    );
}

/// The map is AHEAD of this worker: it names a node that is not here.
#[test]
#[tracing_test::traced_test]
fn a_slot_map_naming_a_node_this_worker_does_not_hold_is_loud_and_still_binds_the_rest() {
    let mut s = scheduler_with(&["scan"]);
    let spy = Arc::new(SpyMarker::default());

    assert!(
        s.set_wedge_page(spy.clone(), &slots(&[("scan", 0), ("ghost", 1)])),
        "a partial alarm beats none — the page is still installed"
    );
    s.step_ms(10);

    assert_eq!(
        spy.entered_slots(),
        vec![0],
        "the node that DID match must still be watched"
    );
    assert!(
        logs_contain("does not hold"),
        "a slot naming an absent node must be reported"
    );
    assert!(logs_contain("ghost"), "…and must NAME it");
    assert!(
        !logs_contain("OMITS"),
        "nothing was omitted — the two directions must not be conflated"
    );
}

/// **THE ONE THAT WENT SILENT.** The map is BEHIND this worker: it omits a node
/// the scheduler holds.
///
/// That node then never marks, so its slot is one the supervisor never reads and
/// the omission looks exactly like a rank with fewer nodes. Before this report
/// the condition was computed (`bound`) and used for nothing.
#[test]
#[tracing_test::traced_test]
fn a_slot_map_omitting_a_node_this_worker_holds_is_loud_about_the_node_it_left_unwatched() {
    let mut s = scheduler_with(&["scan", "forgotten"]);
    let spy = Arc::new(SpyMarker::default());

    assert!(s.set_wedge_page(spy.clone(), &slots(&[("scan", 0)])));
    s.step_ms(10);

    assert_eq!(
        spy.entered_slots(),
        vec![0],
        "only the mapped node marks — which is exactly why the supervisor cannot \
         see the omission from its own side"
    );
    assert!(
        logs_contain("OMITS"),
        "an omitted node must be reported: it gets NO alarm, and unlike an unknown \
         slot there is no unmoving slot for the observer to notice"
    );
    assert!(
        logs_contain("forgotten"),
        "…and the report must NAME the node left unwatched, not merely count it"
    );
    assert!(
        !logs_contain("does not hold"),
        "no slot named an absent node — the two directions must not be conflated"
    );
}

/// A SECOND install is refused, and the refusal is REPORTED BACK.
///
/// The contract is install-once-before-step-0: the supervisor's observer is
/// already accumulating dwell against the first page's counters, so a swap would
/// re-anchor every node's pair mid-run. Returning the verdict is what stops a
/// caller logging "wedge page opened and bound" for a binding that never
/// happened — the misleading-surface class, in the one log line that is a
/// reader's only evidence the rank is watched.
#[test]
#[tracing_test::traced_test]
fn a_second_install_is_refused_and_reports_the_refusal_to_its_caller() {
    let mut s = scheduler_with(&["scan"]);
    let first = Arc::new(SpyMarker::default());
    let second = Arc::new(SpyMarker::default());

    assert!(s.set_wedge_page(first.clone(), &slots(&[("scan", 0)])));
    assert!(
        !s.set_wedge_page(second.clone(), &slots(&[("scan", 7)])),
        "a re-install must report FAILURE, or its caller logs a binding that was \
         refused"
    );
    s.step_ms(10);

    assert_eq!(
        first.entered_slots(),
        vec![0],
        "the FIRST page keeps the binding"
    );
    assert!(
        second.entered_slots().is_empty(),
        "the refused page must never be marked — a swap would silently re-anchor \
         every node's seq pair under an observer already accumulating dwell"
    );
    assert!(logs_contain("REFUSING the re-install"));
}
