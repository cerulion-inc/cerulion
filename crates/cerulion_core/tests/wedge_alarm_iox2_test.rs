// SPDX-License-Identifier: AGPL-3.0-only
//! The wedge alarm's OBSERVABLE half, over a REAL `GraphRuntime`.
//!
//! `wedge_page`'s own unit tests drive the SHM page directly; `cerulion_cli_engine`'s
//! `wedge_alarm` tests drive the dwell rule against hand vectors. Neither can see the
//! thing that actually matters: whether a node that is INSIDE a tick right now says so
//! — in process, through [`NodeHandle::in_tick_since_ns`], and across processes,
//! through the page's seq pair. That claim needs a tick that is genuinely still
//! running while somebody else looks, which needs two threads and a real runtime.
//!
//! Every arm here uses a tick that BLOCKS on a channel rather than one that sleeps.
//! A sleep would make the assertion a race against a wall — the class, where
//! a nominal 150 ms is charged as 1100–1696 ms under macOS background QoS — whereas a
//! blocked tick is still blocked however slow the machine is, so the observation is
//! ordered by the channel rather than by luck. Every wait the test itself does is a
//! generous liveness ceiling in seconds, which load can delay but never invert.
//!
//! ```bash
//! cargo test -p cerulion_core --test wedge_alarm_iox2_test
//! ```
//!
//! Parallel-safe: `build_for_test` mints a per-test SHM root, and each page tag is
//! pid+name scoped.

#![cfg(unix)]

use std::sync::mpsc;
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, NodeDef};
use cerulion_core::graph::node::{ClosureNodeEntry, MacroPolicy, NodeEntry, NodeInfo};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::wedge_page::{MappedWedgePage, SlotReading};
use indexmap::IndexMap;

/// A generous liveness ceiling. Nothing here is timed against it — it exists so a
/// wedged harness FAILS instead of hanging CI to its job timeout.
const CEILING: Duration = Duration::from_secs(30);

fn tag(what: &str) -> String {
    format!("it_{what}_{}", std::process::id())
}

/// The step size every arm drives, matched to the node's period so ONE step is
/// exactly ONE fire.
///
/// Load-bearing rather than cosmetic: a period SHORTER than the step makes
/// `evaluate_node` fire the node repeatedly to catch up WITHIN one step, so a
/// blocking tick would block again on the very next catch-up fire — with nothing
/// left to release it, the harness deadlocks (MEASURED, at `period_ms = 1` against
/// this 10 ms step, before the periods were matched).
const STEP: Duration = Duration::from_millis(10);

/// A one-node graph whose `tick` runs `body`.
///
/// `period_ms` matches [`STEP`], so the node fires exactly once per step and the
/// arms below never have to reason about a catch-up burst.
fn one_node_graph(
    node_id: &str,
    body: impl FnMut() + Send + 'static,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "wedge_test".to_string(),
        prefix: "wt".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: node_id.to_string(),
            node_type: "closure".to_string(),
            inputs: vec![],
            outputs: vec![],
        }],
    };
    let mut body = body;
    let entry = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec![]).with_policy(MacroPolicy::Period { period_ms: 10 }),
        move |_ctx| {
            body();
            Ok(())
        },
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(node_id.to_string(), Box::new(entry));
    (config, factories)
}

/// THE headline: a tick that has not returned is visible AS SUCH, from inside the
/// process and from another one, WHILE it is still running.
///
/// This is the condition `tick_within_missed_count` structurally cannot report — that
/// counter is bumped from an elapsed read AFTER the callback returns, so a tick that
/// never returns increments nothing. It is asserted IN THE SAME BODY here, because
/// "the new observable moved" is a much weaker claim than "the new observable moved
/// where the existing one is blind".
#[test]
fn a_tick_that_has_not_returned_is_visible_in_process_and_across_the_page() {
    let (entered_tx, entered_rx) = mpsc::channel::<()>();
    let (release_tx, release_rx) = mpsc::channel::<()>();
    let fires = Arc::new(std::sync::atomic::AtomicU64::new(0));
    let f = fires.clone();
    let (config, factories) = one_node_graph("slow", move || {
        // Announce, then BLOCK — on the FIRST fire only. The tick is genuinely
        // still on the stack for as long as this receive waits, so every assertion
        // below is ordered by the channel rather than by a wall. Later fires must
        // return: one release cannot free two blocked ticks, and a tick still
        // waiting for a second one would DEADLOCK the harness rather than fail it.
        if f.fetch_add(1, std::sync::atomic::Ordering::SeqCst) == 0 {
            let _ = entered_tx.send(());
            let _ = release_rx.recv();
        }
    });
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build the graph");

    // The supervisor's half: a page this test owns, bound to the runtime exactly
    // as `graph run-worker` binds the one its plan names.
    let t = tag("headline");
    let owner = Arc::new(MappedWedgePage::create_owned(&t, 0, 1).expect("create the page"));
    let peer = MappedWedgePage::open_unowned(&t, 0).expect("open it as the supervisor would");
    let mut slots = IndexMap::new();
    slots.insert("slow".to_string(), 0usize);
    runtime.set_wedge_page(owner.clone(), &slots);

    let handle = runtime.node_handle("slow").expect("node handle").clone();

    // Before the first fire: not in a tick, and the pair is level.
    assert_eq!(handle.in_tick_since_ns(), None);
    assert_eq!(
        peer.read_slot(0),
        Some(SlotReading {
            entered: 0,
            exited: 0
        })
    );

    let stepper = std::thread::spawn(move || {
        runtime.step(STEP);
        runtime
    });

    entered_rx.recv_timeout(CEILING).expect("the tick started");

    // WHILE the tick is on the stack.
    let during = handle
        .in_tick_since_ns()
        .expect("a node inside a tick must say so — this is the whole observable");
    assert!(
        during > 0,
        "0 is the not-in-a-tick sentinel and must never be served as a stamp"
    );
    let reading = peer.read_slot(0).expect("slot 0");
    assert_eq!(
        reading,
        SlotReading {
            entered: 1,
            exited: 0
        },
        "the cross-process pair must show entry WITHOUT exit — this is the shape a \
         supervisor accumulates dwell against"
    );
    assert!(reading.in_tick());
    // The existing counter is blind here, and that is why this feature exists.
    assert_eq!(
        handle.tick_within_missed_count(),
        0,
        "`tick_within_ms` times only ticks that RETURN, so it reports nothing about \
         a tick that has not — the premise of the wedge alarm, asserted rather than \
         assumed"
    );

    release_tx.send(()).expect("release the tick");
    let runtime = stepper.join().expect("the step thread must not panic");
    drop(runtime);

    // After it returns: cleared, and the pair is level again.
    assert_eq!(
        handle.in_tick_since_ns(),
        None,
        "a returned tick must clear the marker — a leaked one would report every \
         healthy node as permanently wedged"
    );
    assert_eq!(
        peer.read_slot(0),
        Some(SlotReading {
            entered: 1,
            exited: 1
        })
    );
    assert!(!peer.read_slot(0).expect("slot 0").in_tick());
}

/// A tick that PANICS still clears the marker.
///
/// The exit store sits beside the post-callback `elapsed_ns` read, which control
/// reaches even on a caught panic (the panic is contained by the `catch_unwind`
/// above it — the invariant its replay-suppress clear rests on in the same
/// function). Without that placement a single panicking tick would leave the node
/// permanently marked in-tick and its supervisor would report a wedge for a node that
/// is firing perfectly well ever after — which is exactly what the second half of
/// this test drives.
#[test]
fn a_panicking_tick_clears_the_marker_and_the_next_fires_still_pair_up() {
    let panics = Arc::new(std::sync::atomic::AtomicU64::new(0));
    let p = panics.clone();
    let (config, factories) = one_node_graph("boom", move || {
        // Panic on the FIRST fire only, so the arms below can watch the node
        // recover rather than only observing the crash.
        if p.fetch_add(1, std::sync::atomic::Ordering::SeqCst) == 0 {
            panic!("wedge alarm test: a tick that panics");
        }
    });
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build the graph");

    let t = tag("panic");
    let owner = Arc::new(MappedWedgePage::create_owned(&t, 0, 1).expect("create the page"));
    let peer = MappedWedgePage::open_unowned(&t, 0).expect("open");
    let mut slots = IndexMap::new();
    slots.insert("boom".to_string(), 0usize);
    runtime.set_wedge_page(owner.clone(), &slots);
    let handle = runtime.node_handle("boom").expect("node handle").clone();

    // The panic is caught by the scheduler, so this step returns normally.
    runtime.step(STEP);

    assert_eq!(
        handle.panic_count(),
        1,
        "precondition: the fire really did panic (otherwise this test proves nothing)"
    );
    assert_eq!(
        handle.in_tick_since_ns(),
        None,
        "a CAUGHT panic must still clear the marker — a leaked one marks this node \
         wedged forever while it keeps firing"
    );
    assert_eq!(
        peer.read_slot(0),
        Some(SlotReading {
            entered: 1,
            exited: 1
        }),
        "the seq pair must close on the panic path too"
    );

    // And the node keeps firing cleanly, pairing up every time.
    for expected in 2..=5u64 {
        runtime.step(STEP);
        assert_eq!(
            peer.read_slot(0),
            Some(SlotReading {
                entered: expected,
                exited: expected
            }),
            "fire {expected} must pair up"
        );
        assert_eq!(handle.in_tick_since_ns(), None);
    }
    assert_eq!(handle.panic_count(), 1, "only the first fire panicked");
}

/// A healthy node's pair advances once per fire, and the rank's step word advances
/// once per STEP — the two halves, measured against a hand oracle.
///
/// The step word is the companion for a rank wedged OUTSIDE any tick; without an
/// independent count there would be no way to tell "the loop stopped" from "a tick
/// is long", which is the whole reason it is a second word rather than a derived
/// one.
#[test]
fn a_healthy_node_pairs_once_per_fire_while_the_rank_word_counts_steps() {
    let (config, factories) = one_node_graph("healthy", || {});
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build the graph");

    let t = tag("healthy");
    let owner = Arc::new(MappedWedgePage::create_owned(&t, 0, 1).expect("create the page"));
    let peer = MappedWedgePage::open_unowned(&t, 0).expect("open");
    let mut slots = IndexMap::new();
    slots.insert("healthy".to_string(), 0usize);
    runtime.set_wedge_page(owner.clone(), &slots);

    const STEPS: u64 = 12;
    for _ in 0..STEPS {
        runtime.step(STEP);
    }
    assert_eq!(
        peer.read_slot(0),
        Some(SlotReading {
            entered: STEPS,
            exited: STEPS
        }),
        "the node fires once per step (its period matches STEP), and every fire \
         must close its pair"
    );
    assert_eq!(
        peer.step_progress(),
        STEPS,
        "the rank word counts STEPS, independently of what its nodes did"
    );
    assert!(!peer.read_slot(0).expect("slot 0").in_tick());
}

/// A node the slot map does not name is left un-marked, and the page is still
/// installed for the one that IS named.
///
/// The supervisor/worker desync arm: a partial alarm beats none, and the un-covered
/// node must be un-covered rather than silently landing in somebody else's slot.
#[test]
fn a_slot_map_naming_an_unknown_node_still_binds_the_ones_it_got_right() {
    let (config, factories) = one_node_graph("real", || {});
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build the graph");

    let t = tag("desync");
    let owner = Arc::new(MappedWedgePage::create_owned(&t, 0, 2).expect("create the page"));
    let peer = MappedWedgePage::open_unowned(&t, 0).expect("open");
    let mut slots = IndexMap::new();
    slots.insert("real".to_string(), 0usize);
    // A node this worker does not hold — the plan desync shape.
    slots.insert("ghost".to_string(), 1usize);
    runtime.set_wedge_page(owner.clone(), &slots);

    runtime.step(STEP);

    assert_eq!(
        peer.read_slot(0),
        Some(SlotReading {
            entered: 1,
            exited: 1
        }),
        "the node that DID match must still be watched"
    );
    assert_eq!(
        peer.read_slot(1),
        Some(SlotReading {
            entered: 0,
            exited: 0
        }),
        "the unknown node's slot must stay untouched — never quietly written by \
         whichever node happened to be nearby"
    );
}

/// An UN-installed page costs nothing, and the in-process observable works without
/// one.
///
/// The monolith / every-test path. `in_tick_since_ns` is in-process and needs no
/// SHM, so it must be live whether or not a supervisor ever created a page — this is
/// the arm that would fail if the marker were folded into the page rather than kept
/// beside it.
#[test]
fn the_in_process_marker_works_with_no_page_installed_at_all() {
    let (entered_tx, entered_rx) = mpsc::channel::<()>();
    let (release_tx, release_rx) = mpsc::channel::<()>();
    let fires = Arc::new(std::sync::atomic::AtomicU64::new(0));
    let f = fires.clone();
    let (config, factories) = one_node_graph("lonely", move || {
        if f.fetch_add(1, std::sync::atomic::Ordering::SeqCst) == 0 {
            let _ = entered_tx.send(());
            let _ = release_rx.recv();
        }
    });
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build the graph");
    let handle = runtime.node_handle("lonely").expect("node handle").clone();

    let stepper = std::thread::spawn(move || {
        runtime.step(STEP);
        runtime
    });
    entered_rx.recv_timeout(CEILING).expect("the tick started");
    assert!(
        handle.in_tick_since_ns().is_some(),
        "the in-process marker must not depend on a wedge page existing"
    );
    release_tx.send(()).expect("release");
    drop(stepper.join().expect("the step thread must not panic"));
    assert_eq!(handle.in_tick_since_ns(), None);
}
