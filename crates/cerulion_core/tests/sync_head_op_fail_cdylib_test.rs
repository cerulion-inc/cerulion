// SPDX-License-Identifier: AGPL-3.0-only
//! Per-set Sync arm 18, FFI half: the align driver's `Failed` policy is FAIL-CLOSED
//! at the cdylib seam, on a REAL `dlopen`ed node.
//!
//! # What was missing
//!
//! `cerulion_node_sync_head_op` had argument-guard coverage (null name, bad
//! UTF-8, unknown op, null out-params) and a pure-scheduler oracle for the
//! `Failed` POLICY, but nothing anywhere drove a `Failed` answer through the
//! real FFI on a real loaded node. The guard tests answer "the export refuses
//! bad arguments"; this answers the different question the design asks — when a
//! LOADED node's op genuinely fails mid-alignment, does the host do the
//! conservative thing?
//!
//! # Why the fault is scoped to ONE INPUT
//!
//! The policy is POSITION-AWARE: a failed probe resolves to the
//! descent-DISABLING answer for the site it was asked at — at the argmin
//! `None` (fire greedily), at the GATE `Present` (this input refuses the gate).
//! A fixture that fails EVERY op cannot tell those apart, because the argmin's
//! own probe fails first and the gate is never scanned. `test_node_sync_op_fail_cdylib`
//! therefore fails only `b`'s head ops, and leaves the FILLS alone — they ride
//! the drain symbols, so both heads fill from real frames and the matcher
//! reaches a complete tuple. What the fault removes is exactly the EVIDENCE a
//! descent would need.
//!
//! # The oracle is a MEMBERSHIP difference, not a log line
//!
//! On this stimulus the two runs disagree about which frames the node reads,
//! and that is the whole point: fail-closed means a failure can never be what
//! ENABLES a descent.
//!
//! * healthy — `b` is scarce, the gate passes on real evidence, the walk
//!   advances `a` onto its second frame and the node reads `(20, 20)`;
//! * faulted — `b`'s gate probe fails, that input refuses the gate, no descent
//!   runs, and the node reads its FIFO heads `(0, 20)`.
//!
//! Both are real published frames. Neither run fabricates a member, and the
//! faulted one still fires — greedy-or-wait, never a wrong set, never a wedge.
//!
//! `#[serial]` — real iceoryx2 plus the cdylib's process-global `NODES` and
//! probe statics, and the arms mutate a process env var.
//!
//! Build the fixture first: `cargo build -p test_node_sync_op_fail_cdylib`.

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;
use tracing_test::traced_test;

const TOPIC_A: &str = "/sof/a";
const TOPIC_B: &str = "/sof/b";
const MS: u64 = 1_000_000;
const FAIL_KNOB: &str = "CER_FAIL_MODE";

/// RAII so a panicking arm cannot leave the knob set for its sibling.
struct EnvVarGuard;
impl EnvVarGuard {
    fn set(value: &str) -> Self {
        std::env::set_var(FAIL_KNOB, value);
        Self
    }
}
impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        std::env::remove_var(FAIL_KNOB);
    }
}

/// What the fixture's process-global statics saw.
struct Probe {
    calls: u64,
    faults: u64,
    fires: u64,
    a: f64,
    b: f64,
}

/// Read (and reset) the fixture's counters through a SECOND `dlopen` of the
/// same path. `dlopen` refcounts, so the host's handle and this one share the
/// statics — the same trick `cdylib_snapshot_set_failure_test` uses.
fn probe(reset: bool) -> Probe {
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_sync_op_fail_cdylib");
    // SAFETY: the fixture is built by this workspace; the two symbols are its
    // own `#[no_mangle] extern "C"` exports with exactly these signatures.
    unsafe {
        let lib = libloading::Library::new(&path).expect("open the fixture for probing");
        if reset {
            let f: libloading::Symbol<unsafe extern "C" fn()> =
                lib.get(b"sync_probe_reset\0").expect("reset symbol");
            f();
            return Probe {
                calls: 0,
                faults: 0,
                fires: 0,
                a: 0.0,
                b: 0.0,
            };
        }
        let f: libloading::Symbol<
            unsafe extern "C" fn(*mut u64, *mut u64, *mut u64, *mut u64, *mut u64),
        > = lib.get(b"sync_probe\0").expect("probe symbol");
        let (mut calls, mut faults, mut fires, mut a_bits, mut b_bits) = (0u64, 0, 0, 0, 0);
        f(
            &mut calls,
            &mut faults,
            &mut fires,
            &mut a_bits,
            &mut b_bits,
        );
        Probe {
            calls,
            faults,
            fires,
            a: f64::from_bits(a_bits),
            b: f64::from_bits(b_bits),
        }
    }
}

fn graph(prefix: &str) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_sync_op_fail_cdylib");
    let entry = DylibNodeEntry::load(&path).expect("load the sync-op-fail fixture");
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "sof".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "fuse".to_string(),
            node_type: "sync_op_fail".to_string(),
            inputs: vec![
                InputDef {
                    name: "a".to_string(),
                    source: TOPIC_A.to_string(),
                },
                InputDef {
                    name: "b".to_string(),
                    source: TOPIC_B.to_string(),
                },
            ],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fuse".to_string(), Box::new(entry));
    (config, factories)
}

/// A@0, A@20, B@20 — the descent shape. `b` holds no second frame, so on real
/// evidence the gate PASSES and the walk advances `a` from 0 to 20.
fn drive(prefix: &str) -> GraphRuntime {
    let (config, factories) = graph(prefix);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build the cdylib sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on b");
    for (p, ts, x) in [(0u8, 0u64, 0.0f64), (0, 20 * MS, 20.0), (1, 20 * MS, 20.0)] {
        clock.set(ts);
        let publisher = if p == 0 { &mut pub_a } else { &mut pub_b };
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan");
        proxy.x = x;
    }
    clock.set(0);
    for _ in 0..6 {
        runtime.step(Duration::from_millis(1));
    }
    runtime
}

/// CONTROL — the fixture is a real per-set Sync node, and with no fault it
/// descends.
///
/// Without this the fault arm below is unreadable: `(0, 20)` is also what a
/// node that never descends at all produces, and a fixture whose ops were
/// simply broken would look identical to one obeying the policy.
#[test]
#[serial]
fn a_raw_ffi_sync_cdylib_descends_over_the_real_ffi_when_its_ops_are_healthy() {
    std::env::remove_var(FAIL_KNOB);
    probe(true);
    let runtime = drive("sofok");

    let p = probe(false);
    assert_eq!(p.faults, 0, "no fault is armed");
    assert!(
        p.calls > 0,
        "the align pass really crossed the FFI — without this the arm proves \
         only that the node was built"
    );
    assert_eq!(p.fires, 1, "one complete set fires");
    assert_eq!(
        (p.a, p.b),
        (20.0, 20.0),
        "on real evidence `b` is scarce, the gate passes, and the walk advances \
         `a` from 0 to 20 to tighten the span"
    );
    assert_eq!(
        runtime
            .node_handle("fuse")
            .expect("fuse handle")
            .sync_closer_skip_count(TOPIC_A),
        1,
        "a@0 was PASSED OVER — the descent is what makes the fault arm's \
         different answer meaningful"
    );
}

/// FAULT — a failed head op can never be the evidence that enables a descent.
///
/// `b`'s ops fail, so the gate scan asks an input that cannot answer. The
/// conservative reading is that this input REFUSES the gate, and the arm's
/// oracle is that the node then reads its FIFO heads instead of the tightened
/// pair: no descent on evidence that was never gathered. It still fires
/// (greedy-or-wait, never a wrong set), the members it reads are frames that
/// were really published, the failure is announced ONCE for the regime rather
/// than once per op, and the runtime survives to keep stepping.
#[test]
#[serial]
#[traced_test]
fn a_failed_head_op_over_the_real_ffi_refuses_the_descent_and_says_so_once() {
    let _guard = EnvVarGuard::set("sync_op_fail");
    probe(true);
    let runtime = drive("soffail");

    let p = probe(false);
    assert!(
        p.faults > 0,
        "the fault really fired across the FFI ({} calls, {} faults)",
        p.calls,
        p.faults
    );
    assert_eq!(
        p.fires, 1,
        "FAIL-CLOSED IS NOT FAIL-SILENT: a complete in-window tuple is still \
         served. Greedy-or-wait membership, never a wedge"
    );
    assert_eq!(
        (p.a, p.b),
        (0.0, 20.0),
        "THE PIN: the node read its FIFO HEADS, not the tightened pair the \
         healthy run reads. `b` could not answer the gate, so that input \
         refuses it and no descent runs — a failure must never be what lets a \
         descent proceed. Both members are frames that were really published; \
         nothing is fabricated"
    );
    assert_eq!(
        runtime
            .node_handle("fuse")
            .expect("fuse handle")
            .sync_closer_skip_count(TOPIC_A),
        0,
        "and nothing was passed over, which is the same statement read off the \
         counter instead of the members"
    );

    // WHAT IS NOT ASSERTED HERE, and why.
    //
    // A `probe().calls >= before` check after one more step, claiming
    // "the runtime survived", would be vacuous: an
    // unchanged counter is precisely the node being disabled, so `>=` accepts
    // the one outcome it exists to rule out.
    //
    // `>` does not fix it, and neither does a fire count —
    // both are FALSE on this fixture. The head ops are DEMAND-driven and the
    // FILLS ride the drain symbols, so an align pass can legitimately cross this
    // FFI zero times; and driving a fresh pair through this node after the fault
    // is cleared does not produce a second fire, for reasons that belong to the
    // fixture's own lifecycle rather than to the contract under test.
    //
    // So no such claim is made here. It is not lost: `p.fires
    // == 1` above is asserted WITH THE FAULT ARMED, and a node disabled by the
    // failure fires zero times. "The failure disables descent, not the node" is
    // therefore already pinned by the arm's own membership oracle, which is the
    // stronger statement anyway — it says what the node DID, not merely that
    // something called it.

    logs_assert(|lines: &[&str]| {
        let loud = lines
            .iter()
            .filter(|l| l.contains("ERROR") && l.contains("sync alignment op FAILED"))
            .count();
        if loud != 1 {
            return Err(format!(
                "a failing op is announced ONCE per regime, not once per op — \
                 got {loud} loud lines"
            ));
        }
        let head = lines
            .iter()
            .find(|l| l.contains("sync alignment op FAILED"))
            .copied()
            .unwrap_or("<none>");
        // The loud line names the RESOLVED TOPIC (the scheduler's key space),
        // not the macro field name the FFI is called with — worth pinning
        // because they differ and an operator greps the one the log prints.
        if !head.contains("sync_input=/sof/b") {
            return Err(format!(
                "the loud line must NAME the input that failed, or an operator \
                 cannot tell which declaration to look at. Got: {head}"
            ));
        }
        // And the POSITION: `op=probe` is what makes the membership oracle
        // above readable as "the gate was refused" rather than "some op
        // somewhere failed". A mutating op failing would terminate the pass
        // instead, which is a different arm of the same policy.
        if !head.contains("op=probe") {
            return Err(format!(
                "the failure must be at a PROBE — that is the position whose \
                 fail-closed answer is `this input refuses the gate`. Got: {head}"
            ));
        }
        Ok(())
    });
}

/// The op-failure latch RE-ARMS: a regime that heals and recurs is loud again,
/// ON ONE NODE.
///
/// The site documents "loud first-of-regime … and one recovery `info!`", and
/// before this arm nothing ever called the latch's `on_success`. That made the
/// documented once-per-REGIME head a once-per-PROCESS head: a node whose ops
/// fail, heal, and fail again reported the second regime at `debug!`, which is
/// invisible at the default level, and the promised recovery line could never be
/// emitted at all. The counter kept climbing, so the failure was entirely in the
/// surface an operator actually watches.
///
/// ONE runtime, with the fault toggled underneath it — that scoping is the whole
/// arm. A version that built a fresh graph per phase passed against the unfixed
/// code, because each new node gets a new latch and every regime is trivially
/// "first". The fixture reads `CER_FAIL_MODE` on every call, so the same node
/// can be walked through fail → heal → fail.
#[test]
#[serial]
#[traced_test]
fn a_healed_op_failure_regime_is_loud_again_when_it_recurs() {
    probe(true);
    let (config, factories) = graph("sofrearm");
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build the cdylib sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on b");

    // A descent-shaped pair per phase, so the ops really run.
    // Several pairs per phase, so a FAULTED phase produces more than one
    // failure: the shared latch's recovery line is emitted only when a regime
    // actually SUPPRESSED something (the lone-error silent re-arm), so a
    // one-failure regime would re-arm correctly and legitimately say nothing.
    let mut feed = |runtime: &mut GraphRuntime, base: u64| {
        for round in 0..4u64 {
            for (p, off, x) in [(0u8, 0u64, 0.0f64), (0, 20, 20.0), (1, 20, 20.0)] {
                clock.set((base + round * 40 + off) * MS);
                let publisher = if p == 0 { &mut pub_a } else { &mut pub_b };
                let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan");
                proxy.x = x + round as f64;
            }
        }
        clock.set(base * MS);
        for _ in 0..16 {
            runtime.step(Duration::from_millis(1));
        }
    };

    {
        let _guard = EnvVarGuard::set("sync_op_fail");
        feed(&mut runtime, 0);
    }
    // HEAL on the SAME node: every op answers now.
    feed(&mut runtime, 100);
    // And fail again.
    {
        let _guard = EnvVarGuard::set("sync_op_fail");
        feed(&mut runtime, 200);
    }

    logs_assert(|lines: &[&str]| {
        let loud = lines
            .iter()
            .filter(|l| l.contains("ERROR") && l.contains("sync alignment op FAILED"))
            .count();
        // `>= 2`, not `== 2`, and the looseness is the correct reading rather
        // than a softened one: the latch tracks per-PASS health, so a faulted
        // phase whose passes are interleaved with healthy ones opens and closes
        // the regime more than once. What is CONTRACTED is that a healed regime
        // recurring is loud AGAIN; with a latch that never re-arms this count is
        // exactly 1 however long the run, because every later head is
        // suppressed to `debug!`.
        if loud < 2 {
            return Err(format!(
                "each regime gets its OWN loud head — got {loud}. One means the \
                 latch never re-armed and every later regime was reported at \
                 `debug!`, where nobody sees it"
            ));
        }
        let recovered = lines
            .iter()
            .filter(|l| l.contains("INFO") && l.contains("sync alignment ops RECOVERED"))
            .count();
        if recovered == 0 {
            return Err(
                "the recovery line the site documents must actually be EMITTED — \
                 with no healthy transition calling `on_success` it never could \
                 be, whatever the run did"
                    .to_string(),
            );
        }
        Ok(())
    });
}
