// SPDX-License-Identifier: AGPL-3.0-only
//! A data-trigger consumer's THROUGHPUT is not capped at one frame
//! per scheduler step — a queued burst is served WITHIN the step that sees it.
//!
//! # The defect this file exists to catch
//!
//! Per-message FIFO firing made a data-trigger node fire once per
//! queued frame, in arrival order, and lost no frame — but it consumed exactly
//! ONE pending arrival per step and CARRIED the rest. That is a THROUGHPUT cap,
//! and behind a producer that can publish more than one frame per step it is a
//! PERMANENT lag rather than a transient one: one frame in, one frame out, per
//! step, forever. The consumer's input queue fills to its depth, `drop_oldest`
//! starts evicting, and every frame the consumer serves is a whole queue-depth
//! old.
//!
//! It is not a corner: an in-graph `period_ms = 1` producer against a live loop
//! stepping at ~1 kHz needs ONE late wake — any wake — to catch-up-fire k > 1
//! frames in a single step, and from that instant the consumer is k behind with
//! no mechanism to work it off. MEASURED on the merge commit's own main-push
//! run, on BOTH runners: the CLI e2e graph-latency gate went from ~25 us p50 to
//! **8752 us p50**, floor 7.5 ms, max ~9.9 ms at every payload size — which is
//! exactly `depth (10) x period (1 ms)` — accompanied by a steady stream of
//! `drop_oldest` eviction warnings on the consumer's trigger input.
//!
//! # What each arm pins, and why there are several
//!
//! The arms drive DIFFERENT seams and reach the burst by different means, so
//! none subsumes the other:
//!
//! * The POLLED arm reproduces the CI failure's own MECHANISM — an in-graph
//!   `period_ms = 1` producer CATCH-UP-FIRING a burst inside one step, exactly
//!   as a late wake makes it. Its oracle is the full delivered sequence.
//! * The LIVE arm drives `run_live_step_once_for_test`, the seam a real robot
//!   runs on, and reaches the burst DETERMINISTICALLY (a foreign publisher's
//!   frames land before the step begins) rather than depending on how much wall
//!   time a live step happens to advance the clock by.
//! * The cdylib arm carries both across the FFI, through the production surface.
//!
//! Those fail on `52125241e` with the same signature: the consumer serves ONE
//! frame and the rest sit in the queue.
//!
//! * The COLLAPSED-TICK arm pins the OTHER side of the same seam: a fire whose
//!   tick never reached the trigger's read must NOT be refilled on the head it
//!   left behind. Without that guard, this arm fails with 64 fires per
//!   step on ONE frame — a same-frame re-fire storm the throughput arms above
//!   cannot see, because their ticks all consume.
//!
//! # Section (c): SERVE-MANY — the burst reaches the CONTEXT input too
//!
//! The arms above all use a trigger-ONLY consumer, and that is exactly why they
//! could not see the residual this file used to record: a consumer that also has
//! a plain non-trigger `#[input]` context was STILL capped at one message per
//! step, because `snapshot_inputs` freezes such an input once per level pass
//! while `try_view` CONSUMED the frozen slot — so fire 2 of the burst found it
//! empty and its tick chain collapsed. That is the motivating shape (a
//! data-triggered controller with a slow `/map` context), i.e. the common one.
//!
//! The frozen-slot reuse rule closes it: the frozen slot serves every fire
//! of the step, the same bytes to each. Section (c)'s arms drive that from four
//! directions — the headline throughput+identity oracle, the same-step-publish
//! DISCRIMINATOR (which is the only one that can tell the fix from a serve-many
//! that re-reads LIVE), determinism, and the never-delivered context under a
//! real burst — plus a cdylib parity arm whose exact scope is stated on it.
//!
//! Every oracle is HAND-WRITTEN, and every measured window is anchored on the
//! PRODUCER's own fire count rather than on a warm-up observation count.
//!
//! `#[serial]`: real iceoryx2 + the live WaitSet seam.
//!
//! # Running
//!
//! ```bash
//! cargo build -p test_node_macro_data_trigger_cdylib \
//!             -p test_node_macro_burst_ctx_cdylib
//! cargo test -p cerulion_core --test fifo_burst_within_step_iox2_test \
//!     -- --test-threads=1
//! ```

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{
    BackpressurePolicy, ClosureNodeEntry, DylibNodeEntry, InputMeta, NodeEntry, NodeInfo,
};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::MacroPolicy;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// The burst every arm drives. Chosen to match the CI failure's own shape (a
/// `period_ms = 1` producer catching up over a 10 ms step) while staying well
/// inside the consumer's provisioned queue, so `drop_oldest == 0` is a real
/// assertion about firing rather than an accident of headroom.
const BURST: u64 = 10;

/// Deeper than `BURST`, so nothing in these arms can be explained by eviction.
const CONSUMER_DEPTH: usize = 16;

// ===========================================================================
// The producer: the CI failure's own shape — `period_ms = 1`, publishing its
// own fire index, so the delivered VALUES are self-describing.
// ===========================================================================

#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct BurstPing {
    #[output]
    out: Vector3,
    n: u64,
}

#[cerulion_node_impl]
impl BurstPing {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// A data-trigger consumer recording every value its `try_view` observes, in
/// order. `try_view` is the frozen-slot-served read path the `#[cerulion_node]`
/// macro's generated tick uses, so this is the production read semantics.
fn recording_consumer(seen: Arc<Mutex<Vec<u64>>>) -> ClosureNodeEntry {
    let info = NodeInfo::with_meta(
        vec![InputMeta {
            name: "inp".to_string(),
            schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
            trigger: true,
            depth: CONSUMER_DEPTH,
            backpressure: BackpressurePolicy::DropOldest,
            expect_within_ms: None,
        }],
        vec![],
    )
    .with_policy(MacroPolicy::DataTrigger {
        input_name: "inp".to_string(),
    });
    ClosureNodeEntry::new(info, move |ctx| {
        if let Some(s) = ctx.subscriber_mut("inp") {
            if let Ok(Some(v)) = s.try_view::<Vector3, _>(|view| view.x as u64) {
                seen.lock().unwrap().push(v);
            }
        }
        Ok(())
    })
    .with_label("burst_pong")
}

/// The hand oracle: a contiguous run `a..=b`, which is what "every frame the
/// producer committed over that span, in order, exactly once" looks like.
fn oracle(a: u64, b: u64) -> Vec<u64> {
    (a..=b).collect()
}

/// Warm-up steps to spend before a measured window. Generous: on a healthy desk
/// the first 1 ms step already delivers, and the loop below stops the instant
/// the graph is provably caught up.
const WARMUP_STEP_BUDGET: usize = 64;

/// Step 1 ms at a time until the consumer is provably CAUGHT UP — it has served
/// the NEWEST frame the producer has committed — and return the producer's fire
/// count at that instant.
///
/// The measured windows below anchor on that returned count, NOT on how many
/// frames the consumer happened to see: iceoryx2 connection warm-up can drop the
/// first frames, so a warm-up count folded into an oracle makes the arm pass
/// only when the warm-up it performs was unnecessary (the `lazy_loan_iox2_test`
/// class). The convergence CONDITION is drop-tolerant — a dropped frame simply
/// means the producer commits another one on the next step and the loop keeps
/// going — and it is a liveness bound in steps, never a wall.
fn warm_up_until_caught_up(runtime: &mut GraphRuntime, seen: &Arc<Mutex<Vec<u64>>>) -> u64 {
    for _ in 0..WARMUP_STEP_BUDGET {
        runtime.step(Duration::from_millis(1));
        let fires = runtime
            .node_handle("ping")
            .expect("the producer is a scheduler node")
            .fire_count();
        if seen.lock().unwrap().last().copied() == Some(fires) {
            return fires;
        }
    }
    panic!(
        "the consumer never caught up to the producer within {WARMUP_STEP_BUDGET} warm-up \
         steps (observed {:?})",
        seen.lock().unwrap()
    );
}

// ===========================================================================
// (a) THE REGRESSION PIN — polled twin: the CI failure's own mechanism.
// ===========================================================================

/// A `period_ms = 1` producer catch-up-firing `BURST` times inside ONE step
/// publishes `BURST` frames, and the data-trigger consumer downstream of it
/// must serve ALL of them in that same step.
///
/// This is the CI failure reproduced in miniature: the producer is in-graph on
/// the level above, so its whole burst is published before the consumer's level
/// is drained, and a consumer capped at one frame per step ends the step
/// `BURST - 1` frames behind with nothing that can work the deficit off. On
/// `52125241e` the observed sequence is `[1]`.
///
/// `drop_oldest == 0` is asserted in the same body so "it served everything"
/// cannot be satisfied by a queue that quietly threw the rest away.
#[test]
#[serial]
fn a_catch_up_burst_is_served_within_the_step_that_publishes_it() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut runtime = build_in_graph_chain("fbwsPolled", Arc::clone(&seen));

    // Warm-up: step until the consumer has served the producer's NEWEST frame,
    // so the measured step below is a pure burst. `base` is the PRODUCER's own
    // fire count at that instant — never the consumer's observation count,
    // which a dropped warm-up frame would make disagree with it.
    let base = warm_up_until_caught_up(&mut runtime, &seen);
    let base_seen = seen.lock().unwrap().len();

    // ONE step advancing BURST periods ⇒ the Period producer catches up
    // `BURST` times (max_catchup defaults to unbounded) and publishes `BURST`
    // frames, all before the consumer's level is drained.
    runtime.step(Duration::from_millis(BURST));

    let observed_new: Vec<u64> = seen.lock().unwrap()[base_seen..].to_vec();
    let ping_fires = runtime
        .node_handle("ping")
        .expect("the producer is a scheduler node")
        .fire_count();
    assert_eq!(
        ping_fires - base,
        BURST,
        "precondition: the Period producer must really have caught up BURST \
         times in the measured step (its own fire count, before and after, is \
         the evidence — the consumer's count is what is under test)"
    );
    assert_eq!(
        observed_new,
        oracle(base + 1, base + BURST),
        "every frame the producer committed in the measured step must be \
         served, in order, by the end of that step — a consumer capped at one \
         frame per step serves exactly one of them and never catches up"
    );
    assert_eq!(
        runtime
            .node_handle("sink")
            .expect("the consumer is a scheduler node")
            .backpressure_drop_oldest_count("inp"),
        0,
        "nothing was evicted — the burst was SERVED, not discarded"
    );
}

/// The lag really is worked off, not merely masked by a one-off: after the
/// burst step the consumer is CAUGHT UP, so a single further frame is served by
/// the very next step.
///
/// Under one-fire-per-step this is the assertion that cannot be satisfied at
/// all — the consumer is permanently `BURST - 1` frames behind, so the frame it
/// serves next step is an old one and the newest frame stays queued forever.
#[test]
#[serial]
fn after_a_burst_the_consumer_is_caught_up_and_serves_the_next_frame_at_once() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut runtime = build_in_graph_chain("fbwsCaught", Arc::clone(&seen));

    let base = warm_up_until_caught_up(&mut runtime, &seen);
    let base_seen = seen.lock().unwrap().len();
    runtime.step(Duration::from_millis(BURST));
    // Anchor the intermediate window so this arm kills a carry regression on
    // its own: without it the final oracle ADAPTS to however few frames the
    // burst step served and stays self-consistent (deleting the refill
    // passes this arm while failing its sibling).
    assert_eq!(
        seen.lock().unwrap()[base_seen..].to_vec(),
        oracle(base + 1, base + BURST),
        "precondition: the burst step must have served the whole burst"
    );

    // One more period: the producer commits exactly ONE further frame.
    runtime.step(Duration::from_millis(1));

    let observed_new: Vec<u64> = seen.lock().unwrap()[base_seen..].to_vec();
    assert_eq!(
        observed_new,
        oracle(base + 1, base + BURST + 1),
        "a caught-up consumer serves the newest frame the step after it is \
         published; a consumer still carrying a backlog serves a stale one"
    );
}

// ===========================================================================
// (a) THE REGRESSION PIN — the LIVE seam.
// ===========================================================================

/// The seam a robot actually runs on. A foreign publisher's burst is already
/// queued when the live step begins, so this reaches the same state as the
/// catch-up arm without depending on how far a live step advances the clock.
///
/// On `52125241e` the observed sequence is `[1]`.
#[test]
#[serial]
fn a_queued_burst_is_served_within_one_live_step() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let (config, factories) = external_source_graph("fbwsLive", Arc::clone(&seen));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build live burst graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher("/fbws/ext", MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");

    // Warm-up: one frame + one live step establishes the connection.
    publish(&mut ext, 0);
    runtime.run_live_step_once_for_test(Duration::from_millis(200));
    seen.lock().unwrap().clear();

    for v in 1..=BURST {
        publish(&mut ext, v);
    }
    runtime.run_live_step_once_for_test(Duration::from_millis(200));

    let observed = seen.lock().unwrap().clone();
    assert_eq!(
        observed,
        oracle(1, BURST),
        "one live step must serve the whole queued burst; a consumer capped \
         at one frame per step observes only [1]"
    );
    assert_eq!(
        runtime
            .node_handle("sink")
            .expect("the consumer is a scheduler node")
            .backpressure_drop_oldest_count("inp"),
        0,
        "nothing was evicted — the burst was SERVED, not discarded"
    );
}

// ===========================================================================
// (b) THE COLLAPSED-TICK PIN — a fire that did NOT consume the head must not
//     be refilled on that same head.
// ===========================================================================

/// The shape the burst refill can get wrong: a NON-TRIGGER `#[input]` declared
/// BEFORE the trigger.
///
/// `#[cerulion_node_impl]` nests one `try_view` per input in DECLARATION order,
/// so while `ctx_in` has never been delivered (the pre-first-delivery
/// WAIT) the whole chain collapses and `inp`'s read never runs — the frame the
/// boundary drain popped and froze for it stays UNSERVED.
///
/// `depth = 32` is deeper than anything this arm queues, so `drop_oldest == 0`
/// is an assertion about firing and not an accident of headroom.
#[cerulion_node]
#[derive(Default)]
struct CollapsedPong {
    #[input]
    ctx_in: Vector3,
    #[input(trigger, depth = 32)]
    inp: Vector3,
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl CollapsedPong {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Forwarding the trigger's value is what makes the downstream sink's
        // record an oracle for what this node actually CONSUMED.
        self.out.x = self.inp.x;
        self.out.y = self.ctx_in.x;
        Ok(())
    }
}

/// Steps to hold the collapse open for. Comfortably inside the trigger input's
/// declared depth, so the backlog it builds is queued rather than evicted.
const COLLAPSE_STEPS: u64 = 8;

/// A fire whose tick never reached the trigger's `try_view` must be followed by
/// EXACTLY NO refill — one fire per step while the collapse lasts — and the
/// backlog it built must drain WITHIN one step once the collapse clears.
///
/// # The defect
///
/// The boundary drain RE-OFFERS an unserved frozen head (`(1, head_ts)`, popping
/// nothing) so a fire that could not consume it keeps coming back — Principle #6,
/// and without it such a node WEDGES. The burst loop's refill called that same
/// entry point and read `popped != 0` as a fresh frame, so it re-fired on the
/// SAME frame until the per-step cap (`DATA_PENDING_CARRY_CLAMP` = 64), set
/// `data_backlog_hint`, and had `ns_until_next_fire` report due-NOW — pinning the
/// live loop at its 1 ms floor for ~64k no-op ticks/s, staging 64 `TraceEntry`s
/// per frame, and reaching `MAX_CONSECUTIVE_PANICS` (3) inside ONE step for a
/// panicking tick instead of across three.
///
/// Pre-branch semantics for exactly this shape were ONE fire per step; that is
/// what the first assertion restores and pins.
///
/// # The residual this arm MEASURED — now CLOSED (serve-many)
///
/// This arm used to record a residual it deliberately did not fix: a node with a
/// NON-TRIGGER `#[input]` could not serve a burst within one step AT ALL,
/// because `snapshot_inputs` freezes such an input ONCE per level pass while
/// `try_view` CONSUMED the frozen slot (`self.frozen.take()`) — so the SECOND
/// fire of a burst found the slot empty, and (correctly) did not live-read
/// instead, collapsing the tick again. The measured shape was 2 fires per step:
/// ONE served frame plus ONE popped frame whose tick collapsed, with the backlog
/// draining one frame per step. That is the earlier throughput, and it is
/// exactly the shape the motivating use case has (`TrigCtxConsumer`: a
/// data-triggered controller with a slow `/map` context).
///
/// The frozen-slot reuse rule closes it: the frozen slot now serves every
/// fire of the step (the same frozen bytes to each — a non-trigger input is
/// latest-value by contract), cleared and recaptured only at its capture sites.
/// So the HEAL half below inverted, and it is now the strongest oracle in this
/// file for the fix: the whole held backlog drains in ONE step, in arrival
/// order, one fire per served frame — no collapsed fire at all.
///
/// The COLLAPSE half above it is UNCHANGED and still pins what it always did: a
/// context input that has NEVER delivered still collapses the chain (the
/// pre-first-delivery WAIT — no fabricated default), the head is RE-OFFERED
/// rather than lost, and the refill must read that re-offer as "nothing new".
/// Serve-many does not weaken it: the frozen slot is `Empty` while nothing has
/// ever been delivered, and a re-served `Empty` is still `Ok(None)`.
///
/// # Why the oracles are shaped this way
///
/// The heal window's oracle is anchored on the node's own FIRE COUNT and on
/// contiguity, never on a measured warm-up count: a dropped warm-up frame shifts
/// which frame the collapse froze, and an oracle that baked that in would pass
/// only when the warm-up was unnecessary. The fire count is also what
/// DISCRIMINATES — the delivered sequence alone does not, because every fire the
/// defect adds is a collapsed tick that publishes nothing.
#[test]
#[serial]
fn a_fire_that_did_not_consume_the_head_is_not_refilled_on_it() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut runtime = build_collapsed_chain("fbwsCollapse", Arc::clone(&seen));
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ctx_pub = mgr
        .create_publisher("/fbwsCollapse/ctx", MaxSliceLen::const_new(64), 0)
        .expect("the absolute context source accepts a publisher");

    // Reach the collapsed state: step until the consumer has fired at least
    // once, which proves its trigger connection is live and a frame really was
    // popped and frozen for it.
    let mut warmed = false;
    for _ in 0..WARMUP_STEP_BUDGET {
        runtime.step(Duration::from_millis(1));
        if fire_count(&runtime, "collapsed") > 0 {
            warmed = true;
            break;
        }
    }
    assert!(
        warmed,
        "precondition: the collapsed consumer must fire at least once before \
         the measured window (its trigger connection has to be live)"
    );

    let fires_before = fire_count(&runtime, "collapsed");
    for _ in 0..COLLAPSE_STEPS {
        runtime.step(Duration::from_millis(1));
    }
    let fires_after = fire_count(&runtime, "collapsed");

    assert_eq!(
        fires_after - fires_before,
        COLLAPSE_STEPS,
        "a node whose tick collapses before reading its trigger fires EXACTLY \
         once per step: the boundary drain re-offers the head it is still \
         holding, and the burst loop's refill must read that re-offer as \
         'nothing new', not as a fresh frame to fire on"
    );
    assert!(
        seen.lock().unwrap().is_empty(),
        "anti-tautology: the collapse is REAL — the tick never ran to its \
         write, so nothing was published downstream"
    );

    // Heal: one context frame, then ONE step. The held head is finally
    // CONSUMED, and under serve-many so is everything queued behind
    // it: the frozen context is re-served to every fire of the burst instead of
    // collapsing fire 2.
    publish(&mut ctx_pub, 7);
    let fires_at_heal_start = fire_count(&runtime, "collapsed");
    runtime.step(Duration::from_millis(1));

    let healed = seen.lock().unwrap().clone();
    let head = *healed
        .first()
        .expect("the healing step must serve at least the head it was holding");
    // The producer's OWN fire count is the hand oracle for "what had been
    // committed by the end of this step" — `BurstPing` publishes its fire index,
    // so a caught-up consumer's last observation IS that number. Anchoring on it
    // rather than on a measured count is what makes this an oracle and not a
    // self-compare (a dropped warm-up frame shifts `head`, never the identity).
    assert_eq!(
        healed,
        oracle(head, fire_count(&runtime, "ping")),
        "ONE healing step must serve the head AND the whole backlog queued \
         behind it, in ARRIVAL ORDER, nothing skipped and nothing served twice \
         — leaving the consumer caught up with the producer. Under serve-ONCE \
         this reads a single frame: fire 2 of the burst found the frozen \
         context slot consumed and collapsed"
    );
    assert!(
        healed.len() as u64 > COLLAPSE_STEPS,
        "precondition: the collapse really did build a multi-frame backlog for \
         the healing step to drain, so the assertion above is about BURSTING \
         and not about a queue that happened to hold one frame — observed \
         {healed:?}"
    );
    assert_eq!(
        fire_count(&runtime, "collapsed") - fires_at_heal_start,
        healed.len() as u64,
        "and it fires EXACTLY once per frame it served: no collapsed fire (the \
         pre-serve-many shape fired twice and served one), and no same-frame \
         re-fire (which shows up here as the per-step cap, {})",
        cerulion_core::graph::topology::MAX_CONSUMER_DEPTH,
    );

    // The node is now CAUGHT UP: it tracks the producer one frame per step, and
    // its fire rate is flat at ONE — the collapsed second fire is gone too.
    let fires_before_drain = fire_count(&runtime, "collapsed");
    for _ in 0..COLLAPSE_STEPS {
        runtime.step(Duration::from_millis(1));
    }
    assert_eq!(
        seen.lock().unwrap().clone(),
        oracle(head, fire_count(&runtime, "ping")),
        "a caught-up consumer keeps serving every committed frame in ARRIVAL \
         ORDER with nothing skipped and nothing served twice"
    );
    assert_eq!(
        fire_count(&runtime, "collapsed") - fires_before_drain,
        COLLAPSE_STEPS,
        "and the fire rate stays flat at ONE per step while it tracks — a \
         same-frame re-fire regression shows up here as the per-step cap, \
         every step"
    );
    assert_eq!(
        runtime
            .node_handle("collapsed")
            .expect("the consumer is a scheduler node")
            .backpressure_drop_oldest_count("inp"),
        0,
        "nothing was evicted — the backlog was HELD, then served"
    );
}

// ===========================================================================
// cdylib parity: the refill crosses the FFI through its OWN export.
// ===========================================================================

/// The production surface. A cdylib's body subscriber lives inside the cdylib
/// (its `NodeContext` is transferred across the FFI at `init`), so the refill
/// can only reach it through `NodeEntry::refill_trigger_input` — which for a
/// `DylibNodeEntry` is the `cerulion_node_refill_trigger_input` export.
///
/// That export is ADDITIVE (symbol presence is the capability; the ABI version
/// is NOT bumped), so this arm is also the no-inert-shipping proof that the
/// macro really emits it and the host really resolves it: a cdylib without it
/// reports `refills_trigger_input() == false`, installs no hook, and serves this
/// burst one frame per step — which is what the oracle below refuses.
///
/// The forwarder reads `trigger_in.x` and writes it to `cmd.x`; a closure sink
/// records what arrives, so the oracle is the full sequence THROUGH the cdylib.
#[test]
#[serial]
fn a_cdylib_consumer_serves_a_queued_burst_within_one_step() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_macro_data_trigger_cdylib");
    let fwd = DylibNodeEntry::load(&path).expect("load the macro data-trigger cdylib");
    assert!(
        fwd.unifies_trigger_drain(),
        "precondition: the fixture must be Unified, or this arm would be \
         exercising the Separate path instead"
    );

    let sink = ClosureNodeEntry::new(
        NodeInfo::from_names(vec!["in".to_string()], vec![]).with_policy(
            MacroPolicy::DataTrigger {
                input_name: "in".to_string(),
            },
        ),
        {
            let seen = Arc::clone(&seen);
            move |ctx| {
                if let Some(s) = ctx.subscriber_mut("in") {
                    if let Ok(Some(v)) = s.try_view::<Vector3, _>(|view| view.x as u64) {
                        seen.lock().unwrap().push(v);
                    }
                }
                Ok(())
            }
        },
    )
    .with_label("fbws_cdylib_sink");

    let config = GraphConfig {
        name: None,
        identity: "fbws_cdylib".to_string(),
        prefix: "fbwsCdylib".to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "fwd".to_string(),
                node_type: "data_trigger_node".to_string(),
                inputs: vec![InputDef {
                    name: "trigger_in".to_string(),
                    source: "/fbwsc/ext".to_string(),
                }],
                outputs: vec![OutputDef {
                    name: "cmd".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                ros2: None,
                id: "sink".to_string(),
                node_type: "fbws_cdylib_sink".to_string(),
                inputs: vec![InputDef {
                    name: "in".to_string(),
                    source: "fwd/cmd".to_string(),
                }],
                outputs: vec![],
            },
        ],
        multi_publisher_topics: Vec::new(),
        process_groups: Default::default(),
        process_group_order: Default::default(),
        level_assignments: None,
        network: None,
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fwd".to_string(), Box::new(fwd));
    factories.insert("sink".to_string(), Box::new(sink));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 16)
        .expect("build the cdylib burst graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher("/fbwsc/ext", MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");

    publish(&mut ext, 0);
    runtime.step(Duration::from_millis(1));
    runtime.step(Duration::from_millis(1));
    seen.lock().unwrap().clear();

    for v in 1..=BURST {
        publish(&mut ext, v);
    }
    // ONE step for the cdylib to serve the burst; the sink is a level below it,
    // so the frames it forwards reach the sink within the SAME step.
    runtime.step(Duration::from_millis(1));

    let observed = seen.lock().unwrap().clone();
    assert_eq!(
        observed,
        oracle(1, BURST),
        "the burst must cross the FFI drain export once per frame, in order, \
         within one step"
    );
}

// ===========================================================================
// (c) SERVE-MANY: the step-frozen NON-TRIGGER context is served to
//     EVERY fire of the burst.
//
// Before frozen-slot reuse, `try_view` consumed the frozen
// slot, so the first fire took the context and fires 2..k found it empty and
// FELL TO A LIVE DRAIN. On a QUIET context topic — the motivating shape,
// a data-triggered controller with a slow `/map` context — that drain found the
// queue empty and returned `Ok(None)`, so the tick chain collapsed at the
// macro's declaration-ordered `try_view` and the shape was capped at one message
// per step; that collapse is the residual the collapsed-tick arm above used to
// record. On a NON-quiet context topic the same drain really POPPED, leaking a
// mid-step read so the fires of ONE step disagreed about their context
// (Principle #7) — measured on the Period catch-up shape in
// `read_outcome_capture_iox2_test`'s arm (g), whose oracle this decision
// renegotiated.
//
// The arms below drive the shape from four directions; none subsumes another.
// ===========================================================================

/// A distinctive context value. Far outside the trigger indices these arms
/// produce (which run from 1), so a pair oracle cannot be satisfied by the two
/// halves being confused for one another.
const CTX_VALUE: u64 = 700;

/// The base a same-level context producer counts up from — likewise disjoint
/// from the trigger indices.
const CTX_BASE: u64 = 900;

/// The hand oracle for a burst read through a `(trigger, context)` recorder:
/// the contiguous trigger run `a..=b`, each fire carrying the IDENTICAL
/// context `ctx`.
fn pair_oracle(a: u64, b: u64, ctx: u64) -> Vec<(u64, u64)> {
    (a..=b).map(|n| (n, ctx)).collect()
}

/// Step until the pair-recording consumer has served the producer's NEWEST
/// committed frame, and return the producer's fire count at that instant.
/// Same discipline (and same reason) as [`warm_up_until_caught_up`].
fn warm_up_pairs_until_caught_up(
    runtime: &mut GraphRuntime,
    seen: &Arc<Mutex<Vec<(u64, u64)>>>,
) -> u64 {
    for _ in 0..WARMUP_STEP_BUDGET {
        runtime.step(Duration::from_millis(1));
        let fires = fire_count(runtime, "ping");
        if seen.lock().unwrap().last().map(|(t, _)| *t) == Some(fires) {
            return fires;
        }
    }
    panic!(
        "the pair consumer never caught up to the producer within \
         {WARMUP_STEP_BUDGET} warm-up steps (observed {:?})",
        seen.lock().unwrap()
    );
}

/// THE HEADLINE. A data-trigger node with a plain non-trigger `#[input]`
/// context, given `BURST` queued trigger frames and a DELIVERED context, must
/// process ALL of them in ONE step — and every fire must read the IDENTICAL
/// context bytes.
///
/// # Why the oracle is a PAIR
///
/// `CollapsedPong` forwards both reads (`out.x = inp.x`, `out.y = ctx_in.x`),
/// so the downstream recorder observes, per fire, which trigger frame was
/// consumed AND which context value was served. Asserting the trigger run alone
/// would pass a fix that served the burst while feeding fires 2..k a stale or
/// zeroed context; asserting the context alone would pass one that served one
/// frame k times. The pair pins both halves of the decision at once.
///
/// Under serve-ONCE (variant M2) this arm reads a SINGLE pair: fire 2 finds the
/// frozen context consumed and falls to the LIVE drain, which on this
/// deliberately QUIET context topic finds an empty queue (the live arm never
/// reaches the held sample), so `try_view` returns `Ok(None)` and the
/// macro's declaration-ordered chain collapses before it ever reaches `inp`.
#[test]
#[serial]
fn every_fire_of_a_burst_reads_the_same_frozen_context() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut runtime = build_ctx_chain("fbwsCtx", Arc::clone(&seen));
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ctx_pub = mgr
        .create_publisher(CTX_TOPIC, MaxSliceLen::const_new(64), 0)
        .expect("the absolute context source accepts a publisher");

    // Deliver the context ONCE. Every later step's snapshot drain is Empty and
    // replays the held value, so the frozen context is `CTX_VALUE` for
    // the rest of the run — which is what makes it a fixed oracle.
    publish(&mut ctx_pub, CTX_VALUE);

    let base = warm_up_pairs_until_caught_up(&mut runtime, &seen);
    let base_seen = seen.lock().unwrap().len();

    // ONE step advancing BURST periods ⇒ the Period producer catches up BURST
    // times and publishes BURST frames, all before the consumer's level runs.
    runtime.step(Duration::from_millis(BURST));

    let observed: Vec<(u64, u64)> = seen.lock().unwrap()[base_seen..].to_vec();
    assert_eq!(
        fire_count(&runtime, "ping") - base,
        BURST,
        "precondition: the Period producer must really have caught up BURST \
         times in the measured step"
    );
    assert_eq!(
        observed,
        pair_oracle(base + 1, base + BURST, CTX_VALUE),
        "every frame of the burst must be served in ONE step, in arrival \
         order, and EVERY fire must read the same frozen context ({CTX_VALUE}). \
         Under serve-once this reads a single pair — fire 2 found the context \
         slot consumed and its tick chain collapsed"
    );
    assert_eq!(
        runtime
            .node_handle("consumer")
            .expect("the consumer is a scheduler node")
            .backpressure_drop_oldest_count("inp"),
        0,
        "nothing was evicted — the burst was SERVED, not discarded"
    );
}

/// THE DISCRIMINATOR. A context frame published in the SAME step must be
/// invisible to every fire of the burst, fires 2..k included.
///
/// # Why this arm exists, and what it kills that the headline cannot
///
/// "Serve every fire" has a WRONG implementation — let fires 2..k fall through
/// to a LIVE drain (variant M1) — and this arm is the only one that can see it
/// on a NON-QUIET context topic, which is the shape that carries the real
/// hazard.
///
/// The headline arm's context topic is deliberately QUIET (it publishes
/// `CTX_VALUE` once, before the measured step), so under M1 the live drain finds
/// an empty queue. `try_view`'s live arm returns `Ok(None)` there — it does NOT
/// reach the held sample, which is served only through the FROZEN slot —
/// so the tick chain collapses and the headline arm fails too. M1 is therefore
/// killed by both arms, and this one is not the sole guard against it.
///
/// What this arm alone can see is the OTHER outcome of the same fall-through:
/// with fresh frames really queued, the live drain POPS, and fires 2..k serve a
/// SAME-STEP publish that fire 1 could not see. The fires of ONE step then
/// disagree about their context and record/replay can diverge (Principle #7) —
/// an M1 that leaks rather than collapses, which every quiet-context arm in the
/// file is structurally blind to.
///
/// # The construction
///
/// `ctxsrc` is triggered by the SAME producer as the consumer, so it levelizes
/// onto the consumer's level (a plain non-trigger edge does not levelize —
/// by design), and it is declared BEFORE the consumer so it ticks first within
/// that level. The level's ordering is what does the work: the snapshot phase
/// freezes the consumer's `ctx_in` BEFORE the tick phase in which `ctxsrc`
/// publishes. This is the `snapshot_wiring_iox2_test` prior-value shape, driven
/// under a BURST.
///
/// `ctxsrc` bursts too, so by the time the consumer's second fire runs there are
/// BURST fresh context frames sitting in its queue — the strongest possible bait
/// for a live re-read.
#[test]
#[serial]
fn a_same_step_context_publish_is_invisible_to_every_fire_of_the_burst() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut runtime = build_same_level_ctx_chain("fbwsSameLvl", Arc::clone(&seen));

    let base = warm_up_pairs_until_caught_up(&mut runtime, &seen);
    let base_seen = seen.lock().unwrap().len();

    // The frozen value the measured step MUST serve: `ctxsrc` publishes
    // `CTX_BASE + its own fire index`, and the newest frame queued at the
    // measured step's snapshot is the last one it published on the PRIOR step.
    let ctx_before = fire_count(&runtime, "ctxsrc");
    let frozen_ctx = CTX_BASE + ctx_before;

    runtime.step(Duration::from_millis(BURST));

    let observed: Vec<(u64, u64)> = seen.lock().unwrap()[base_seen..].to_vec();
    let ctx_after = fire_count(&runtime, "ctxsrc");
    let same_step_ctx = CTX_BASE + ctx_after;

    // Anti-vacuity: the bait was REAL — `ctxsrc` published fresh context frames
    // during the measured step, and they are DIFFERENT from the frozen value.
    // Without this, an arm asserting "the frozen value was served" would pass on
    // a run where nothing new was ever published.
    assert_eq!(
        ctx_after - ctx_before,
        BURST,
        "precondition: the same-level context source must really have burst in \
         the measured step, or there is no same-step publish to be blind to"
    );
    assert_ne!(
        same_step_ctx, frozen_ctx,
        "precondition: the same-step context value must DIFFER from the frozen \
         one, or the oracle below cannot discriminate"
    );

    assert_eq!(
        observed,
        pair_oracle(base + 1, base + BURST, frozen_ctx),
        "EVERY fire of the burst — not just the first — must read the context \
         frozen at the step boundary ({frozen_ctx}), never the {same_step_ctx} \
         its same-level producer published later in the SAME step. A serve-many \
         that re-reads LIVE serves the same-step value to fires 2..k and makes \
         the fires of one step disagree"
    );

    // And the frames really were reachable: the NEXT step's snapshot picks them
    // up, so the measured step's blindness was the freeze, not an empty queue.
    let before_next = seen.lock().unwrap().len();
    runtime.step(Duration::from_millis(1));
    let next: Vec<(u64, u64)> = seen.lock().unwrap()[before_next..].to_vec();
    assert!(
        !next.is_empty() && next.iter().all(|(_, c)| *c == same_step_ctx),
        "the context frames the measured step was blind to are picked up by the \
         NEXT step's capture — proving they were queued and readable all along, \
         so the blindness was the FREEZE and not an empty queue. observed \
         {next:?}, expected context {same_step_ctx}"
    );
}

/// Determinism (Principle #7): two runs of the burst-plus-context sequence are
/// byte-identical AND equal to the hand oracle.
///
/// Each run gets a fresh prefix (and `build_for_test` a fresh SHM root), and the
/// recorded sequence is NORMALISED against the producer's own warm-up fire count
/// so the comparison is run-independent — a warm-up frame dropped by iceoryx2
/// connection establishment shifts the absolute indices, never the shape.
///
/// The oracle equality is what keeps this from being a self-compare: two runs of
/// a deterministically-broken implementation also agree with each other.
#[test]
#[serial]
fn burst_context_reads_are_deterministic_and_match_the_oracle() {
    let a = run_ctx_burst("fbwsDet1");
    let b = run_ctx_burst("fbwsDet2");
    let expected = pair_oracle(1, BURST, CTX_VALUE);
    assert_eq!(
        a, b,
        "the burst's (trigger, context) sequence must be bit-identical across \
         runs. a={a:?} b={b:?}"
    );
    assert_eq!(
        a, expected,
        "and it must equal the hand oracle — two runs of a deterministically \
         broken implementation would agree with each other too"
    );
}

/// The NEVER-DELIVERED context, under a real burst.
///
/// Serve-many must not fabricate anything on an input that has never delivered:
/// the frozen slot is `Empty`, a re-served `Empty` is still `Ok(None)`, and the
/// pre-first-delivery WAIT still collapses the chain. So a step in which
/// the producer commits BURST frames must yield EXACTLY ONE fire (the boundary's
/// re-offer of a head no tick can take), not `BURST` of them — and the head must
/// be HELD, not lost.
///
/// The collapsed-tick arm above drives the same collapse one frame per step;
/// this one drives it under the burst the fix enables, which is the state where
/// a serve-many that fabricated a context would run away to the per-step cap.
#[test]
#[serial]
fn a_burst_on_an_undelivered_context_fires_once_and_holds_its_head() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut runtime = build_collapsed_chain("fbwsBurstColl", Arc::clone(&seen));
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ctx_pub = mgr
        .create_publisher("/fbwsCollapse/ctx", MaxSliceLen::const_new(64), 0)
        .expect("the absolute context source accepts a publisher");

    // The head the consumer freezes is PROVABLY ping's frame number
    // `ping_at_first_collapse`, and that derivation is what anchors the heal
    // oracle below on something OTHER than the healed output itself:
    //
    //   * `collapsed` is a data-trigger node, so its FIRST fire requires its
    //     FIRST yielding boundary drain — a re-offer needs an already-frozen
    //     head, so nothing else can mint that first arrival signal.
    //   * an `EachFifo` drain pops the OLDEST queued frame, so that first
    //     yield freezes the first frame the consumer was ever DELIVERED.
    //   * `ping` is `period_ms = 1` publishing its own fire index, and this
    //     warm-up steps 1 ms at a time, so it commits exactly one frame per
    //     step and its fire count on that step IS that frame's value.
    //
    // The derivation holds for ANY k: `ping_at_first_collapse == k` means
    // frames 1..k-1 were never delivered (or `collapsed` would have fired
    // earlier) and frame k was — so it is robust to iceoryx2 connection
    // warm-up dropping early frames, not an assumption that none are dropped.
    let mut ping_at_first_collapse = 0;
    for _ in 0..WARMUP_STEP_BUDGET {
        runtime.step(Duration::from_millis(1));
        if fire_count(&runtime, "collapsed") > 0 {
            ping_at_first_collapse = fire_count(&runtime, "ping");
            break;
        }
    }
    assert!(
        ping_at_first_collapse > 0,
        "precondition: the collapsed consumer must fire at least once before \
         the measured window (its trigger connection has to be live)"
    );

    // ONE step in which the producer catches up BURST times.
    let ping_before = fire_count(&runtime, "ping");
    let fires_before = fire_count(&runtime, "collapsed");
    runtime.step(Duration::from_millis(BURST));

    assert_eq!(
        fire_count(&runtime, "ping") - ping_before,
        BURST,
        "precondition: the producer really committed a burst for the collapsed \
         consumer to NOT serve"
    );
    assert_eq!(
        fire_count(&runtime, "collapsed") - fires_before,
        1,
        "a node whose context has NEVER delivered fires exactly ONCE even when \
         a whole burst is queued: the boundary re-offers the head it is still \
         holding and the refill reads that re-offer as 'nothing new'. A \
         serve-many that fabricated a context — or one whose re-served Empty \
         stopped collapsing the chain — runs the burst here instead"
    );
    assert!(
        seen.lock().unwrap().is_empty(),
        "anti-tautology: the collapse is REAL — the tick never ran to its \
         write, so nothing was published downstream"
    );

    // The head was HELD, not lost: it is the first thing served once the
    // context finally delivers, and the whole backlog follows it in order.
    //
    // The oracle is anchored on `ping_at_first_collapse` — the PRE-COLLAPSE
    // head, derived above — and NEVER on `healed.first()`: a self-anchored
    // oracle (`oracle(healed[0], ping_after)`) is satisfied by an
    // implementation that DROPS the held head and serves the next contiguous
    // frame, since that output is also a contiguous run ending at the newest
    // committed frame.
    publish(&mut ctx_pub, CTX_VALUE);
    runtime.step(Duration::from_millis(1));
    let healed = seen.lock().unwrap().clone();
    assert_eq!(
        healed.first().copied(),
        Some(ping_at_first_collapse),
        "the HELD head is the first thing served once the context delivers — \
         and it is the frame the collapse froze, not merely whatever the heal \
         happened to start with. observed {healed:?}"
    );
    assert_eq!(
        healed,
        oracle(ping_at_first_collapse, fire_count(&runtime, "ping")),
        "and nothing the collapse held was LOST: the backlog drains in arrival \
         order, contiguously, from the frozen head up to the newest committed \
         frame"
    );
}

/// cdylib PARITY — the production surface.
///
/// A cdylib's body subscriber lives inside the cdylib (its `NodeContext` is
/// transferred across the FFI at `init`), so the step-frozen context slot under
/// test is reached only through the `cerulion_node_{set_,}snapshot_inputs`
/// exports, and the burst only through the serve-many
/// `cerulion_node_refill_trigger_input` export. This arm is therefore the
/// no-inert-shipping proof that the decision reaches a real deployed node and not
/// only in-process closures.
///
/// # SCOPE — mutating the host copy CANNOT kill this arm
///
/// `test_node_macro_burst_ctx_cdylib` statically links its OWN `cerulion_core`,
/// so mutating `cerulion_core/src/transport/subscriber.rs` and running
/// `cargo test -p cerulion_core` leaves the fixture holding the OLD, unchanged
/// `try_view` — this arm stays green regardless of that change unless the
/// fixture is rebuilt too. Its value is PARITY (the FFI pair really forwards the
/// freeze, and the re-serve really survives the boundary), not mutation
/// sensitivity; the mutation-sensitive arms are the in-process ones above.
#[test]
#[serial]
fn a_cdylib_burst_reads_the_same_frozen_context_on_every_fire() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_macro_burst_ctx_cdylib");
    let node = DylibNodeEntry::load(&path).expect("load the burst-context cdylib");
    assert!(
        node.unifies_trigger_drain(),
        "precondition: the fixture must be Unified, or this arm would exercise \
         the Separate path instead"
    );
    assert!(
        node.holds_input_snapshot(),
        "precondition: the fixture must export the snapshot pair, or \
         there is no frozen context slot to re-serve across the FFI"
    );

    let config = GraphConfig {
        name: None,
        identity: "fifo_burst_ctx_cdylib".to_string(),
        prefix: "fbwsCtxDl".to_string(),
        nodes: vec![
            ping_node(),
            NodeDef {
                ros2: None,
                id: "consumer".to_string(),
                node_type: "burst_ctx_node".to_string(),
                inputs: vec![
                    InputDef {
                        name: "ctx_in".to_string(),
                        source: CTX_TOPIC.to_string(),
                    },
                    InputDef {
                        name: "inp".to_string(),
                        source: "ping/out".to_string(),
                    },
                ],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            record_node(),
        ],
        multi_publisher_topics: Vec::new(),
        process_groups: Default::default(),
        process_group_order: Default::default(),
        level_assignments: None,
        network: None,
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("ping".to_string(), Box::new(BurstPingEntry::new()));
    factories.insert("consumer".to_string(), Box::new(node));
    factories.insert(
        "record".to_string(),
        Box::new(recording_pair_consumer(Arc::clone(&seen))),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 16)
        .expect("build the cdylib burst-context chain");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ctx_pub = mgr
        .create_publisher(CTX_TOPIC, MaxSliceLen::const_new(64), 0)
        .expect("the absolute context source accepts a publisher");

    publish(&mut ctx_pub, CTX_VALUE);
    let base = warm_up_pairs_until_caught_up(&mut runtime, &seen);
    let base_seen = seen.lock().unwrap().len();

    runtime.step(Duration::from_millis(BURST));

    let observed: Vec<(u64, u64)> = seen.lock().unwrap()[base_seen..].to_vec();
    assert_eq!(
        fire_count(&runtime, "ping") - base,
        BURST,
        "precondition: the Period producer must really have caught up BURST times"
    );
    assert_eq!(
        observed,
        pair_oracle(base + 1, base + BURST, CTX_VALUE),
        "the burst must cross the FFI once per frame, in order, within ONE \
         step, with the frozen context re-served to every fire"
    );
}

// ===========================================================================
// Shared harness.
// ===========================================================================

/// The absolute context topic the serve-many arms publish to by hand, so the
/// test decides exactly when (and whether) the context delivers.
const CTX_TOPIC: &str = "/fbwsCtx/ctx";

/// A data-triggered CONTEXT producer, publishing `CTX_BASE + its own fire
/// index`.
///
/// Data-triggered on purpose: sharing the consumer's trigger levelizes it onto
/// the consumer's LEVEL, which is what puts its publish AFTER the consumer's
/// snapshot within one step. A `period_ms` source with no inputs would sit on
/// level 0 and publish BEFORE the consumer's level was even reached — legitimate
/// DAG flow, and no discriminator at all.
#[cerulion_node]
#[derive(Default)]
struct BurstCtxSource {
    #[input(trigger, depth = 32)]
    tick_in: Vector3,
    #[output]
    ctx: Vector3,
    n: u64,
}

#[cerulion_node_impl]
impl BurstCtxSource {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.ctx.x = (CTX_BASE + self.n) as f64;
        Ok(())
    }
}

/// A data-trigger recorder capturing the `(x, y)` PAIR — which trigger frame a
/// fire consumed and which context value it read — so one oracle covers both
/// halves of the serve-many decision.
///
/// Declared with meta (and a deep queue) for the same two reasons as
/// `build_collapsed_chain`'s recorder: a whole burst arrives here in ONE step,
/// so a default-depth queue could evict and break contiguity; and a
/// meta-carrying closure is Unified, so it bursts through the refill under test.
fn recording_pair_consumer(seen: Arc<Mutex<Vec<(u64, u64)>>>) -> ClosureNodeEntry {
    ClosureNodeEntry::new(
        NodeInfo::with_meta(
            vec![InputMeta {
                name: "in".to_string(),
                schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
                trigger: true,
                depth: 32,
                backpressure: BackpressurePolicy::DropOldest,
                expect_within_ms: None,
            }],
            vec![],
        )
        .with_policy(MacroPolicy::DataTrigger {
            input_name: "in".to_string(),
        }),
        move |ctx| {
            if let Some(s) = ctx.subscriber_mut("in") {
                if let Ok(Some(pair)) =
                    s.try_view::<Vector3, _>(|view| (view.x as u64, view.y as u64))
                {
                    seen.lock().unwrap().push(pair);
                }
            }
            Ok(())
        },
    )
    .with_label("fbws_pair_record")
}

/// The `period_ms = 1` trigger producer every chain in this file shares.
fn ping_node() -> NodeDef {
    NodeDef {
        ros2: None,
        id: "ping".to_string(),
        node_type: "burst_ping".to_string(),
        inputs: vec![],
        outputs: vec![OutputDef {
            name: "out".to_string(),
            schema: "Vector3".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: None,
        }],
    }
}

/// The pair recorder wired to `consumer/out`.
fn record_node() -> NodeDef {
    NodeDef {
        ros2: None,
        id: "record".to_string(),
        node_type: "fbws_pair_record".to_string(),
        inputs: vec![InputDef {
            name: "in".to_string(),
            source: "consumer/out".to_string(),
        }],
        outputs: vec![],
    }
}

/// `ping` (`period_ms = 1`) -> `consumer` (`CollapsedPong`: non-trigger
/// `ctx_in` on an absolute source declared FIRST, trigger `inp` from `ping`)
/// -> `record` (pairs).
fn build_ctx_chain(prefix: &str, seen: Arc<Mutex<Vec<(u64, u64)>>>) -> GraphRuntime {
    let config = GraphConfig {
        name: None,
        identity: "fifo_burst_frozen_context".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            ping_node(),
            NodeDef {
                ros2: None,
                id: "consumer".to_string(),
                node_type: "collapsed_pong".to_string(),
                inputs: vec![
                    InputDef {
                        name: "ctx_in".to_string(),
                        source: CTX_TOPIC.to_string(),
                    },
                    InputDef {
                        name: "inp".to_string(),
                        source: "ping/out".to_string(),
                    },
                ],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            record_node(),
        ],
        multi_publisher_topics: Vec::new(),
        process_groups: Default::default(),
        process_group_order: Default::default(),
        level_assignments: None,
        network: None,
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("ping".to_string(), Box::new(BurstPingEntry::new()));
    factories.insert("consumer".to_string(), Box::new(CollapsedPongEntry::new()));
    factories.insert(
        "record".to_string(),
        Box::new(recording_pair_consumer(seen)),
    );
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 16)
        .expect("build the frozen-context burst chain")
}

/// The prior-value discriminator's chain: `ctxsrc` and `consumer` share the
/// `ping` trigger, so they share a LEVEL, and `ctxsrc` is declared first so it
/// ticks first within it — publishing its context AFTER the consumer's snapshot
/// phase has already frozen the value the consumer must read.
fn build_same_level_ctx_chain(prefix: &str, seen: Arc<Mutex<Vec<(u64, u64)>>>) -> GraphRuntime {
    let config = GraphConfig {
        name: None,
        identity: "fifo_burst_same_level_context".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            ping_node(),
            NodeDef {
                ros2: None,
                id: "ctxsrc".to_string(),
                node_type: "burst_ctx_source".to_string(),
                inputs: vec![InputDef {
                    name: "tick_in".to_string(),
                    source: "ping/out".to_string(),
                }],
                outputs: vec![OutputDef {
                    name: "ctx".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                ros2: None,
                id: "consumer".to_string(),
                node_type: "collapsed_pong".to_string(),
                inputs: vec![
                    InputDef {
                        name: "ctx_in".to_string(),
                        source: "ctxsrc/ctx".to_string(),
                    },
                    InputDef {
                        name: "inp".to_string(),
                        source: "ping/out".to_string(),
                    },
                ],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            record_node(),
        ],
        multi_publisher_topics: Vec::new(),
        process_groups: Default::default(),
        process_group_order: Default::default(),
        level_assignments: None,
        network: None,
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("ping".to_string(), Box::new(BurstPingEntry::new()));
    factories.insert("ctxsrc".to_string(), Box::new(BurstCtxSourceEntry::new()));
    factories.insert("consumer".to_string(), Box::new(CollapsedPongEntry::new()));
    factories.insert(
        "record".to_string(),
        Box::new(recording_pair_consumer(seen)),
    );
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 16)
        .expect("build the same-level context burst chain")
}

/// One determinism run: warm up, burst once, and return the measured window
/// NORMALISED against the producer's warm-up fire count (so the shape, not the
/// absolute indices, is what is compared).
fn run_ctx_burst(prefix: &str) -> Vec<(u64, u64)> {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut runtime = build_ctx_chain(prefix, Arc::clone(&seen));
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ctx_pub = mgr
        .create_publisher(CTX_TOPIC, MaxSliceLen::const_new(64), 0)
        .expect("the absolute context source accepts a publisher");
    publish(&mut ctx_pub, CTX_VALUE);

    let base = warm_up_pairs_until_caught_up(&mut runtime, &seen);
    let base_seen = seen.lock().unwrap().len();
    runtime.step(Duration::from_millis(BURST));
    let normalised: Vec<(u64, u64)> = seen.lock().unwrap()[base_seen..]
        .iter()
        .map(|(t, c)| (t - base, *c))
        .collect();
    normalised
}

fn publish(p: &mut cerulion_core::transport::publisher::CerulionPublisher, v: u64) {
    let mut proxy = p.loan_proxy::<Vector3>().expect("loan");
    proxy.x = v as f64;
    drop(proxy);
}

fn fire_count(runtime: &GraphRuntime, node: &str) -> u64 {
    runtime
        .node_handle(node)
        .unwrap_or_else(|| panic!("'{node}' is a scheduler node"))
        .fire_count()
}

/// `ping` (`period_ms = 1`) -> `collapsed` (a macro node whose non-trigger
/// `ctx_in` is declared FIRST, so its tick collapses until that input delivers)
/// -> `record` (a closure recording what `collapsed` forwarded).
///
/// `ctx_in` reads an ABSOLUTE source with no in-graph producer, which is what
/// lets the test decide exactly when the collapse clears.
fn build_collapsed_chain(prefix: &str, seen: Arc<Mutex<Vec<u64>>>) -> GraphRuntime {
    // Declared with meta (and a deep queue) rather than `from_names`, for two
    // reasons: the whole backlog the collapse built arrives here in ONE step, so
    // a `DEFAULT_CONSUMER_DEPTH` queue could evict and break the contiguity
    // oracle; and a meta-carrying closure is Unified, so this node's own burst
    // rides the refill under test instead of the Separate boundary-drain path.
    let record = ClosureNodeEntry::new(
        NodeInfo::with_meta(
            vec![InputMeta {
                name: "in".to_string(),
                schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
                trigger: true,
                depth: 32,
                backpressure: BackpressurePolicy::DropOldest,
                expect_within_ms: None,
            }],
            vec![],
        )
        .with_policy(MacroPolicy::DataTrigger {
            input_name: "in".to_string(),
        }),
        move |ctx| {
            if let Some(s) = ctx.subscriber_mut("in") {
                if let Ok(Some(v)) = s.try_view::<Vector3, _>(|view| view.x as u64) {
                    seen.lock().unwrap().push(v);
                }
            }
            Ok(())
        },
    )
    .with_label("fbws_collapse_record");

    let config = GraphConfig {
        name: None,
        identity: "fifo_burst_collapsed_tick".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "ping".to_string(),
                node_type: "burst_ping".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                ros2: None,
                id: "collapsed".to_string(),
                node_type: "collapsed_pong".to_string(),
                inputs: vec![
                    InputDef {
                        name: "ctx_in".to_string(),
                        source: "/fbwsCollapse/ctx".to_string(),
                    },
                    InputDef {
                        name: "inp".to_string(),
                        source: "ping/out".to_string(),
                    },
                ],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                ros2: None,
                id: "record".to_string(),
                node_type: "fbws_collapse_record".to_string(),
                inputs: vec![InputDef {
                    name: "in".to_string(),
                    source: "collapsed/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
        multi_publisher_topics: Vec::new(),
        process_groups: Default::default(),
        process_group_order: Default::default(),
        level_assignments: None,
        network: None,
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("ping".to_string(), Box::new(BurstPingEntry::new()));
    factories.insert("collapsed".to_string(), Box::new(CollapsedPongEntry::new()));
    factories.insert("record".to_string(), Box::new(record));
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 16)
        .expect("build the collapsed-tick chain")
}

/// `ping` (`period_ms = 1`, in-graph) -> `sink` (data-trigger recorder).
fn build_in_graph_chain(prefix: &str, seen: Arc<Mutex<Vec<u64>>>) -> GraphRuntime {
    let config = GraphConfig {
        name: None,
        identity: "fifo_burst_within_step".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "ping".to_string(),
                node_type: "burst_ping".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                ros2: None,
                id: "sink".to_string(),
                node_type: "burst_pong".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "ping/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
        multi_publisher_topics: Vec::new(),
        process_groups: Default::default(),
        process_group_order: Default::default(),
        level_assignments: None,
        network: None,
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("ping".to_string(), Box::new(BurstPingEntry::new()));
    factories.insert("sink".to_string(), Box::new(recording_consumer(seen)));
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 16).expect("build in-graph burst chain")
}

/// A lone data-trigger recorder on the absolute external source `/fbws/ext`.
fn external_source_graph(
    prefix: &str,
    seen: Arc<Mutex<Vec<u64>>>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        name: None,
        identity: "fifo_burst_within_step_live".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "sink".to_string(),
            node_type: "burst_pong".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: "/fbws/ext".to_string(),
            }],
            outputs: vec![],
        }],
        multi_publisher_topics: Vec::new(),
        process_groups: Default::default(),
        process_group_order: Default::default(),
        level_assignments: None,
        network: None,
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("sink".to_string(), Box::new(recording_consumer(seen)));
    (config, factories)
}
