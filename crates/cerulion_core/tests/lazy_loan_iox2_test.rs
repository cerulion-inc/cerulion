// SPDX-License-Identifier: AGPL-3.0-only
//! Macro-node LAZY-LOAN — the `#[cerulion_node_impl]` tick loans an
//! `#[output]` port's proxy on the FIRST write instead of pre-loaning every
//! output at tick start. Over real iceoryx2 (`GraphRuntime::build_for_test`,
//! per-test SHM root).
//!
//! # The bug this pins
//!
//! A generated tick preamble that pre-LOANS an `OutputProxy` for
//! EVERY declared output and arms publishing on every fully-Ok tick breaks a SPARSE
//! writer — a node whose tick leaves some outputs untouched on a given tick
//! (a DDS bridge whose empty-queue tick writes nothing; a multi-output node
//! writing a different subset each tick) — because it Drop-DISCARDS the
//! untouched proxies every tick. For a VARIABLE-schema output the loud
//! "dropped without writing all declared variable fields" `error!` fires, and
//! because the flood latch re-arms on every COMPLETE publish,
//! alternating write/empty ticks make every empty-tick discard a fresh regime
//! head = one loud `error!` per empty tick (measured on a robot: 3118 errors in ~4 min).
//! A FIXED-schema untouched output silently publishes a zero-default frame
//! every tick (fabricated data).
//!
//! So the macro's write shims route through `LazyOutput::__cer_loan`, which
//! loans on the FIRST write. An output the tick never writes never loans →
//! nothing to discard, nothing to publish → a zero-traffic non-event.
//!
//! # What each test pins (hand oracles, never a self-compare)
//!
//! | Test | Pin |
//! |------|-----|
//! | `sparse_writer_skips_untouched_output_no_publish_no_discard` | A 2-output node writing only ONE output every tick: the written output delivers every fire, the untouched FIXED output delivers ZERO (with a pre-loan: a zero-default frame every fire) and fires ZERO discard errors. |
//! | `alternating_write_empty_ticks_never_flood_discard_errors` | The flood shape — a variable-schema output written completely on odd ticks and NOT AT ALL on even ticks fires ZERO discard errors across the run (with a pre-loan: one loud `error!` per empty tick). |
//! | `partial_write_still_discards_loudly` | Anti-tautology control: a loaned-but-INCOMPLETE output (a real write of some, not all, variable fields) keeps the loud discard `error!` + once-per-regime latch EXACTLY. Lazy-loan must not weaken the loud partial-write safety net. |
//! | `sparse_delivery_is_deterministic` | Two runs of the sparse scenario (warmup-then-N-step delta) are byte-identical AND equal the hand oracle. |
//! | `zero_field_emit_publishes_every_tick_unemitted_is_silent` | A fieldless `std_msgs/Empty` output IS emittable via the explicit `self.<port>.emit()?` gesture — it delivers a real frame (wire `schema_hash == Empty::SCHEMA_HASH`) every tick; a second Empty output NOT emitted delivers ZERO frames and fires no discard error (the lazy-loan non-event rule holds for fieldless outputs). |
//! | `read_back_after_write_sees_just_written_value` | Read-back-after-write within a tick — a node writes `out.x = count` then `out.y = self.out.x * 2.0`; the delivered frame carries (x=count, y=2*count), proving the second assignment read the just-written `out.x` through the same lazy get-or-loan receiver. The file COMPILING is the E0499 regression guard (without the RHS hoist the 2-segment read-back emits two overlapping `&mut` loans). |
//!
//! # Wire timestamp
//!
//! Because the loan happens at first-write time, the wire `timestamp_ns`
//! (stamped by `loan_proxy` at loan) is a first-write stamp, not a tick-start one. In
//! a DETERMINISTIC run this is UNOBSERVABLE — the gating clock does not advance
//! mid-step, so first-write-time == tick-start-time (both read the same
//! `now_ns()`); this is exactly why first-write stamping is replay-safe. The timestamp
//! STAMPING mechanism itself (loan-time `clock.now_ns()`) is
//! pinned exhaustively by `non_trigger_hold_iox2_test.rs`'s wire-timestamp arms; here
//! we structurally confirm a written output's frame still carries a nonzero,
//! monotonically-increasing stamp across steps (a regression guard that the
//! lazy-loan wiring does not break stamping), while an UNWRITTEN output carries
//! no frame at all (the loan — and thus the stamp — only happens on write).
//!
//! # Running (iceoryx2 SHM singleton → serial)
//!
//! ```bash
//! cargo test -p cerulion_core --test lazy_loan_iox2_test -- --test-threads=1
//! ```

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::testing::{count_at_exclusively, debug_lines_expected, line_level};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use native_ros2_messages::sensor_msgs::Image;
use native_ros2_messages::std_msgs::Empty;
use native_ros2_messages::std_msgs::String as RosString;
use serial_test::serial;
use tracing_test::traced_test;

/// The step delta; also every node's period so a Period node fires per step.
const STEP: Duration = Duration::from_millis(10);

/// The loud discard `error!` substring OutputProxy::Drop emits when a
/// loaned-but-incomplete variable output is dropped (also used by
/// `output_proxy_test.rs`).
const DISCARD_ERROR: &str = "dropped without writing all declared variable fields";

/// Monotonic prefix counter so re-builds within one process never collide on
/// an iceoryx2 service name (mirrors `collapse_no_publish_test.rs`).
static PREFIX_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_prefix(stem: &str) -> String {
    format!("{stem}{}", PREFIX_COUNTER.fetch_add(1, Ordering::Relaxed))
}

fn vec3_out(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "Vector3".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
    }
}

fn str_out(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "std_msgs/String".to_string(),
        max_slice_len: Some(256),
        history_size: 0,
        topic: None,
    }
}

fn image_out(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "sensor_msgs/Image".to_string(),
        max_slice_len: Some(4096),
        history_size: 0,
        topic: None,
    }
}

fn empty_out(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "std_msgs/Empty".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
    }
}

// ===========================================================================
// Node types
// ===========================================================================

/// A 2-output Period source that writes ONLY `written` every tick and NEVER
/// touches `skipped`. `written` carries an increasing scalar (a value oracle);
/// `skipped` (a FIXED Vector3) is the untouched port whose zero-default
/// publish lazy-loan eliminates.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct SparseTwoOut {
    #[output]
    written: Vector3,
    #[output]
    skipped: Vector3,
    count: u64,
}

#[cerulion_node_impl]
impl SparseTwoOut {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.count += 1;
        // Write ONLY `written`. `skipped` is intentionally never touched → it
        // must never loan → never publish, never discard.
        self.written.x = self.count as f64;
        Ok(())
    }
}

/// A single variable-schema (`std_msgs/String`) output that is written
/// COMPLETELY on odd ticks and NOT AT ALL on even ticks — the flood shape
/// (alternating complete-publish / empty). With a pre-loan each even tick's
/// proxy Drop-discards loudly, and the odd tick's complete publish
/// re-arms the latch → one loud `error!` per empty tick.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct AlternatingVarWriter {
    #[output]
    msg: RosString,
    count: u64,
}

#[cerulion_node_impl]
impl AlternatingVarWriter {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.count += 1;
        if self.count % 2 == 1 {
            // Odd tick — write the ONLY variable field completely → publishes.
            self.msg.data = "probe";
        }
        // Even tick — write NOTHING → no loan → no discard, no flood.
        Ok(())
    }
}

/// A variable-schema (`sensor_msgs/Image`) output that is LOANED every tick (it
/// writes `data`) but left INCOMPLETE (`encoding` + `header` unwritten). This
/// is the loaned-but-incomplete PARTIAL-write case — the loud discard
/// path must stay EXACTLY as loud as with a pre-loan (the anti-tautology control that
/// lazy-loan does not silence real partial writes). Mirrors
/// `test_node_discard_probe_cdylib`.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct PartialVarWriter {
    #[output]
    image: Image,
    count: u64,
}

#[cerulion_node_impl]
impl PartialVarWriter {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.count += 1;
        // Touch a fixed field so the port LOANS, then write ONLY `data`,
        // leaving `encoding` + `header` unwritten → the loaned proxy Drop
        // fires the loud all-variables-gate discard error.
        self.image.height = self.count as u32;
        let payload = [1u8, 2, 3];
        self.image.data = &payload[..];
        Ok(())
    }
}

/// A DataTrigger sink over a FIXED Vector3 topic: `delivered` counts every fire
/// (an unambiguous "a publish reached here" oracle — trigger-gated delivery),
/// `last_value` records the value read, and `first_ts`/`last_ts` capture the
/// frame's wire timestamp (the stamping regression guard).
#[cerulion_node]
#[derive(Default)]
struct Vec3Sink {
    #[input(trigger)]
    inp: Vector3,
    delivered: Arc<AtomicU64>,
    last_value: Arc<AtomicU64>,
    first_ts: Arc<AtomicU64>,
    last_ts: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl Vec3Sink {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.delivered.fetch_add(1, Ordering::Relaxed);
        self.last_value.store(self.inp.x as u64, Ordering::Relaxed);
        let ts = self.inp.wire_timestamp_ns();
        // Set `first_ts` exactly once (0 is the sentinel — no real frame is
        // stamped 0 in these runs since the clock has advanced past 0).
        let _ = self
            .first_ts
            .compare_exchange(0, ts, Ordering::Relaxed, Ordering::Relaxed);
        self.last_ts.store(ts, Ordering::Relaxed);
        Ok(())
    }
}

/// A DataTrigger sink over a variable `std_msgs/String` topic: `delivered`
/// counts every fire (proves a COMPLETE publish reached here on odd ticks).
#[cerulion_node]
#[derive(Default)]
struct StrSink {
    #[input(trigger)]
    inp: RosString,
    delivered: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl StrSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.delivered.fetch_add(1, Ordering::Relaxed);
        // Touch the payload so a broken read surfaces (utf-8 decode).
        let _ = self.inp.data();
        Ok(())
    }
}

/// A 2-output Period source over ZERO-FIELD `std_msgs/Empty`
/// heartbeats. `pulse` is EMITTED every tick via the explicit `emit()` gesture
/// (a fieldless output has no field-write to trigger the lazy loan, so `emit()`
/// is its ONLY publish path); `quiet` is DECLARED but never emitted — it must
/// never loan, never publish (the lazy-loan non-event rule holds for fieldless
/// outputs too). Pins that a fieldless output IS emittable (the gap the
/// `emit()` gesture closes).
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct EmptyHeartbeat {
    #[output]
    pulse: Empty,
    #[output]
    quiet: Empty,
}

#[cerulion_node_impl]
impl EmptyHeartbeat {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Emit ONLY `pulse`. `quiet` is never touched → never loans → never
        // publishes, exactly like an unwritten fielded output.
        self.pulse.emit()?;
        Ok(())
    }
}

/// A DataTrigger sink over a ZERO-FIELD `std_msgs/Empty` topic: `delivered`
/// counts every fire (proves an Empty frame reached here), `schema_hash`
/// records the delivered frame's wire-header schema hash (must equal
/// `Empty::SCHEMA_HASH` — the wire-shape pin for a fieldless publish).
#[cerulion_node]
#[derive(Default)]
struct EmptySink {
    #[input(trigger)]
    inp: Empty,
    delivered: Arc<AtomicU64>,
    schema_hash: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl EmptySink {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.delivered.fetch_add(1, Ordering::Relaxed);
        self.schema_hash
            .store(self.inp.wire_header().schema_hash, Ordering::Relaxed);
        Ok(())
    }
}

/// READ-BACK-AFTER-WRITE within a tick. Writes
/// `out.x = count`, then `out.y = self.out.x * 2.0` — the SECOND assignment
/// READS the just-written `out.x` off the SAME lazy-loaned proxy (an output-field
/// read routes through the same get-or-loan receiver). Without the RHS hoist this 2-segment
/// read-back-after-write does not COMPILE — the macro would emit two overlapping
/// `&mut` loans of `__cer_out` in one call expression → E0499; the RHS
/// hoist prevents it. The delivered frame must carry (x=count, y=2*count), proving
/// the read saw the just-written value (not a stale/zero slot).
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct ReadBackWriter {
    #[output]
    out: Vector3,
    count: u64,
}

#[cerulion_node_impl]
impl ReadBackWriter {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.count += 1;
        self.out.x = self.count as f64;
        // READ-BACK: `self.out.x` was just written on THIS tick's lazy-loaned
        // proxy; reading it here must see `count`, so `y == 2 * count`. This line
        // is the E0499 compile-regression guard (the file failing to compile ==
        // the regression) AND the behavioral read-back assertion below.
        self.out.y = self.out.x * 2.0;
        Ok(())
    }
}

/// A DataTrigger sink over a FIXED Vector3 topic recording BOTH `x` and `y` of
/// the last delivered frame — the read-back oracle for `ReadBackWriter`.
#[cerulion_node]
#[derive(Default)]
struct Vec3XYSink {
    #[input(trigger)]
    inp: Vector3,
    delivered: Arc<AtomicU64>,
    last_x: Arc<AtomicU64>,
    last_y: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl Vec3XYSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.delivered.fetch_add(1, Ordering::Relaxed);
        self.last_x.store(self.inp.x as u64, Ordering::Relaxed);
        self.last_y.store(self.inp.y as u64, Ordering::Relaxed);
        Ok(())
    }
}

// ===========================================================================
// Shared handles bundle for the sparse graph
// ===========================================================================

struct SparseHandles {
    w_delivered: Arc<AtomicU64>,
    w_last_value: Arc<AtomicU64>,
    w_first_ts: Arc<AtomicU64>,
    w_last_ts: Arc<AtomicU64>,
    s_delivered: Arc<AtomicU64>,
}

/// `dut` (Period, writes only `written`) → `sink_w` (on `dut/written`) +
/// `sink_s` (on `dut/skipped`). `skipped` has NO consumer-relevant data —
/// `sink_s` counts whether any frame is ever published on it.
fn build_sparse_graph(prefix: &str) -> (GraphRuntime, SparseHandles) {
    let w_delivered = Arc::new(AtomicU64::new(0));
    let w_last_value = Arc::new(AtomicU64::new(u64::MAX));
    let w_first_ts = Arc::new(AtomicU64::new(0));
    let w_last_ts = Arc::new(AtomicU64::new(0));
    let s_delivered = Arc::new(AtomicU64::new(0));

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "lazy_loan_sparse".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "dut".to_string(),
                node_type: "sparse_two_out".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("written"), vec3_out("skipped")],
            },
            NodeDef {
                ros2: None,
                id: "sink_w".to_string(),
                node_type: "vec3_sink".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "dut/written".to_string(),
                }],
                outputs: vec![],
            },
            NodeDef {
                ros2: None,
                id: "sink_s".to_string(),
                node_type: "vec3_sink".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "dut/skipped".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("dut".to_string(), Box::new(SparseTwoOutEntry::new()));
    factories.insert(
        "sink_w".to_string(),
        Box::new(Vec3SinkEntry::with_state(Vec3Sink {
            delivered: Arc::clone(&w_delivered),
            last_value: Arc::clone(&w_last_value),
            first_ts: Arc::clone(&w_first_ts),
            last_ts: Arc::clone(&w_last_ts),
            ..Default::default()
        })),
    );
    factories.insert(
        "sink_s".to_string(),
        Box::new(Vec3SinkEntry::with_state(Vec3Sink {
            delivered: Arc::clone(&s_delivered),
            ..Default::default()
        })),
    );

    let clock = Arc::new(VirtualClock::new());
    let runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build sparse graph");
    (
        runtime,
        SparseHandles {
            w_delivered,
            w_last_value,
            w_first_ts,
            w_last_ts,
            s_delivered,
        },
    )
}

// ===========================================================================
// PIN 1 (HEADLINE): a sparse writer's untouched FIXED output never publishes
// and never discards; the written output delivers every fire.
// ===========================================================================

#[test]
#[serial]
#[traced_test]
fn sparse_writer_skips_untouched_output_no_publish_no_discard() {
    const N: u32 = 20;
    let (mut rt, h) = build_sparse_graph(&unique_prefix("lazy_sparse"));

    // Establish: step until the WRITTEN output first delivers (bounded —
    // tolerates iceoryx2 connection warmup, mirrors collapse_no_publish's
    // establish loops). The DUT is Period(10ms) firing every step, so it writes
    // `written` every step; `total_steps` counts DUT ticks (== its `count`).
    let mut total_steps: u64 = 0;
    let mut tries = 0;
    loop {
        rt.step(STEP);
        total_steps += 1;
        if h.w_delivered.load(Ordering::Relaxed) >= 1 {
            break;
        }
        tries += 1;
        assert!(
            tries < 200,
            "written output never delivered within 200 steps"
        );
    }
    let w_established = h.w_delivered.load(Ordering::Relaxed);

    // Step N more; the written output must deliver exactly N more (1:1 once the
    // connection is warm — the proven pattern from
    // `genuine_success_still_publishes_every_tick`).
    for _ in 0..N {
        rt.step(STEP);
        total_steps += 1;
    }
    assert_eq!(
        h.w_delivered.load(Ordering::Relaxed) - w_established,
        u64::from(N),
        "the written output must publish on every one of {N} post-warmup Period \
         ticks (1:1 delivery)"
    );
    // Value oracle: the last delivered value is the DUT's `count` at the last
    // step, which (Period fires every step) equals `total_steps` — a real write
    // flowed, not a fabricated zero frame.
    assert_eq!(
        h.w_last_value.load(Ordering::Relaxed),
        total_steps,
        "the last delivered `written.x` must equal the DUT's tick count \
         ({total_steps}), proving real writes flow (a fabricated frame would \
         carry x=0)"
    );

    // The UNTOUCHED FIXED output publishes ZERO frames throughout.
    // Without lazy loan this fixed-schema output would publish a zero-default
    // frame on EVERY fire, so the sink would record a delivery per step
    // (hand oracle: 0).
    assert_eq!(
        h.s_delivered.load(Ordering::Relaxed),
        0,
        "the untouched FIXED output must NEVER loan → NEVER publish; \
         the downstream sink must record 0 deliveries across all {total_steps} \
         ticks (without lazy loan: one zero-default frame per fire)"
    );

    // No discard error may fire — neither the written output (it completes)
    // nor the untouched output (it never loans, so nothing to discard).
    logs_assert(|lines: &[&str]| {
        let errors = lines.iter().filter(|l| l.contains(DISCARD_ERROR)).count();
        if errors != 0 {
            return Err(format!(
                "a sparse writer must fire ZERO discard errors — the \
                 untouched output never loans and the written one completes; \
                 got {errors}"
            ));
        }
        Ok(())
    });

    // Timestamp stamping regression guard: the written output's
    // frames carry a nonzero, monotonically-increasing wire timestamp across
    // steps — the loan (and thus the loan-time `clock.now_ns()` stamp) tracks
    // the clock at first-write. The exact first-write-vs-tick-start value is
    // UNOBSERVABLE deterministically (the clock is frozen mid-step); see the
    // module docs. Full stamping semantics live in non_trigger_hold's wire-timestamp arms.
    let first_ts = h.w_first_ts.load(Ordering::Relaxed);
    let last_ts = h.w_last_ts.load(Ordering::Relaxed);
    assert!(
        first_ts != 0,
        "the first written frame must carry a nonzero loan-time wire timestamp"
    );
    assert!(
        last_ts > first_ts,
        "the wire timestamp must advance across steps (loan-time stamp tracks \
         the clock at first-write); first={first_ts} last={last_ts}"
    );
}

// ===========================================================================
// PIN 2 (REGRESSION — the discard flood): alternating complete/empty ticks
// on a variable-schema output fire ZERO discard errors.
// ===========================================================================

#[test]
#[serial]
#[traced_test]
fn alternating_write_empty_ticks_never_flood_discard_errors() {
    const PAIRS: u32 = 15; // 15 odd (write) + 15 even (empty) = 30 ticks
    let delivered = Arc::new(AtomicU64::new(0));

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "lazy_loan_alternating".to_string(),
        prefix: unique_prefix("lazy_alt"),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "dut".to_string(),
                node_type: "alternating_var_writer".to_string(),
                inputs: vec![],
                outputs: vec![str_out("msg")],
            },
            NodeDef {
                ros2: None,
                id: "sink".to_string(),
                node_type: "str_sink".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "dut/msg".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "dut".to_string(),
        Box::new(AlternatingVarWriterEntry::new()),
    );
    factories.insert(
        "sink".to_string(),
        Box::new(StrSinkEntry::with_state(StrSink {
            delivered: Arc::clone(&delivered),
            ..Default::default()
        })),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut rt =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build alternating graph");

    for _ in 0..(PAIRS * 2) {
        rt.step(STEP);
    }

    // THE regression pin: not one discard error across the whole run.
    // With a pre-loan the even (empty) ticks each fire a fresh-regime loud
    // `error!` (the odd tick's complete publish re-arms the latch),
    // so this would be `PAIRS` (15) loud errors.
    logs_assert(|lines: &[&str]| {
        let errors = lines.iter().filter(|l| l.contains(DISCARD_ERROR)).count();
        if errors != 0 {
            return Err(format!(
                "alternating write/empty ticks must fire ZERO discard \
                 errors (an empty tick never loans, so there is nothing to \
                 discard) — got {errors} (without lazy loan: {PAIRS}, the flood)"
            ));
        }
        Ok(())
    });

    // Anti-tautology / non-vacuous: the odd ticks DID publish completely (the
    // "0 errors" pin must not be trivially satisfied by a writer that never
    // publishes). `>= 1` is warmup-robust (iceoryx2 connection warmup may drop
    // the earliest frames); the exact-count publish path is pinned elsewhere.
    // A near-full count also confirms most odd ticks landed.
    let got = delivered.load(Ordering::Relaxed);
    assert!(
        got >= 1,
        "the odd (write) ticks must publish complete frames — the sink must \
         record at least one delivery (got {got}); otherwise the zero-errors \
         pin is vacuous"
    );
    assert!(
        got <= u64::from(PAIRS),
        "the sink cannot receive more than the {PAIRS} complete (odd-tick) \
         publishes — got {got}, which would mean an even (empty) tick published"
    );
}

// ===========================================================================
// PIN 3 (ANTI-TAUTOLOGY CONTROL): a loaned-but-INCOMPLETE output still
// discards loudly (the loud path + the once-per-regime latch).
// ===========================================================================

#[test]
#[serial]
#[traced_test]
fn partial_write_still_discards_loudly() {
    const N: u32 = 6;

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "lazy_loan_partial".to_string(),
        prefix: unique_prefix("lazy_partial"),
        nodes: vec![NodeDef {
            ros2: None,
            id: "dut".to_string(),
            node_type: "partial_var_writer".to_string(),
            inputs: vec![],
            outputs: vec![image_out("image")],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("dut".to_string(), Box::new(PartialVarWriterEntry::new()));
    let clock = Arc::new(VirtualClock::new());
    let mut rt =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build partial graph");

    for _ in 0..N {
        rt.step(STEP);
    }

    // RELEASE-OBSERVABLE FIRST: the per-output discard
    // COUNTER is unconditional — it bumps on every discard whatever the log
    // level regime downgraded — so the N discards are pinned in BOTH profiles.
    // The DEBUG half of the log oracle below reads 0 where `debug!` is compiled
    // out and can carry nothing there; this is what does.
    assert_eq!(
        rt.node_handle("dut")
            .expect("dut handle")
            .output_discard_count("image"),
        u64::from(N),
        "every incomplete tick must be COUNTED as a discard, log level regime aside"
    );

    // The output IS loaned every tick (it writes `data`) but left incomplete
    // (`encoding` + `header` unwritten) → the loud discard path must
    // fire. Per the flood latch this regime NEVER re-arms (no complete
    // publish ever), so EXACTLY ONE loud `error!` (regime head) + N-1
    // debug-downgraded "suppressed" lines — proving lazy-loan kept the loud
    // safety net intact, not that it silenced it.
    logs_assert(|lines: &[&str]| {
        // Level-free twin: a suppressed discard repeat must never be LOUD — the half of the
        // contract that survives `release_max_level_info`, where the gated
        // DEBUG count reads 0.
        for level in ["WARN", "INFO", "ERROR"] {
            let loud = lines
                .iter()
                .filter(|l| {
                    line_level(l) == Some(level) && (l.contains("OutputProxy discard suppressed"))
                })
                .count();
            if loud != 0 {
                return Err(format!(
                    "a suppressed discard repeat was emitted at {level} ({loud} line(s))"
                ));
            }
        }
        // The loud head, matched WITH its level token AND against the level-free
        // total of the same marker: a head demoted to `warn!`/`info!` is not the
        // loud safety net this pins, and neither is a second copy of it at
        // another level.
        let errors = count_at_exclusively(lines, "ERROR", &[DISCARD_ERROR])?;
        let suppressed = count_at_exclusively(lines, "DEBUG", &["OutputProxy discard suppressed"])?;
        if errors != 1 {
            return Err(format!(
                "a loaned-but-incomplete output must fire EXACTLY ONE loud \
                 discard error (the regime head; the regime never re-arms \
                 because no complete publish follows) — got {errors}. Lazy-loan \
                 must not weaken the loud partial-write path."
            ));
        }
        let want_suppressed = debug_lines_expected(usize::try_from(N - 1).unwrap());
        if suppressed != want_suppressed {
            return Err(format!(
                "expected {want_suppressed} debug-downgraded 'discard suppressed' lines (the \
                 2nd..Nth discards of the single regime), got {suppressed}"
            ));
        }
        Ok(())
    });
}

// ===========================================================================
// PIN 4 (DETERMINISM): the sparse scenario's delivery counts are byte-identical
// across two runs AND equal the hand oracle.
// ===========================================================================

#[test]
#[serial]
fn sparse_delivery_is_deterministic() {
    const N: u32 = 20;

    // Warm up until the written output first delivers, snapshot that baseline,
    // THEN measure the next N steps' delta — the SAME warmup-then-count shape as
    // PIN 1 and the sibling family (`collapse_no_publish_test`, PIN 1 above). An
    // EXACT cold-start count would flake: iceoryx2 connection warmup can drop the
    // earliest frames, so the first few steps are not guaranteed 1:1.
    let run = || {
        let (mut rt, h) = build_sparse_graph(&unique_prefix("lazy_sparse_det"));
        let mut tries = 0;
        loop {
            rt.step(STEP);
            if h.w_delivered.load(Ordering::Relaxed) >= 1 {
                break;
            }
            tries += 1;
            assert!(
                tries < 200,
                "written output never delivered within 200 steps"
            );
        }
        let established = h.w_delivered.load(Ordering::Relaxed);
        for _ in 0..N {
            rt.step(STEP);
        }
        (
            h.w_delivered.load(Ordering::Relaxed) - established,
            h.s_delivered.load(Ordering::Relaxed),
        )
    };

    let a = run();
    let b = run();

    assert_eq!(
        a, b,
        "two runs of the sparse scenario must be byte-identical"
    );
    assert_eq!(
        a,
        (u64::from(N), 0),
        "AND both must equal the hand oracle (anti-tautology): the written \
         output delivers N={N} post-warmup (1:1), the untouched output delivers \
         0 throughout. got {a:?}"
    );
}

// ===========================================================================
// PIN 5: a fieldless `std_msgs/Empty` output IS emittable via
// the explicit `emit()` gesture — it delivers a real frame every tick; a second
// Empty output NOT emitted is a zero-traffic non-event (no publish, no discard).
// ===========================================================================

#[test]
#[serial]
#[traced_test]
fn zero_field_emit_publishes_every_tick_unemitted_is_silent() {
    const N: u32 = 20;
    let p_delivered = Arc::new(AtomicU64::new(0));
    let p_hash = Arc::new(AtomicU64::new(0));
    let q_delivered = Arc::new(AtomicU64::new(0));
    let q_hash = Arc::new(AtomicU64::new(0));

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "lazy_loan_empty_emit".to_string(),
        prefix: unique_prefix("lazy_empty"),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "dut".to_string(),
                node_type: "empty_heartbeat".to_string(),
                inputs: vec![],
                outputs: vec![empty_out("pulse"), empty_out("quiet")],
            },
            NodeDef {
                ros2: None,
                id: "sink_p".to_string(),
                node_type: "empty_sink".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "dut/pulse".to_string(),
                }],
                outputs: vec![],
            },
            NodeDef {
                ros2: None,
                id: "sink_q".to_string(),
                node_type: "empty_sink".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "dut/quiet".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("dut".to_string(), Box::new(EmptyHeartbeatEntry::new()));
    factories.insert(
        "sink_p".to_string(),
        Box::new(EmptySinkEntry::with_state(EmptySink {
            delivered: Arc::clone(&p_delivered),
            schema_hash: Arc::clone(&p_hash),
            ..Default::default()
        })),
    );
    factories.insert(
        "sink_q".to_string(),
        Box::new(EmptySinkEntry::with_state(EmptySink {
            delivered: Arc::clone(&q_delivered),
            schema_hash: Arc::clone(&q_hash),
            ..Default::default()
        })),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut rt =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build empty-emit graph");

    // Warm up until the emitted `pulse` first delivers (bounded — tolerates
    // iceoryx2 connection warmup, mirrors PIN 1), then step N more and require
    // EXACTLY N more deliveries (post-warmup 1:1 — a fieldless output IS
    // emittable, and delivers a real frame every tick).
    let mut tries = 0;
    loop {
        rt.step(STEP);
        if p_delivered.load(Ordering::Relaxed) >= 1 {
            break;
        }
        tries += 1;
        assert!(
            tries < 200,
            "the emitted Empty output never delivered within 200 steps"
        );
    }
    let established = p_delivered.load(Ordering::Relaxed);
    for _ in 0..N {
        rt.step(STEP);
    }
    assert_eq!(
        p_delivered.load(Ordering::Relaxed) - established,
        u64::from(N),
        "an `emit()`ed fieldless output must publish on every one of {N} \
         post-warmup Period ticks (1:1 delivery) — a fieldless output IS emittable"
    );

    // Wire-shape pin: the delivered Empty frame carries Empty's schema hash (a
    // real, correctly-typed frame flowed — not a phantom). Hand oracle:
    // `Empty::SCHEMA_HASH` (never a self-compare).
    assert_eq!(
        p_hash.load(Ordering::Relaxed),
        Empty::SCHEMA_HASH,
        "the delivered frame's wire schema_hash must equal Empty::SCHEMA_HASH"
    );

    // CONTROL: `quiet` is declared but never `emit()`ed → it must never loan,
    // never publish. Its sink records ZERO deliveries (hand oracle: 0) and never
    // sees a frame (its recorded hash stays the 0 sentinel).
    assert_eq!(
        q_delivered.load(Ordering::Relaxed),
        0,
        "an un-emitted fieldless output must NEVER loan → NEVER publish; \
         its sink must record 0 deliveries (a zero-traffic non-event)"
    );
    assert_eq!(
        q_hash.load(Ordering::Relaxed),
        0,
        "the un-emitted output's sink must never see a frame (hash stays 0)"
    );

    // No discard error may fire — `pulse` is vacuously complete (fieldless →
    // VARIABLE_FIELD_COUNT 0, so the all-variables gate cannot trip) and `quiet`
    // never loans, so there is nothing to discard.
    logs_assert(|lines: &[&str]| {
        let errors = lines.iter().filter(|l| l.contains(DISCARD_ERROR)).count();
        if errors != 0 {
            return Err(format!(
                "a fieldless emit()/non-emit pair must fire ZERO discard \
                 errors — the emitted proxy is vacuously complete and the un-emitted \
                 one never loans; got {errors}"
            ));
        }
        Ok(())
    });
}

// ===========================================================================
// PIN 6: read-back-after-write within a tick —
// `self.out.y = self.out.x * 2.0` sees the just-written `out.x` through the
// get-or-loan receiver. The file COMPILING is itself the E0499 regression guard
// (without the RHS hoist the 2-segment read-back emits two overlapping `&mut` loans of
// `__cer_out`); the delivered (x, y=2x) frame is the behavioral half.
// ===========================================================================

#[test]
#[serial]
fn read_back_after_write_sees_just_written_value() {
    const N: u32 = 20;
    let delivered = Arc::new(AtomicU64::new(0));
    let last_x = Arc::new(AtomicU64::new(u64::MAX));
    let last_y = Arc::new(AtomicU64::new(u64::MAX));

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "lazy_loan_readback".to_string(),
        prefix: unique_prefix("lazy_readback"),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "dut".to_string(),
                node_type: "read_back_writer".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                ros2: None,
                id: "sink".to_string(),
                node_type: "vec3_xy_sink".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "dut/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("dut".to_string(), Box::new(ReadBackWriterEntry::new()));
    factories.insert(
        "sink".to_string(),
        Box::new(Vec3XYSinkEntry::with_state(Vec3XYSink {
            delivered: Arc::clone(&delivered),
            last_x: Arc::clone(&last_x),
            last_y: Arc::clone(&last_y),
            ..Default::default()
        })),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut rt =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build readback graph");

    // Warm up until the first delivery, then step N more (1:1 post-warmup,
    // mirrors PIN 1). The DUT is Period(10ms) firing every step, so `count`
    // equals the total DUT ticks == `total_steps`.
    let mut total_steps: u64 = 0;
    let mut tries = 0;
    loop {
        rt.step(STEP);
        total_steps += 1;
        if delivered.load(Ordering::Relaxed) >= 1 {
            break;
        }
        tries += 1;
        assert!(
            tries < 200,
            "read-back output never delivered within 200 steps"
        );
    }
    for _ in 0..N {
        rt.step(STEP);
        total_steps += 1;
    }

    // Hand oracle: the last delivered frame's `x` equals the DUT tick count
    // (`total_steps`) — the DUT (level 0) publishes and the data-trigger sink
    // (level 1) reads it in the SAME step, so the last value tracks the last
    // tick (the exact pattern PIN 1 proves).
    let x = last_x.load(Ordering::Relaxed);
    let y = last_y.load(Ordering::Relaxed);
    assert!(
        delivered.load(Ordering::Relaxed) >= 1,
        "a real frame delivered"
    );
    assert_eq!(
        x, total_steps,
        "the delivered out.x must equal the DUT tick count ({total_steps}) — a \
         real write flowed (not a fabricated zero frame)"
    );
    // THE read-back proof: `out.y == 2 * out.x`. The second assignment READ the
    // `out.x` written earlier THIS tick, off the same lazy-loaned proxy (the
    // get-or-loan receiver). A stale/zero read would make y == 0 ≠ 2*x.
    assert_eq!(
        y,
        2 * total_steps,
        "the delivered out.y must equal 2*out.x ({}) — the read-back saw the \
         just-written out.x through the get-or-loan receiver",
        2 * total_steps
    );
}
