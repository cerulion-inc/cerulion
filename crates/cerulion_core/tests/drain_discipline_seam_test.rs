// SPDX-License-Identifier: AGPL-3.0-only
//! The drain-discipline measurement seam regression pin.
//!
//! The unified drain moves an eligible macro data-trigger consumer's trigger drain onto
//! its BODY subscriber (`DrainSource::Unified` — one iceoryx2 receive per hop),
//! eliminating the legacy dual-subscriber double-read (`DrainSource::Separate`).
//! The hidden env knob `CERULION_DRAIN_DISCIPLINE=separate` forces every
//! otherwise-Unified binding back to Separate on ONE graph, so the
//! Separate-vs-Unified per-hop delta (the ~1.3µs/receive figure)
//! can be A/B-measured. The seam is PERMANENT (a benchmarking /
//! regression lever) and HIDDEN (deliberately absent from USER_API.md — it
//! mirrors the `CERULION_MW_SINGLE_PARK` hidden-measurement-knob precedent).
//!
//! What this file pins (all over `GraphRuntime::build_for_test`, per-test SHM
//! root, real iceoryx2 — no fake data, Principle #13):
//!
//! - **(a) default (env unset):** the eligible consumer's binding is Unified
//!   (`unified_binding_count_for_test() >= 1`), and it delivers the contiguous
//!   producer sequence (a HAND ORACLE `warmup+1 ..= warmup+measured`, NOT a
//!   self-compare).
//! - **(b) `=separate`:** the SAME graph builds with `unified_binding_count ==
//!   0`, still fires, and delivers a sequence equal to the SAME hand oracle AND
//!   byte-identical to leg (a). The unified-drain invariant: the drain discipline
//!   changes HOW a trigger is drained, never WHAT is delivered.
//! - **(c) `=bogus` / `=Separate` / `=" separate"`:** a loud `tracing::warn!`
//!   naming the knob fires and the build STAYS Unified (a typo never silently
//!   degrades to Separate). The match is EXACT — capitalization and whitespace
//!   are not forgiven — and the warn fires exactly ONCE per build (the env is
//!   read once per build and threaded to both sites, not re-read per eligible
//!   binding per pass, which would fire it 2× per build).
//! - **(d) effectiveness breadcrumb:** the loud one-per-build `tracing::info!`
//!   fires ONLY when the knob actually flipped ≥1 binding (stale-knob-class
//!   prevention) — absent on a default build, present (with `forced=1`) on a
//!   forced-Separate build.
//! - **(e) empty string:** `CERULION_DRAIN_DISCIPLINE=""` is the deliberate
//!   SILENT no-op arm (`Ok("") | Err(_)`) — Unified stays, no warn, no
//!   breadcrumb.
//! - **(f) count-pass/build-pass desync (the key pin):** a
//!   `FANOUT_CONSUMERS`-wide fan-out on ONE producer topic makes a count-site
//!   desync FAIL the build. Forced-Separate wires 2K subscribers (K body + K
//!   trigger drain); a count pass that wrongly thinks Unified provisions only
//!   K + `INTROSPECTION_SUBSCRIBER_HEADROOM` slots, and
//!   2K > K + HEADROOM ⇔ K > HEADROOM, so K = HEADROOM + 1 pushes the
//!   under-provisioning past the headroom — iceoryx2 rejects the last
//!   subscriber at open and the build errors. K is DERIVED from the constant
//!   (if the headroom rises, a hard-coded K keeps passing while
//!   silently no longer catching the regression). Under the CORRECT code (ONE env
//!   read threaded to both sites) the forced build provisions 2K + HEADROOM,
//!   builds, delivers the hand oracle to ALL K consumers, and breadcrumbs the
//!   accumulated `forced=K` exactly once. (The single-consumer graphs above sit
//!   inside the headroom — 2 wired vs 1 + HEADROOM provisioned — and can NEVER
//!   catch this desync class.)
//! - **(g) CLOSURE unification:** a `ClosureNodeEntry`
//!   data-trigger consumer with DropOldest `input_meta` now builds Unified —
//!   `NodeEntry::unifies_trigger_drain` decoupled drain eligibility from the
//!   rayon flag (`performs_input_snapshot`) — and delivers the hand oracle via
//!   `try_view` (the frozen-slot-served read path); the same graph under
//!   `=separate` is byte-identical (the seam invariant extended to
//!   closures); and `.with_unified_drain(false)` opts a closure back to
//!   Separate (the escape hatch for accumulate-all `try_receive` ticks, which
//!   the frozen slot does NOT serve — see the trait method's READ-PATH
//!   CONTRACT).
//!
//! NOT unified (a deliberate decision; full analysis at
//! `NodeInfo::unifies_data_trigger`): `Sample(N)`, whose fire-rate
//! concern was REFUTED by the drain code (`DrainOutcome.popped` counts RAW
//! pre-decimation pops, so fire-per-arrival would hold), but unification
//! would skip the per-arrival `signal_input_received` watchdog reset on
//! decimated drains (`latest_ts == None` vs the Separate arm's ungated
//! per-frame reset), silently changing `expect_within_ms` semantics for
//! sample-gated trigger inputs. `Block` — the pre-fire mirror/pacing
//! analysis. Both stay Separate.
//!
//! `#[serial]` because `CERULION_DRAIN_DISCIPLINE` is process-global; an RAII
//! `EnvVarGuard` removes it on drop (panic-safe). `build_for_test` reads the env
//! FRESH at build time, so the guard must be live across the build call.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test drain_discipline_seam_test -- --test-threads=1
//! ```

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::INTROSPECTION_SUBSCRIBER_HEADROOM;
use cerulion_core::MacroPolicy;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;
use tracing_test::traced_test;

/// The knob under test.
const KNOB: &str = "CERULION_DRAIN_DISCIPLINE";

/// Steps discarded before the measured window (let the data-trigger chain reach
/// steady state).
const WARMUP: u32 = 3;

/// Measured steps: one producer publish + one consumer fire each.
const MEASURED: u32 = 8;

/// A sentinel the producer never publishes (it publishes 1, 2, 3, ...). If a
/// measured-step read records this, the consumer's tick did NOT run — the
/// oracle assert then fails loud instead of the test silently degrading.
const MISSING: u64 = u64::MAX;

/// RAII guard that sets `CERULION_DRAIN_DISCIPLINE` and removes it on drop —
/// even if an assertion panics mid-test. Mirrors `fire_threads_env_test`'s
/// `FireThreadsGuard` / `chunk_c_ffi_codes_3_4_test`'s `EnvVarGuard`.
struct EnvVarGuard;
impl EnvVarGuard {
    fn set(value: &str) -> Self {
        std::env::set_var(KNOB, value);
        Self
    }
}
impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        std::env::remove_var(KNOB);
    }
}

/// The hand oracle for the measured window: the producer ticks every step
/// (Period), so on the i-th measured step (global step `WARMUP + i`) it
/// publishes `WARMUP + i`; the data-trigger consumer reads that LIVE value.
fn oracle() -> Vec<u64> {
    ((WARMUP as u64 + 1)..=(WARMUP as u64 + MEASURED as u64)).collect()
}

// ===========================================================================
// Nodes: a Period(10) producer publishing an incrementing counter into a fixed
// Vector3 field, and a data-trigger consumer that records the LIVE trigger read
// each tick. A macro node (`performs_input_snapshot == true`) with a default
// (`DropOldest`) trigger input is the eligibility shape `unifies_data_trigger`
// accepts — so its binding is Unified by default.
// ===========================================================================

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct DrainProducer {
    #[output]
    out: Vector3,
    n: u64,
}

#[cerulion_node_impl]
impl DrainProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Increment FIRST so the first publish carries 1, not 0 (keeps reads
        // strictly positive and distinct from the MISSING sentinel).
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

#[cerulion_node]
#[derive(Default)]
struct DrainConsumer {
    #[input(trigger)]
    inp: Vector3,
    /// Records the LIVE trigger read on each fire (shared with the harness).
    last_read: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl DrainConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last_read.store(self.inp.x as u64, Ordering::Relaxed);
        Ok(())
    }
}

/// Build the `producer -> data-trigger consumer` graph over an isolated test
/// transport, record `unified_binding_count_for_test()` at build (before any
/// step), then drive `WARMUP + MEASURED` steps and return
/// `(unified_binding_count, delivered_sequence)`. Reads the env FRESH at build,
/// so the caller sets/clears the knob before calling.
fn run_chain(prefix: &str) -> (usize, Vec<u64>) {
    let last_read = Arc::new(AtomicU64::new(MISSING));
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "drain_discipline_seam".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "producer".to_string(),
                node_type: "drain_producer".to_string(),
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
                id: "consumer".to_string(),
                node_type: "drain_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(DrainProducerEntry::new()));
    let consumer = DrainConsumer {
        last_read: Arc::clone(&last_read),
        ..Default::default()
    };
    factories.insert(
        "consumer".to_string(),
        Box::new(DrainConsumerEntry::with_state(consumer)),
    );

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build drain-discipline seam graph");

    let unified = runtime.unified_binding_count_for_test();

    for _ in 0..WARMUP {
        runtime.step(Duration::from_millis(10));
    }
    let mut seq = Vec::with_capacity(MEASURED as usize);
    for _ in 0..MEASURED {
        // Reset so a non-running consumer tick leaves MISSING (detectable).
        last_read.store(MISSING, Ordering::Relaxed);
        runtime.step(Duration::from_millis(10));
        seq.push(last_read.load(Ordering::Relaxed));
    }
    (unified, seq)
}

/// Fan-out width for the count-pass/build-pass desync pin: the SMALLEST K
/// where a count-site desync escapes the introspection headroom. Forced-Separate
/// wires 2K subscribers on the producer topic (K body + K trigger-drain) while a
/// desynced count pass provisions K + `INTROSPECTION_SUBSCRIBER_HEADROOM`, and
/// 2K > K + HEADROOM ⇔ K > HEADROOM.
///
/// DERIVED from the constant, never hard-coded: the headroom was once raised
/// 4 → 5, and a literal `5` here would have gone on passing while quietly no
/// longer catching the regression it exists to catch.
const FANOUT_CONSUMERS: usize = INTROSPECTION_SUBSCRIBER_HEADROOM + 1;

/// [`run_chain`]'s K-consumer fan-out twin: ONE producer topic, K macro
/// data-trigger consumers (all otherwise-Unified-eligible). Returns
/// `(unified_binding_count, per-consumer delivered sequences)`.
fn run_fanout(prefix: &str) -> (usize, Vec<Vec<u64>>) {
    let readers: Vec<Arc<AtomicU64>> = (0..FANOUT_CONSUMERS)
        .map(|_| Arc::new(AtomicU64::new(MISSING)))
        .collect();

    let mut nodes = vec![NodeDef {
        ros2: None,
        id: "producer".to_string(),
        node_type: "drain_producer".to_string(),
        inputs: vec![],
        outputs: vec![OutputDef {
            name: "out".to_string(),
            schema: "Vector3".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: None,
        }],
    }];
    for i in 0..FANOUT_CONSUMERS {
        nodes.push(NodeDef {
            ros2: None,
            id: format!("consumer{i}"),
            node_type: "drain_consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: "producer/out".to_string(),
            }],
            outputs: vec![],
        });
    }
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "drain_discipline_fanout".to_string(),
        prefix: prefix.to_string(),
        nodes,
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(DrainProducerEntry::new()));
    for (i, reader) in readers.iter().enumerate() {
        let consumer = DrainConsumer {
            last_read: Arc::clone(reader),
            ..Default::default()
        };
        factories.insert(
            format!("consumer{i}"),
            Box::new(DrainConsumerEntry::with_state(consumer)),
        );
    }

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build drain-discipline fan-out graph");

    let unified = runtime.unified_binding_count_for_test();

    for _ in 0..WARMUP {
        runtime.step(Duration::from_millis(10));
    }
    let mut seqs: Vec<Vec<u64>> = (0..FANOUT_CONSUMERS)
        .map(|_| Vec::with_capacity(MEASURED as usize))
        .collect();
    for _ in 0..MEASURED {
        for reader in &readers {
            reader.store(MISSING, Ordering::Relaxed);
        }
        runtime.step(Duration::from_millis(10));
        for (i, reader) in readers.iter().enumerate() {
            seqs[i].push(reader.load(Ordering::Relaxed));
        }
    }
    (unified, seqs)
}

// ===========================================================================
// (a) default (env unset): the eligible consumer's binding is Unified.
// ===========================================================================
#[test]
#[serial]
fn default_unset_uses_unified_and_delivers() {
    // Belt-and-suspenders: `#[serial]` orders tests but does not guarantee a
    // sibling's guard already dropped, so clear the knob explicitly.
    std::env::remove_var(KNOB);
    let (unified, seq) = run_chain("dd_default");
    assert!(
        unified >= 1,
        "the default build must UNIFY the eligible macro data-trigger consumer; \
         got unified_binding_count = {unified}"
    );
    assert_eq!(
        seq,
        oracle(),
        "Unified delivery must be the contiguous producer sequence (hand oracle)"
    );
}

// ===========================================================================
// (b) =separate: 0 Unified, still fires, byte-identical to the default leg.
// ===========================================================================
#[test]
#[serial]
fn separate_forces_no_unified_and_is_byte_identical_to_unified() {
    let expected = oracle();

    // Leg A — default (Unified).
    std::env::remove_var(KNOB);
    let (unified_a, seq_a) = run_chain("dd_ab_uni");
    assert!(
        unified_a >= 1,
        "leg A (default) must be Unified; got {unified_a}"
    );
    assert_eq!(seq_a, expected, "leg A delivers the hand oracle");

    // Leg B — forced Separate.
    let _guard = EnvVarGuard::set("separate");
    let (unified_b, seq_b) = run_chain("dd_ab_sep");
    assert_eq!(
        unified_b, 0,
        "leg B (=separate) must force ALL bindings to Separate (0 Unified)"
    );
    assert_eq!(seq_b, expected, "leg B delivers the SAME hand oracle");

    // The unified-drain invariant: the drain discipline changes HOW a trigger is
    // drained, never WHAT flows — Separate must be byte-identical to Unified.
    assert_eq!(
        seq_a, seq_b,
        "Separate delivery must be byte-identical to Unified"
    );
}

// ===========================================================================
// (c) unrecognized values: loud warn, build stays Unified (never silently to
// Separate). Three legs — "bogus", "Separate" (capitalization not forgiven),
// " separate" (whitespace not trimmed) — pin the EXACT-match contract, then a
// logs_assert pins the once-per-BUILD warn discipline: 3 builds ⇒ exactly 3
// warns (re-reading the env per eligible binding per pass would give 2/build).
// ===========================================================================
#[test]
#[serial]
#[traced_test]
fn unrecognized_values_warn_once_per_build_and_stay_unified() {
    // Leg 1 — "bogus" (the arbitrary-typo arm).
    {
        let _guard = EnvVarGuard::set("bogus");
        let (unified, seq) = run_chain("dd_bogus");
        assert!(
            unified >= 1,
            "a bogus CERULION_DRAIN_DISCIPLINE value must default to UNIFIED \
             (never silently to Separate); got {unified}"
        );
        assert_eq!(
            seq,
            oracle(),
            "the bogus-value fallback still delivers correctly"
        );
    }
    assert!(
        logs_contain("CERULION_DRAIN_DISCIPLINE set to an unrecognized value"),
        "a bogus value must emit the loud warn naming the knob"
    );
    assert!(
        logs_contain("value=bogus"),
        "the warn must name the offending value 'bogus'"
    );

    // Leg 2 — "Separate": the match is EXACT, capitalization is not forgiven.
    {
        let _guard = EnvVarGuard::set("Separate");
        let (unified, seq) = run_chain("dd_cap");
        assert!(
            unified >= 1,
            "'Separate' (capitalized) must hit the warn arm and stay UNIFIED; \
             got {unified}"
        );
        assert_eq!(seq, oracle(), "the capitalized-value leg still delivers");
    }
    assert!(
        logs_contain("value=Separate"),
        "the warn must name the offending capitalized value 'Separate'"
    );

    // Leg 3 — " separate": whitespace is not trimmed.
    {
        let _guard = EnvVarGuard::set(" separate");
        let (unified, seq) = run_chain("dd_ws");
        assert!(
            unified >= 1,
            "' separate' (leading whitespace) must hit the warn arm and stay \
             UNIFIED; got {unified}"
        );
        assert_eq!(seq, oracle(), "the whitespace-value leg still delivers");
    }
    assert!(
        logs_contain("value= separate"),
        "the warn must name the offending whitespace value ' separate'"
    );

    // Once-per-BUILD discipline: 3 builds above ⇒ exactly 3 warn lines. A
    // per-call read would fire the warn at BOTH the count pass and the
    // build pass (2 per build with 1 eligible binding ⇒ 6 here).
    logs_assert(|lines: &[&str]| {
        let warns = lines
            .iter()
            .filter(|l| l.contains("CERULION_DRAIN_DISCIPLINE set to an unrecognized value"))
            .count();
        if warns == 3 {
            Ok(())
        } else {
            Err(format!(
                "expected exactly 3 warns (one per build across 3 builds), got {warns}"
            ))
        }
    });
}

// ===========================================================================
// (e) empty string: the DELIBERATE silent no-op arm (`Ok("") | Err(_)`) —
// Unified stays, no warn, no breadcrumb.
// ===========================================================================
#[test]
#[serial]
#[traced_test]
fn empty_value_is_silent_and_stays_unified() {
    let _guard = EnvVarGuard::set("");
    let (unified, seq) = run_chain("dd_empty");
    assert!(
        unified >= 1,
        "an EMPTY CERULION_DRAIN_DISCIPLINE must stay UNIFIED; got {unified}"
    );
    assert_eq!(seq, oracle(), "the empty-value build still delivers");
    assert!(
        !logs_contain("CERULION_DRAIN_DISCIPLINE set to an unrecognized value"),
        "empty string is the deliberate SILENT arm — no unrecognized-value warn"
    );
    assert!(
        !logs_contain("drain discipline: forcing Separate"),
        "empty string flips nothing — no effectiveness breadcrumb"
    );
}

// ===========================================================================
// (d) effectiveness breadcrumb: one-per-build info!, ONLY on a real flip.
// ===========================================================================
#[test]
#[serial]
#[traced_test]
fn effectiveness_breadcrumb_fires_only_on_a_real_flip() {
    // Default build: nothing flips → NO breadcrumb (checked BEFORE the forced
    // leg pollutes the shared per-test log buffer).
    std::env::remove_var(KNOB);
    let (unified, _seq) = run_chain("dd_bc_default");
    assert!(unified >= 1, "default leg must be Unified; got {unified}");
    assert!(
        !logs_contain("drain discipline: forcing Separate"),
        "no effectiveness breadcrumb when the knob is unset (nothing flipped)"
    );

    // Forced Separate: exactly one binding flips → exactly one breadcrumb.
    let _guard = EnvVarGuard::set("separate");
    let (unified2, _seq2) = run_chain("dd_bc_sep");
    assert_eq!(
        unified2, 0,
        "forced leg must report 0 Unified; got {unified2}"
    );
    assert!(
        logs_contain("drain discipline: forcing Separate on otherwise-Unified bindings"),
        "a real flip must emit the loud effectiveness breadcrumb"
    );
    assert!(
        logs_contain("forced=1"),
        "the breadcrumb must name how many bindings flipped (1 here)"
    );
}

// ===========================================================================
// (f) The count-pass/build-pass desync pin: a
// 5-consumer fan-out makes a count-site desync fail the BUILD.
//
// Forced-Separate wires 2K subscribers on the producer topic (K body + K
// trigger-drain). A count pass that wrongly thinks Unified (the mutation:
// dropping the count-site `!force_separate_discipline` wrap) provisions only
// K + INTROSPECTION_SUBSCRIBER_HEADROOM slots < the 2K wired — iceoryx2 rejects
// the last subscriber at open and `build_for_test` errors. The single-consumer
// graphs above wire 2 vs 1 + HEADROOM provisioned and can NEVER catch this (the
// desync hides inside the headroom). Under the CORRECT code (ONE once-per-build
// env read threaded to both sites) the forced build provisions 2K + HEADROOM,
// builds, and delivers to ALL K consumers.
//
// Also the (2) accumulation pin: K flipped bindings ⇒ ONE breadcrumb carrying
// the accumulated `forced=K`.
// ===========================================================================
#[test]
#[serial]
#[traced_test]
fn forced_separate_fanout_provisions_correctly_and_breadcrumbs_forced_k() {
    let expected = oracle();

    // Leg 1 — default (env unset): all K bindings are otherwise-Unified (the
    // anti-tautology control — the forced leg below genuinely flips K, not 0)
    // and no breadcrumb exists yet in the shared log buffer.
    std::env::remove_var(KNOB);
    let (unified_default, seqs_default) = run_fanout("dd_fan_uni");
    assert_eq!(
        unified_default, FANOUT_CONSUMERS,
        "all {FANOUT_CONSUMERS} fan-out consumers must be Unified by default"
    );
    for (i, seq) in seqs_default.iter().enumerate() {
        assert_eq!(
            seq, &expected,
            "consumer{i} (default leg) delivers the hand oracle"
        );
    }
    assert!(
        !logs_contain("drain discipline: forcing Separate"),
        "the default fan-out leg must not breadcrumb (nothing flipped)"
    );

    // Leg 2 — forced Separate: the desync pin. This build MUST succeed (the
    // count pass provisioned the 5 extra trigger-drain subscribers) and every
    // consumer MUST deliver.
    let _guard = EnvVarGuard::set("separate");
    let (unified_forced, seqs_forced) = run_fanout("dd_fan_sep");
    assert_eq!(
        unified_forced, 0,
        "the forced fan-out leg must wire ALL {FANOUT_CONSUMERS} bindings Separate"
    );
    for (i, seq) in seqs_forced.iter().enumerate() {
        assert_eq!(
            seq, &expected,
            "consumer{i} (forced-Separate leg) delivers the hand oracle"
        );
    }
    assert!(
        logs_contain(&format!("forced={FANOUT_CONSUMERS}")),
        "the breadcrumb must carry the ACCUMULATED flip count (forced={FANOUT_CONSUMERS})"
    );
    // Exactly ONE breadcrumb across both legs (leg 1 emits none; leg 2 emits
    // one per build, not one per flipped binding).
    logs_assert(|lines: &[&str]| {
        let crumbs = lines
            .iter()
            .filter(|l| l.contains("drain discipline: forcing Separate on otherwise-Unified"))
            .count();
        if crumbs == 1 {
            Ok(())
        } else {
            Err(format!(
                "expected exactly 1 effectiveness breadcrumb (once per build, \
                 not per binding), got {crumbs}"
            ))
        }
    });
}

// ===========================================================================
// (g) CLOSURE unification. A `ClosureNodeEntry` data-trigger
// consumer with DropOldest `input_meta` builds Unified (the new
// `NodeEntry::unifies_trigger_drain` capability, decoupled from the rayon
// flag) and delivers via `try_view` — the ONLY read path the unified drain's
// frozen slot serves. `run_chain`'s closure twin.
// ===========================================================================

/// Closure twin of [`run_chain`]: a Period(10) closure producer publishing an
/// incrementing counter into `Vector3.x` via `loan_proxy`, and a data-trigger
/// closure consumer (DropOldest `input_meta`) recording its `try_view` read.
/// `unified_capability` is threaded to `.with_unified_drain(..)` on the
/// consumer — `false` pins the accumulate-all escape hatch.
fn run_closure_chain(prefix: &str, unified_capability: bool) -> (usize, Vec<u64>) {
    let last_read = Arc::new(AtomicU64::new(MISSING));
    let produced = std::sync::atomic::AtomicU64::new(0);

    let producer = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec!["out".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 10 }),
        move |ctx| {
            let n = produced.fetch_add(1, Ordering::Relaxed) + 1;
            if let Some(pub_port) = ctx.publisher_mut("out") {
                let mut proxy = pub_port.loan_proxy::<Vector3>()?;
                proxy.x = n as f64;
                proxy.y = 0.0;
                proxy.z = 0.0;
            }
            Ok(())
        },
    )
    .with_label("dd_closure_producer");

    let consumer_info = NodeInfo::with_meta(
        vec![InputMeta {
            name: "inp".to_string(),
            schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
            trigger: true,
            depth: cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
            backpressure: BackpressurePolicy::DropOldest,
            expect_within_ms: None,
        }],
        vec![],
    )
    .with_policy(MacroPolicy::DataTrigger {
        input_name: "inp".to_string(),
    });
    let last_read_c = Arc::clone(&last_read);
    let consumer = ClosureNodeEntry::new(consumer_info, move |ctx| {
        // `try_view` (&mut) — the frozen-slot-served read path. A `try_receive`
        // here would see an EMPTY queue under Unified (the drain consumed it).
        let v = ctx
            .subscriber_mut("inp")
            .and_then(|s| {
                s.try_view::<Vector3, _>(|view| view.x as u64)
                    .ok()
                    .flatten()
            })
            .unwrap_or(MISSING);
        last_read_c.store(v, Ordering::Relaxed);
        Ok(())
    })
    .with_label("dd_closure_consumer")
    .with_unified_drain(unified_capability);

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "drain_discipline_closure".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "producer".to_string(),
                node_type: "dd_closure_producer".to_string(),
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
                id: "consumer".to_string(),
                node_type: "dd_closure_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(producer));
    factories.insert("consumer".to_string(), Box::new(consumer));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build drain-discipline closure graph");

    let unified = runtime.unified_binding_count_for_test();

    for _ in 0..WARMUP {
        runtime.step(Duration::from_millis(10));
    }
    let mut seq = Vec::with_capacity(MEASURED as usize);
    for _ in 0..MEASURED {
        last_read.store(MISSING, Ordering::Relaxed);
        runtime.step(Duration::from_millis(10));
        seq.push(last_read.load(Ordering::Relaxed));
    }
    (unified, seq)
}

#[test]
#[serial]
fn closure_consumer_builds_unified_and_delivers() {
    std::env::remove_var(KNOB);
    let (unified, seq) = run_closure_chain("ddc_default", true);
    assert!(
        unified >= 1,
        "a ClosureNodeEntry data-trigger consumer with DropOldest input_meta \
         must build UNIFIED; got unified_binding_count = {unified}"
    );
    assert_eq!(
        seq,
        oracle(),
        "the unified closure must deliver the contiguous producer sequence \
         (hand oracle) via the frozen-slot-served try_view"
    );
}

#[test]
#[serial]
fn closure_separate_vs_unified_is_byte_identical() {
    let expected = oracle();

    // Leg A — default (Unified).
    std::env::remove_var(KNOB);
    let (unified_a, seq_a) = run_closure_chain("ddc_ab_uni", true);
    assert!(
        unified_a >= 1,
        "leg A (default) must be Unified; got {unified_a}"
    );
    assert_eq!(seq_a, expected, "leg A delivers the hand oracle");

    // Leg B — forced Separate via the drain-discipline seam.
    let _guard = EnvVarGuard::set("separate");
    let (unified_b, seq_b) = run_closure_chain("ddc_ab_sep", true);
    assert_eq!(
        unified_b, 0,
        "leg B (=separate) must force the closure binding to Separate"
    );
    assert_eq!(seq_b, expected, "leg B delivers the SAME hand oracle");
    assert_eq!(
        seq_a, seq_b,
        "closure Separate delivery must be byte-identical to Unified \
         (the seam invariant extended to closures)"
    );
}

#[test]
#[serial]
fn closure_opt_out_stays_separate_and_delivers() {
    // `.with_unified_drain(false)` — the accumulate-all escape hatch — must
    // keep the binding Separate even with the env unset (default-Unified
    // conditions), and the Separate path must still deliver. Kills a mutation
    // that ignores the opt-out (the multi_publisher /tf drain-all consumer
    // depends on it: its try_receive tick would silently lose every frame
    // under Unified).
    std::env::remove_var(KNOB);
    let (unified, seq) = run_closure_chain("ddc_optout", false);
    assert_eq!(
        unified, 0,
        "with_unified_drain(false) must keep the closure binding Separate; \
         got unified_binding_count = {unified}"
    );
    assert_eq!(
        seq,
        oracle(),
        "the opted-out (Separate) closure still delivers the hand oracle"
    );
}

// ===========================================================================
// (h) Warn-once enforcement of the unified READ-PATH CONTRACT (the
// silent-failure class). A unified-bound trigger input's queue is
// consumed by the pre-step drain into the frozen slot, which serves ONLY
// `try_view` — a tick-body `try_receive` observes an EMPTY queue and every
// frame is silently lost to that read. The subscriber now warns ONCE per
// input (`mark_unified_bound` at the runtime's Unified wiring arm), naming
// both remedies. Controls pin that Separate/opted-out inputs and the correct
// `try_view` path never warn (the warn is observability only — delivery and
// the Separate==Unified byte-identity are untouched).
// ===========================================================================

/// The distinctive substring of the misuse warn (see
/// `CerulionSubscriber::warn_unified_receive_misuse`).
const MISUSE_WARN: &str = "UNIFIED-drained trigger input";

/// [`run_closure_chain`]'s misuse twin: the consumer's tick FIRST drains via
/// `try_receive` (counting every frame that read observes — under Unified the
/// hand oracle is ZERO) and THEN reads via `try_view` (the frozen-slot-served
/// path — the frames actually flow). `unified_capability` is threaded to
/// `.with_unified_drain(..)`. Returns `(unified_binding_count,
/// frames_seen_by_try_receive, per-step try_view reads)`.
fn run_closure_receive_chain(prefix: &str, unified_capability: bool) -> (usize, u64, Vec<u64>) {
    let last_read = Arc::new(AtomicU64::new(MISSING));
    let received = Arc::new(AtomicU64::new(0));
    let produced = std::sync::atomic::AtomicU64::new(0);

    let producer = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec!["out".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 10 }),
        move |ctx| {
            let n = produced.fetch_add(1, Ordering::Relaxed) + 1;
            if let Some(pub_port) = ctx.publisher_mut("out") {
                let mut proxy = pub_port.loan_proxy::<Vector3>()?;
                proxy.x = n as f64;
                proxy.y = 0.0;
                proxy.z = 0.0;
            }
            Ok(())
        },
    )
    .with_label("ddw_producer");

    let consumer_info = NodeInfo::with_meta(
        vec![InputMeta {
            name: "inp".to_string(),
            schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
            trigger: true,
            depth: cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
            backpressure: BackpressurePolicy::DropOldest,
            expect_within_ms: None,
        }],
        vec![],
    )
    .with_policy(MacroPolicy::DataTrigger {
        input_name: "inp".to_string(),
    });
    let last_read_c = Arc::clone(&last_read);
    let received_c = Arc::clone(&received);
    let consumer = ClosureNodeEntry::new(consumer_info, move |ctx| {
        // The MISUSE read first: under Unified the pre-step drain already
        // consumed the queue, so this counts 0 frames (and trips the warn,
        // once). Under Separate (opt-out control) it sees every frame.
        let sub = ctx.subscriber("inp").expect("subscriber 'inp' wired");
        let n = sub.try_receive(|msg| {
            let x = f64::from_le_bytes(msg.payload()[0..8].try_into().unwrap());
            last_read_c.store(x as u64, Ordering::Relaxed);
        })?;
        received_c.fetch_add(n as u64, Ordering::Relaxed);
        // Then the CORRECT read: the frozen-slot-served try_view.
        if let Some(v) = ctx.subscriber_mut("inp").and_then(|s| {
            s.try_view::<Vector3, _>(|view| view.x as u64)
                .ok()
                .flatten()
        }) {
            last_read_c.store(v, Ordering::Relaxed);
        }
        Ok(())
    })
    .with_label("ddw_consumer")
    .with_unified_drain(unified_capability);

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "drain_discipline_warn".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "producer".to_string(),
                node_type: "ddw_producer".to_string(),
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
                id: "consumer".to_string(),
                node_type: "ddw_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(producer));
    factories.insert("consumer".to_string(), Box::new(consumer));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build drain-discipline warn graph");

    let unified = runtime.unified_binding_count_for_test();

    for _ in 0..WARMUP {
        runtime.step(Duration::from_millis(10));
    }
    let mut seq = Vec::with_capacity(MEASURED as usize);
    for _ in 0..MEASURED {
        last_read.store(MISSING, Ordering::Relaxed);
        runtime.step(Duration::from_millis(10));
        seq.push(last_read.load(Ordering::Relaxed));
    }
    (unified, received.load(Ordering::Relaxed), seq)
}

/// (h-i) The enforcement + the loss mechanism, in one pin. A unified closure
/// whose tick calls `try_receive` every step: the misuse warn fires EXACTLY
/// ONCE across all `WARMUP + MEASURED` steps (warn-once latch — not per-tick
/// spam, not zero), `try_receive` observes ZERO frames across the entire run
/// (the hand-oracle loss mechanism: the pre-step drain consumed the queue),
/// and the same tick's `try_view` serves the full producer sequence (the
/// frames were never lost to the CORRECT read path).
#[test]
#[serial]
#[traced_test]
fn unified_try_receive_warns_once_and_sees_zero_frames_while_try_view_delivers() {
    std::env::remove_var(KNOB);
    let (unified, received, seq) = run_closure_receive_chain("ddw_misuse", true);
    assert!(
        unified >= 1,
        "premise: the consumer's binding must be Unified (got {unified})"
    );
    assert_eq!(
        received, 0,
        "the loss mechanism: try_receive on the unified-bound input must \
         observe ZERO frames (the pre-step drain consumed the queue); got \
         {received}"
    );
    assert_eq!(
        seq,
        oracle(),
        "the same tick's try_view must serve the full producer sequence \
         (frames flow on the correct read path)"
    );
    logs_assert(|lines: &[&str]| {
        let warns = lines.iter().filter(|l| l.contains(MISUSE_WARN)).count();
        if warns == 1 {
            Ok(())
        } else {
            Err(format!(
                "expected the try_receive misuse warn EXACTLY once across \
                 {} steps (warn-once latch), got {warns}",
                WARMUP + MEASURED
            ))
        }
    });
}

/// (h-ii) Control: the `.with_unified_drain(false)` opted-out (Separate)
/// consumer reading via `try_receive` — the accumulate-all escape hatch — gets
/// NO warn and its `try_receive` sees every frame (hand oracle: one frame per
/// step, values equal to the producer sequence). Kills a mutation that flags
/// the subscriber regardless of the binding arm.
#[test]
#[serial]
#[traced_test]
fn opted_out_separate_try_receive_flows_without_warn() {
    std::env::remove_var(KNOB);
    let (unified, received, seq) = run_closure_receive_chain("ddw_optout", false);
    assert_eq!(
        unified, 0,
        "premise: with_unified_drain(false) must keep the binding Separate"
    );
    assert_eq!(
        received,
        (WARMUP + MEASURED) as u64,
        "Separate: try_receive must see EVERY frame (one per step)"
    );
    assert_eq!(
        seq,
        oracle(),
        "Separate: the per-step try_receive read equals the producer sequence"
    );
    logs_assert(|lines: &[&str]| {
        let warns = lines.iter().filter(|l| l.contains(MISUSE_WARN)).count();
        if warns == 0 {
            Ok(())
        } else {
            Err(format!(
                "an opted-out (Separate) try_receive must NEVER warn, got {warns}"
            ))
        }
    });
}

/// (h-iii) Control: a unified closure reading via `try_view` (the correct,
/// frozen-slot-served path) never trips the warn — the enforcement targets the
/// misuse read, not the Unified binding itself.
#[test]
#[serial]
#[traced_test]
fn unified_try_view_read_does_not_warn() {
    std::env::remove_var(KNOB);
    let (unified, seq) = run_closure_chain("ddw_view", true);
    assert!(
        unified >= 1,
        "premise: the consumer's binding must be Unified (got {unified})"
    );
    assert_eq!(seq, oracle(), "try_view delivers the hand oracle");
    logs_assert(|lines: &[&str]| {
        let warns = lines.iter().filter(|l| l.contains(MISUSE_WARN)).count();
        if warns == 0 {
            Ok(())
        } else {
            Err(format!(
                "a unified try_view read must NEVER warn, got {warns}"
            ))
        }
    });
}

/// (h-iv) `try_receive_one` shares the loss mode AND the latch. A unified
/// closure whose FIRST tick reads via `try_receive_one` (the rmw-style
/// one-message read) and whose LATER ticks read via `try_receive`: the misuse
/// warn fires EXACTLY ONCE total across both methods and all steps (the latch
/// is per-INPUT, shared by the read-path family — the first offender warns,
/// naming itself; a subsequent `try_receive` misuse is latched silent), the
/// receive family observes ZERO frames end-to-end, and `try_view` still
/// serves the full oracle. (`wait_for_message` shares the same hook + latch;
/// no arm for it — a blocking wait inside a scheduler tick is not a realistic
/// test shape, and the hook body is identical.)
#[test]
#[serial]
#[traced_test]
fn unified_try_receive_one_warns_once_and_latch_is_shared_across_methods() {
    std::env::remove_var(KNOB);

    let last_read = Arc::new(AtomicU64::new(MISSING));
    let received = Arc::new(AtomicU64::new(0));
    let produced = std::sync::atomic::AtomicU64::new(0);

    let producer = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec!["out".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 10 }),
        move |ctx| {
            let n = produced.fetch_add(1, Ordering::Relaxed) + 1;
            if let Some(pub_port) = ctx.publisher_mut("out") {
                let mut proxy = pub_port.loan_proxy::<Vector3>()?;
                proxy.x = n as f64;
                proxy.y = 0.0;
                proxy.z = 0.0;
            }
            Ok(())
        },
    )
    .with_label("ddw1_producer");

    let consumer_info = NodeInfo::with_meta(
        vec![InputMeta {
            name: "inp".to_string(),
            schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
            trigger: true,
            depth: cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
            backpressure: BackpressurePolicy::DropOldest,
            expect_within_ms: None,
        }],
        vec![],
    )
    .with_policy(MacroPolicy::DataTrigger {
        input_name: "inp".to_string(),
    });
    let last_read_c = Arc::clone(&last_read);
    let received_c = Arc::clone(&received);
    let ticks = std::sync::atomic::AtomicU64::new(0);
    let consumer = ClosureNodeEntry::new(consumer_info, move |ctx| {
        let tick = ticks.fetch_add(1, Ordering::Relaxed);
        let sub = ctx.subscriber("inp").expect("subscriber 'inp' wired");
        if tick == 0 {
            // FIRST misuse via try_receive_one (AnySubscriber has no
            // passthrough — reach the Ipc subscriber via the public variant).
            let AnySubscriber::Ipc(ipc) = sub;
            let got = ipc.try_receive_one(|_msg| {})?;
            if got {
                received_c.fetch_add(1, Ordering::Relaxed);
            }
        } else {
            // LATER misuse via try_receive: the SHARED latch must keep this
            // silent (no second warn from a different method).
            let n = sub.try_receive(|_msg| {})?;
            received_c.fetch_add(n as u64, Ordering::Relaxed);
        }
        // The correct read still serves the frame.
        if let Some(v) = ctx.subscriber_mut("inp").and_then(|s| {
            s.try_view::<Vector3, _>(|view| view.x as u64)
                .ok()
                .flatten()
        }) {
            last_read_c.store(v, Ordering::Relaxed);
        }
        Ok(())
    })
    .with_label("ddw1_consumer")
    .with_unified_drain(true);

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "drain_discipline_warn_one".to_string(),
        prefix: "ddw_one".to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "producer".to_string(),
                node_type: "ddw1_producer".to_string(),
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
                id: "consumer".to_string(),
                node_type: "ddw1_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(producer));
    factories.insert("consumer".to_string(), Box::new(consumer));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build drain-discipline try_receive_one graph");
    assert!(
        runtime.unified_binding_count_for_test() >= 1,
        "premise: the consumer's binding must be Unified"
    );

    for _ in 0..WARMUP {
        runtime.step(Duration::from_millis(10));
    }
    let mut seq = Vec::with_capacity(MEASURED as usize);
    for _ in 0..MEASURED {
        last_read.store(MISSING, Ordering::Relaxed);
        runtime.step(Duration::from_millis(10));
        seq.push(last_read.load(Ordering::Relaxed));
    }

    assert_eq!(
        received.load(Ordering::Relaxed),
        0,
        "the receive family (try_receive_one then try_receive) must observe \
         ZERO frames on the unified-bound input"
    );
    assert_eq!(seq, oracle(), "try_view still serves the full oracle");
    logs_assert(|lines: &[&str]| {
        let warns = lines.iter().filter(|l| l.contains(MISUSE_WARN)).count();
        let named = lines
            .iter()
            .filter(|l| l.contains(MISUSE_WARN) && l.contains("try_receive_one"))
            .count();
        if warns == 1 && named == 1 {
            Ok(())
        } else {
            Err(format!(
                "expected EXACTLY one misuse warn total (shared latch across \
                 methods), naming try_receive_one (the first offender); got \
                 {warns} warn(s), {named} naming try_receive_one"
            ))
        }
    });
}

/// (h-v) `wait_for_message` first-offender runtime coverage.
/// Only the runtime can mark a subscriber unified-bound, so the cheapest seam
/// is a minimal unified graph whose consumer tick calls two queue-draining
/// reads DIRECTLY on the marked input's subscriber: tick 0 blocks on
/// `wait_for_message` with a 1 ms timeout (the FIRST offender — the warn must
/// fire once and carry method="wait_for_message"), tick 1 misuses `try_receive`
/// (a sibling family member — the SHARED latch must keep it silent). One warn
/// total across both methods; the receive family observes zero frames;
/// `try_view` still serves the oracle. (This arm's unique coverage is
/// `wait_for_message` as the FIRST, named offender — the sibling test covers a
/// `try_receive_one`-first pairing.)
#[test]
#[serial]
#[traced_test]
fn unified_wait_for_message_warns_once_named_and_latch_covers_try_receive() {
    std::env::remove_var(KNOB);

    let last_read = Arc::new(AtomicU64::new(MISSING));
    let received = Arc::new(AtomicU64::new(0));
    let produced = std::sync::atomic::AtomicU64::new(0);

    let producer = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec!["out".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 10 }),
        move |ctx| {
            let n = produced.fetch_add(1, Ordering::Relaxed) + 1;
            if let Some(pub_port) = ctx.publisher_mut("out") {
                let mut proxy = pub_port.loan_proxy::<Vector3>()?;
                proxy.x = n as f64;
                proxy.y = 0.0;
                proxy.z = 0.0;
            }
            Ok(())
        },
    )
    .with_label("ddw2_producer");

    let consumer_info = NodeInfo::with_meta(
        vec![InputMeta {
            name: "inp".to_string(),
            schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
            trigger: true,
            depth: cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
            backpressure: BackpressurePolicy::DropOldest,
            expect_within_ms: None,
        }],
        vec![],
    )
    .with_policy(MacroPolicy::DataTrigger {
        input_name: "inp".to_string(),
    });
    let last_read_c = Arc::clone(&last_read);
    let received_c = Arc::clone(&received);
    let ticks = std::sync::atomic::AtomicU64::new(0);
    let consumer = ClosureNodeEntry::new(consumer_info, move |ctx| {
        let tick = ticks.fetch_add(1, Ordering::Relaxed);
        if tick == 0 {
            // FIRST offender: the blocking read, bounded to 1 ms so the tick
            // shape stays cheap. Must warn, naming wait_for_message.
            let sub = ctx.subscriber("inp").expect("subscriber 'inp' wired");
            let n = sub.wait_for_message(Duration::from_millis(1), |_msg| {})?;
            received_c.fetch_add(n as u64, Ordering::Relaxed);
        } else if tick == 1 {
            // SECOND offender, different method: the shared latch keeps it
            // silent (one warn total across the whole read family).
            let sub = ctx.subscriber_mut("inp").expect("subscriber 'inp' wired");
            let AnySubscriber::Ipc(ipc) = sub;
            let mut n = 0usize;
            ipc.try_receive(|_msg| {
                n += 1;
            })?;
            received_c.fetch_add(n as u64, Ordering::Relaxed);
        }
        // The correct read still serves the frame.
        if let Some(v) = ctx.subscriber_mut("inp").and_then(|s| {
            s.try_view::<Vector3, _>(|view| view.x as u64)
                .ok()
                .flatten()
        }) {
            last_read_c.store(v, Ordering::Relaxed);
        }
        Ok(())
    })
    .with_label("ddw2_consumer")
    .with_unified_drain(true);

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "drain_discipline_warn_wait".to_string(),
        prefix: "ddw_wait".to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "producer".to_string(),
                node_type: "ddw2_producer".to_string(),
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
                id: "consumer".to_string(),
                node_type: "ddw2_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(producer));
    factories.insert("consumer".to_string(), Box::new(consumer));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build drain-discipline wait_for_message graph");
    assert!(
        runtime.unified_binding_count_for_test() >= 1,
        "premise: the consumer's binding must be Unified"
    );

    for _ in 0..WARMUP {
        runtime.step(Duration::from_millis(10));
    }
    let mut seq = Vec::with_capacity(MEASURED as usize);
    for _ in 0..MEASURED {
        last_read.store(MISSING, Ordering::Relaxed);
        runtime.step(Duration::from_millis(10));
        seq.push(last_read.load(Ordering::Relaxed));
    }

    assert_eq!(
        received.load(Ordering::Relaxed),
        0,
        "wait_for_message + try_receive must observe ZERO frames on the \
         unified-bound input"
    );
    assert_eq!(seq, oracle(), "try_view still serves the full oracle");
    logs_assert(|lines: &[&str]| {
        let warns = lines.iter().filter(|l| l.contains(MISUSE_WARN)).count();
        let named = lines
            .iter()
            .filter(|l| l.contains(MISUSE_WARN) && l.contains("wait_for_message"))
            .count();
        if warns == 1 && named == 1 {
            Ok(())
        } else {
            Err(format!(
                "expected EXACTLY one misuse warn total (shared latch across \
                 wait_for_message + try_receive), naming wait_for_message (the \
                 first offender); got {warns} warn(s), {named} naming it"
            ))
        }
    });
}
