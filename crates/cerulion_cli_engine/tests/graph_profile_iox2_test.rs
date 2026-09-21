// SPDX-License-Identifier: AGPL-3.0-only
//! END-TO-END pins for `cerulion graph profile` over REAL
//! iceoryx2 shared memory — the engine-fn precedent (`topic_observer_iox2_test`):
//! call [`graph_profile`] in-process against a hand-built tempdir workspace
//! whose node cdylibs are COPIES of the prebuilt macro fixtures (no in-test
//! cargo build).
//!
//! Graph under profile: `ticker` (`#[cerulion_node(period_ms = 50)]`, output
//! `cmd: Vector3` — the `test_node_macro_period_cdylib` fixture) →
//! `sink` (`#[input(trigger)] trigger_in: Vector3` — the
//! `test_node_macro_data_trigger_cdylib` fixture).
//!
//! Arms:
//! 1. **Gate-met happy path** — a low `fires_target` gates fast: the report
//!    has NO isolated nodes, the artifact exists at the DEFAULT path
//!    (`graphs/<name>.costs.yaml`), parses (v2), costs BOTH nodes (p50 > 0),
//!    carries the (ticker → sink) edge with a plausible rate, its hop
//!    block is the profiling machine's platform default, and the
//!    frozen `derived_budget_ns`/`profile_cores` pair is present and equals
//!    `ceil(Σ p50 / cores)` of the artifact's own numbers.
//! 2. **Cap-hit adversarial** — an unreachable `fires_target` under a short
//!    cap: the run stops at the cap, EVERY node is isolated (+ report
//!    matches), and the artifact is STILL written (empty `nodes:`/`edges:`,
//!    full `isolated:`, and NO frozen budget: nothing costed) —
//!    a partial profile is a durable, accurate artifact, never a crash.
//! 3. **Shape determinism** — two profile runs on the same graph agree on the
//!    STRUCTURE: same costed-node key set, same edge pair set, same isolated
//!    set, and `nodes ∪ isolated == all graph nodes` (the harvest totality
//!    contract). The p50/rate VALUES are wall-clock measurements on the live
//!    `RealClock` path and are NOT asserted equal — value-level
//!    recording-on/off byte-identity is pinned at the right layer by the core
//!    B-dur suite (`recording_honest_clock_iox2_test` + `scheduler_test`).
//! 4. **Pre-stopped run (the Ctrl-C composition seam)** — `running` already
//!    false makes `run_live` return immediately: zero fires, everything
//!    isolated, artifact still written. Pins that an early Ctrl-C yields a
//!    durable (if empty) artifact instead of a lost run.
//! 5. **Capped ring** — a heterogeneous graph (flooding
//!    ticker→sink + a never-firing data-trigger node on a producer-less
//!    absolute external source) out-fires the fires_target-sized ring over
//!    the full cap: the loud capped-ring warn fires, the flooding nodes are
//!    STILL costed from the newest retained samples, and the artifact writes.
//! 6. **AUTO-derive low-rate** — the same 20 Hz graph
//!    profiled with `fires_override = None` over a short cap: the warm-up
//!    derives per-node targets scaled to the observed rate, so the low-rate
//!    nodes are COSTED, not blanket-isolated (the earlier uniform
//!    default of 1000 would have isolated both), + totality.
//! 7. **AUTO-derive silent node** — auto mode over the
//!    3-node silent-workspace: the silent node is isolated with the
//!    `target: None` "silent through warm-up" marker AND the starved-trigger
//!    hint naming its zero-rate trigger input; the live pair is costed and
//!    the gate stops the run early (the silent node cannot hold it hostage).
//! 8. **AUTO-derive EARLY STOP** — a mid-run Ctrl-C (the
//!    `running` flag flipped by a stopper thread AFTER warm-up completed,
//!    well before the cap) must COST the sampled nodes, not isolate them:
//!    the harvest gate re-projects the warm-up observation to the ACTUAL
//!    window (cap-horizon gating would spuriously isolate everything at
//!    ~1/4 of the cap), and the no-warm-up fallback breadcrumb must NOT
//!    appear (a `unwrap_or_default` that discards the snapshot would emit it).
//! 9. **AUTO-derive ring sizing** — the silent-workspace run in
//!    auto mode never trips the capped-ring warn: the ring is sized from the
//!    derivation CEILING (`max_samples × nodes × 4` = 12 000 ≫ the ~160
//!    fires a 4 s cap floods) and the flooding pair still costs. Kills an
//!    `unwrap_or(0)` ring-sizing regression (ring collapses to 1 → the warn
//!    fires + a flooding node demotes for lack of duration samples).
//! 10. **Ambiguous `schema:` refused before profiling** — the
//!     lookup's refusal, the same as `graph run`'s, in this binary because
//!     the verb sweeps dead iceoryx2 nodes before it reads the graph.
//!
//! **Load robustness.** Arms 6-9 profile in AUTO mode, where each
//! node's fire target is derived from the rate the machine actually delivered
//! during warm-up. On a machine running five concurrent builds those four
//! arms go red with the code unchanged — measured `fires: 13, target:
//! Some(20)` (2.2 Hz) and `sink fires=8
//! target=25` (0.94 Hz, 88 drop_oldest evictions) — while an idle machine is 9/9
//! green. That is a statement about the MACHINE, not the code under test.
//!
//! Each of those arms routes ONLY its costed-node assertions through
//! [`auto_mode_starved_isolation`], which excuses exactly one shape: every
//! isolation was gated against a REAL target it fell short of, the node ran
//! below [`LOAD_DEGRADE_HZ`] (half the fixtures' 20 Hz nominal), AND that
//! target was itself projected from a warm-up under [`STARVED_WARMUP_HZ`]
//! (`0.75 ×` nominal) — so the machine, not one node, was slow. Keying on the
//! target VALUE would not work (the reproduced failure's target was 25, not the
//! policy floor), and keying on the observed rate ALONE would excuse an
//! ASYMMETRIC regression that collapses `sink` after a healthy warm-up while
//! `ticker` stays at 20 Hz; requiring the warm-up to have been starved too is
//! what refuses that.
//!
//! **The 0.75 margin is load-bearing.** Comparing the recovered warm-up rate
//! against nominal with NO margin is satisfied on every real run — the engine's
//! `derive_fire_targets` floors twice and the profiler's measured warm-up
//! always overshoots its nominal window, so a HEALTHY machine recovers 19.667 Hz
//! (0.983 × nominal) and would be blessed as "starved". Measured
//! separation: healthy 0.90–0.98 × nominal, the two starved regimes 0.295 and
//! 0.333 ×. See [`STARVED_WARMUP_FRACTION`] for the full bias arithmetic.
//!
//! **A second load signature, MEASURED, and closed in
//! the engine rather than excused here.** A fast runner can isolate
//! `ticker` in arm 7 with `fires=160 target=176` over a 6022 ms window: an
//! observed 26.57 Hz and a recovered warm-up of 58.45 Hz, i.e. **1.33 ×** and
//! **2.92 ×** nominal. That machine is not starved — it is FAST — so the
//! classifier above refuses it (correctly) and the arm's isolated-SET equality
//! fires. The cause is a BRING-UP BURST: a `period_ms = 50` node owes a fire
//! for every 50 ms of scheduler time since its last, the graph build is elapsed
//! time in which it cannot fire, and the debt is replayed as a catch-up burst
//! that lands inside a 1 s warm-up measured from run start
//! (38.45 implied burst fires × 50 ms = 1.95 s, against a ~2.75 s bring-up on
//! that runner).
//!
//! The engine closes that shape itself: each node's warm-up window
//! starts at THAT NODE'S FIRST FIRE, so the whole burst falls outside the
//! observation and the derived target is projected from the sustained rate.
//! The engine cannot produce the shape, so there is no test-side burst gate
//! to excuse it; the arms consult
//! [`auto_mode_starved_isolation`] directly. The pointer lives in that
//! classifier's docs. Arm 7's set equality
//! is SPLIT — "`silent` is the only node carrying the `target: None` marker"
//! is the derivation-poisoning kill and is load-INSENSITIVE, so it is asserted
//! unconditionally; only "nothing else isolated at all" — the half a starved
//! `ticker` JOINS — sits behind the gate.
//!
//! Every other shape still fails hard, so no arm's defect-detection is weakened. The
//! classifier is oracle-pinned by
//! `starved_isolation_gate_classifies_only_the_rate_collapse_signature`, whose
//! margin boundary pair (`Some(45)` refused / `Some(44)` excused) is derived
//! from the engine's own arithmetic so both sides are REACHABLE on a real run:
//! the EXCUSED side from an ordinary 10 ms-quantised warm-up (`F = 15` over
//! `w ≈ 1.005 s`), the REFUSED side from a warm-up window that overshot its
//! nominal 1.000 s by >= 4.4 % (`F = 16` over `w ≈ 1.05 s`) — a modest
//! overshoot the poll quantisation alone does not reach, but a LOADED machine (this
//! gate's whole reason for existing) routinely produces.
//!
//! **What an excused run still asserts.** The gate is consulted LAST and skips
//! as little as possible: each arm asserts its whole load-INSENSITIVE half
//! first and unconditionally — mode/window facts, the harvest TOTALITY
//! partition, the no-phantom-edge shape, "every emitted p50 is a real nonzero
//! measurement", the silent node's `target: None` marker, its starved-trigger
//! hint and its artifact-side isolation, the capped-ring warn's
//! presence/absence, and arm 8's `run ended before warm-up completed`
//! breadcrumb-absence (that arm's
//! only defense against a regression there, and a WALL-clock fact independent of fire rates). Only
//! the claims that name WHICH nodes made the gate sit behind it.
//!
//! **Loudness caveat.** A skip prints a `LOAD
//! DEGRADE` line to stderr and returns; libtest CAPTURES and DISCARDS stderr
//! for a PASSING test, so on a green CI run — which never passes `--nocapture`
//! — the skip is INVISIBLE. The `cdylib_qos_behavioral_test` precedent has the
//! same property. That is precisely why the skip was shrunk to the smallest
//! possible set of assertions rather than being relied on to announce itself:
//! run with `-- --nocapture` when you need to know whether an arm degraded.
//!
//! Prerequisites (the repo's fixture pattern — each test PANICS with the
//! build instruction if an artifact is missing):
//! ```bash
//! cargo build -p test_node_macro_period_cdylib -p test_node_macro_data_trigger_cdylib
//! ```
//!
//! These tests run the production `graph_profile` path, which initializes the
//! process-global `TransportManager` singleton (GLOBAL iceoryx2 namespace) —
//! `#[serial]` + pid/nanos-unique graph prefixes, and run with
//! `--test-threads=1`. Run them yourself with:
//! ```bash
//! cargo test -p cerulion_cli_engine --test graph_profile_iox2_test -- --test-threads=1
//! ```

#![cfg(unix)]

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use cerulion_cli_engine::graph_cmd::{
    default_artifact_path, graph_profile, ProfileArtifact, ProfileReport,
};
use cerulion_core::graph::{FireTargetPolicy, HopCosts};
use serial_test::serial;

/// The platform cdylib filename for a crate/node name.
fn dylib_file(name: &str) -> String {
    if cfg!(target_os = "macos") {
        format!("lib{name}.dylib")
    } else {
        format!("lib{name}.so")
    }
}

/// A prebuilt fixture cdylib from THIS run's own cargo target directory
/// (derived from the test binary's location, so an isolated `CARGO_TARGET_DIR`
/// resolves correctly), PANICKING with the build instruction when missing.
fn fixture_cdylib(fixture: &str) -> PathBuf {
    cerulion_core::testing::find_fixture_cdylib(fixture)
}

/// A fixture's source file (copied into the workspace so the CLI's
/// source-is-truth metadata walker can parse the node's trigger policy).
fn fixture_src(fixture: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("repo root")
        .join("test_fixtures")
        .join(fixture)
        .join("src/lib.rs")
}

/// A pid + nanosecond unique graph prefix so runs on the shared global
/// iceoryx2 namespace can never collide across tests/processes.
fn unique_prefix(tag: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    format!("prof{tag}{}n{}", std::process::id(), nanos)
}

/// Hand-build the profiling workspace in `root`: a `[workspace]` Cargo.toml,
/// `graphs/demo.yaml` (ticker → sink under `prefix`), the two node types'
/// sources (copies of the fixture sources, for the metadata walker), and the
/// prebuilt fixture cdylibs under `target/debug/` under the node-type names.
fn build_workspace(root: &Path, prefix: &str) {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::create_dir_all(root.join("nodes/ticker/src")).unwrap();
    std::fs::create_dir_all(root.join("nodes/sink/src")).unwrap();
    std::fs::create_dir_all(root.join("target/debug")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();
    std::fs::write(
        root.join("graphs/demo.yaml"),
        format!(
            "name: demo\nprefix: {prefix}\nnodes:\n\
             - id: ticker\n  type: ticker\n  inputs: []\n  outputs:\n\
             \x20 - name: cmd\n    schema: geometry_msgs/Vector3\n\
             - id: sink\n  type: sink\n  inputs:\n\
             \x20 - name: trigger_in\n    source: ticker/cmd\n  outputs:\n\
             \x20 - name: cmd\n    schema: geometry_msgs/Vector3\n"
        ),
    )
    .unwrap();
    std::fs::copy(
        fixture_src("test_node_macro_period_cdylib"),
        root.join("nodes/ticker/src/lib.rs"),
    )
    .expect("copy ticker fixture src");
    std::fs::copy(
        fixture_src("test_node_macro_data_trigger_cdylib"),
        root.join("nodes/sink/src/lib.rs"),
    )
    .expect("copy sink fixture src");
    std::fs::copy(
        fixture_cdylib("test_node_macro_period_cdylib"),
        root.join("target/debug").join(dylib_file("ticker")),
    )
    .expect("copy ticker cdylib");
    std::fs::copy(
        fixture_cdylib("test_node_macro_data_trigger_cdylib"),
        root.join("target/debug").join(dylib_file("sink")),
    )
    .expect("copy sink cdylib");
}

/// RAII guard: clear `CARGO_TARGET_DIR` for the test body (the in-workspace
/// `target/` lookup must win) and restore the prior value on drop —
/// panic-safe (the repo's `EnvVarGuard` pattern; `#[serial]` serializes the
/// process-global env mutation).
struct TargetDirGuard(Option<std::ffi::OsString>);
impl TargetDirGuard {
    fn clear() -> Self {
        let prior = std::env::var_os("CARGO_TARGET_DIR");
        std::env::remove_var("CARGO_TARGET_DIR");
        Self(prior)
    }
}
impl Drop for TargetDirGuard {
    fn drop(&mut self) {
        if let Some(v) = self.0.take() {
            std::env::set_var("CARGO_TARGET_DIR", v);
        }
    }
}

/// Read + parse the artifact at `path`.
fn read_artifact(path: &Path) -> ProfileArtifact {
    let yaml = std::fs::read_to_string(path).expect("artifact file exists and is readable");
    serde_yaml::from_str(&yaml).expect("artifact parses back")
}

const ALL_NODES: [&str; 2] = ["sink", "ticker"];

/// The 3-node silent workspace ([`build_workspace_with_silent`]).
const SILENT_WS_NODES: [&str; 3] = ["sink", "silent", "ticker"];

/// `nodes ∪ isolated` must cover EVERY graph node exactly once (the harvest
/// totality contract — a node is costed or isolated, never neither/both).
///
/// This contract is LOAD-INSENSITIVE — it constrains the PARTITION of
/// the node set, never which side any node lands on — so every caller asserts
/// it BEFORE consulting [`auto_mode_starved_isolation`].
fn assert_totality_over(artifact: &ProfileArtifact, nodes: &[&str]) {
    let mut covered: BTreeSet<String> = artifact.nodes.keys().cloned().collect();
    for iso in &artifact.isolated {
        assert!(
            covered.insert(iso.clone()),
            "node '{iso}' is BOTH costed and isolated"
        );
    }
    let expected: BTreeSet<String> = nodes.iter().map(|s| s.to_string()).collect();
    assert_eq!(
        covered, expected,
        "nodes: ∪ isolated: must cover every graph node exactly once"
    );
}

/// [`assert_totality_over`] for the 2-node `ticker → sink` workspace.
fn assert_totality(artifact: &ProfileArtifact) {
    assert_totality_over(artifact, &ALL_NODES);
}

/// Every edge the harvest emitted must be the ONE real trigger edge
/// (`ticker → sink`); an isolated endpoint legitimately DROPS it, so the count
/// is `<= 1`, never pinned to 1 here.
///
/// The load-INSENSITIVE half of the edge-shape contract (no phantom
/// edge, no wrong endpoints, no self-edge) — asserted before the degrade gate.
/// The load-SENSITIVE half (the edge is actually PRESENT, with a nonzero
/// observed rate) stays behind it.
fn assert_no_phantom_edges(artifact: &ProfileArtifact) {
    assert!(
        artifact.edges.len() <= 1,
        "at most the one ticker → sink trigger edge can be harvested; got {:?}",
        artifact
            .edges
            .iter()
            .map(|e| (&e.producer, &e.consumer))
            .collect::<Vec<_>>()
    );
    for edge in &artifact.edges {
        assert_eq!(edge.producer, "ticker", "the only producer is ticker");
        assert_eq!(edge.consumer, "sink", "the only consumer is sink");
    }
}

/// The NOMINAL fire rate of both fixture nodes: `ticker` is
/// `#[cerulion_node(period_ms = 50)]` = 20 Hz, and `sink` is data-triggered by
/// it (one fire per delivered ticker frame), so the same 20 Hz.
const FIXTURE_NOMINAL_HZ: f64 = 20.0;

/// The rate below which a node provably did not get the CPU these
/// contracts assume. HALF the fixture nominal — an idle machine delivers ~20 Hz and
/// a merely-busy one stays well above 10 Hz, while the measured starved regimes
/// are 2.2 Hz (`fires: 13` over a 6 s window) and 0.94 Hz
/// (`sink fires=8 target=25` over 8.5 s, reproduced under five concurrent builds).
const LOAD_DEGRADE_HZ: f64 = FIXTURE_NOMINAL_HZ / 2.0;

/// The fraction of [`FIXTURE_NOMINAL_HZ`] a run's
/// recovered warm-up rate must fall below before the isolation is excused as
/// load. **This margin is load-bearing, not decorative** — without it the
/// warm-up condition is satisfied on every real run and the classifier collapses
/// back to "observed rate alone".
///
/// **Why a bare `< FIXTURE_NOMINAL_HZ` is always true.** The recovered rate
/// inverts the engine's target derivation, which FLOORS twice
/// (`floor(floor(F·W/w)·num/den)`) over the node's warm-up fire count `F` and
/// window `w`. With first-fire anchoring `F` is the
/// fires observed after the node's first sighting and `w` its own active window;
/// the arithmetic is the same: `F` is an
/// integer count over a window it does not exactly tile, so `F/w <= R` for a
/// node at true rate `R`. Writing `E = F·W/w`, the two floors give
/// `t >= E/2 − 1`, hence
///
/// ```text
/// recovered = 2t/W  >  F/w − 2/W        (an UNDER-estimate, by < 2/W Hz)
/// ```
///
/// Both biases push the recovered value
/// strictly BELOW nominal, so a zero-margin comparison can never fail. Measured
/// on the arm-6 shape (`W = 6 s`, `w ≈ 1.005 s`, 20 fires): `t = 59`, recovered
/// **19.667 Hz = 0.983 × nominal** — a fully HEALTHY machine, silently blessed as
/// "starved" by a zero-margin cut.
///
/// **Why 0.75.** The two regimes are far apart and the cut is chosen from the
/// measured arithmetic, not by feel:
///
/// | regime | recovered / nominal |
/// |---|---|
/// | healthy machine (`F = 20`, `w ≈ 1.005 s`) | 0.983 |
/// | healthy machine losing 1 warm-up fire (`F = 19`, `w = 1.01 s`) | 0.933 |
/// | healthy machine losing 2 fires + a 5 % long warm-up | 0.900 |
/// | the original filing (`fires 13`, `target 20`, 6 s) | 0.333 |
/// | the loaded reproduction (`sink 8/25`, 8.5 s) | 0.295 |
///
/// 0.75 sits ~0.15 below the worst plausible HEALTHY value and 2.25× above the
/// worst measured STARVED one. The quantisation bias bounded above is `< 2/W`
/// Hz (0.33 Hz = 0.017 × nominal at `W = 6 s`) plus at most one dropped warm-up
/// fire (`1/w` ≈ 1 Hz = 0.05 × nominal), i.e. ~0.07 × nominal total — the margin
/// is 0.25, so quantisation CANNOT eat it.
///
/// **What this deliberately refuses.** A machine healthy enough to recover
/// `>= 15 Hz` through warm-up and then collapse below 10 Hz is not a slow machine;
/// it is a delivery regression, and the arm hard-fails as it should. Excusing
/// that shape is exactly the defect this margin closes.
const STARVED_WARMUP_FRACTION: f64 = 0.75;

/// The absolute recovered-warm-up ceiling for an excused run — see
/// [`STARVED_WARMUP_FRACTION`] for the derivation. 15 Hz at these
/// constants.
const STARVED_WARMUP_HZ: f64 = FIXTURE_NOMINAL_HZ * STARVED_WARMUP_FRACTION;

/// COMPILE-TIME guard that the margin is a REAL one. At `>= 1.0` the warm-up
/// condition is satisfied by the floor bias alone (see
/// [`STARVED_WARMUP_FRACTION`]) and the classifier silently degrades to its
/// observed-rate half. A retune past 1.0 fails the
/// build rather than shipping a vacuous conjunct.
const _: () = assert!(
    STARVED_WARMUP_FRACTION < 1.0,
    "STARVED_WARMUP_FRACTION must leave headroom below nominal: at >= 1.0 the recovered \
     warm-up rate (a floor-biased UNDER-estimate) is below the cut on every real run, so the \
     warm-up conjunct becomes vacuously true and the starved-machine classifier stops \
     discriminating the asymmetric post-warm-up collapse"
);

/// Does this report's isolation of `expected_costed` carry the
/// STARVED-BOX signature rather than a real regression?
///
/// AUTO mode derives each node's target from the rate observed during warm-up
/// and projects it over the window, so *any* isolation means "fired less than
/// half as fast after warm-up as during it" — that is the definition of the
/// gate, not a discriminator. Two independent facts ARE discriminators, and an
/// excused run must show BOTH:
///
/// 1. **The observed rate COLLAPSED** (below [`LOAD_DEGRADE_HZ`]). A regression
///    in the derivation (a fabricated blanket target, a cap-horizon
///    projection) isolates a node still firing at its healthy nominal rate.
/// 2. **The WARM-UP was starved too, BY A MARGIN** — i.e. the machine was never
///    delivering anything close to nominal, so the collapse is a property of the
///    machine and not of one node. The target IS the warm-up observation (the
///    engine derives it as `clamp(warmup_rate × window × fraction, min, max)`),
///    so inverting the engine's own [`FireTargetPolicy`] recovers the rate the
///    warm-up ran at: `warmup_hz ≈ target × den / num / window_s`. **First-fire
///    anchoring sharpens what that inversion recovers**: the engine anchors each
///    node's warm-up at its FIRST FIRE, so the recovered value is the node's
///    SUSTAINED rate rather than a figure inflated by the bring-up catch-up
///    burst — which is what this condition means. Requiring
///    that to be below [`STARVED_WARMUP_HZ`] — `0.75 ×` nominal, NOT nominal
///    itself — is what rules out the asymmetric shape, walked
///    through here: a delivery regression that starves `sink` after warm-up while
///    `ticker` stays costed at 20 Hz leaves `sink`'s target at ~the NOMINAL
///    projection, so condition 2 fails and the arm hard-fails.
///
/// **The margin is the whole condition (do not delete it).** A gate that
/// compares the recovered rate against
/// [`FIXTURE_NOMINAL_HZ`] with ZERO margin is satisfied on every real
/// run: `derive_fire_targets` floors twice and the profiler's measured warm-up
/// always overshoots its nominal window, so the recovered rate is a strict
/// UNDER-estimate of an already-sub-nominal measurement (a healthy machine recovers
/// 19.667 Hz on the arm-6 shape). Condition 2 is then vacuously TRUE, the
/// classifier silently degrades to "observed rate alone", and the asymmetric
/// shape it exists to refuse is excused. [`STARVED_WARMUP_FRACTION`] carries
/// the full bias arithmetic and the measured healthy-vs-starved separation.
///
/// The inversion's error is bounded and the margin absorbs it in BOTH
/// directions: the floors under-estimate by `< 2/window_s` Hz, and where the
/// `min_samples` clamp bit it OVER-estimates (the safe, refusing direction). A
/// target AT/ABOVE `max_samples` carries no upper bound at all (the clamp moved
/// it DOWN), which is definitionally a high-rate node, so it is never excused —
/// that guard SHORT-CIRCUITS before either rate condition. Neither is a
/// uniform-mode (`--fires N`) report excused: its scalar target is not an
/// observation and cannot be inverted.
///
/// **Which condition binds (do not mis-read the pair as independent).** Under
/// the SHIPPED constants condition 2 IMPLIES condition 1: `fires < target` and
/// `target < STARVED_WARMUP_HZ × window × num/den` give `observed_hz <
/// STARVED_WARMUP_HZ × num/den = 7.5 Hz <= ` [`LOAD_DEGRADE_HZ`]. So condition 2
/// is the only one that can ever be the sole reason a run is refused an excuse,
/// and no oracle case can attribute a refusal to condition 1 alone — the gate
/// self-test says so explicitly and asserts the constant relationship
/// (`STARVED_WARMUP_HZ × num/den <= LOAD_DEGRADE_HZ`) that makes the implication
/// hold, so this claim cannot rot silently. Condition 1 is KEPT anyway: it is
/// the direct, derivation-independent statement of the property a reader cares
/// about ("this node's rate collapsed"), computed from the run's OWN fires and
/// window rather than from a model of the engine's arithmetic, so it still
/// bounds the damage if [`FireTargetPolicy`] or the harvest horizon ever change
/// out from under the inversion.
///
/// Returns `Some(diagnostic)` only for that signature; the caller then LOUDLY
/// skips its costed-node assertions (never silently: the `LOAD DEGRADE`
/// marker prints under `--nocapture`, the repo's `cdylib_qos_behavioral_test`
/// precedent for an un-assertable-on-this-machine arm). Note the whole point of the
/// assert-first ordering: by the time a caller consults this gate, everything
/// load-INSENSITIVE has already been asserted, so an excused run still pins the
/// bulk of its contract.
///
/// Returns `None` for every other shape, so the caller's assertion still fires
/// and the defects these arms exist to catch are NOT masked:
/// * nothing isolated → the arm asserts its full contract as before;
/// * a FABRICATED / non-scaling target (a blanket 1000, a
///   cap-horizon projection) → the node still fired at ~nominal → hard
///   fail;
/// * an isolation whose fire count actually MET its target (the
///   no-usable-duration-samples demotion a `unwrap_or(0)` ring-sizing variant
///   produces) → hard fail;
/// * a `target: None` "silent through warm-up" marker on a node that is supposed
///   to fire → hard fail;
/// * an offender whose warm-up ran at or near nominal — anything recovering
///   `>= ` [`STARVED_WARMUP_HZ`], which INCLUDES a healthy machine's own 0.983 ×
///   nominal signature (the asymmetric post-warm-up collapse) → hard fail.
///
/// **This is the ONLY load gate, and that is the point.**
/// A SECOND signature exists (a bring-up catch-up burst counted as
/// warm-up rate, which this classifier correctly refuses because the machine is
/// fast, not starved), and it is closed in the engine, not excused by a
/// sibling gate: the
/// engine anchors each node's warm-up at its own first fire, so the burst is
/// outside the observation and that shape cannot be
/// produced. If a burst-shaped isolation is ever seen, the
/// engine regressed — do not add an excuse for it here.
///
/// **Residual.** A regression present from the very FIRST fire depresses
/// the warm-up equally, and when its projection floors at `min_samples` the
/// target retains no rate information at all — that shape is indistinguishable
/// from load *in the report*. Distinguishing it needs a COSTED sibling's
/// absolute rate, which no artifact of this run carries: [`ProfileReport`] has
/// fire counts for ISOLATED nodes only, and `harvest_costs` drops every edge
/// touching an isolated node, so the `ticker → sink` rate vanishes in exactly
/// that case. Closing it means adding per-node observed fires to the report —
/// an engine change, deliberately not made from a test file.
fn auto_mode_starved_isolation(report: &ProfileReport, expected_costed: &[&str]) -> Option<String> {
    // Uniform mode's target is a caller-supplied scalar, not an observation of
    // this machine — nothing to invert, so nothing is ever excused there.
    if report.fires_override.is_some() {
        return None;
    }
    let offenders: Vec<&_> = report
        .isolated
        .iter()
        .filter(|iso| expected_costed.contains(&iso.node.as_str()))
        .collect();
    if offenders.is_empty() {
        return None;
    }
    let window_s = report.window_ns as f64 / 1e9;
    if window_s <= 0.0 {
        return None;
    }
    let policy = FireTargetPolicy::default();
    let observed_hz = |fires: u64| fires as f64 / window_s;
    // The rate the warm-up ran at, recovered from the target the engine derived
    // from it. Floor-biased LOW by `< 2/window_s` Hz, which is why the
    // comparison below carries a MARGIN instead of testing against nominal —
    // see `STARVED_WARMUP_FRACTION`.
    let warmup_hz = |target: u64| {
        target as f64 * policy.fraction_den as f64 / policy.fraction_num as f64 / window_s
    };
    if !offenders.iter().all(|iso| match iso.target {
        Some(t) => {
            iso.fires < t
                && t < policy.max_samples
                && observed_hz(iso.fires) < LOAD_DEGRADE_HZ
                && warmup_hz(t) < STARVED_WARMUP_HZ
        }
        // "Silent through warm-up" on a node that must fire is a real defect.
        None => false,
    }) {
        return None;
    }
    let detail: Vec<String> = offenders
        .iter()
        .map(|iso| {
            format!(
                "{} (fires {}, target {:?}, {:.2} Hz observed, ~{:.2} Hz through warm-up)",
                iso.node,
                iso.fires,
                iso.target,
                observed_hz(iso.fires),
                iso.target.map(warmup_hz).unwrap_or(f64::NAN),
            )
        })
        .collect();
    Some(format!(
        "this machine never delivered the fixtures' {FIXTURE_NOMINAL_HZ} Hz nominal — every isolation \
         collapsed below {LOAD_DEGRADE_HZ} Hz AND was derived from a warm-up under \
         {STARVED_WARMUP_HZ} Hz ({STARVED_WARMUP_FRACTION} x nominal), so it is a RATE COLLAPSE, \
         not a derivation regression: {}; window {window_s:.2} s",
        detail.join(", "),
    ))
}

// ==========================================================================
// Load-degrade gate self-test: the degrade classifier is itself oracle-pinned, so
// "we skipped under load" can never quietly become "we skip everything". Pure
// (no transport, no `#[serial]` need) — hand-built reports vs hand-written
// expectations, never a self-compare.
// ==========================================================================

#[test]
fn starved_isolation_gate_classifies_only_the_rate_collapse_signature() {
    let iso = |node: &str, fires: u64, target: Option<u64>| {
        cerulion_cli_engine::graph_cmd::IsolatedNodeReport {
            node: node.to_string(),
            fires,
            target,
            starved_trigger_inputs: Vec::new(),
        }
    };
    // A 6 s window: nominal 20 Hz = 120 fires, the degrade line = 60 fires.
    let report =
        |isolated: Vec<cerulion_cli_engine::graph_cmd::IsolatedNodeReport>| ProfileReport {
            artifact_path: PathBuf::from("/dev/null"),
            backup_path: None,
            isolated,
            fires_override: None,
            node_count: 2,
            window_ns: 6_000_000_000,
        };

    // Nothing isolated => the caller asserts its full contract.
    assert!(auto_mode_starved_isolation(&report(vec![]), &ALL_NODES).is_none());

    // Only an UNRELATED node isolated (arm 7/9's `silent`) => not our concern.
    // The fixture must carry a FULLY EXCUSABLE shape so this arm pins the
    // `expected_costed` FILTER itself: delete the filter and `silent` becomes an
    // offender that matches the signature, the gate returns `Some`, and this
    // assert fails. `Some(40)` recovers 40*2/6 = 13.33 Hz — under the 15 Hz
    // margin, so the warm-up half passes; 5 fires / 6 s = 0.83 Hz clears the
    // observed half. (A fixture of `iso("silent", 0, None)` would be
    // rejected by the None-target rule anyway, and `Some(50)`
    // recovers 16.67 Hz, which the margin rejects — either would pass for the
    // wrong reason.)
    assert!(auto_mode_starved_isolation(
        &report(vec![iso("silent", 5, Some(40))]),
        &["ticker", "sink"]
    )
    .is_none());

    // THE load signature, both measured regimes: the original filing's
    // `fires: 13, target: Some(20)` (2.2 Hz) …
    let degraded = auto_mode_starved_isolation(
        &report(vec![iso("ticker", 13, Some(20)), iso("sink", 13, Some(20))]),
        &ALL_NODES,
    )
    .expect("2.2 Hz against a real target is a starved machine");
    assert!(
        degraded.contains(
            "ticker (fires 13, target Some(20), 2.17 Hz observed, ~6.67 Hz through warm-up)"
        ),
        "the degrade diagnostic names the offender, its numbers, its observed rate AND the \
         warm-up rate the excuse rests on; got {degraded}"
    );
    // … and the loaded reproduction (`sink fires=8 target=25`, 0.94 Hz over
    // 8.5 s — the shape whose target is NOT the policy floor, which is exactly
    // why the gate keys on the RATE and not on the floor).
    let long_window = ProfileReport {
        window_ns: 8_470_000_000,
        ..report(vec![iso("sink", 8, Some(25))])
    };
    assert!(
        auto_mode_starved_isolation(&long_window, &ALL_NODES).is_some(),
        "a non-floor target with a collapsed rate is still the load signature"
    );

    // A FABRICATED non-scaling target (the earlier blanket 1000) must NOT be
    // excused. Scope: `1000 == max_samples`, so this pair
    // is refused by the `t < policy.max_samples` guard, which short-circuits
    // before either rate condition — it pins the max_samples guard, not the rate
    // property. The rate property is pinned by the `Some(200)` arm below (a
    // sub-`max_samples` fabrication on a node still at 20 Hz) and by the
    // margin-boundary pair.
    assert!(
        auto_mode_starved_isolation(&report(vec![iso("ticker", 120, Some(1000))]), &ALL_NODES)
            .is_none(),
        "a blanket target AT max_samples carries no rate information — never excused"
    );

    // MIXED: one starved + one at-max_samples fabrication => still a hard fail
    // (the `all()` is over EVERY offender; one non-matching entry refuses).
    assert!(auto_mode_starved_isolation(
        &report(vec![
            iso("ticker", 13, Some(20)),
            iso("sink", 120, Some(1000))
        ]),
        &ALL_NODES
    )
    .is_none());

    // An isolation whose fires MET its target (the no-usable-duration-samples
    // demotion a ring-sizing variant produces) is a real defect: even at a
    // collapsed 1.67 Hz it is NOT excused.
    assert!(
        auto_mode_starved_isolation(&report(vec![iso("ticker", 10, Some(10))]), &ALL_NODES)
            .is_none(),
        "fires == target is the demotion arm, not starvation"
    );

    // A costed-expected node marked SILENT-through-warm-up (target None) is a
    // genuine bug — the fixture ticker always fires.
    assert!(
        auto_mode_starved_isolation(&report(vec![iso("ticker", 0, None)]), &ALL_NODES).is_none(),
        "a silent-marker isolation of a live node is never excused"
    );

    // ---- The asymmetric shape -------------------------
    // A delivery regression that collapses ONE node AFTER warm-up: `sink`
    // observed at 2 Hz (12 fires / 6 s — deep under LOAD_DEGRADE_HZ, so the
    // observed-rate half is fully satisfied) while its target was projected
    // from a HEALTHY warm-up. The machine was fine; the node broke. NOT excused —
    // this is what the warm-up half exists for. Every arm below holds `fires`
    // at 12 so the ONLY variable is the recovered warm-up rate.
    //
    // (a) An IDEAL 20 Hz warm-up: 20 Hz × 6 s ÷ 2 = 60 ⇒ recovers 20.00 Hz.
    assert!(
        auto_mode_starved_isolation(&report(vec![iso("sink", 12, Some(60))]), &ALL_NODES).is_none(),
        "a collapse measured against a NOMINAL warm-up is a regression, not load"
    );
    // (b) The regression arm. `Some(59)` is not a hypothetical, but mind
    // which horizon it comes from: it is what a healthy warm-up projects over
    // the arm-6 shape's CAP horizon (20 warm-up fires over a `w ≈ 1.005 s`
    // poll-quantised window ⇒ `floor(floor(20·6/1.005)/2) = 59`, recovering
    // 19.667 Hz = 0.983 × nominal). A healthy RUN never puts that number in a
    // report — it meets the per-node stop gate at ~3 s, the harvest re-projects
    // the SAME observation over that ~3 s window (target ~29) and isolates
    // nothing at all. 59 reaches `IsolatedNodeReport.target` in exactly the
    // scenario this arm models: a healthy warm-up followed by a post-warm-up
    // collapse, so the gate is never met, the run goes the full 6 s cap, and the
    // harvest horizon IS the cap. The RATE it encodes is a healthy machine's own
    // signature — which the first gate, comparing against nominal with no
    // margin, excused, and with it the whole asymmetric shape. It must be
    // refused. Reverting `STARVED_WARMUP_HZ` to `FIXTURE_NOMINAL_HZ` fails
    // exactly here.
    assert!(
        auto_mode_starved_isolation(&report(vec![iso("sink", 12, Some(59))]), &ALL_NODES).is_none(),
        "19.667 Hz through warm-up is what a HEALTHY machine recovers — never a slow machine"
    );

    // ---- THE MARGIN BOUNDARY (both sides REACHABLE by the engine) ---------
    // The cut is `warmup_hz(t) < STARVED_WARMUP_HZ`, i.e. `2t/W < 15` at
    // `W = 6 s` ⇒ `t < 45`. Both fixtures are producible by the real engine
    // arithmetic `t = clamp(floor(floor(F·W/w)·num/den), 20, 1000)`, but they
    // come from DIFFERENT warm-up regimes — `t = 45` needs
    // `floor(F·6/w) ∈ {90, 91}`, which `F = 15` reaches only at `w <= 1.000 s`,
    // a window the poll model above (`w` STRICTLY overshoots its nominal 1.000 s)
    // rules out:
    //
    //   t = 44  ⟸  F = 15 warm-up fires over w = 1.005 s  (15·6/1.005 = 89.55
    //              → floor 89 → 44)  — the ORDINARY 10 ms-quantised overshoot,
    //              i.e. a warm-up just under the cut on any machine;
    //   t = 45  ⟸  F = 16 warm-up fires over w ∈ (1.0435, 1.0667] s, e.g.
    //              w = 1.050 s  (16·6/1.050 = 91.43 → floor 91 → 45)  — a
    //              warm-up at EXACTLY 0.75 × nominal, requiring a window that
    //              overshot its nominal 1.000 s by >= 4.4 %. The 10 ms poll
    //              alone does not stretch that far, but a LOADED machine — the
    //              exact regime this gate exists for — routinely does (a single
    //              scheduler preemption of the watcher thread is enough).
    //
    // Adjacent integers straddling the cut, with the oracle's `fires` held at 12
    // so the recovered warm-up rate is the only variable, and no clamp involved
    // (both are strictly inside `[min_samples, max_samples]`) — so the pair
    // isolates the margin and nothing else. Deleting the margin flips the (b)
    // `Some(59)` arm above AND this `Some(45)` boundary; the (a) `Some(60)`
    // ideal-warm-up arm SURVIVES the revert (`20.0 < 20.0` is false), so do not
    // reach for it as the mutation witness.
    assert!(
        auto_mode_starved_isolation(&report(vec![iso("sink", 12, Some(45))]), &ALL_NODES).is_none(),
        "recovered exactly 15.00 Hz == STARVED_WARMUP_HZ: the comparison is strict `<`, refused"
    );
    assert!(
        auto_mode_starved_isolation(&report(vec![iso("sink", 12, Some(44))]), &ALL_NODES).is_some(),
        "recovered 14.67 Hz is under the margin — a genuinely starved warm-up plus a \
         collapsed window is the starved-machine signature"
    );

    // A node still running at NOMINAL is never excused whatever its target —
    // the plain-language property the gate is FOR.
    assert!(
        auto_mode_starved_isolation(&report(vec![iso("ticker", 120, Some(200))]), &ALL_NODES)
            .is_none(),
        "20 Hz observed is a healthy machine; isolating it is a real defect"
    );

    // DRIFT TRIPWIRE for the doc's "condition 2 implies condition 1" claim: the
    // implication holds exactly while `STARVED_WARMUP_HZ × num/den <=
    // LOAD_DEGRADE_HZ` (`fires < target` + `target < STARVED_WARMUP_HZ × window
    // × num/den` ⇒ `observed < STARVED_WARMUP_HZ × num/den` = 7.5 Hz). While it
    // holds, NO oracle case can attribute a refusal to the observed-rate half
    // alone — which is why the boundary pair above moves the WARM-UP number, not
    // the fire count. The margin only STRENGTHENS the implication (7.5 <= 10 vs
    // the first gate's 10 <= 10 equality), but the assert is re-anchored on the
    // constant the predicate actually uses so retuning EITHER fails here,
    // forcing the doc + the pair to be revisited.
    let policy = FireTargetPolicy::default();
    assert!(
        STARVED_WARMUP_HZ * policy.fraction_num as f64 / policy.fraction_den as f64
            <= LOAD_DEGRADE_HZ,
        "the classifier doc claims the warm-up condition implies the observed-rate \
         condition; that only holds while STARVED_WARMUP_HZ x num/den <= LOAD_DEGRADE_HZ"
    );
    // (The companion "the margin must be a REAL one" guard — `fraction < 1.0` —
    // is a COMPILE-TIME `const _: () = assert!(..)` next to the constant itself,
    // so a retune past 1.0 fails the build rather than this test.)

    // A zero/absurd window can never be read as a rate — never excused.
    let zero_window = ProfileReport {
        window_ns: 0,
        ..report(vec![iso("ticker", 0, Some(20))])
    };
    assert!(auto_mode_starved_isolation(&zero_window, &ALL_NODES).is_none());
}

// ==========================================================================
// Arm 1: gate-met happy path.
// ==========================================================================

#[test]
#[serial]
fn gate_met_profile_writes_full_artifact_no_isolated() {
    let _guard = TargetDirGuard::clear();
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = tmp.path();
    build_workspace(ws, &unique_prefix("a"));

    // period 50ms → 5 fires ≈ 250ms; the 30s cap is a generous backstop, the
    // GATE is what stops this run.
    let running = Arc::new(AtomicBool::new(true));
    let report = graph_profile(
        ws,
        "demo",
        Arc::clone(&running),
        Duration::from_secs(30),
        Some(5),
        None,
    )
    .expect("gate-met profile run succeeds");

    // The gate stopped the run: no node is isolated.
    assert!(
        report.isolated.is_empty(),
        "every node reached the target — no isolation; got {:?}",
        report.isolated
    );
    assert_eq!(report.node_count, 2);
    assert!(report.window_ns > 0, "a real observation window elapsed");

    // Artifact at the DEFAULT path, parseable, complete.
    let expected_path = default_artifact_path(ws, "demo");
    assert_eq!(report.artifact_path, expected_path);
    let artifact = read_artifact(&expected_path);
    assert_eq!(artifact.version, 2, "the profiler writes v2");
    assert_eq!(artifact.graph, "demo");
    assert_totality(&artifact);
    assert!(artifact.isolated.is_empty());

    // With both nodes costed (Σ p50 > 0) the default budget is
    // FROZEN into the artifact together with its core-count provenance. The
    // values are machine-dependent (real p50s / real cores) — pin presence +
    // internal consistency, not literals.
    let frozen_budget = artifact
        .derived_budget_ns
        .expect("a costed profile freezes derived_budget_ns");
    let frozen_cores = artifact
        .profile_cores
        .expect("the frozen budget carries its core-count provenance");
    assert!(frozen_budget > 0 && frozen_cores > 0);
    let total: u64 = artifact.nodes.values().sum();
    assert_eq!(
        frozen_budget,
        total.div_ceil(frozen_cores as u64).max(1),
        "the frozen budget is exactly ceil(Σ p50 / cores) of THIS artifact"
    );

    // Both nodes costed with a REAL (nonzero) observed p50 — duration
    // recording was on and the trace carried samples.
    for node in ALL_NODES {
        let p50 = artifact
            .nodes
            .get(node)
            .unwrap_or_else(|| panic!("node '{node}' must carry a p50 cost"));
        assert!(
            *p50 > 0,
            "node '{node}' p50 must be a real measured duration"
        );
    }

    // The (ticker → sink) trigger edge is present with a plausible rate: the
    // producer runs at 20 Hz (period 50ms) = 20_000 mHz. Wall jitter and the
    // window boundary make the exact value machine-dependent — assert the sane
    // band (nonzero, under 10× nominal), not equality.
    assert_eq!(artifact.edges.len(), 1, "exactly the one trigger edge");
    let edge = &artifact.edges[0];
    assert_eq!(
        (edge.producer.as_str(), edge.consumer.as_str()),
        ("ticker", "sink")
    );
    assert!(
        edge.rate_mhz > 0 && edge.rate_mhz < 200_000,
        "edge rate must be a real observed rate near 20_000 mHz; got {}",
        edge.rate_mhz
    );

    // The self-contained hop block carries THIS machine's platform defaults.
    let hop = artifact
        .hop
        .expect("the profiler always writes the hop block");
    let expected_hop = HopCosts::platform_default();
    assert_eq!(hop.intra_ns, expected_hop.intra_ns);
    assert_eq!(hop.cross_ns, expected_hop.cross_ns);
}

// ==========================================================================
// Arm 2: cap-hit adversarial — unreachable target, artifact still written.
// ==========================================================================

#[test]
#[serial]
fn cap_hit_isolates_undersampled_nodes_and_still_writes_artifact() {
    let _guard = TargetDirGuard::clear();
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = tmp.path();
    build_workspace(ws, &unique_prefix("b"));

    // 100_000 fires at 20 Hz would need ~83 minutes — the 1.5s cap stops the
    // run first, leaving BOTH nodes under-sampled.
    let running = Arc::new(AtomicBool::new(true));
    let report = graph_profile(
        ws,
        "demo",
        Arc::clone(&running),
        Duration::from_millis(1500),
        Some(100_000),
        None,
    )
    .expect("a cap-hit run is NOT an error — the partial profile is the artifact");

    let expected_isolated: Vec<String> = ALL_NODES.iter().map(|s| s.to_string()).collect();
    let isolated_nodes: Vec<String> = report.isolated.iter().map(|i| i.node.clone()).collect();
    assert_eq!(
        isolated_nodes, expected_isolated,
        "every node fell short of the unreachable target"
    );
    assert_eq!(report.fires_override, Some(100_000));
    for iso in &report.isolated {
        assert_eq!(
            iso.target,
            Some(100_000),
            "uniform mode: every isolated entry carries the scalar target"
        );
        assert!(
            iso.fires < 100_000,
            "isolated node '{}' must report an under-target fire count; got {}",
            iso.node,
            iso.fires
        );
    }

    // The artifact is STILL written — durable and accurate: no costs, no
    // edges (both endpoints isolated), the full isolated list.
    let artifact = read_artifact(&report.artifact_path);
    assert_totality(&artifact);
    assert!(
        artifact.nodes.is_empty(),
        "no node may carry a fabricated cost; got {:?}",
        artifact.nodes
    );
    assert!(
        artifact.edges.is_empty(),
        "an edge touching an isolated node is omitted; got {:?}",
        artifact.edges
    );
    assert_eq!(artifact.isolated, expected_isolated);
    // Nothing costed (total 0) => NO frozen budget, no cores
    // provenance — the degenerate case is never smuggled into the file.
    assert_eq!(
        artifact.derived_budget_ns, None,
        "an all-isolated profile freezes no budget"
    );
    assert_eq!(artifact.profile_cores, None);
}

// ==========================================================================
// Arm 3: shape determinism across two runs (+ the out_path override).
// ==========================================================================

#[test]
#[serial]
fn two_runs_agree_on_artifact_shape() {
    let _guard = TargetDirGuard::clear();
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = tmp.path();
    build_workspace(ws, &unique_prefix("c"));

    let out_a = ws.join("run_a.costs.yaml");
    let out_b = ws.join("run_b.costs.yaml");
    for out in [&out_a, &out_b] {
        let running = Arc::new(AtomicBool::new(true));
        let report = graph_profile(
            ws,
            "demo",
            running,
            Duration::from_secs(30),
            Some(5),
            Some(out),
        )
        .expect("profile run succeeds");
        // out_path override wins over the default location.
        assert_eq!(&report.artifact_path, out);
    }
    assert!(
        !default_artifact_path(ws, "demo").exists(),
        "with out_path given, nothing lands at the default location"
    );

    let a = read_artifact(&out_a);
    let b = read_artifact(&out_b);
    // STRUCTURE agrees run-to-run (the deterministic part of a live profile);
    // p50/rate VALUES are wall measurements and legitimately differ — the
    // value-level recording-on/off firewall is pinned by the core B-dur suite.
    assert_eq!(
        a.nodes.keys().collect::<Vec<_>>(),
        b.nodes.keys().collect::<Vec<_>>(),
        "same costed-node key set"
    );
    let pairs = |art: &ProfileArtifact| -> Vec<(String, String)> {
        art.edges
            .iter()
            .map(|e| (e.producer.clone(), e.consumer.clone()))
            .collect()
    };
    assert_eq!(pairs(&a), pairs(&b), "same trigger-edge pair set");
    assert_eq!(a.isolated, b.isolated, "same isolated set");
    assert_eq!(a.graph, b.graph);
    assert_eq!(a.version, b.version);
    assert_totality(&a);
    assert_totality(&b);
}

// ==========================================================================
// Arm 4: a pre-stopped `running` (the Ctrl-C composition seam) still yields
// a durable artifact.
// ==========================================================================

#[test]
#[serial]
fn pre_stopped_running_flag_still_writes_isolated_artifact() {
    let _guard = TargetDirGuard::clear();
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = tmp.path();
    build_workspace(ws, &unique_prefix("d"));

    // `running` is ALREADY false — the same flag a Ctrl-C handler flips.
    // `run_live` returns immediately; the harvest sees zero fires everywhere.
    let running = Arc::new(AtomicBool::new(false));
    let report = graph_profile(ws, "demo", running, Duration::from_secs(30), Some(5), None)
        .expect("an externally-stopped run still completes the harvest + write");

    let expected_isolated: Vec<String> = ALL_NODES.iter().map(|s| s.to_string()).collect();
    let isolated_nodes: Vec<String> = report.isolated.iter().map(|i| i.node.clone()).collect();
    assert_eq!(
        isolated_nodes, expected_isolated,
        "zero fires anywhere — everything isolated"
    );
    for iso in &report.isolated {
        assert_eq!(iso.fires, 0, "a pre-stopped run observes zero fires");
    }
    let artifact = read_artifact(&report.artifact_path);
    assert_totality(&artifact);
    assert!(artifact.nodes.is_empty());
    assert_eq!(artifact.isolated, expected_isolated);
}

// ==========================================================================
// Arm 5: the CAPPED-RING warn + newest-retained-samples p50
// path. A heterogeneous graph: the ticker→sink pair floods fires while a
// THIRD data-trigger node on a producer-less absolute external source
// (`/ext/...`, the absolute-source validation exemption) NEVER fires, so
// the gate is never met and the run rides the full duration cap. With
// fires_target=3 the ring is sized 3 nodes × 3 × 4 = 36 entries, while the
// 20 Hz ticker + its sink produce ~40 fires/s over the ~4 s cap (~160 total)
// — the run out-fires the ring. Pins: the loud capped-ring warn fired, the
// flooding nodes' costs are STILL computed (p50 over the NEWEST retained
// samples), the silent node isolates, and the artifact still writes.
// ==========================================================================

/// The 3-node workspace for the capped-ring arm: ticker → sink (both fire at
/// 20 Hz) + `silent` (a second instance of the sink's data-trigger type whose
/// trigger input reads a producer-less absolute external topic — never fires).
fn build_workspace_with_silent(root: &Path, prefix: &str) {
    build_workspace(root, prefix);
    std::fs::write(
        root.join("graphs/demo.yaml"),
        format!(
            "name: demo\nprefix: {prefix}\nnodes:\n\
             - id: ticker\n  type: ticker\n  inputs: []\n  outputs:\n\
             \x20 - name: cmd\n    schema: geometry_msgs/Vector3\n\
             - id: sink\n  type: sink\n  inputs:\n\
             \x20 - name: trigger_in\n    source: ticker/cmd\n  outputs:\n\
             \x20 - name: cmd\n    schema: geometry_msgs/Vector3\n\
             - id: silent\n  type: sink\n  inputs:\n\
             \x20 - name: trigger_in\n    source: /ext/{prefix}/nobody\n  outputs:\n\
             \x20 - name: cmd\n    schema: geometry_msgs/Vector3\n"
        ),
    )
    .unwrap();
}

#[tracing_test::traced_test]
#[test]
#[serial]
fn capped_ring_warns_and_still_costs_flooding_nodes() {
    let _guard = TargetDirGuard::clear();
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = tmp.path();
    build_workspace_with_silent(ws, &unique_prefix("e"));

    // fires_target 3 → ring = 3 nodes × 3 × 4 = 36 entries. `silent` never
    // fires (producer-less external source), so the gate is never met and the
    // run rides the 4 s cap while ticker+sink flood ~160 fires >> 36.
    let running = Arc::new(AtomicBool::new(true));
    let report = graph_profile(
        ws,
        "demo",
        Arc::clone(&running),
        Duration::from_secs(4),
        Some(3),
        None,
    )
    .expect("a capped-ring run is not an error");

    // The loud eviction warn fired (fire counters are cap-immune, so the
    // total-fires > retained-entries comparison is exact).
    assert!(
        logs_contain("trace ring capped"),
        "the capped-ring warn must fire when the run out-fires the ring"
    );
    // The remediation must name the REAL flag (`--fires`); the exact phrase is
    // pinned so a regression to a nonexistent flag name (such as
    // `--fires-target`) fails here instead of reaching users. Note:
    // the phrase is mode-neutral ("set --fires N", not "raise") — in AUTO
    // mode there is no existing --fires value to raise, but an explicit
    // `--fires N` sizes the ring to N in either mode.
    assert!(
        logs_contain("set --fires N for a larger ring"),
        "the capped-ring remediation must name the real --fires flag"
    );

    // Only the silent node isolated; the flooding pair is STILL costed from
    // the newest retained samples.
    let isolated_nodes: Vec<String> = report.isolated.iter().map(|i| i.node.clone()).collect();
    assert_eq!(
        isolated_nodes,
        vec!["silent".to_string()],
        "only the never-firing node isolates"
    );

    let artifact = read_artifact(&report.artifact_path);
    for node in ["ticker", "sink"] {
        let p50 = artifact
            .nodes
            .get(node)
            .unwrap_or_else(|| panic!("flooding node '{node}' must still be costed"));
        assert!(
            *p50 > 0,
            "node '{node}' p50 must come from real retained samples; got {p50}"
        );
    }
    assert_eq!(artifact.isolated, vec!["silent".to_string()]);
    // Totality over the 3-node graph: 2 costed + 1 isolated, no overlap.
    assert_eq!(artifact.nodes.len(), 2);
    assert!(!artifact.nodes.contains_key("silent"));
    // The in-graph trigger edge survives (neither endpoint isolated); the
    // silent node's external-source edge has no in-graph producer => no pair.
    assert_eq!(artifact.edges.len(), 1);
    assert_eq!(artifact.edges[0].producer, "ticker");
    assert_eq!(artifact.edges[0].consumer, "sink");
}

// ==========================================================================
// Arm 6: AUTO-derive mode costs a LOW-RATE graph the
// uniform default would have blanket-isolated. `fires_override = None` over
// a 6 s cap: warm-up = clamp(6s/10, 1s, 3s) = 1 s, during which the 20 Hz
// ticker (period 50 ms) fires ~20× → derived target ≈ 20 × 6 / 1 / 2 = 60
// (well inside [20, 1000]) — reached in ~3 s, so BOTH nodes are costed. The
// earlier uniform default (1000 fires) needed 50 s > any short cap and
// would have isolated the whole graph.
// ==========================================================================

#[test]
#[serial]
fn auto_derive_costs_low_rate_graph_not_isolated() {
    let _guard = TargetDirGuard::clear();
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = tmp.path();
    build_workspace(ws, &unique_prefix("f"));

    // The observation cap for the run, and nothing else.
    const CAP: Duration = Duration::from_secs(6);
    let running = Arc::new(AtomicBool::new(true));
    let report = graph_profile(ws, "demo", Arc::clone(&running), CAP, None, None)
        .expect("auto-derive profile run succeeds");

    assert_eq!(
        report.fires_override, None,
        "the report says which mode gated the run"
    );
    // ---- LOAD-INSENSITIVE half: asserted UNCONDITIONALLY -----
    // None of these constrain WHICH nodes were costed, so a degraded machine still
    // pins them; only the "nothing was isolated" claim below is load-sensitive.
    assert_eq!(report.node_count, 2);
    assert!(report.window_ns > 0, "a real observation window elapsed");
    let artifact = read_artifact(&report.artifact_path);
    assert_totality(&artifact);
    assert_no_phantom_edges(&artifact);
    // Costed or not, an emitted p50 is a REAL measurement — never a fabricated
    // zero (Principle #13). Independent of how many nodes made the gate.
    for (node, p50) in &artifact.nodes {
        assert!(
            *p50 > 0,
            "costed node '{node}' p50 must be a real measured duration; got {p50}"
        );
    }

    // ---- LOAD-SENSITIVE half ---------------------------------------------
    // A machine too loaded to deliver the policy floor inside the cap
    // cannot exercise this contract — skip LOUDLY, never silently green (the
    // marker needs `--nocapture`; see the module header). Any other isolation
    // shape falls through to the assertions below.
    if let Some(why) = auto_mode_starved_isolation(&report, &ALL_NODES) {
        eprintln!("LOAD DEGRADE auto_derive_costs_low_rate_graph_not_isolated: {why}");
        return;
    }
    // The headline: the low-rate nodes are COSTED, not isolated — the per-node
    // derived targets scaled to the observed 20 Hz rate.
    assert!(
        report.isolated.is_empty(),
        "auto-derived targets are reachable at the node's own rate — no \
         isolation; got {:?}",
        report.isolated
    );
    assert!(artifact.isolated.is_empty());
    for node in ALL_NODES {
        assert!(
            artifact.nodes.contains_key(node),
            "node '{node}' must carry a p50 cost"
        );
    }
    // The trigger edge survives with a real observed rate.
    assert_eq!(artifact.edges.len(), 1, "exactly the one trigger edge");
    assert!(artifact.edges[0].rate_mhz > 0);
}

// ==========================================================================
// Arm 7: AUTO-derive + a silent node. The 3-node
// silent-workspace under `fires_override = None`: `silent` never fires
// (producer-less absolute external source), so it derives NO target at
// warm-up end — it is EXCLUDED from the stop gate (the live pair's targets
// stop the run early, the silent node cannot hold it to the cap) and
// isolates with the `target: None` "silent through warm-up" marker + the
// starved-trigger hint naming its zero-rate trigger input. The live pair is
// costed.
// ==========================================================================

#[tracing_test::traced_test]
#[test]
#[serial]
fn auto_derive_silent_node_isolates_with_no_target_marker_and_starved_hint() {
    let _guard = TargetDirGuard::clear();
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = tmp.path();
    build_workspace_with_silent(ws, &unique_prefix("g"));

    // The observation cap for the run (see arm 6).
    const CAP: Duration = Duration::from_secs(6);
    let running = Arc::new(AtomicBool::new(true));
    let report = graph_profile(ws, "demo", Arc::clone(&running), CAP, None, None)
        .expect("an auto-derive run with a silent node is not an error");

    // The silent node isolates, with the no-target marker (silent through
    // warm-up ⇒ no rate to project ⇒ NO fabricated target number). Note:
    // this half is load-INSENSITIVE — the node fires zero times on any machine —
    // so it is asserted in full BEFORE the live pair's load-sensitive gate.
    assert_eq!(report.fires_override, None);
    let silent = report
        .isolated
        .iter()
        .find(|i| i.node == "silent")
        .unwrap_or_else(|| {
            panic!(
                "the never-firing node must isolate; isolated was {:?}",
                report.isolated
            )
        });
    assert_eq!(silent.fires, 0, "the silent node never fired");
    assert_eq!(
        silent.target, None,
        "silent through warm-up => NO derived target (the distinct marker, \
         never a fabricated number)"
    );

    // The starved-trigger hint: the node never fired AND its triggering
    // input's topic saw a zero harvested rate (producer-less absolute
    // source) — the report names the input + topic.
    assert_eq!(
        silent.starved_trigger_inputs.len(),
        1,
        "exactly the one starved triggering input; got {:?}",
        silent.starved_trigger_inputs
    );
    assert!(
        silent.starved_trigger_inputs[0].starts_with("trigger_in (topic "),
        "the hint names the input port + its topic; got {:?}",
        silent.starved_trigger_inputs[0]
    );
    assert!(
        silent.starved_trigger_inputs[0].contains("nobody"),
        "the hint names the never-published topic; got {:?}",
        silent.starved_trigger_inputs[0]
    );

    // The loud warn surface: the silent-through-warm-up marker + the
    // starved-trigger remediation hint both reached the logs.
    assert!(
        logs_contain("silent through warm-up"),
        "the isolation warn must carry the no-target marker"
    );
    assert!(
        logs_contain("an upstream goal/driver-fed input may be silent"),
        "the starved-trigger hint must fire for the 0-fire node"
    );

    // The production warm-up snapshot really is
    // anchored at each node's first fire — the no-inert-shipping pin, and
    // load-INSENSITIVE (a 1 s warm-up under a 6 s cap completes on any machine, and
    // `all_targets_met` is FALSE until it does, so the run cannot stop first).
    //
    // The arithmetic itself is oracle-pinned purely (`graph_profile_test.rs`
    // for the anchoring, `auto_partition_test.rs` for the projection); what no
    // pure arm can see is whether the WATCHER feeds them. `burst_fires_excluded`
    // is computed from the sightings, so a watcher reverted to observing every
    // node over the full elapsed wall emits no such field at all. It is
    // structurally `>= 1` per anchored node (a sighting exists only because the
    // counter moved), and this workspace has two live nodes.
    logs_assert(|lines: &[&str]| {
        let line = lines
            .iter()
            .find(|l| l.contains("graph profile: warm-up complete"))
            .ok_or_else(|| "the warm-up-complete line never appeared".to_string())?;
        let excluded = line
            .split_whitespace()
            .find_map(|tok| tok.strip_prefix("burst_fires_excluded="))
            .ok_or_else(|| {
                format!(
                    "the warm-up snapshot must account for the burst it excluded \
                     — no `burst_fires_excluded=` field on: {line}"
                )
            })?;
        let excluded: u64 = excluded
            .parse()
            .map_err(|_| format!("`burst_fires_excluded` is not a number on: {line}"))?;
        if excluded < 2 {
            return Err(format!(
                "two live nodes each contribute at least their first fire, so at least \
                 2 fires must have been excluded; got {excluded} on: {line}"
            ));
        }
        Ok(())
    });

    // The ARTIFACT half of the silent-node contract is equally
    // load-INSENSITIVE (a node that never fires is never costed on any machine),
    // as are totality over the 3-node graph and the no-phantom-edge shape.
    let artifact = read_artifact(&report.artifact_path);
    assert_totality_over(&artifact, &SILENT_WS_NODES);
    assert_no_phantom_edges(&artifact);
    assert!(
        artifact.isolated.contains(&"silent".to_string()),
        "the never-firing node is isolated in the artifact too; got {:?}",
        artifact.isolated
    );
    assert!(
        !artifact.nodes.contains_key("silent"),
        "a never-firing node must carry NO fabricated cost"
    );
    for (node, p50) in &artifact.nodes {
        assert!(*p50 > 0, "costed node '{node}' p50 must be real; got {p50}");
    }

    // The arm's structural claim about the LIVE pair, split out of
    // the set equality below because THIS half is load-INSENSITIVE. `silent`
    // must be the only node that failed to DERIVE a target: any other
    // isolation has to be a RATE judgement (`target: Some(_)`, fires short of
    // it), which a slow machine can legitimately produce, and never the
    // `target: None` marker — which would mean the silent node's zero warm-up
    // rate POISONED its neighbours' derivation (the derivation bailing for the
    // whole graph rather than skipping the one silent node). First-fire anchoring makes
    // this stricter still: MEMBERSHIP of the observation map is the evidence a
    // node fired, so a live node reported as `target: None` means the watcher
    // never anchored it at all. A regression there fails on any machine, so it is
    // before the gate rather than behind it.
    for iso in &report.isolated {
        if iso.node == "silent" {
            continue;
        }
        assert!(
            iso.target.is_some(),
            "only the never-firing node may carry the no-target marker — '{}' was isolated \
             WITHOUT a derived target, so the silent node poisoned the derivation; isolated \
             was {:?}",
            iso.node,
            report.isolated
        );
    }

    // The live pair is COSTED — the silent node neither held the gate open
    // to the cap nor dragged its neighbors into isolation. Note: only
    // reachable on a machine that delivered the policy floor, so a genuinely
    // starved run skips LOUDLY here (any other isolation shape still fails
    // below). A slow BRING-UP needs no excuse: the engine anchors each
    // node's warm-up at its first fire, so the catch-up burst is outside the
    // observation entirely.
    if let Some(why) = auto_mode_starved_isolation(&report, &["ticker", "sink"]) {
        eprintln!(
            "LOAD DEGRADE \
             auto_derive_silent_node_isolates_with_no_target_marker_and_starved_hint: {why}"
        );
        return;
    }
    let isolated_nodes: Vec<String> = report.isolated.iter().map(|i| i.node.clone()).collect();
    assert_eq!(
        isolated_nodes,
        vec!["silent".to_string()],
        "only the never-firing node isolates"
    );
    for node in ["ticker", "sink"] {
        assert!(
            artifact.nodes.contains_key(node),
            "live node '{node}' must be costed"
        );
    }
    // Totality over the 3-node graph: 2 costed + 1 isolated, no overlap.
    assert_eq!(artifact.nodes.len(), 2);
    assert_eq!(artifact.isolated, vec!["silent".to_string()]);
    assert_eq!(artifact.edges.len(), 1);
}

// ==========================================================================
// Arm 8: AUTO-mode EARLY STOP — a mid-run Ctrl-C after warm-up
// costs the sampled nodes instead of isolating them. Cap 20 s ⇒ warm-up 2 s;
// a stopper thread flips `running` at ~5 s (the Ctrl-C seam, crib of arm 4
// but MID-run). At 20 Hz: warm-up ≈ 40 fires ⇒ the watcher's CAP-horizon
// stop-gate target ≈ 40×20/2/2 = 200 (unreachable by t=5 s — the stopper
// wins, so this run IS an early stop, not gate-met); the WINDOW-horizon
// harvest target ≈ 40×5.5/2/2 ≈ 55 ≤ the ~100 observed fires ⇒ COSTED.
// Gating harvest against the cap-horizon 200 instead would leave everything
// isolated — this arm fails on that regression. The no-warm-up fallback
// breadcrumb must NOT appear (a snapshot-discarded variant would emit it and
// still cost via clamp(fires/2), so the breadcrumb-absence assert is the
// only defense against it).
// ==========================================================================

#[tracing_test::traced_test]
#[test]
#[serial]
fn auto_mode_early_stop_costs_sampled_nodes_not_isolated() {
    let _guard = TargetDirGuard::clear();
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = tmp.path();
    build_workspace(ws, &unique_prefix("h"));

    // The Ctrl-C seam: the SAME shared flag a SIGINT handler would flip,
    // flipped mid-run by a stopper thread — after the 2 s warm-up completes,
    // well before the 20 s cap.
    let running = Arc::new(AtomicBool::new(true));
    let stopper_flag = Arc::clone(&running);
    let stopper = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_secs(5));
        stopper_flag.store(false, std::sync::atomic::Ordering::Relaxed);
    });

    // The observation cap for the run (see arm 6).
    const CAP: Duration = Duration::from_secs(20);
    let report = graph_profile(ws, "demo", Arc::clone(&running), CAP, None, None)
        .expect("a mid-run Ctrl-C is not an error");
    stopper.join().expect("stopper thread joins");

    // The early stop really happened (the stopper, not the cap): the window
    // is far short of the 20 s cap (generous bound for slow machines).
    assert!(
        report.window_ns < 18_000_000_000,
        "the stopper must end the run early; window was {} ns",
        report.window_ns
    );

    assert_eq!(report.fires_override, None);

    // ---- LOAD-INSENSITIVE half: asserted UNCONDITIONALLY -----
    // Warm-up COMPLETED (a 2 s WALL warm-up under a 5 s WALL stopper — true on
    // any machine regardless of how fast the nodes fired), so the harvest
    // re-projected the watcher's snapshot and the no-warm-up fallback
    // breadcrumb must be absent. A variant that discards the watcher's snapshot
    // (`unwrap_or_default`) falls into the fallback arm and emits it — while
    // still costing via clamp(fires/2), which is why the costed asserts alone
    // cannot catch it. THIS is the arm's only defense against it, so it must never
    // sit behind a load gate.
    assert!(
        !logs_contain("run ended before warm-up completed"),
        "warm-up completed — the no-warm-up fallback must not have run"
    );
    let artifact = read_artifact(&report.artifact_path);
    assert_totality(&artifact);
    assert_no_phantom_edges(&artifact);
    for (node, p50) in &artifact.nodes {
        assert!(*p50 > 0, "costed node '{node}' p50 must be real; got {p50}");
    }

    // ---- LOAD-SENSITIVE half ---------------------------------------------
    // THE window-horizon pin: sampled nodes are COSTED — isolation is judged against the
    // ACTUAL window, not the cap-length projection.
    // On a machine too slow to reach the policy floor inside the ~5.5 s
    // window, "well-sampled" is counterfactual — skip LOUDLY. A cap-horizon
    // regression isolates a node still firing at its healthy nominal rate, so
    // the starved gate REFUSES to excuse it and the arm still fails.
    if let Some(why) = auto_mode_starved_isolation(&report, &ALL_NODES) {
        eprintln!("LOAD DEGRADE auto_mode_early_stop_costs_sampled_nodes_not_isolated: {why}");
        return;
    }
    assert!(
        report.isolated.is_empty(),
        "a mid-run stop must not spuriously isolate well-sampled nodes \
         (cap-horizon harvest gating regression); got {:?}",
        report.isolated
    );
    for node in ALL_NODES {
        assert!(
            artifact.nodes.contains_key(node),
            "node '{node}' must be costed after an early stop"
        );
    }
}

// ==========================================================================
// Arm 9: AUTO-mode ring sizing from the derivation CEILING.
// The same 3-node silent-workspace as arm 5, but in AUTO mode: the ring is
// max_samples × 3 nodes × 4 = 12 000 entries, while the 4 s cap floods only
// ~160 fires (and the stop gate ends the run even earlier, ≈2 s) — so the
// capped-ring warn MUST NOT fire and the flooding pair still costs. NOTE:
// the warn itself is UNREACHABLE in auto mode at test scale (it would take
// > 12 000 fires inside the cap), so this arm pins its ABSENCE + costing —
// which still catches a `unwrap_or(0)` ring-sizing regression: ring collapses to
// max(0×3×4, 1) = 1 entry ⇒ the warn FIRES (absence assert dies) AND one
// flooding node loses every duration sample ⇒ demoted to isolated (the
// costed assert dies too).
// ==========================================================================

#[tracing_test::traced_test]
#[test]
#[serial]
fn auto_mode_ring_sized_from_max_samples_no_capped_warn() {
    let _guard = TargetDirGuard::clear();
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = tmp.path();
    build_workspace_with_silent(ws, &unique_prefix("i"));

    // The observation cap for the run (see arm 6).
    // AUTO-derive: ring sized from FireTargetPolicy max_samples.
    const CAP: Duration = Duration::from_secs(4);
    let running = Arc::new(AtomicBool::new(true));
    let report = graph_profile(ws, "demo", Arc::clone(&running), CAP, None, None)
        .expect("an auto-mode run over the silent workspace is not an error");

    // ---- LOAD-INSENSITIVE half: asserted UNCONDITIONALLY -----
    // The max_samples-sized ring absorbs the whole run — no eviction warn. A
    // SLOWER machine floods FEWER fires, so warn-absence can only get easier under
    // load: this half of the `unwrap_or(0)` ring-sizing check (ring
    // collapses to 1 entry ⇒ the warn FIRES) is never gated.
    assert!(
        !logs_contain("trace ring capped"),
        "auto-mode ring (max_samples × nodes × 4) must not cap on ~160 fires"
    );
    assert_eq!(report.fires_override, None);
    let artifact = read_artifact(&report.artifact_path);
    assert_totality_over(&artifact, &SILENT_WS_NODES);
    assert_no_phantom_edges(&artifact);
    // The never-firing node is isolated + un-costed on every machine.
    assert!(
        report.isolated.iter().any(|i| i.node == "silent"),
        "the never-firing node must isolate; got {:?}",
        report.isolated
    );
    assert!(artifact.isolated.contains(&"silent".to_string()));
    assert!(
        !artifact.nodes.contains_key("silent"),
        "a never-firing node must carry NO fabricated cost"
    );
    for (node, p50) in &artifact.nodes {
        assert!(*p50 > 0, "costed node '{node}' p50 must be real; got {p50}");
    }

    // ---- LOAD-SENSITIVE half ---------------------------------------------
    // The flooding pair is costed from intact samples; only the silent node
    // isolates (no target derived). Note: a machine that never reached the
    // policy floor cannot show this — skip LOUDLY. The `unwrap_or(0)`
    // ring-sizing regression demotes a node for lack of duration
    // SAMPLES (fires met its target), never for floor starvation, so it is
    // still caught here.
    if let Some(why) = auto_mode_starved_isolation(&report, &["ticker", "sink"]) {
        eprintln!("LOAD DEGRADE auto_mode_ring_sized_from_max_samples_no_capped_warn: {why}");
        return;
    }
    let isolated_nodes: Vec<String> = report.isolated.iter().map(|i| i.node.clone()).collect();
    assert_eq!(
        isolated_nodes,
        vec!["silent".to_string()],
        "only the never-firing node isolates"
    );
    for node in ["ticker", "sink"] {
        assert!(
            artifact.nodes.contains_key(node),
            "flooding node '{node}' must be costed"
        );
    }
    assert_eq!(artifact.nodes.len(), 2);
    assert_eq!(artifact.isolated, vec!["silent".to_string()]);
}

/// The class sibling of `graph run --no-validate`: `graph profile`
/// EXECUTES the graph without the validation report, so an ambiguous
/// `schema:` — one the workspace defines more than once — is refused by the
/// lookup itself (`refuse_ambiguous_output_schemas`), naming the port and
/// every source, before any node library is loaded (such a
/// spelling has no meaning, so no run may proceed on a last-wins pick).
/// Controls: with one definition — and with a spelling NOTHING defines, which
/// is the report's business, not the lookup's — the profile passes the
/// resolution and fails LATER, at the node-SOURCE gate (`nodes/<type>` does
/// not exist in this bare workspace), naming the type — never for ambiguity.
/// Lives in THIS binary, not the pure `graph_profile_test`, because
/// `graph_profile` sweeps dead iceoryx2 nodes on the default namespace before
/// it reads the graph (`STARTUP_DEAD_NODE_SWEEP_BUDGET`) — a shared-memory
/// side effect that belongs under `#[serial]`. The node TYPE is unique to
/// this arm as hygiene: nothing else in the tree can have built it.
#[test]
#[serial]
fn an_ambiguous_output_schema_refuses_the_profile_before_node_libraries_load() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = tmp.path();
    std::fs::create_dir_all(ws.join("graphs")).expect("graphs dir");
    std::fs::create_dir_all(ws.join("schemas")).expect("schemas dir");
    const FOO: &str = "schemas:\n  Foo:\n    fields:\n      \"uint32 x\":\n";
    std::fs::write(ws.join("schemas").join("a.yaml"), FOO).expect("a.yaml");
    std::fs::write(ws.join("schemas").join("b.yaml"), FOO).expect("b.yaml");
    let prefix = unique_prefix("r21");
    let write_graph = |schema: &str| {
        std::fs::write(
            ws.join("graphs").join("ambig.yaml"),
            format!(
                "prefix: {prefix}\nnodes:\n  - id: cam\n    type: r21_profile_probe\n    \
                 outputs:\n      - name: out\n        schema: {schema}\n"
            ),
        )
        .expect("graph");
    };
    write_graph("Foo");
    let profile = || {
        cerulion_cli_engine::graph_cmd::graph_profile(
            ws,
            "ambig",
            Arc::new(AtomicBool::new(true)),
            Duration::from_secs(1),
            None,
            None,
        )
    };
    let msg = profile()
        .expect_err("an ambiguous `schema:` must refuse the profile")
        .to_string();
    assert!(
        msg.starts_with("graph 'ambig': ")
            && msg.contains(
                "output 'cam'.out declares `schema: Foo`, which this workspace refuses: "
            )
            && msg.contains("'Foo' is ambiguous in this workspace")
            && msg.contains("schemas/a.yaml (entry Foo)")
            && msg.contains("schemas/b.yaml (entry Foo)"),
        "the profile must refuse naming the graph, the port and every source: {msg}"
    );

    std::fs::remove_file(ws.join("schemas").join("b.yaml")).expect("remove b.yaml");
    let passes_to_the_source_gate = |what: &str| {
        let msg = profile()
            .expect_err("a bare workspace has no node source to profile")
            .to_string();
        assert!(
            !msg.contains("is ambiguous")
                && !msg.contains("this workspace refuses")
                && !msg.contains("could not check"),
            "{what} passes the profile's resolution: {msg}"
        );
        assert!(
            msg.contains("r21_profile_probe"),
            "{what}: the control must fail at the node-source gate, naming the type: {msg}"
        );
    };
    passes_to_the_source_gate("an unambiguous spelling");
    write_graph("NoSuchSchemaAnywhere");
    passes_to_the_source_gate("an absent spelling");
}
