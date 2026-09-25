// SPDX-License-Identifier: AGPL-3.0-only
//! Full user-journey end-to-end CLI + cdylib graph latency test.
//!
//! This is the closest-to-real-user test in the suite. It exercises the
//! ENTIRE journey a roboticist takes — workspace/node/graph scaffolding
//! through the public `cerulion_cli_engine` library surface (the exact
//! code the `cerulion` binary calls), then RUNS THE GRAPH BY SPAWNING THE
//! `cerulion` BINARY AS A SUBPROCESS — with REAL compiled cdylibs and a
//! REAL iceoryx2 graph:
//!
//! 1. **Create a workspace** — `workspace::workspace_create`. The
//!    generated workspace `Cargo.toml` gets ABSOLUTE path deps to THIS
//!    repo's `cerulion_core` / `native_ros2_messages` (the CLI walks up
//!    from the test binary's `current_exe()` — `target/<profile>/deps/...`,
//!    i.e. `target/release/deps/...` under `cargo test --release` —
//!    to find `cerulion_core/Cargo.toml`), so the cdylibs built in the
//!    tmpdir compile against the LOCAL crate, not a published version.
//! 2. **Scaffold the 3 node crates** — `node_cmd::node_create_with_options`
//!    declares each node's `#[input]`/`#[output]` ports + trigger +
//!    period through the CLI surface (Cargo.toml, workspace-member wiring,
//!    and a valid `#[cerulion_node]` macro `src/lib.rs` are all
//!    CLI-generated). The per-node `tick()`/`init()` bodies are then
//!    written into the generated `src/lib.rs` (overwritten in place —
//!    `node_modify_add_port` can declare ports but cannot express the
//!    `init()` env-config read, the latency node's sample-dump helper,
//!    or the variable-field publish logic, so the body is hand-authored,
//!    modeled verbatim on the deleted `benches/cerulion_round_trip/graph_rtt_bench`;
//!    its successor is `benches/latency/workspace/`).
//! 3. **Build the cdylibs** — `node_cmd::node_build` (= `cargo build -p
//!    <node> --release` in the workspace root). Compiles against LOCAL
//!    cerulion_core in RELEASE mode — production-latency numbers only make
//!    sense in release (debug is 10-50x slower). The spawned binary below
//!    loads these RELEASE cdylibs (it runs `cerulion graph run --release`,
//!    which makes `find_cdylib` search `target/release` first).
//!    **The repo's `Cargo.lock` is copied into the workspace first** so the
//!    cdylibs link the IDENTICAL iceoryx2 (and cerulion_core) as the
//!    binary — a version skew (binary 0.9.1 vs a fresh-resolve 0.9.2)
//!    breaks the iceoryx2 event protocol and silently starves the
//!    data-trigger chain (see the step-3 code comment).
//! 4. **Wire the graph** — `graph_cmd::graph_create` + `node_stage` for
//!    each node, building the `NodeDef`s programmatically (the CLI surface
//!    `build_node_def` / `node_stage` round-trips them through the graph
//!    YAML on disk, including `validate_graph`).
//! 5. **Run it — AS A SEPARATE PROCESS.** We spawn the `cerulion` binary
//!    (`cerulion graph run rtt_bench --single-process [--time-source
//!    virtual]`) with `current_dir(ws.root)` so its workspace-discovery
//!    finds the graph. THIS IS THE CRITICAL FIX over the prior in-process
//!    design.
//!
//!    `--single-process`: this gate measures + pins the
//!    MONOLITH latency number. Without the flag, the multi-process
//!    default would derive a process-per-node partition for the
//!    unpartitioned `rtt_bench` graph on this no-TTY subprocess (in-memory,
//!    quantum-timed supervisor run) — a different measurement entirely.
//!
//!    Why a subprocess, not the in-process `graph_run()` library call: the
//!    test binary statically links `cerulion_core`, AND it `dlopen`-loads
//!    cdylibs that EACH statically link their OWN copy of `cerulion_core`.
//!    Calling `graph_run()` in-process means the host's `TransportManager`
//!    /iceoryx2 singleton and each cdylib's singleton are DIFFERENT
//!    instances that never rendezvous — "Unable to establish connection to
//!    new receiver", no data flows. A clean separate `cerulion` process
//!    has exactly ONE `cerulion_core` (its own) driving the graph and
//!    `dlopen`s the cdylibs into THAT address space — the proven bench
//!    path. So we spawn the binary instead.
//!
//!    We build the binary first (`cargo build -p cerulion_cli --release`) so
//!    the spawned process reflects local code, not a stale on-disk artifact.
//!    Each spawn is watchdog-bounded (a polled `try_wait()` loop that
//!    `child.kill()`s after a generous absolute timeout, then BOUNDED-reaps
//!    the corpse — see `run_mode`) so a hang can NEVER spin CI forever — a
//!    hang turns into a LOUD failure (the `.bin`-missing / low-sample
//!    asserts below). The healthy exit is the latency node's
//!    `request_shutdown()` making the binary exit on its own.
//! 6. **Measure latency** — the latency node dumps raw LE `u64` RTT samples
//!    (nanoseconds) to `${CER_BENCH_RAW_DUMP_DIR}/${CER_BENCH_RAW_NAME}_<size>.bin`
//!    on shutdown; the test reads the `.bin`, computes p50/p99, and asserts.
//!
//! # Two clock modes — what each one actually gates
//!
//! We run the graph TWICE per payload size — once per clock source the
//! binary supports. The two modes are BOTH real iceoryx2 round trips with
//! real `real_ns()` RTTs; the headline gate over both is the FLATNESS ratio
//! (below), not an absolute-µs ceiling.
//!
//! The numbers below are from a CI run
//! (ubuntu, n=1000, 64 B).
//!
//! - **virtual** (`--time-source virtual`) drives the graph via the
//!   deterministic `VirtualClock` poll loop, pumping it as fast as
//!   transport + macro + dispatch allow. Measured (Linux CI, 64 B): floor
//!   ≈ 23.2 µs, p50 ≈ 23.7 µs, p99 ≈ 39.9 µs. (The in-tree
//!   `graph_rtt_bench` runs this same mode.)
//! - **real** (default, no flag) drives the graph via the live WaitSet
//!   reactor. Measured (Linux CI, 64 B): floor ≈ 23.4 µs, p50 ≈ 25.8 µs,
//!   p99 ≈ 51.1 µs.
//!
//! **virtual ≈ real (≈ 23 µs) ON THIS CI VM — the live-WaitSet wakeup cost
//! is MASKED here, NOT absent.** This virtualized Azure runner has no real
//! hardware C-states (the guest cores can't deep-idle the way bare metal
//! does) and per-syscall / VM-scheduling jitter sets a high floor, so the
//! C-state-exit wakeup cost is hidden under VM noise — and the CPU DMA /
//! C-state lock is INERT here (a guest PM_QoS write does not
//! govern the HOST's physical C-states), which is exactly why CI cannot
//! show the effect. So the "OS wakeup excluded / virtual ≈ real
//! proves the wakeup isn't dominant" reasoning holds ONLY on this VM — do
//! NOT state it as a general truth.
//!
//! On fast bare metal (measured: Intel Ultra 9 285K) the live-WaitSet
//! wakeup DOES dominate: between wakeups the core deep-idles into a cache-flushing
//! state (C2, ~127µs exit on that CPU) and the post-idle chain runs cold.
//! Measured virtual ≈ 5 µs ≪ real-p50 ≈ 18 µs
//! WITHOUT the cap. The live path is DEFAULT-ON and, when the graph declares
//! tight timing, AUTO-CAPS idle to this CPU's cpuidle cliff (a graph-gated,
//! hardware-derived cap): a
//! latency-sensitive graph caps at C1 (~6–13 µs at low power), while
//! `CERULION_CPU_DMA_LOCK=1` pins C0 and recovers the full ~13 µs → ≈ 5.3 µs.
//! The ~23 µs CI-VM subprocess RTT decomposes into the in-process
//! `GraphRuntime::step()` cost (≈ 7.7 µs, see `graph_latency_test`) PLUS
//! subprocess + cdylib-FFI + CLI live-loop overhead — consistent with a
//! transport-bound chain (the per-hop cost is the iceoryx2
//! round trip, not the executor scaffolding). The level executor
//! collapses
//! the ping→pong→latency chain WITHIN a single `step()` (not ~3
//! steps/RTT). ping's `period_ms` paces WHEN
//! ticks happen but does NOT inflate the measured RTT — the RTT is computed
//! from ping's embedded `real_ns()` stamp, so it counts only send→receive,
//! not the inter-tick wait.
//!
//! Note: the CI flatness GATE is Linux (where virtual ≈ real ≈ 23 µs). A
//! dev machine may report different absolute µs (an M-series macOS machine has been
//! anecdotally faster on the virtual path, slower on the live path), but
//! those numbers are NOT grounded by current CI data and are not asserted
//! anywhere — do not treat them as a contract.
//!
//! Each mode writes to its OWN dump dir + `CER_BENCH_RAW_NAME` (and the
//! `.bin` filename is per-payload-size suffixed) so the two runs' `.bin`
//! files never collide, and each is asserted independently.
//!
//! # Payload-size sweep — the FLATNESS gate (the tight contract)
//!
//! Both modes run across a payload sweep (64 B → 1 MiB) and the headline
//! assertion is a max/min FLATNESS RATIO of the per-size latency, NOT the
//! absolute ceilings (those stay loose liveness backstops). The message is
//! published zero-copy — the nodes call `loan_data(N)` and DISCARD the slice
//! (no `set_data`/`fill_from`, i.e. no per-payload write at all); `loan_data`
//! only RESERVES the SHM slot (O(1), no memcpy of the payload). The RTT is
//! measured from the embedded `real_ns()` stamp; the loan sits technically
//! inside the timed window but is O(1) zero-copy, so the per-round RTT should
//! stay ~flat as the payload grows in BOTH modes (the flat virtual curve
//! confirms it empirically — a per-payload memcpy would inflate it O(n)):
//!
//! - **virtual** is the zero-copy moat: measured floor ratio ≈ 1.02×
//!   (a CI run, ubuntu, 64 B → 1 MB), well under the
//!   tight < 1.5× ceiling (matching `graph_latency_test`). A memcpy
//!   regression in the macro/transport stack would blow this up to
//!   tens-of-×.
//! - **real** should ALSO be flat — and IS: measured floor ratio ≈ 1.02×
//!   on the same CI run. A non-flat real curve would expose a hidden
//!   per-payload copy on the live path. A per-payload copy lives on the
//!   (clock-independent) DATA path, so it inflates BOTH modes — the VIRTUAL
//!   gate (< 1.5×, jitter-immune) is the primary copy-detector. The real
//!   ratio keeps a GENEROUS catastrophe backstop (< 5×) because the live
//!   path's FLOOR carries more run-to-run jitter than the virtual poll
//!   loop, and on a noisy CI VM a single mid-size floor can jump without
//!   any copy (a real O(n) copy is monotonic in size and tens-of-× at the
//!   top payload, not a one-off mid-size bump). See `FLATNESS_MAX_REAL`.
//!
//! **CRITICAL — the flatness ratio uses the FLOOR (min over iterations), NOT
//! p50** (mirroring `flat_latency_test` / `cross_thread_rtt_test`). On the
//! noisy, oversubscribed macOS CI runner the *p50* of the top payload row
//! balloons under VM scheduling/memory-pressure jitter even with NO copy —
//! that jitter only ever ADDS latency, it can never lower the floor. The
//! uncontended FLOOR strips the runner noise out while PRESERVING the
//! regression signal: a real O(n) memcpy makes EVERY iteration slower,
//! inflating the floor too (a 1 MiB copy is ~tens-of-× the µs floor). So the
//! floor keeps the gate BINARY (~1× clean vs tens-of-× broken) AND robust to
//! runner noise — strictly better than p50 for a structural "is the path
//! O(1) in payload?" check. The absolute p50/p99 ceilings stay as loose
//! catastrophe backstops; the flatness ratio is the tight gate.
//!
//! **The FLOOR is jitter-RESISTANT but NOT jitter-IMMUNE.** Each
//! payload size is a SEPARATE SUBPROCESS run, and on a shared macOS CI VM an
//! ENTIRE subprocess can be preempted (VM scheduling / memory pressure),
//! inflating EVERY one of its samples — the floor included. If that stalled
//! size is not the min-floor size, the `max/min` ratio blows past the ceiling on
//! byte-identical code (it recurs on the macos-latest runner, each time
//! healed by a plain rerun). So the flatness gate carries window-health
//! robustness — see `run_and_gate_mode`:
//!
//! - **window-health retries** — the SOLE single-window-stall
//!   defense: any size whose floor exceeds `flatness_max × min_floor` is
//!   RE-MEASURED in a fresh subprocess (bounded `MAX_RETRIES`), keeping the
//!   better floor. A transient stall clears in a fresh window; a real copy
//!   re-measures just as slow. A healthy sweep retries nothing.
//! - the gate is the **FULL `max/min`** FLOOR ratio.
//!   **Drop-one-outlier is deliberately NOT used here** (it
//!   IS in the siblings): on this sweep `[64, 4096, 65536, 1048576]` the
//!   2nd-largest size (64 KiB) is 16× smaller than the largest (1 MiB), so a real
//!   linear copy that adds +160 µs at 1 MiB adds only ~10 µs at 64 KiB — BELOW the
//!   ~11-40 µs base RTT. Drop-one would discard the 1 MiB floor (where the copy
//!   shows) and collapse `max` to the barely-inflated 64 KiB floor ⇒ a real copy
//!   would pass. The siblings' geometry
//!   (2nd-largest = 1 MiB under a ~1 µs base) keeps their copy signal above the
//!   drop, which is why drop-one is correct there and not here.
//! - **uniform-stall discriminator**: an unhealed SIZE-INDEPENDENT stall
//!   (`>= 3` elevated floors in a tight band) fails as an ATTRIBUTABLE,
//!   non-probative panic naming the signature + retry history — while a REAL
//!   copy still fails the gate NORMALLY (the healthy-path / catastrophe semantics
//!   are UNCHANGED; the pin is not weakened).
//!
//! **Residuals of the retry-only stall defense** (no drop-one absorbs a
//! stall): (1) a persistent **single- or two-window** stall that survives EVERY
//! retry fails as `RealCopy` — the conservative direction, since it is
//! indistinguishable from a real single-size regression (a re-run on a quiescent
//! host disambiguates; if it clears, it was a stall). (2) A persistent
//! **`>= 3`-window uniform** stall lands in the attributable `UniformStall` arm
//! (non-probative, re-run). Both are far rarer than the single-transient-window
//! flake the retry heals silently.
//!
//! Per-size INTERLEAVING (the lever the sibling `flat_latency_test` applies) is deliberately NOT applied:
//! the measurement unit is a whole subprocess, so there is no cheap
//! round-interleaving, and the retry already re-measures a stalled window in a
//! fresh temporal window — the same reasoning `cross_thread_rtt_test` used to
//! exclude it. A discarded warm-up is not added either: each subprocess already
//! drops `WARMUP` samples, and the flake is a VM stall (size-independent), not a
//! DVFS cold head (the `graph_latency_test` warm-up's motivation).
//! The pure gate-decision math lives in `cerulion_core::testing`
//! (`classify_flatness` + the retry / uniform helpers), oracle-tested there
//! against this gate's actual 4-size sweep.
//!
//! # Clock note — why RTTs are real even under virtual time
//!
//! ping (publish) and latency (receive) both timestamp via
//! `self.real_ns()` (kernel `CLOCK_MONOTONIC`), NOT the runtime's clock.
//! `real_ns()` is independent of the runtime clock injection — same
//! source the in-tree `graph_rtt_bench` uses, so the numbers are
//! comparable across both modes. The runtime clock (`virtual`) only
//! controls scheduler pacing, not the measurement clock. This is why the
//! virtual-clock run still produces real, non-zero RTTs (verified by the
//! per-mode sample-count assert below — if virtual ever produced zero or
//! all-zero RTTs the `.bin`-missing / sample-count assert fails LOUD).
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test cli_e2e_graph_latency_test --release \
//!     -- --test-threads=1 --nocapture
//! ```
//!
//! Must run with `--release` (like `graph_latency_test`): the absolute-µs
//! ceilings only make sense for production-profile artifacts, and this
//! module is gated behind `#![cfg(not(debug_assertions))]` (below) so a
//! plain `cargo test` (no `--release`) compiles it to an empty no-op rather
//! than running meaningless debug numbers. Under `cargo test --release` the
//! test binary lives at `target/release/deps/...`, so the in-test
//! `cargo build -p cerulion_cli --release` and the `current_exe()` walk-up
//! both resolve `target/release/cerulion` — binary build profile and load
//! path stay consistent.
//!
//! Must run single-threaded: the iceoryx2 shared-memory region is a
//! process-wide singleton. The heavy lifting is now in subprocesses, but
//! the two clock-mode subprocesses run SEQUENTIALLY within this one test
//! (iceoryx2 state is wiped between them), and a parallel sibling test
//! touching the same SHM region would still race.
//!
//! # Why this is gated (slow), not a default-CI test
//!
//! This test BUILDS three cdylib crates + the `cerulion` binary at test
//! time (`cargo build -p` each, against local cerulion_core) and runs a
//! real iceoryx2 graph TWICE. That is far slower than the rest of the
//! suite and touches the SHM singleton, so it belongs on a gated
//! push-to-main lane (the human wires it into CI). The fast in-process
//! `e2e_cli_test.rs` covers the file-generation + in-process-runtime
//! path; THIS test is the only one that compiles and loads REAL user
//! cdylibs end to end through the real binary.
//!
//! # Maintenance
//!
//! The hand-written node bodies below mirror the `#[cerulion_node]` macro
//! API, the `native_ros2_messages::sensor_msgs::Image` field/method
//! surface, and the `NodeContext` env/shutdown shims. A breaking change to
//! any of those (CLI scaffold output, macro shim methods, message codegen)
//! requires updating the node source strings in this file.

// Release-only, mirroring `graph_latency_test.rs`: the absolute-µs ceilings
// are production-profile numbers, so under a plain `cargo test` (no
// `--release`) this whole module compiles to an empty no-op instead of
// asserting against meaningless debug latencies. Run it with `--release`.
#![cfg(not(debug_assertions))]

use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::Duration;

use cerulion_cli_engine::graph_cmd::{build_node_def, graph_create, node_stage};
use cerulion_cli_engine::ipc_cleanup::cleanup_dead_iceoryx2_nodes;
use cerulion_cli_engine::node_cmd::{
    node_build, node_create_with_options, NodeCreateOptions, NodeLanguage,
};
use cerulion_cli_engine::workspace::workspace_create;
// The shared, oracle-tested flatness-gate decision (drop-one-outlier
// robust ratio + uniform-stall discriminator) ported from flat_latency_test /
// cross_thread_rtt_test for the shared-macOS-CI-VM stall class.
use cerulion_core::testing::FlatnessVerdict;

// ─── Test knobs ──────────────────────────────────────────────
//
// Small sample count for a fast test (the in-tree bench uses 10_000 +
// 1_000 warmup). 1_000 + 200 is enough to get a stable median + a stable
// floor while keeping each subprocess run short.
const TARGET_SAMPLES: usize = 1_000;
const WARMUP: usize = 200;

/// Payload sizes swept for the FLATNESS gate, 64 B → 1 MiB. Each size is a
/// FULL subprocess run PER MODE (this is a release subprocess test — the
/// binary + 3 cdylibs are already built, but each size respawns the graph),
/// so the count is kept MODEST (4 sizes × 2 modes = 8 graph runs). Small +
/// medium + large is enough to catch an O(n) memcpy regression (which would
/// be tens-of-× at 1 MiB) without ballooning the slow test's wall time.
/// Mirrors `graph_latency_test`'s sweep shape (64 / 64 KiB / 1 MiB) plus a
/// 4 KiB rung for finer resolution between the small and large ends.
const PAYLOAD_SIZES: &[usize] = &[64, 4_096, 65_536, 1_048_576];

/// Per-mode p50 ceilings — generous LIVENESS backstops, set from
/// FIRST-PARTY measured runs on a RELEASE build (NOT placeholders). They are
/// the SECONDARY contract now; the FLATNESS RATIO (below) is the tight gate.
/// Production-latency numbers only make sense in release; this module is
/// `#![cfg(not(debug_assertions))]`-gated so it only runs under `--release`.
/// The two clock modes gate DIFFERENT things (see the "Two clock modes"
/// section above), so they get separate ceilings:
///
/// The numbers below are from a CI run
/// (ubuntu, n=1000, 64 B).
///
/// - **virtual** — the poll-loop path, bounded by transport + macro +
///   dispatch. Measured (Linux CI, 64 B): floor ≈ 23.2 µs, p50 ≈ 23.7 µs,
///   p99 ≈ 39.9 µs. 80 µs is a generous ceiling (~3× headroom over the
///   measured p50) — a transport/macro latency regression trips it.
/// - **real** — a LIVENESS gate, deliberately LOOSE. The live WaitSet
///   reactor path measures (Linux CI, 64 B): floor ≈ 23.4 µs, p50 ≈ 25.8 µs,
///   p99 ≈ 51.1 µs — i.e. virtual ≈ real ON THIS CI VM. That does NOT mean
///   the OS wakeup is cheap in general: the C-state-exit wakeup cost is
///   MASKED on this virtualized runner (no real hardware C-states; high
///   VM-jitter floor), not absent. On bare metal it DOMINATES — measured
///   virtual ≈ 5 µs ≪ real-p50 ≈ 18 µs without the lock, recovered to ≈ 5 µs
///   by the CPU DMA / C-state lock (default-on for the live path).
///   The level executor collapses
///   the chain within a step. 3500 µs (3.5 ms) is a loose ceiling (~135×
///   headroom over the measured p50) that catches a catastrophic regression
///   or a total stall without flaking on a slow, noisy CI runner. Kept
///   generous on purpose: the per-payload FLATNESS RATIO is the real gate;
///   this absolute only catches a gross stall.
///
/// Both modes are real iceoryx2 round trips with real `real_ns()` RTTs. The
/// ~23 µs CI-VM subprocess RTT is the in-process `GraphRuntime::step()` cost
/// (≈ 7.7 µs, see `graph_latency_test`) PLUS subprocess + cdylib-FFI + CLI
/// live-loop overhead — a transport-bound cost.
///
/// Do NOT add a DMA-lock-gated absolute-µs assertion to CI: this runner is
/// virtualized and the lock is INERT here (guest PM_QoS does not govern the
/// host's physical C-states), so the live-path absolute-µs effect cannot be
/// reproduced on CI. The CI GATE stays the per-payload FLATNESS RATIO (which
/// catches O(n) payload copies); absolute-µs A/B validation of the lock is a
/// bare-metal-only manual check.
const P50_CEILING_REAL_US: f64 = 3500.0;
const P50_CEILING_VIRTUAL_US: f64 = 80.0;

/// Per-mode p99 ceilings — loose tail guards (~5× the p50 ceiling) so a
/// CATASTROPHIC tail (a single stall that doesn't move the median) is
/// still caught, without making p99 a tight gate. These are diagnostic
/// backstops, not the primary contract: the p50 ceilings above are the
/// real gate. Set at 5× the respective p50 ceiling, comfortably above the
/// measured p99s (real ≈ 683 µs vs 17500 µs ceiling; virtual ≈ 8 µs vs
/// 400 µs ceiling).
const P99_CEILING_REAL_US: f64 = P50_CEILING_REAL_US * 5.0;
const P99_CEILING_VIRTUAL_US: f64 = P50_CEILING_VIRTUAL_US * 5.0;

/// Per-mode payload FLATNESS RATIO ceilings — the TIGHT gate (the absolute
/// p50/p99 ceilings above are loose liveness backstops). The ratio is
/// max/min of the per-size FLOOR (min RTT over iterations, NOT p50 — see the
/// "Payload-size sweep" doc section), so it is robust to the macOS CI
/// runner's VM jitter (which inflates p50 with no copy) while still catching
/// a real O(n) memcpy (which inflates the floor on every iteration).
///
/// The numbers below are from a CI run
/// (ubuntu, 64 B → 1 MB).
///
/// - **virtual** — the zero-copy moat. `loan_data(N)` only RESERVES the SHM
///   slot (O(1), no payload memcpy; the nodes discard the slice), so the floor
///   is payload-INDEPENDENT: measured floor ratio ≈ 1.02× (Linux CI). The
///   1.5× CEILING matches `graph_latency_test`; the METRIC here is the FLOOR
///   (mirroring `flat_latency_test`) vs that test's p50; a memcpy regression
///   would show tens-of-×.
/// - **real** — the live path is also flat: measured floor ratio ≈ 1.02×
///   (Linux CI, same run). A non-flat real curve would expose a hidden
///   per-payload copy on the live path. The real gate is looser than
///   virtual's 1.5× because the live-reactor path carries more run-to-run
///   jitter than the virtual poll loop — its per-iteration OS wakeup adds
///   noise, so even the FLOOR of a zero-copy live run is less stable than
///   the virtual floor. A real O(n) copy is monotonic in payload and
///   tens-of-µs at the top size, NOT a one-off mid-size bump.
///
///   IMPORTANT — the real gate is a GENEROUS CATASTROPHE BACKSTOP, not the
///   primary copy-detector, for two reasons. (1) Unlike the virtual poll
///   loop, the real path's FLOOR is NOT fully jitter-immune: the WaitSet
///   wakeup's own best-case latency drifts run-to-run, and a shared CI VM is
///   more jittery still — a tight bound would flake there. (2) A per-payload
///   copy lives on the DATA path, which is clock-independent, so it inflates
///   BOTH modes — the VIRTUAL gate (1.5×, jitter-immune) already detects ANY
///   real copy. So `FLATNESS_MAX_REAL` is set well above the jitter band yet
///   still tens-of-× below a gross live-path regression; the tight zero-copy
///   contract is the VIRTUAL gate.
const FLATNESS_MAX_VIRTUAL: f64 = 1.5;
const FLATNESS_MAX_REAL: f64 = 5.0;

/// Absolute per-mode watchdog. The healthy exit is the latency node's
/// `request_shutdown()` making the binary exit on its own (a few seconds
/// at most: `period_ms` pacing × `target+warmup` samples on the real
/// clock; near-instant under virtual). 120s ≫ that, even on a heavily
/// loaded macOS runner, and a hang is killed (turning it into a LOUD
/// `.bin`-missing failure) rather than spinning CI forever.
const WATCHDOG: Duration = Duration::from_secs(120);

/// Per-size window-health RETRY budget. A size whose
/// initial-sweep FLOOR is elevated (a whole-subprocess VM stall lifts even its
/// min) is re-measured in a FRESH subprocess up to this many times, keeping the
/// better floor — a transient VM stall almost always clears within a fresh
/// window. Matches the siblings' budget (2). A healthy sweep retries NOTHING
/// (zero extra subprocess cost), so this only ever spends wall time on a stalled
/// run — bounded by `MAX_RETRIES × PAYLOAD_SIZES.len()` extra subprocesses per
/// mode, each still watchdog-capped at `WATCHDOG`.
const MAX_RETRIES: usize = 2;

// ─── Node source bodies (hand-written, modeled on graph_rtt_bench) ───
//
// Each string fully replaces the CLI-scaffolded `src/lib.rs`. The struct
// name (`{Pascal}Node`), ports, and policy MATCH what
// `node_create_with_options` declared — the overwrite adds only the real
// `init()`/`tick()`/`shutdown()` bodies the CLI scaffold can't express.

/// Periodic source. Stamps `real_ns()` across `Image.height`/`width`
/// (split to preserve full ns precision) and loans `payload_size` bytes.
const PING_SRC: &str = r#"use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct PingNode {
    #[output]
    ping_out: Image,

    payload_size: usize,
}

#[cerulion_node_impl]
impl PingNode {
    fn init(&mut self, ctx: &mut NodeContext) -> Result<(), NodeError> {
        self.payload_size = ctx.env("CER_BENCH_PAYLOAD_SIZE", 64);
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        let t = self.real_ns();
        self.ping_out.height = (t >> 32) as u32;
        self.ping_out.width = (t & 0xFFFF_FFFF) as u32;
        self.ping_out.step = 0;
        self.ping_out.is_bigendian = 0;
        self.ping_out.set_header_bytes(&[])?;
        self.ping_out.set_encoding("rt")?;
        let _ = self.ping_out.loan_data(self.payload_size)?;
        Ok(())
    }
}
"#;

/// Data-triggered echo. Forwards the embedded timestamp and re-loans the
/// same byte count.
const PONG_SRC: &str = r#"use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node]
#[derive(Default)]
struct PongNode {
    #[input(trigger)]
    ping_in: Image,
    #[output]
    echo_out: Image,

    payload_size: usize,
}

#[cerulion_node_impl]
impl PongNode {
    fn init(&mut self, ctx: &mut NodeContext) -> Result<(), NodeError> {
        self.payload_size = ctx.env("CER_BENCH_PAYLOAD_SIZE", 64);
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        let h = self.ping_in.height;
        let w = self.ping_in.width;
        self.echo_out.height = h;
        self.echo_out.width = w;
        self.echo_out.step = 0;
        self.echo_out.is_bigendian = 0;
        self.echo_out.set_header_bytes(&[])?;
        self.echo_out.set_encoding("rt")?;
        let _ = self.echo_out.loan_data(self.payload_size)?;
        Ok(())
    }
}
"#;

/// Data-triggered sink. Computes RTT against `real_ns()`, collects into a
/// Vec, and after `target_samples + warmup` calls `request_shutdown()` and
/// dumps raw LE `u64` ns samples to the env-specified `.bin`.
const LATENCY_SRC: &str = r#"use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

#[cerulion_node]
#[derive(Default)]
struct LatencyNode {
    #[input(trigger)]
    echo_in: Image,

    samples: Vec<u64>,
    target_samples: usize,
    warmup: usize,
    payload_size: usize,
    shutdown_requested: bool,
}

#[cerulion_node_impl]
impl LatencyNode {
    fn init(&mut self, ctx: &mut NodeContext) -> Result<(), NodeError> {
        self.target_samples = ctx.env("CER_BENCH_TARGET_SAMPLES", 10_000);
        self.warmup = ctx.env("CER_BENCH_WARMUP", 1_000);
        self.payload_size = ctx.env("CER_BENCH_PAYLOAD_SIZE", 64);
        self.samples = Vec::with_capacity(self.target_samples + self.warmup);
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        let h = self.echo_in.height as u64;
        let w = self.echo_in.width as u64;
        let send_ns = (h << 32) | w;

        let now_ns = self.real_ns();
        let rtt = now_ns.saturating_sub(send_ns);
        if send_ns == 0 || rtt == 0 {
            return Ok(());
        }

        self.samples.push(rtt);
        let total_needed = self.target_samples + self.warmup;
        if !self.shutdown_requested && self.samples.len() >= total_needed {
            self.shutdown_requested = true;
            self.request_shutdown();
        }
        Ok(())
    }

    fn shutdown(&mut self) -> Result<(), NodeError> {
        if self.samples.len() <= self.warmup {
            return Ok(());
        }
        self.samples.drain(..self.warmup);
        let n = self.samples.len();
        dump_raw_samples(&self.samples, self.payload_size);
        eprintln!(
            "latency_node: payload={} dumped {} samples",
            self.payload_size, n,
        );
        let _ = std::io::stderr().flush();
        Ok(())
    }
}

/// Write raw round-trip samples (LE u64 ns) to
/// `${CER_BENCH_RAW_DUMP_DIR}/${CER_BENCH_RAW_NAME}_${payload_size}.bin`.
fn dump_raw_samples(samples: &[u64], payload_size: usize) {
    let Ok(dir) = std::env::var("CER_BENCH_RAW_DUMP_DIR") else {
        return;
    };
    let Ok(name) = std::env::var("CER_BENCH_RAW_NAME") else {
        return;
    };
    let path = PathBuf::from(dir).join(format!("{name}_{payload_size}.bin"));
    let mut buf = Vec::with_capacity(samples.len() * 8);
    for &s in samples {
        buf.extend_from_slice(&s.to_le_bytes());
    }
    let mut f = match File::create(&path) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("dump_raw_samples: create {}: {e}", path.display());
            return;
        }
    };
    if let Err(e) = f.write_all(&buf) {
        eprintln!("dump_raw_samples: write {}: {e}", path.display());
    }
}
"#;

/// Overwrite a CLI-scaffolded node crate's `src/lib.rs` with the real
/// tick logic. The CLI's `node_create_with_options` produced the
/// Cargo.toml, workspace-member wiring, and a placeholder macro lib.rs
/// with matching ports/policy; this swaps in the body.
fn write_node_body(nodes_dir: &Path, node_type: &str, source: &str) {
    let lib_rs = nodes_dir.join(node_type).join("src").join("lib.rs");
    assert!(
        lib_rs.exists(),
        "CLI scaffold should have created {} — got nothing",
        lib_rs.display()
    );
    std::fs::write(&lib_rs, source)
        .unwrap_or_else(|e| panic!("write node body to {}: {e}", lib_rs.display()));
}

/// Extract the resolved `iceoryx2` version from a `Cargo.lock`.
///
/// Used to PROVE the lock pin (step 3) is effective: after the cdylib
/// build, the workspace lock's iceoryx2 version must still match the repo
/// lock's. If `cargo build` silently re-resolved (lock ignored / a member
/// forced a bump), this catches it — closing the "lock silently
/// re-resolved" path that would otherwise reappear only as the
/// connection-failure flood. Returns `None` if no `[[package]] name =
/// "iceoryx2"` stanza is present.
fn iceoryx2_version_in_lock(lock_path: &Path) -> Option<String> {
    let text = std::fs::read_to_string(lock_path).ok()?;
    let mut in_iceoryx2 = false;
    for line in text.lines() {
        let line = line.trim();
        if line == "[[package]]" {
            in_iceoryx2 = false; // entering a new package stanza
            continue;
        }
        if line == r#"name = "iceoryx2""# {
            in_iceoryx2 = true;
            continue;
        }
        if in_iceoryx2 {
            if let Some(rest) = line.strip_prefix("version = ") {
                return Some(rest.trim_matches('"').to_string());
            }
        }
    }
    None
}

/// Parse a raw LE-`u64` sample dump and return the sorted nanosecond
/// samples (ascending) for percentile extraction.
fn read_samples_sorted_ns(path: &Path) -> Vec<u64> {
    let bytes =
        std::fs::read(path).unwrap_or_else(|e| panic!("read sample dump {}: {e}", path.display()));
    assert!(
        bytes.len().is_multiple_of(8),
        "sample dump {} is not a whole number of u64s ({} bytes)",
        path.display(),
        bytes.len()
    );
    let mut samples: Vec<u64> = bytes
        .as_chunks::<8>()
        .0
        .iter()
        .map(|c| u64::from_le_bytes(*c))
        .collect();
    samples.sort_unstable();
    samples
}

/// Nearest-rank percentile from an ascending-sorted slice (`p` in 0.0..=1.0).
fn percentile_ns(sorted: &[u64], p: f64) -> u64 {
    assert!(!sorted.is_empty(), "no samples");
    let rank = ((sorted.len() as f64 - 1.0) * p).round() as usize;
    sorted[rank.min(sorted.len() - 1)]
}

/// Resolve the `cerulion` binary path — the EXACT artifact the
/// `cargo build -p cerulion_cli --release` in step 5 produced.
///
/// That build strips `CARGO_TARGET_DIR` and runs with
/// `current_dir(repo_root)`, so the binary deterministically lands at
/// `<repo_root>/target/release/cerulion` regardless of any ambient
/// `CARGO_TARGET_DIR`. We resolve that SAME canonical path here, keyed
/// off the same `repo_root` (derived from the compile-time
/// `CARGO_MANIFEST_DIR`, itself `CARGO_TARGET_DIR`-immune) — so build
/// output and resolution stay in lockstep.
///
/// We deliberately do NOT walk up from this test binary's
/// `current_exe()`: `cargo test` DOES honor `CARGO_TARGET_DIR` for the
/// test binary's own location, so a walk-up would resolve
/// `$CARGO_TARGET_DIR/release/cerulion` while the build wrote to
/// `<repo_root>/target/release/cerulion` — diverging whenever
/// `CARGO_TARGET_DIR` is set and firing the `exists()` assert below with
/// a confusing message. This module is release-only
/// (`#![cfg(not(debug_assertions))]`), so `release` is always the right
/// profile.
///
/// `CARGO_BIN_EXE_cerulion` (injected only for the binary-owning crate's
/// own tests — never for this `cerulion_core` test) is honored first as
/// an explicit override should a wrapping harness ever set it.
fn cerulion_bin_path(repo_root: &Path) -> PathBuf {
    if let Some(p) = std::env::var_os("CARGO_BIN_EXE_cerulion") {
        return PathBuf::from(p);
    }
    repo_root.join("target").join("release").join("cerulion")
}

/// The iceoryx2 connection-failure flood that this test exists to catch.
///
/// A binary↔cdylib iceoryx2 VERSION SKEW (e.g. binary 0.9.1 vs a cdylib
/// fresh-resolved to 0.9.2; see the Cargo.lock pin in step 3) breaks the
/// shared-memory event protocol — the binary's Notifier cannot establish
/// event connections to the cdylib's Listeners, emitting this exact
/// error line at iceoryx2 ERROR level, often 100k+ times, while ZERO
/// data flows. We KEEP `IOX2_LOG_LEVEL=error` on the child precisely so
/// this error-level spam still surfaces, then assert (in `run_mode`) that
/// the captured child stderr carries no flood of it. A handful during
/// startup is tolerable; hundreds+ is the bug.
const IOX2_CONN_FAIL_NEEDLE: &str = "Unable to establish connection";

/// Max tolerated count of [`IOX2_CONN_FAIL_NEEDLE`] lines in the child's
/// stderr before we treat the run as the version-skew failure. A few may
/// legitimately appear during connection setup races at startup; the bug
/// produces them by the hundred-thousand.
const MAX_CONN_FAIL_LINES: usize = 16;

/// Bounded reap deadline after `child.kill()`. A `SIGKILL`ed child should
/// be reaped near-instantly; if it somehow survives this long, that is
/// itself a finding (a child surviving SIGKILL / a stuck wait) and we
/// panic LOUD rather than block forever in an unbounded `wait()`.
const REAP_DEADLINE: Duration = Duration::from_secs(5);

/// Run one clock mode at one payload size end to end and return the sorted
/// RTT samples (ns).
///
/// Spawns the `cerulion` binary in `mode` (real = no time-source flag;
/// virtual = `--time-source virtual`), always with `--release` so the
/// release cdylibs are loaded, with the `CER_BENCH_*` env wired onto the
/// CHILD only (never `std::env::set_var` — that is process-global + racy).
/// `payload_size` is wired onto the child via `CER_BENCH_PAYLOAD_SIZE` (the
/// ping/pong/latency nodes loan that many bytes per round). Each (mode,size)
/// gets its OWN dump dir + per-size `.bin` (`CER_BENCH_RAW_NAME` is per-mode;
/// the dump filename is size-suffixed by the node) so the runs' `.bin` files
/// never collide.
///
/// The child's stderr is PIPED and drained on a dedicated thread (so a
/// full pipe can never block the child) and, after a successful run,
/// asserted free of the [`IOX2_CONN_FAIL_NEEDLE`] flood — the error-flood
/// half of the version-skew bug this test exists to catch.
///
/// A watchdog (a polled `try_wait()` loop on this thread) `child.kill()`s
/// after [`WATCHDOG`] so a hang becomes a loud failure, not an infinite
/// spin — and the post-kill reap is itself BOUNDED ([`REAP_DEADLINE`]) so
/// the watchdog can never hang in an unbounded `wait()`.
fn run_mode(
    cerulion_bin: &Path,
    ws_root: &Path,
    mode_name: &str,
    virtual_clock: bool,
    payload_size: usize,
) -> Vec<u64> {
    let dump_dir = ws_root.join(format!("raw_dumps_{mode_name}"));
    std::fs::create_dir_all(&dump_dir).expect("create dump dir");
    let raw_name = format!("cli_e2e_{mode_name}");

    // Reclaim ONLY the stale resources of DEAD iceoryx2 nodes (the prior
    // mode's `cerulion` subprocess has fully exited by the time we run the
    // next mode, so its node is dead → reclaimable). We deliberately do
    // NOT `remove_dir_all("/tmp/iceoryx2")`: `ipc_cleanup.rs` documents
    // that a blanket wipe corrupts ANY concurrent live Cerulion process on
    // the host AND that this smart dead-node sweep is the correct tool.
    // (It walks iceoryx2's node registry; it does not itself clear
    // `/dev/shm`, but a cleanly-exited child releases its own SHM segments
    // on drop, and the next mode uses fresh derived service names anyway.)
    let cleanup = cleanup_dead_iceoryx2_nodes();
    eprintln!(
        "[{mode_name}] iceoryx2 dead-node cleanup: {} cleaned, {} failed",
        cleanup.cleanups, cleanup.failed_cleanups,
    );

    let mut cmd = Command::new(cerulion_bin);
    cmd.args(["graph", "run", "rtt_bench"]);
    // `--single-process`: this gate pins the MONOLITH number.
    // The unpartitioned rtt_bench graph on a no-TTY subprocess would
    // otherwise hit the multi-process auto-partition default (in-memory
    // process-per-node supervisor run) — a different measurement.
    cmd.arg("--single-process");
    // `--release` for BOTH modes: the cdylibs were built with
    // `node_build(.., true)`, so the binary must prefer `target/release`
    // when resolving + loading them (find_cdylib release-first).
    cmd.arg("--release");
    if virtual_clock {
        cmd.args(["--time-source", "virtual"]);
    }
    cmd.current_dir(ws_root)
        // CARGO_TARGET_DIR is deliberately INHERITED (not stripped): the
        // cdylibs were built by `node_build` (cargo with
        // `current_dir(ws_root)`), whose child cargo honors any ambient
        // CARGO_TARGET_DIR — and the binary's `find_cdylib` resolves
        // through `cdylib_target_base`,
        // which honors the SAME env with the SAME `ws_root` anchor for
        // relative values. Build and resolution therefore agree wherever
        // the artifacts actually landed. (Stripping the env here
        // would point the resolver at the canonical
        // `<ws_root>/target` while the artifacts sit in the redirected
        // target dir: a build/resolve divergence.)
        // Hermetic — no scouting session/gateway in CI (a real-clock
        // run is permissive-by-default; the kill-switch env keeps this latency
        // gate LOCAL-ONLY so no network overlay perturbs the measurement).
        .env("CERULION_NETWORK", "off")
        // Quiet iceoryx2's per-notification WARN spam on the CHILD only —
        // but KEEP error level so the version-skew connection-failure flood
        // (asserted-absent below) still surfaces in the captured stderr.
        .env("IOX2_LOG_LEVEL", "error")
        .env("CER_BENCH_TARGET_SAMPLES", TARGET_SAMPLES.to_string())
        .env("CER_BENCH_WARMUP", WARMUP.to_string())
        .env("CER_BENCH_PAYLOAD_SIZE", payload_size.to_string())
        .env("CER_BENCH_RAW_DUMP_DIR", &dump_dir)
        .env("CER_BENCH_RAW_NAME", &raw_name)
        // Pipe stderr so we can assert the connection-failure flood is
        // absent. It is drained on a thread below so it can never fill and
        // block the child. iceoryx2 errors and the latency node's progress
        // `eprintln!`s both land on stderr, so this captures everything we
        // need; stdout stays inherited.
        .stderr(std::process::Stdio::piped());

    let mut child = cmd
        .spawn()
        .unwrap_or_else(|e| panic!("spawn `cerulion graph run rtt_bench` ({mode_name}): {e}"));

    // Drain the child's stderr on a dedicated thread. This is mandatory
    // for correctness, not just observability: with a piped stderr that we
    // never read, the OS pipe buffer fills and the child BLOCKS on its next
    // write — which under the version-skew bug (100k+ error lines) would
    // deadlock the child and only ever surface as a watchdog timeout. The
    // thread reads to EOF (EOF arrives when the child exits and closes its
    // write end), so the watchdog's `try_wait()` polling on THIS thread is
    // never blocked by the draining. We also tee the lines to our own
    // stderr (prefixed) so `--nocapture` runs still show the child's logs.
    let child_stderr = child.stderr.take().expect("child stderr was piped");
    let mode_for_thread = mode_name.to_string();
    let drain = std::thread::spawn(move || {
        use std::io::BufRead;
        let reader = std::io::BufReader::new(child_stderr);
        let mut captured = String::new();
        for line in reader.lines() {
            let Ok(line) = line else { break };
            eprintln!("[{mode_for_thread} child] {line}");
            captured.push_str(&line);
            captured.push('\n');
        }
        captured
    });

    // Watchdog via polled `try_wait()` on the MAIN thread — no mutex, no
    // separate kill thread. Wrapping the child in an
    // `Arc<Mutex<Child>>` and blocking the main thread in `wait()` while a
    // watchdog thread tries to `lock()` to call `kill()` deadlocks: `wait()` holds
    // the lock for the whole run, so the watchdog can
    // never fire (an 11-minute hang, measured). Polling `try_wait()`
    // ourselves keeps the kill path entirely on this thread and bounded.
    let deadline = std::time::Instant::now() + WATCHDOG;
    let status: Option<std::process::ExitStatus> = loop {
        match child.try_wait().expect("try_wait on cerulion child") {
            Some(s) => break Some(s), // healthy exit (request_shutdown → process exit)
            None => {
                if std::time::Instant::now() >= deadline {
                    let _ = child.kill();
                    // BOUNDED reap. A plain `child.wait()` here is an
                    // UNBOUNDED blocking reap — if the child somehow won't
                    // reap, the watchdog ITSELF hangs, reverting to the
                    // 30-min CI job timeout with no diagnostic. Instead poll
                    // `try_wait()` against a short [`REAP_DEADLINE`]; if the
                    // child survives SIGKILL / won't reap, that is its own
                    // finding — panic LOUD rather than block forever.
                    let reap_deadline = std::time::Instant::now() + REAP_DEADLINE;
                    loop {
                        match child.try_wait().expect("try_wait reaping killed child") {
                            Some(_) => break, // reaped
                            None => {
                                if std::time::Instant::now() >= reap_deadline {
                                    panic!(
                                        "[{mode_name}] child survived SIGKILL / won't reap \
                                         within {REAP_DEADLINE:?} after the {WATCHDOG:?} \
                                         watchdog fired — the child is stuck unkillable \
                                         (this is itself a finding, not a normal stall)"
                                    );
                                }
                                std::thread::sleep(Duration::from_millis(20));
                            }
                        }
                    }
                    break None; // watchdog fired (child reaped)
                }
                std::thread::sleep(Duration::from_millis(50));
            }
        }
    };

    let status = status.unwrap_or_else(|| {
        panic!(
            "[{mode_name}] watchdog fired after {:?} — the graph never reached \
             request_shutdown() (no data flowed / chain stalled). Killed the child.",
            WATCHDOG
        )
    });

    // Collect the drained child stderr now that the child has exited (its
    // write end is closed → the drain thread saw EOF and returned). Joining
    // here is bounded: the thread only blocks on a pipe whose writer is now
    // gone. This point is never reached on a watchdog kill (that path panics above),
    // so a join here always follows a real child exit.
    let child_stderr = drain.join().expect("stderr drain thread panicked");

    // Exit status is NOT the data-flow gate. `graph_run` can exit 0 with no
    // data: graph validation / reactor-build failures warn-and-continue and
    // still return Ok, so a clean status only proves the binary didn't hit a
    // HARD build/transport-init error. The REAL data-flow gates are the
    // `.bin`-exists check + the sample-count assert (in `assert_mode`) + the
    // no-flood stderr assert just below. We still assert success because a
    // non-success exit IS a hard failure worth surfacing — just not the
    // thing that proves data flowed.
    assert!(
        status.success(),
        "[{mode_name}] `cerulion graph run rtt_bench` exited with {status} \
         (a hard build/transport-init failure — NOT the data-flow gate; that \
         is the .bin + sample-count + no-flood asserts)"
    );

    // The bug this test exists to catch: an iceoryx2 binary↔cdylib version
    // skew floods stderr with the connection-failure line while ZERO data
    // flows. Assert the flood is absent. A handful at startup is tolerable
    // (connection-setup races); hundreds+ is the bug. Counting per-line
    // keeps a single multi-line message from tripping it.
    let conn_fail_lines = child_stderr
        .lines()
        .filter(|l| l.contains(IOX2_CONN_FAIL_NEEDLE))
        .count();
    assert!(
        conn_fail_lines <= MAX_CONN_FAIL_LINES,
        "[{mode_name}] child stderr carried {conn_fail_lines} \"{IOX2_CONN_FAIL_NEEDLE}\" \
         lines (> {MAX_CONN_FAIL_LINES}) — this is the iceoryx2 binary↔cdylib version-skew \
         flood (Cargo.lock pin failed?), under which the event protocol breaks and no data \
         flows. See the step-3 Cargo.lock-pin comment."
    );

    let bin_path = dump_dir.join(format!("{raw_name}_{payload_size}.bin"));
    assert!(
        bin_path.exists(),
        "[{mode_name}] latency node should have dumped {} (payload={payload_size}B) \
         — graph did not complete (no samples flowed?)",
        bin_path.display()
    );
    read_samples_sorted_ns(&bin_path)
}

/// Minimum (uncontended floor) of an ascending-sorted slice (ns). This is
/// the metric the FLATNESS ratio is built on — robust to CI-VM jitter (noise
/// only adds latency, never lowers the floor) while still catching a real
/// O(n) copy (which inflates every iteration, the floor included). See
/// `flat_latency_test` / the "Payload-size sweep" doc section for why min,
/// not p50.
fn floor_ns(sorted: &[u64]) -> u64 {
    *sorted.first().expect("no samples")
}

/// Max (worst) RTT of an ascending-sorted slice (ns) — reported alongside
/// floor/p50/p99 for tail visibility. Not a gate input; the flatness ratio is
/// built on the FLOOR (see `floor_ns`).
fn max_ns(sorted: &[u64]) -> u64 {
    *sorted.last().expect("no samples")
}

/// Per-size measured stats in NANOSECONDS (f64): `(floor, p50, p99, max)`.
///
/// Also asserts the DATA-FLOW gate (`>= TARGET_SAMPLES`): a window that produced
/// no data is a real failure (the graph broke / a version skew starved the
/// chain), NOT a VM stall, so this check runs on EVERY window — the initial
/// sweep AND every window-health re-measurement (a retry that flows no data
/// still surfaces loudly).
fn measured_stats_ns(mode_name: &str, payload_size: usize, sorted: &[u64]) -> (f64, f64, f64, f64) {
    assert!(
        sorted.len() >= TARGET_SAMPLES,
        "[{mode_name}] expected >= {TARGET_SAMPLES} samples after warmup drop, got {} \
         (payload={payload_size}B) — data did NOT flow through the {mode_name}-clock graph",
        sorted.len()
    );
    (
        floor_ns(sorted) as f64,
        percentile_ns(sorted, 0.50) as f64,
        percentile_ns(sorted, 0.99) as f64,
        max_ns(sorted) as f64,
    )
}

/// Print the per-size stats table (the FLOOR gates flatness; p50/p99/max are for
/// human insight). Shared by the initial-sweep and post-retry prints.
fn print_stats_table(mode_name: &str, label: &str, stats: &[(usize, f64, f64, f64, f64)]) {
    println!(
        "=== CLI e2e graph RTT by payload size [{mode_name} clock] ({label}) \
         (FLOOR gates flatness) ==="
    );
    for &(size, floor, p50, p99, max) in stats {
        println!(
            "  {:>10} bytes : floor={:>8.2}µs  p50={:>8.2}µs  p99={:>8.2}µs  max={:>8.2}µs",
            size,
            floor / 1000.0,
            p50 / 1000.0,
            p99 / 1000.0,
            max / 1000.0,
        );
    }
}

/// Loose per-size liveness backstops on one (mode, size)'s FINAL (post-retry)
/// p50/p99. Runs AFTER window-health retries so a size whose window was elevated
/// by a VM stall is gated on its HEALED numbers, not the stalled ones. The TIGHT
/// gate is the cross-size flatness ratio; these only catch a gross stall / total
/// regression (see the ceiling constants' docs).
fn assert_liveness(
    mode_name: &str,
    size: usize,
    p50_ns: f64,
    p99_ns: f64,
    p50_ceiling_us: f64,
    p99_ceiling_us: f64,
) {
    let p50_us = p50_ns / 1000.0;
    let p99_us = p99_ns / 1000.0;
    assert!(
        p50_us < p50_ceiling_us,
        "[{mode_name}] p50 graph RTT {p50_us:.2}µs (payload={size}B) exceeds the \
         {p50_ceiling_us}µs liveness ceiling — CLI/cdylib graph latency regressed (or the \
         box is heavily loaded)"
    );
    assert!(
        p99_us < p99_ceiling_us,
        "[{mode_name}] p99 graph RTT {p99_us:.2}µs (payload={size}B) exceeds the loose \
         {p99_ceiling_us}µs tail backstop — a catastrophic latency tail (not just a shifted \
         median)"
    );
}

/// Run ONE clock mode across the payload sweep and gate it.
///
/// Hardens the shared-macOS-CI-VM stall class — the gate at line ~992 flaked
/// because a whole-subprocess VM stall lifts even one size's FLOOR, tripping the
/// max/min ratio on byte-identical code:
///
/// - the metric stays the per-size FLOOR (min RTT) — robust to jitter that only
///   ADDS latency (unchanged);
/// - WINDOW-HEALTH RETRIES are the SOLE single-window-stall
///   defense: any size whose floor exceeds `flatness_max × min_floor` (the
///   would-fail set) is RE-MEASURED in a fresh subprocess (bounded per-size
///   budget `MAX_RETRIES`), keeping the better floor — a transient stall clears
///   in a fresh window, a real copy re-measures just as slow. A healthy sweep
///   retries NOTHING (zero extra subprocess cost);
/// - the gate is the FULL `max/min` FLOOR ratio (NO drop-one — see the
///   `cerulion_core::testing::classify_flatness` rustdoc: this sweep's 2nd-largest
///   size is 16× smaller than its largest, so a real copy's 2nd-largest floor is
///   below the base RTT and drop-one would DISCARD the very floor the copy lives
///   in ⇒ a real regression would pass);
/// - the UNIFORM-STALL discriminator: an unhealed SIZE-INDEPENDENT stall
///   (`>= UNIFORM_STALL_MIN_SIZES` elevated floors in a tight band) fails as an
///   ATTRIBUTABLE, non-probative panic naming the signature + retry history —
///   while a real O(n) copy still fails the gate NORMALLY (the ceilings are
///   unchanged; the pin is not weakened).
///
/// The window-health-retry LIMITS drive two residuals (see the module header):
/// (1) a single/two-window stall that survives EVERY retry fails as `RealCopy`
/// (conservative — indistinguishable from a real single-size regression; a
/// re-run on a quiescent host disambiguates), and (2) a `>= 3`-window uniform
/// stall that survives lands in the attributable `UniformStall` arm.
///
/// (c) per-size INTERLEAVING is deliberately NOT applied (mirroring
/// `cross_thread_rtt_test`, and see the module header): the measurement unit here
/// is a whole SUBPROCESS, so there is no cheap round-interleaving, and the (b)
/// retry already re-measures a stalled size in a fresh temporal window. A
/// discarded WARM-UP is also not added — each subprocess already drops `WARMUP`
/// samples, and the flake is a VM stall (size-independent), not a DVFS cold head.
fn run_and_gate_mode(
    cerulion_bin: &Path,
    ws_root: &Path,
    mode_name: &str,
    virtual_clock: bool,
    p50_ceiling_us: f64,
    p99_ceiling_us: f64,
    flatness_max: f64,
) {
    // 1. Initial sweep: one watchdog-bounded subprocess per size + data-flow gate.
    let mut stats: Vec<(usize, f64, f64, f64, f64)> = PAYLOAD_SIZES
        .iter()
        .map(|&size| {
            let sorted = run_mode(cerulion_bin, ws_root, mode_name, virtual_clock, size);
            let (floor, p50, p99, max) = measured_stats_ns(mode_name, size, &sorted);
            (size, floor, p50, p99, max)
        })
        .collect();
    print_stats_table(mode_name, "initial sweep", &stats);

    // 2. Window-health retries. The pure orchestration lives in
    //    cerulion_core::testing (unit-tested there); the subprocess re-measurement
    //    is injected here. Only elevated sizes are re-run; a flat sweep is a no-op.
    let retry_log = cerulion_core::testing::run_window_health_retries(
        &mut stats,
        flatness_max,
        MAX_RETRIES,
        |size| {
            let sorted = run_mode(cerulion_bin, ws_root, mode_name, virtual_clock, size);
            measured_stats_ns(mode_name, size, &sorted)
        },
    );
    if retry_log.is_empty() {
        println!(
            "[{mode_name}] window-health: all per-size floors within {flatness_max:.1}x of the \
             min floor — no retries"
        );
    } else {
        println!(
            "[{mode_name}] window-health: {} re-measurement attempt(s) on elevated size(s):",
            retry_log.len()
        );
        for r in &retry_log {
            println!(
                "  [{mode_name}] retry {} bytes round {} attempt {}/{}: old floor={:.2}µs -> \
                 measured={:.2}µs, kept={:.2}µs",
                r.size,
                r.round,
                r.attempt,
                MAX_RETRIES,
                r.old_floor / 1000.0,
                r.measured_floor / 1000.0,
                r.kept_floor / 1000.0,
            );
        }
        print_stats_table(mode_name, "after window-health retries", &stats);
    }

    // 3. Loose per-size liveness backstops on the FINAL (post-retry) stats.
    for &(size, _floor, p50, p99, _max) in &stats {
        assert_liveness(mode_name, size, p50, p99, p50_ceiling_us, p99_ceiling_us);
    }

    // 4. The TIGHT gate: FULL max/min FLOOR flatness (NO drop-one — see the
    //    `classify_flatness` rustdoc; this sweep's 2nd-largest size is 16× smaller
    //    than its largest, so a real copy's 2nd-largest signal is below the base
    //    RTT and drop-one would hide it) + the uniform-stall discriminator, via the
    //    pure, oracle-tested `classify_flatness`. Single-window stalls were already
    //    absorbed in step 2 by the retry; a stall that survived every retry fails
    //    here as RealCopy (the conservative residual — a human re-runs).
    let floors: Vec<(usize, f64)> = stats.iter().map(|&(s, f, ..)| (s, f)).collect();
    match cerulion_core::testing::classify_flatness(&floors, flatness_max) {
        FlatnessVerdict::Pass(ff) => {
            println!(
                "CLI e2e graph FLATNESS [{mode_name} clock]: PASS — max/min ratio={:.3}x \
                 (min={:.2}µs @ {}B, max={:.2}µs @ {}B, ceiling {:.1}x). A single VM-stalled \
                 window is absorbed by the window-health retry, not by dropping a floor.",
                ff.ratio,
                ff.min.1 / 1000.0,
                ff.min.0,
                ff.max.1 / 1000.0,
                ff.max.0,
                flatness_max,
            );
        }
        FlatnessVerdict::UniformStall(ff) => {
            let floors_str = stats
                .iter()
                .map(|&(s, f, ..)| format!("{s}B={:.2}µs", f / 1000.0))
                .collect::<Vec<_>>()
                .join(", ");
            let retry_str = if retry_log.is_empty() {
                "none".to_string()
            } else {
                retry_log
                    .iter()
                    .map(|r| {
                        format!(
                            "{}B a{}: {:.2}->{:.2} kept {:.2}µs",
                            r.size,
                            r.attempt,
                            r.old_floor / 1000.0,
                            r.measured_floor / 1000.0,
                            r.kept_floor / 1000.0,
                        )
                    })
                    .collect::<Vec<_>>()
                    .join("; ")
            };
            panic!(
                "NON-PROBATIVE: the [{mode_name}] CLI-e2e flatness gate failed with a \
                 SIZE-INDEPENDENT uniform-stall signature, NOT a payload-size-dependent copy. \
                 >= {} per-size floors are elevated (> {:.1}x the {:.2}µs min floor @ {}B) yet \
                 fall within a tight {:.2}x band of EACH OTHER — the shape of a shared-CI-VM \
                 stall spanning multiple SUBPROCESS windows that survived every retry, which a \
                 real O(n) copy (adjacent floors differ by the >= 16x payload-size step) can \
                 never produce. This is an ATTRIBUTABLE infrastructure red, not a zero-copy \
                 regression — re-run on a quiescent host. Final per-size floors: [{}]. \
                 Window-health retry history ({} attempt(s)): [{}]. max/min ratio = {:.3}x, \
                 ceiling = {:.1}x.",
                cerulion_core::testing::UNIFORM_STALL_MIN_SIZES,
                flatness_max,
                ff.min.1 / 1000.0,
                ff.min.0,
                cerulion_core::testing::UNIFORM_STALL_BAND,
                floors_str,
                retry_log.len(),
                retry_str,
                ff.ratio,
                flatness_max,
            );
        }
        FlatnessVerdict::RealCopy(ff) => {
            panic!(
                "[{mode_name}] cross-payload FLOOR flatness {:.3}x (FULL max/min, NO outlier \
                 dropped) exceeds the {:.1}x ceiling — the variable payload is being COPIED \
                 somewhere on the {mode_name} graph path (a zero-copy path is O(1) in payload ⇒ \
                 the floor is flat; a memcpy regression shows tens-of-×). The shape is NOT the \
                 uniform-stall signature and the window-health retry did NOT heal it, so this is \
                 a REAL regression. (Residual: a RARE persistent single/two-window VM stall that \
                 survives every retry is indistinguishable from a real single-size regression and \
                 lands here too — if a re-run on a quiescent host clears it, it was a stall, not a \
                 copy.) max floor = {:.2}µs @ {}B, min floor = {:.2}µs @ {}B.",
                ff.ratio,
                flatness_max,
                ff.max.1 / 1000.0,
                ff.max.0,
                ff.min.1 / 1000.0,
                ff.min.0,
            );
        }
    }
}

#[test]
fn test_cli_e2e_user_graph_latency() {
    let tmp = tempfile::tempdir().expect("tempdir");

    // ── 1. Workspace via the CLI ─────────────────────────────
    // workspace_create walks up from this test binary's current_exe()
    // (target/<profile>/deps/..., i.e. target/release/deps/... under
    // `cargo test --release`) to find cerulion_core/Cargo.toml, so the
    // generated workspace Cargo.toml gets ABSOLUTE path deps to the
    // LOCAL crates — cdylibs built below compile against local changes.
    let ws = workspace_create(tmp.path(), "rtt_ws").expect("workspace_create");
    let cargo_toml = ws.root.join("Cargo.toml");

    // ── 2. Scaffold the 3 node crates via the CLI ───────────
    // node_create_with_options declares the ports/policy/trigger through
    // the CLI surface (Cargo.toml + workspace member + a valid macro
    // lib.rs are all CLI-generated). We then overwrite each src/lib.rs
    // with the real tick body (init env-read + variable-field publish +
    // sample dump can't be expressed via node_modify_add_port).

    // ping: periodic source, one Image output.
    node_create_with_options(
        &ws.nodes_dir,
        &cargo_toml,
        "ping",
        Some(cerulion_core::MacroPolicy::Period { period_ms: 1 }),
        &NodeCreateOptions {
            outputs: vec![("sensor_msgs::Image".to_string(), "ping_out".to_string())],
            inputs: vec![],
            trigger: None,
            raw_ffi: false,
            language: NodeLanguage::Rust,
        },
    )
    .expect("node_create ping");
    write_node_body(&ws.nodes_dir, "ping", PING_SRC);

    // pong: data-triggered echo, Image in (trigger) + Image out.
    node_create_with_options(
        &ws.nodes_dir,
        &cargo_toml,
        "pong",
        None, // data-trigger is field-driven via `trigger`
        &NodeCreateOptions {
            outputs: vec![("sensor_msgs::Image".to_string(), "echo_out".to_string())],
            inputs: vec![("sensor_msgs::Image".to_string(), "ping_in".to_string())],
            trigger: Some("ping_in".to_string()),
            raw_ffi: false,
            language: NodeLanguage::Rust,
        },
    )
    .expect("node_create pong");
    write_node_body(&ws.nodes_dir, "pong", PONG_SRC);

    // latency: data-triggered sink, Image in (trigger), no outputs.
    node_create_with_options(
        &ws.nodes_dir,
        &cargo_toml,
        "latency",
        None,
        &NodeCreateOptions {
            outputs: vec![],
            inputs: vec![("sensor_msgs::Image".to_string(), "echo_in".to_string())],
            trigger: Some("echo_in".to_string()),
            raw_ffi: false,
            language: NodeLanguage::Rust,
        },
    )
    .expect("node_create latency");
    write_node_body(&ws.nodes_dir, "latency", LATENCY_SRC);

    // ── 3. Build the cdylibs via the CLI ────────────────────
    // node_build = `cargo build -p <node> --release` in the workspace root,
    // against the LOCAL cerulion_core path dep. RELEASE build (release=true)
    // — production-latency numbers only make sense in release — AND it
    // matches the subprocess load profile below (`cerulion graph run
    // --release` prefers `target/release`).
    //
    // CRITICAL: pin the workspace's dependency versions to THIS repo's
    // `Cargo.lock` *before* building. The `cerulion` binary we spawn below
    // is built from the repo's lock (iceoryx2 0.9.1 today). A freshly
    // scaffolded workspace has NO `Cargo.lock`, so `cargo build` would
    // re-resolve to the newest compatible iceoryx2 (0.9.2) — and the
    // cdylibs would then link a DIFFERENT iceoryx2 than the binary that
    // dlopen-loads them. The two iceoryx2 versions use incompatible
    // shared-memory event protocols: the binary's Notifier cannot
    // establish event connections to the cdylib's Listeners ("Unable to
    // establish connection to new receiver" × 120k), so events never
    // fire, the data-trigger chain never wakes, and REAL-clock mode
    // (event-driven WaitSet) flows ZERO samples (virtual-clock polling
    // limps along but still spams). Copying the repo lock forces the
    // cdylibs onto the identical iceoryx2 as the binary — exactly what
    // the in-tree `graph_rtt_bench` workspace does via its committed
    // `Cargo.lock`. See run-mechanism comment in step 5.
    let repo_root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("cerulion_core's parent is crates/")
        .parent()
        .expect("crates/ has a parent (the repo root)")
        .to_path_buf();
    let repo_lock = repo_root.join("Cargo.lock");
    assert!(
        repo_lock.exists(),
        "repo Cargo.lock not found at {} — needed to pin the workspace's \
         iceoryx2 to the binary's version",
        repo_lock.display()
    );
    std::fs::copy(&repo_lock, ws.root.join("Cargo.lock"))
        .unwrap_or_else(|e| panic!("copy repo Cargo.lock into workspace: {e}"));

    for node in ["ping", "pong", "latency"] {
        node_build(&ws.root, node, true)
            .unwrap_or_else(|e| panic!("node_build {node} failed: {e}"));
    }

    // Verify the lock pin was EFFECTIVE, not just present. `node_build`
    // doesn't pass `--locked` (no such param), so a member crate forcing a
    // bump, or cargo re-resolving, would silently change the workspace
    // lock's iceoryx2 away from the binary's — exactly the skew the copy
    // above is meant to prevent. Cheap, deterministic assert closes that
    // gap: after the build, the workspace iceoryx2 must STILL equal the
    // repo's. (The no-flood stderr assert later is the behavioral backstop;
    // this is the early, precise one.)
    let repo_iox2 =
        iceoryx2_version_in_lock(&repo_lock).expect("repo Cargo.lock must pin an iceoryx2 version");
    let ws_iox2 = iceoryx2_version_in_lock(&ws.root.join("Cargo.lock"))
        .expect("workspace Cargo.lock must pin an iceoryx2 version after the cdylib build");
    assert_eq!(
        ws_iox2, repo_iox2,
        "cdylib build re-resolved iceoryx2 to {ws_iox2} despite the Cargo.lock pin \
         (repo pins {repo_iox2}) — the lock copy was ineffective; the cdylibs would link \
         a DIFFERENT iceoryx2 than the binary and the event protocol would break"
    );

    // ── 4. Wire the graph via the CLI ───────────────────────
    // graph_create writes the YAML shell; node_stage appends each NodeDef
    // and round-trips through validate_graph + disk. A fixed prefix keeps
    // the derived iceoryx2 topic names stable.
    graph_create(&ws.graphs_dir, "rtt_bench", Some("rtt_ws")).expect("graph_create");

    // ping: one output. max_slice_len is intentionally OMITTED — it resolves to
    // sensor_msgs/Image's tier-2 codegen default (16 MiB, silently, via
    // `<Image as ShmMessage>::MAX_SLICE_LEN`), and the 64B test payload fits
    // trivially. Leaving it unspecified is the point: a user shouldn't hand-size
    // SHM for a known message type. (This also exercises the tier-2 auto-default
    // resolution end to end.)
    let ping_def = build_node_def(
        "ping",
        Some("ping"),
        &[(
            "ping_out".to_string(),
            Some("sensor_msgs::Image".to_string()),
        )],
        &[],
    );
    node_stage(&ws.graphs_dir, "rtt_bench", ping_def).expect("stage ping");

    // pong: input bound to ping/ping_out, one output. max_slice_len omitted
    // too (tier-2 Image default, as for ping).
    let pong_def = build_node_def(
        "pong",
        Some("pong"),
        &[(
            "echo_out".to_string(),
            Some("sensor_msgs::Image".to_string()),
        )],
        &[("ping_in".to_string(), "[ping,ping_out]".to_string())],
    );
    node_stage(&ws.graphs_dir, "rtt_bench", pong_def).expect("stage pong");

    // latency: input bound to pong/echo_out, no outputs.
    let latency_def = build_node_def(
        "latency",
        Some("latency"),
        &[],
        &[("echo_in".to_string(), "[pong,echo_out]".to_string())],
    );
    node_stage(&ws.graphs_dir, "rtt_bench", latency_def).expect("stage latency");

    // ── 5. Build the `cerulion` binary, then RUN the graph as a SUBPROCESS ──
    // Build the binary first so the spawned process reflects local code
    // (the on-disk target/release/cerulion may be stale). `cargo build -p
    // cerulion_cli --release` from the repo root (`repo_root` computed in
    // step 3). RELEASE so the binary matches both the release cdylibs it
    // loads and the `target/release/cerulion` path the `current_exe()`
    // walk-up resolves under `cargo test --release`.
    // `.env_remove("CARGO_TARGET_DIR")`: strip any ambient CARGO_TARGET_DIR
    // so the build output lands at the canonical `<repo_root>/target/release/`
    // — exactly the path `cerulion_bin_path(&repo_root)` resolves below. Both
    // sides key off `repo_root` (the compile-time `CARGO_MANIFEST_DIR`), so
    // build output and resolution stay in lockstep regardless of ambient env
    // (a `current_exe()` walk-up WOULD honor CARGO_TARGET_DIR and diverge).
    let build_status = Command::new(env!("CARGO"))
        .args(["build", "-p", "cerulion_cli", "--release"])
        .current_dir(&repo_root)
        .env_remove("CARGO_TARGET_DIR")
        .status()
        .expect("spawn `cargo build -p cerulion_cli --release`");
    assert!(
        build_status.success(),
        "`cargo build -p cerulion_cli --release` failed — cannot run the subprocess graph"
    );

    let cerulion_bin = cerulion_bin_path(&repo_root);
    assert!(
        cerulion_bin.exists(),
        "cerulion binary not found at {} after build — the release build did \
         not produce it at the canonical <repo_root>/target/release path",
        cerulion_bin.display()
    );

    // ── 6. Run BOTH clock modes across the payload sweep + assert ──
    // For EACH mode we sweep PAYLOAD_SIZES (64 B → 1 MiB), running a fresh
    // watchdog-bounded subprocess per size, asserting the loose per-size
    // liveness ceilings, and collecting the per-size FLOOR for the tight
    // cross-size FLATNESS gate.
    //
    // real (default, no flag): live WaitSet reactor — a LIVENESS gate (loose
    //   absolute ceiling). On Linux CI virtual ≈ real ≈ ~23µs (real floor
    //   ~23.4µs), so the live-WaitSet OS-wakeup is NOT the dominant cost; the
    //   flatness across payloads is what matters (a non-flat curve = a hidden
    //   per-payload copy).
    // virtual (--time-source virtual): uncapped poll loop — the PRODUCTION-
    //   latency regression gate. Floor ~23.2µs through the full real binary +
    //   cdylib + subprocess path (≈ in-process step() ~7.7µs + subprocess /
    //   cdylib-FFI / CLI overhead; transport-bound). RTTs are real
    //   in both modes because the nodes timestamp via real_ns() (independent of
    //   the runtime clock). Measured on a CI run.
    //
    // The flatness ratio (FLOOR-based, FULL max/min — deliberately NO drop-one
    // (on this sweep drop-one discards the 1 MiB floor a
    // real copy lives in) — + window-health retries + uniform-stall
    // discriminator) is the tight gate; the
    // per-size p50/p99 ceilings are loose liveness backstops. See the module-doc
    // "Payload-size sweep" section and `run_and_gate_mode`.

    // real first (the slower, looser mode), then virtual. Each call sweeps
    // PAYLOAD_SIZES, re-measures any VM-stalled window, asserts the loose
    // liveness ceilings on the healed stats, and gates the robust flatness ratio.
    run_and_gate_mode(
        &cerulion_bin,
        &ws.root,
        "real",
        false,
        P50_CEILING_REAL_US,
        P99_CEILING_REAL_US,
        FLATNESS_MAX_REAL,
    );
    run_and_gate_mode(
        &cerulion_bin,
        &ws.root,
        "virtual",
        true,
        P50_CEILING_VIRTUAL_US,
        P99_CEILING_VIRTUAL_US,
        FLATNESS_MAX_VIRTUAL,
    );
}
