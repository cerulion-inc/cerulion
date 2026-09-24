// SPDX-License-Identifier: AGPL-3.0-only
//! Cerulion CLI engine — all command logic as a testable library.
//!
//! This crate contains the implementation for every CLI command.
//! The binary crate (`cerulion_cli`) provides clap dispatch.

#![deny(dead_code, unused_imports, unused_variables)]
// Principle 12 (logging; see the Logging convention in `AGENTS.md`): library
// code never prints. It logs through `tracing`. Scoped `not(test)` so unit
// tests keep printing diagnostics, and applied at the crate root rather than
// in `[workspace.lints]` because that table cannot distinguish a lib target
// from a test binary. Pinned by
// `crates/cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

// The CLI's LOCAL account state (`~/.cerulion/auth.json`) + the pure
// runtime login-gate classifier. Network-free — the device-code
// login FLOW that populates this state lives in `login_cmd`.
pub mod auth;
// Account self-service device management (list/revoke) + the robot-ACL
// revocation engine seam (Studio / web account page; there is no CLI verb).
pub mod account_cmd;
// `cerulion bag play` / `bag info` / `bag record` — the bag-as-a-data-source
// verbs. `play` re-publishes recorded frames BYTE-VERBATIM onto local SHM, wall-paced
// (the robot-substitute source for desk-side work); `record` is the desk-side
// observational capture that produces such a bag. Distinct from `bag play --resim`,
// which RE-EXECUTES the bagged graph (and with `--verify`, verifies it). Reads bags
// via `cerulion_bag`, which is `#![cfg(unix)]`, so the module is gated here exactly
// like `replay_cmd`.
#[cfg(unix)]
pub mod bag_cmd;
// `cerulion bag migrate` — rewrite a bag whose embedded `graph.yaml`
// carries keys the graph format no longer defines (an older bag embeds the
// ON-DISK graph file, which could still carry a legacy `policy:` block, and
// an MCAP attachment is sealed). Writes a NEW bag through `cerulion_bag`'s
// writer — never in place — so the module is `cfg(unix)` like `bag_cmd`.
#[cfg(unix)]
pub mod bag_migrate;
// Making a signal Cerulion SENDS a child actually REACH it. An ignored
// disposition AND the blocked set both survive `execve`, so a launcher that
// ignores or blocks SIGINT (a shell script's `cmd &`, `nohup`) would otherwise
// make every graceful stop undeliverable. `pub(crate)`: an internal spawn
// rule, not CLI surface.
pub(crate) mod child_signals;
// The `cerulion connect` verb's PURE resolution (robot eid +
// argv + the `cerulion-connectd` sibling-binary path). The CLI stays iroh-free —
// it spawns the resolved binary.
pub mod connect_cmd;
// The value sources behind shell tab-completion — live topic / node /
// graph / schema / robot names for a TAB press. Every source is an INSTANT
// LOCAL read under a hard wall-clock budget: no process is ever spawned (netd
// least of all) and no network is ever touched, because this runs on the
// keystroke. The clap tree completes itself; this is the half clap cannot know.
pub mod completions;
// The device-code login FLOW + account-service HTTP client + the
// first-need runtime gate entry (`ensure_login_gate`). Dials the account service
// (`CERULION_ACCOUNT_SERVICE` / the production default) via reqwest.
pub mod login_cmd;
// The robot-gateway discovery LADDER — the shared peer types + the
// parallel gather engine (mDNS / cache / hostname rungs under one ceiling).
pub mod discovery_ladder;
pub mod error;
// `cerulion flashback` — the operator's manual trigger, and the
// one surface a capture refusal's loud, numbered report has to reach.
#[cfg(unix)]
pub mod flashback_cmd;
pub mod graph_cmd;
// The hostname-convention discovery rung (well-known robot names).
pub mod hostname_peers;
pub mod ipc_cleanup;
// The mDNS discovery rung (pure-Rust `mdns-sd` browse — the primary rung).
pub mod mdns_discovery;
// The LOCAL ament harvest rung — a filesystem-only
// `SchemaAcquirer` over the host's `$AMENT_PREFIX_PATH` ROS install.
pub mod local_harvest;
pub mod multiprocess;
pub mod node_cmd;
pub mod node_metadata;
// The `cerulion pair` verb — PURE eid/name resolution ladder + the
// `cerulion-connectd pair` argv builder + robots.toml pinning (the CLI stays
// iroh-free; the CPace ceremony lives in the spawned sibling).
pub mod pair_cmd;
// Login-gated ownership at install: `register_robot` (`POST
// /v1/robots`) binds the logged-in account as THIS machine's owner and returns
// the offline-verifiable owner grant. The install-funnel entry (a robot install,
// not a desk); dials the account service via reqwest.
pub mod robot_cmd;
// The desk-side device↔account binding resolver: reads the cached
// `SignedDeviceCert` (~/.cerulion/device.cert) and resolves this machine's VERIFIED
// cloud AccountId (asserting the cert attests THIS machine's device key). The
// desk half of the binding the robot side enforces via `device_index`.
pub mod device_binding;
// The peer-cache discovery rung + best-effort persist to
// `~/.cerulion/peers.json` (warm peers for the next run).
pub mod peer_cache;
// Rung 4: the OPT-IN unicast subnet-sweep discovery rung (`--scan`).
// Structurally gated behind the flag (never fires on a default run — a
// horizontal SYN sweep reads as port-scan recon to corporate IDS).
pub mod node_inspector;
pub mod subnet_sweep;
pub mod workspace_lock;
// The surgical top-level-block splice machinery shared by
// `partition_emit`'s `process_groups:` rewrite and `graph_cmd`'s `node stage`
// (comment-preserving; never a lossy serde round-trip). Crate-internal — the
// callers are the public surface.
pub(crate) mod yaml_splice;
// Auto-partition emit glue + surgical `process_groups:` YAML
// rewrite (comment-preserving; never a lossy serde round-trip).
pub mod partition_emit;
// The re-execution ENGINE entry point (driven today by `cerulion bag
// play --resim`; the module keeps its original name) — reads bags via
// `cerulion_bag`, which is
// `#![cfg(unix)]` (it reuses `cerulion_core`'s Unix-only `trace_ring`), so the
// module is Unix-gated. The CLI dispatcher returns a loud "unsupported on this
// platform" error off Unix (mirrors `cerulion bagd`).
#[cfg(unix)]
pub mod replay_cmd;
// The replay ENGINE (runtime construction, injection, capture,
// diff, report) that `replay_cmd` calls after its entry gates pass. Same
// Unix-gating rationale as `replay_cmd` (it drives `cerulion_bag` + the
// transport).
#[cfg(unix)]
pub mod replay_engine;
// The `cerulion bag play --resim` SURFACE
// over that engine — flag legality and the neutral-vs-`--verify` exit contract.
// A verb layer only: it calls `replay_cmd::run_replay` and changes nothing
// inside it.
//
// NOT Unix-gated, unlike its callee: parsing a selection and judging a flag
// combination are platform-independent, and `main` refuses a malformed
// invocation (above the login gate) on EVERY platform the binary builds for.
// Only the two functions that touch the bag reader carry `#[cfg(unix)]`, inside
// the module. `crates/cerulion_cli/src/cfg_symmetry_tests.rs` pins that split.
pub mod resim_cmd;
// The `--tolerance` YAML schema + strict validation (exit 4)
// and the by-name field registry / read-only frame decoder it validates
// against. Platform-independent (pure schema/registry logic, no bag reader),
// so NOT Unix-gated — `replay_cmd` (Unix) wires them into the pre-engine gate.
pub mod replay_field_registry;
// The PURE per-rank replay planner (rank membership from
// the recorded trace manifests, per-rank subgraph restriction, cross-rank vs
// external read classification, the recorded step slice). Platform-independent
// for the same reason as `replay_field_registry` — plain data + `GraphConfig`,
// no bag reader and no transport — so NOT Unix-gated, unlike the
// `replay_engine` executor that consumes it.
pub mod replay_rank;
// The PURE read-log-steered injection planner — which
// recorded frames of a cross-rank topic are due before which step of the rank
// being replayed, derived from that rank's own kind-6 read log.
// NOT Unix-gated, unlike `replay_engine`: it takes decoded records and frame
// sequences and returns a schedule, touching neither `cerulion_bag` nor the
// transport, so it builds and tests on every platform the crate does.
pub mod replay_inject;
// The PURE rank-local re-derivation VERIFIER. Replay fires
// are trace-driven, so the structural "would this schedule have
// happened?" verdict is produced here — Period / `throttle_ms` / Sync / FIFO
// pop counts re-derived from the recorded clock + reads and compared
// positionally against the trace, with the cross-rank `block` gate DEMOTED to a
// non-gating conservation assert. It is pure (numbers in, decisions out) and
// would compile anywhere; the `#[cfg(unix)]` is INHERITED from the one
// vocabulary it slots into (`replay_engine::DivergenceClass`), a second copy of
// which is exactly the drift the one-vocabulary rule forbids.
#[cfg(unix)]
pub mod replay_rederive;
// The READER that turns a recorded bag's `__cerulion/state`
// records + `state_coverage.json` manifest into the `AnchorFact`s the pure
// decision layer judges. Unix-gated for the SAME reason as `replay_cmd` /
// `replay_engine`, and it is a HARD requirement rather than a convention: the
// module's types come from `cerulion_bag` (`BagReader`, `STATE_TOPIC`) and
// `cerulion_bagd` (`StateCoverage`, `STATE_COVERAGE_ATTACHMENT`), both of which
// are declared under `[target.'cfg(unix)'.dependencies]` because their own
// crates are `#![cfg(unix)]` (they reuse `cerulion_core`'s POSIX trace rings).
// Ungated, a non-Unix build fails at this module's `use` lines.
#[cfg(unix)]
pub mod replay_state;
// The RUN DIRECTORY + its `/__cerulion/runs` registry record
// — written on EVERY `graph run`, not only under `--record`, so an attaching
// recorder can find a live run and read the same description a
// `graph run --record` bag embeds. UNIX ONLY as a FEATURE: the owner-only
// permissions it exists to guarantee are a `std::os::unix` implementation, the
// artifact renderers it is fed are Unix-gated, and so is the `graph_cmd` call
// site (a non-Unix run says it is undescribed, loudly, rather than skipping
// silently). The module itself is not `#[cfg]`-gated — it compiles everywhere,
// with the mode-setting arms inert — so this declaration is unconditional; do
// NOT read that as the feature being portable. See the module docs.
pub mod run_dir;
// The `flock`-based liveness oracle a run-directory sweeper
// reads, and the sweeper itself. Split from `run_dir` because they answer a
// question ABOUT run directories rather than writing one, and because the
// sweep's policy half is pure and wants its own oracle vectors.
pub mod run_lock;
// `#[cfg(unix)]`, and NOT for the same reason as its sibling. `run_lock`
// compiles everywhere (its non-Unix arm reports UNKNOWN, which is the
// never-sweep answer), but the sweeper RESOLVES POSIX SHM NAMES — it names
// `cerulion_core::{shm_ring, state_arm, state_ring}`, every one of which is
// itself `#[cfg(unix)]` because POSIX shared memory is what they are. Declaring
// it unconditionally is a non-Unix BUILD FAILURE, not a feature that degrades.
// The mp machinery is gated the same way for the same reason.
#[cfg(unix)]
pub mod run_sweep;
// `cerulion ros2 attach` — DDS discovery report + bridge config/graph
// generation. PURE over `cerulion_dds`'s DDS-free discovery types (the live
// backend is a separate crate); fully oracle-testable without a DDS peer.
pub mod ros_cmd;
// `cerulion ros2 run` — env orchestration + transparent exec() for running
// bare ROS 2 launch files on Cerulion transport (`RMW_IMPLEMENTATION=
// rmw_cerulion`). Platform-independent plan building; the exec() half is
// `#[cfg(unix)]` with a loud non-Unix stub (mirrors `bagd`'s stub).
pub mod ros2_cmd;
// `ros2:` graph entries — spawn + supervise stock ROS 2 processes beside a
// native graph under `cerulion graph run` (v1: spawn/supervise only, no
// scheduling). Reuses the multi-process supervisor's `ChildGuard`.
pub mod ros2_graph;
// `cerulion ros2 migrate` — rewrite a colcon workspace's C++
// publish call sites to the upstream loaned-message API where the clang
// prover (tools/ros2_migrate/, container-built — the Rust tree never links
// libclang) proves it safe; dry-run default + manual-candidates report +
// rclpy report-only + machine-readable manifest; `--write` = consent gate,
// clean-git-tree refusal, ONE commit + patch file, then an automatic
// `colcon build --packages-select` of the affected packages.
pub mod ros2_migrate;
pub mod schema_cmd;
pub mod schema_serve;
// The macOS `/tmp/*.shm_state` population — detect it, price it, and
// reclaim only what is PROVABLY dead. `iceoryx2-pal-posix` keeps one such file
// per SHM segment on a platform with no `/dev/shm`, a SIGKILLed owner leaks it
// forever, and `shm_list()` is a FULL READDIR of `/tmp` that every startup path
// sweeping dead nodes walks into. Pure over an injected directory + system
// probe, so the whole thing is oracle-testable with no real processes and no
// real shared memory.
pub mod shm_state;
// The checkpoint arm TAG a running graph process is handed,
// and the (best-effort, never fatal) attach it drives.
pub mod state_arm_attach;
// The workspace `.msg` schema store reader
// (`schemas/<pkg>/msg/<Type>.msg`), shared by attach's resolvability chain
// and `schema list`/`info`. Pure (`parse_rosmsg`); no DDS, no transport.
pub mod schema_store;
// OPTIONAL SYSTEM dependencies for node crates. A Cargo build script
// cannot enable a feature of its own crate, so "link GStreamer if this machine
// has it" cannot be expressed in Cargo at all — the probe lives here, and
// `node build` consults it. Pure decision core + an injected pkg-config probe.
pub mod system_deps;
// The shared near-miss key predicate for hand-authored config
// files whose FOREIGN sibling keys must stay legal (`~/.cerulion/config.toml`,
// `~/.cerulion/robots.toml`), plus the Levenshtein bound `system_deps`'
// `package.metadata` twin also uses. Crate-internal — the warns are the surface.
pub(crate) mod near_miss;
pub mod templates;
pub mod tolerance;
pub mod tolerance_metrics;
pub mod topic_cmd;
pub mod utils;
// `cerulion viz [TOPIC…]` is a THIN CLIENT of the
// long-lived `cerulion-vizd` DAEMON (it no longer compiles/materializes a graph
// per run). `viz_client` is the rerun-free daemon control client (socket
// resolution, detached spawn, connect, one `attach` per topic); `viz_cmd` keeps
// the `$CERULION_RERUN_URL` mirror. The Rerun SDK stays in the isolated
// `cerulion_viz/` workspace; these modules MIRROR its constants (never link it).
pub mod viz_client;
pub mod viz_cmd;
// The ZERO-CONFIG wedge alarm's two PURE halves — the per-node
// threshold ladder (declared `tick_within_ms` → `period_ms` → profiled p50 → a
// 5 s floor) and the clock-free dwell rule the supervisor's JOIN loop decides
// against. It watches the one condition `tick_within_ms` structurally cannot see:
// a tick that never RETURNS. The cross-process transport is
// `cerulion_core::wedge_page`.
pub mod wedge_alarm;
pub mod workspace;

/// The ONE crate-wide lock serializing every env-mutating
/// **lib** test in `cerulion_cli_engine`.
///
/// libtest runs all `#[cfg(test)] mod tests` of a library in a SINGLE test binary,
/// so the process environment is shared across every module. Before this module
/// each file kept its own private `env_lock()` mutex — six of them — which
/// serialized a module against ITSELF and against nothing else. Two modules that
/// both set `HOME` (`connect_cmd::tests::plan_keys_the_epoch_cache_by_the_desk_verified_name_or_by_nothing`
/// and `account_cmd::tests::the_writer_resolves_the_shared_epoch_cache_path`)
/// therefore raced 12/12 under `--test-threads=2`: each held a different mutex,
/// each pointed `HOME` at its own tempdir, and whichever ran second silently
/// re-homed the first.
///
/// Every env-mutating lib test in this crate must take THIS guard (module-level
/// `env_lock()` helpers are thin delegates to it), so adding a new env-mutating
/// test in any module is automatically serialized against all the others.
///
/// **`#[serial_test::serial]` is NOT a substitute.** The crate DOES carry a
/// `serial_test` dev-dep and several lib tests use it (for process-global state
/// that is not the environment: the SIGINT disposition, the iceoryx2 SHM
/// singleton). But `#[serial]` takes serial_test's OWN mutex, so a `#[serial]`
/// env test and a `test_env` env test are in two INDEPENDENT domains and run
/// concurrently: exactly the shape this lock exists to kill. Every env-reading
/// test takes it, `mdns_discovery`'s `CERULION_STATE_ROOT` tests
/// and `viz_client::tests::socket_env_override_wins_and_default_ladder`
/// (which reads `HOME`) included. A test that is BOTH env-mutating and
/// `#[serial]` for an unrelated reason takes both (the `graph_cmd`
/// netd-routing tests); lock order is always `#[serial]` first (the attribute)
/// then this guard (the body), so the two can never deadlock.
///
/// Scope note: integration tests under `tests/` are separate binaries with their
/// own processes, so they keep their own file-local locks — a crate-wide static
/// could not reach across process boundaries anyway.
///
/// Poison is deliberately absorbed (`into_inner`): a panicking test leaves the
/// env restored by its own RAII snapshot guard, and poisoning the shared lock
/// would cascade one failure into every other env test in the crate.
#[cfg(test)]
pub(crate) mod test_env {
    /// The one shared mutex. Prefer [`env_lock`]; this accessor exists for the
    /// call sites that acquire the guard themselves (e.g. to pair it with an RAII
    /// env guard in a fixed drop order).
    pub(crate) fn env_mutex() -> &'static std::sync::Mutex<()> {
        static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
        &LOCK
    }

    /// Acquires the crate-wide env lock. Hold the returned guard for the whole
    /// body of any test that reads or writes process-global environment state.
    pub(crate) fn env_lock() -> std::sync::MutexGuard<'static, ()> {
        env_mutex().lock().unwrap_or_else(|p| p.into_inner())
    }
}
