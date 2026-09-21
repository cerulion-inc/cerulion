# cerulion_core internals: the test map

Every integration test binary under `crates/cerulion_core/tests/`, what it pins, and how to run
it. Companions: `core-transport.md`, `core-scheduler-graph.md`. Code on `main` beats this
document; when a row and the file's own header disagree, the header wins; fix the row.

## Ground rules

- **Never `cargo test --workspace`**: it launches ~100 test programs at once and the
  shared-memory tests can deadlock forever (`#[serial]` protects only within one
  program). Run per crate, and shared-memory binaries individually.
- `cargo test -p cerulion_core` runs binaries one after another, but each binary's tests
  run on parallel threads; that is exactly what the `tt1` rows cannot tolerate.
- CI SHARDS this crate 4 ways on Linux and 2 ways on macOS.
  `./tools/scripts/ci_test_shard.sh cerulion_core <n> <count>` enumerates `tests/*.rs` at
  depth 1 and takes every file whose position is `index mod count`, so a new test file
  joins a shard with nothing to edit, with ONE named exception: the script PINS
  `macro_compile_fail_test` to shard `PINNED_SHARD % count` (2 of 4 on Linux, 0 of 2 on
  macOS), because that trybuild harness is the serial tail: one test that is essentially the
  whole of whichever quarter holds it. Unpinned, its position shifts whenever any unrelated
  test file is added, moving which runner is the critical path. Its SIZE is a per-run
  measurement this doc does not restate: the reading lives in `PINNED_TEST` in that script,
  which is also where the bar for adding a second pinned name lives. `--check` proves the
  pin landed where declared, and that the partition is total and disjoint. The script execs
  `cargo nextest run --profile ci` (install: `tools/scripts/install_nextest.sh`). It does
  NOT pass `--test-threads=1`, and the absence is deliberate: nextest gives each test its
  own PROCESS, and what genuinely cannot run beside a sibling is fenced BY NAME in
  `.config/nextest.toml` instead. Every shard (four on Linux, two on macOS) is the
  pre-merge bar; one shard is a targeted smoke check.
- Most binaries here mint a per-test SHM root (`init_for_test` / `generate_isolated_config`
  / `build_for_test`) and share no namespace with anything, so they run in parallel under
  nextest. The `tt1` rows below are the ones that genuinely cannot: under nextest that is
  expressed as membership of a `.config/nextest.toml` fence (the `default-namespace` group
  for binaries touching the default SHM namespace, `threads-required = "num-cpus"` for the
  wall-clock timing gates), not as a flag applied to everything.
- Serial legend:
  - **parallel**: per-test SHM root (`init_for_test` / `build_for_test` /
    `generate_isolated_config()`) or pure; parallel-safe within its own binary.
  - **`#[serial]`**: tests serialize themselves via the `serial_test` dev-dep; safe at
    default threads. `#[serial]` serializes but does NOT order; tests with ordering
    dependencies fold into one `#[test]` body.
  - **tt1**: MUST run alone with `-- --test-threads=1` (global TransportManager
    singleton, exact log-count assertions, or process-global env). Adding the flag to a
    `#[serial]` row is redundant but harmless.
  - **release**: `#![cfg(not(debug_assertions))]`; runs only under `--release`.
  - **hardware-only**: `#[ignore]`'d; run on dedicated hardware with `-- --ignored`.
- Fixture rule: cdylib tests `dlopen` prebuilt fixtures: build the named `-p` crates
  first. After ANY `cerulion_core`/`cerulion_macros` change, rebuild ALL fixtures
  (`cargo build --workspace`): a stale fixture mixes core vintages across the FFI/SHM
  plane and fails arbitrarily (SIGABRT, phantom wiring errors), indistinguishable from
  a real regression. Suspect a stale fixture FIRST.
- Executor/step/drain changes: run EVERY binary that builds a `GraphRuntime` or drives
  `step()` (the whole iceoryx2 e2e set); a hand-curated subset misses regressions.
- libtest DISCARDS stderr from passing tests; degrade/skip lines are invisible without
  `-- --nocapture`.

## Run recipes

```bash
# Default: run a binary by name (parallel and #[serial] rows).
cargo test -p cerulion_core --test <name>

# tt1 rows: the binary alone, single-threaded:
cargo test -p cerulion_core --test <name> -- --test-threads=1

# One CI shard, exactly as CI runs it (execs `cargo nextest run --profile ci`;
# install nextest with tools/scripts/install_nextest.sh):
./tools/scripts/ci_test_shard.sh cerulion_core 0 4

# Release-only latency gates:
cargo test -p cerulion_core --test latency_threshold_test --release -- --test-threads=1
cargo test -p cerulion_core --test graph_latency_test --release -- --test-threads=1

# Trybuild: ignored toolchain-fragile groups; re-bless after diagnostic edits:
cargo test -p cerulion_core --test macro_compile_fail_test -- --ignored
TRYBUILD=overwrite cargo test -p cerulion_core --test macro_compile_fail_test

# After touching notify elision or the live-loop boundary resweep:
cargo test -p cerulion_core --test notify_elision_resweep_iox2_test \
  --test notify_elision_iox2_test -- --test-threads=1

# After touching transport/liveness.rs or the gateway (parallel-safe):
cargo test -p cerulion_core --test topic_liveness_iox2_test
cargo test -p cerulion_core --test gateway_iox2_test

# Hardware-only suites (dedicated Linux hardware unless noted; clean SHM first):
cargo test -p cerulion_core --test shm_footprint_probe -- --ignored
cargo test -p cerulion_core --test shm_guard_madvise_iox2_test -- --ignored --test-threads=1
cargo test -p cerulion_core --test barrier_level_gate_subprocess_iox2_test -- --ignored --nocapture  # any Unix
```

## Fuzz targets

Three fuzz targets live in `crates/cerulion_core/fuzz/fuzz_targets/` (their own workspace,
nightly-only): `fuzz_wire_header` (wire-header deserialization from arbitrary bytes),
`fuzz_parse_graph` (graph YAML from arbitrary input), `fuzz_parse_info_json` (node info
JSON from arbitrary input). The `fuzz-helpers` feature (`crates/cerulion_core/Cargo.toml`, off
by default) exposes the internal helpers the fuzz crate drives.

Use the PINNED nightly (the `PINNED_NIGHTLY` value in `.github/workflows/ci.yml`),
never a stock `+nightly`: current nightlies cannot compile the exact-pinned iceoryx2,
and the failure surfaces as an inscrutable dependency compile error that nothing
connects to the toolchain choice.

```bash
# <pin> = the PINNED_NIGHTLY value in .github/workflows/ci.yml
cd crates/cerulion_core && cargo +<pin> fuzz run fuzz_wire_header -- -max_total_time=30
cd crates/cerulion_core && cargo +<pin> fuzz run fuzz_parse_graph -- -max_total_time=30
cd crates/cerulion_core && cargo +<pin> fuzz run fuzz_parse_info_json -- -max_total_time=30
```

CI runs all three in the `fuzz` job (Linux, `continue-on-error`, non-blocking).

## Test map

### Wire format, codegen, schema

| Test file | Pins | Serial | Fixtures |
|---|---|---|---|
| `wire_test.rs` | Wire-format structures and header validation. | parallel | none |
| `wire_lockstep_test.rs` | Byte-compatibility drift guard between `cerulion_core::wire` and the standalone `cerulion-wire` crate. | parallel | none |
| `codegen_test.rs` | `generate_schema()` emits the SHM-backed types; schema parsing from YAML + `.msg`. | parallel | none |
| `assign_shim_codegen_test.rs` | Codegen-emitted uniform `__cer_assign_<f>` write shims + the write-only diagnostic proxy for variable fields. | parallel | none |
| `fill_from_codegen_test.rs` | `<Name>Shm::fill_from_<f>` cursor bookkeeping, truncation, Err rewind, boundaries (no transport). | parallel | none |
| `fill_from_rewriter_test.rs` | Rewriter no-false-positive coverage: `fill_from` calls + `=` assignment forms preserved. | parallel | none |
| `fill_from_e2e_test.rs` | End-to-end `fill_from` through pub/sub; producer-Err non-publish gate; byte-identical determinism. | tt1 | none |
| `rewriter_var_field_assign_test.rs` | The schema-blind `=` assignment rewriter for variable fields. | parallel | none |
| `variable_schema_fixed_field_test.rs` | Variable-schema fixed fields are direct-access (write into the loaned slot), not setter-mediated. | parallel | none |
| `overflow_redirect_test.rs` | Codegen-level spill helper (`ensure_capacity_for`) + accessors (no transport). | parallel | none |
| `variable_alignment_padding_test.rs` | Generated primitive-array writers zero the alignment gap they skip (typed loan, slice set, first push, `fill_from`; 4-byte and 8-byte alignment; zero-length arrays; recycled bytes; spill; capacity refusal; producer failure), against hand-written payload oracles over poisoned buffers (no transport). | parallel | none |
| `overflow_redirect_e2e_test.rs` | Overflow spill round-trip + counter + determinism; overflow-path recovery `info!` + latch re-arm. | parallel | none |
| `max_slice_len_newtype_test.rs` | `MaxSliceLen` newtype boundary contract. | parallel | none |
| `max_slice_len_resolution_test.rs` | 3-tier `max_slice_len` resolution (YAML wins → schema default → global). | parallel | none |
| `max_slice_len_warn_emission_test.rs` | Warn emission across the resolution ladder (traced). | `#[serial]` | none |
| `adaptive_sizing_test.rs` / `adaptive_sizing_iceoryx2_test.rs` / `adaptive_sizing_proptest.rs` | Adaptive `loan_proxy` sizing: unit, iceoryx2-direct, and property-based adversarial coverage. | parallel / tt1 / `#[serial]` | none |
| `buffer_too_small_test.rs` | Proxy-buffer sizing errors + YAML `max_slice_len` parsing. | parallel | none |
| `misaligned_receive_test.rs` | Misaligned payload receive (header read off unaligned buffers, a bytemuck regression). | parallel | none |
| `frame_walker_production_test.rs` | Production-writer ↔ `FrameWalker` cross-check; generated `SCHEMA_HASH` == parser-derived hash. | parallel | none |
| `frame_walker_count_budget_test.rs` | Hostile-count budget via an ALLOCATION oracle (guard is output-equivalent; work is the point). | `#[serial]` | none |
| `wire_gap_frame_test.rs` | rmw borrow-window wire-legality oracle: `PayloadAudit::Frame` accepts the gap frame (page-aligned big field, out-of-declaration-order placement, dead gaps) byte-exact; refuses a nonzero-length entry below `data_floor`; generated accessors slice the same frame; `(0,0)` stays the unwritten idiom. Fixture: `testing::gap_frame`. | parallel | none |
| `chunk_a_bounds_test.rs` | Adversarial wire-frame bounds validation on receive paths. | parallel | none |
| `std_name_collision_test.rs` | Macro emission survives user types shadowing std names. | parallel | none |
| `error_message_test.rs` | Error messages carry actionable context + suggested fixes. | parallel | none |
| `headline_examples_compile_test.rs` | The headline macro examples in `README.md` and `docs/user-api.md` compile as written. | parallel | none |
| `tutorial_yaml_test.rs` | The tutorial's documented graph YAML pinned against drift (doc-oracle test). | parallel | none |

### Transport core (publisher, subscriber, proxies, history)

| Test file | Pins | Serial | Fixtures |
|---|---|---|---|
| `transport_test.rs` | Zero-copy `loan_proxy`/`try_view` round-trip over real iceoryx2. | tt1 | none |
| `output_proxy_test.rs` | Proxy/view round-trips (fixed + variable); commit-time sequence (discarded loan burns none); discard-latch recovery + `output_discard_count` observability (exact log counts). | tt1 | none |
| `zero_alloc_test.rs` | Zero heap allocations on the receive path (unprobed, probed drop_oldest, sample-gated). | `#[serial]` | none |
| `zero_copy_ci_test.rs` | Structural zero-copy verification (payload sizes, round-trip). | `#[serial]` | none |
| `zero_copy_hot_path_test.rs` | Constant init cost + zero-alloc on the publisher (`loan_proxy`) side. | `#[serial]` | none |
| `history_test.rs` | iceoryx2-native late-joiner history delivery. | tt1 | none |
| `pump_history_runtime_test.rs` | The runtime drives `pump_history` each live-loop iteration. | `#[serial]` | none |
| `pump_history_e2e_test.rs` | Quiescent publisher delivers retained history to a late joiner, in-process AND cdylib nodes. | `#[serial]` | cdylib fixture (header names it) |
| `pump_history_quiescent_test.rs` | Quiescent-state history behavior over an isolated per-test transport. | parallel | none |
| `deliver_history_failure_test.rs` / `deliver_history_failure_iox2_test.rs` | `deliver_history` fault injection + sent-history event suppression (isolated / singleton backends). | parallel / tt1 | none |
| `overflow_iox2_test.rs` | Overflow re-loan path: history push when a payload outgrows its slot; counter on Drop re-loan failure. | tt1 | none |
| `publisher_pool_sizing_test.rs` | Per-schema slot sizing for typed publisher creation. | parallel | none |
| `publisher_recon_teardown_iox2_test.rs` | Node-side producer-reconciliation teardown log (cdylib-parity, no FFI export); at-most-once with the host harvest; a planning-only build is silent while a never-stepped execution build still reports. | `#[serial]` | none |
| `notify_self_drain_iox2_test.rs` | Publishers drain their own event listener on every notify path (`publish_raw` included); anti-tautology saturation arm; latch lifecycle. | parallel | none |
| `notify_elision_iox2_test.rs` | The notify-elision self-healing gate (owned-listener counting; foreign attach resumes notifies). | `#[serial]` tt1 | none |
| `notify_elision_resweep_iox2_test.rs` | Live-loop boundary resweep: fires only on outstanding debt + foreign listener; firing spends the debt. | `#[serial]` tt1 | none |
| `data_only_tap_iox2_test.rs` | Listener-less capture tap: event-level zero proof; frame completeness; NON-consuming `has_samples()` (a consuming probe fails it). | `#[serial]` | none |
| `open_only_subscriber_iox2_test.rs` | `create_subscriber_open_only` cannot create services; no-requirements attach; type-skew diagnostics. | parallel | none |
| `drain_owned_test.rs` | The sample-held-beyond-callback recording tap the recorder daemon consumes. | `#[serial]` | none |
| `fifo_consume_iox2_test.rs` | Per-message FIFO consumption for data-trigger inputs: one fire per queued frame, in arrival order: the delivery contract "fire on each message arriving". A burst between fires must not collapse to ONE fire observing only the newest frame. | `#[serial]` | `test_node_macro_data_trigger_cdylib` |
| `fifo_burst_within_step_iox2_test.rs` | A data-trigger consumer's THROUGHPUT is not capped at one frame per scheduler step; a queued burst is served WITHIN the step that sees it. | `#[serial]` | `test_node_macro_burst_ctx_cdylib`, `test_node_macro_data_trigger_cdylib` |
| `topic_buffer_sizing_test.rs` | Depth-is-real buffers; topology-derived ceilings; max_subscribers exactness + headroom; single-writer provisioning; degraded-open warns. | parallel | none |
| `ingress_route_depth_iox2_test.rs` | `create_ingress_publisher` provisions a SLICE-AWARE receive-queue depth, and a recorder-style tap really inherits it. | parallel | none |
| `max_loaned_samples_test.rs` | Publisher loan-budget knob (`publisher_max_loaned_samples`, which the rmw borrow-window publish path raises): default = 2 loans + 3rd refuses `ExceedsMaxLoans` (`loan_proxy` classifies `LoanCapacity`); `Some(4)` = 4 hold + 5th refuses; `Some(0)`/over-cap refused loudly at creation. | parallel | none |
| `cross_graph_collision_iox2_test.rs` | Cross-graph same-topic publishing refused (derived-name, override, degraded arms); incumbent survives. | parallel | none |
| `multi_publisher_iox2_test.rs` | `multi_publisher_topics` opt-in: cross-graph two-publisher flow, block-all cap, per-stream eviction, fail-fasts. | `#[serial]` | none |
| `absolute_source_external_iox2_test.rs` | Absolute `source:` refs + External publisher provisioning (ceiling-only, create-order independent); external-silence warn; override ownership. | parallel | none |
| `per_test_shm_root_test.rs` | The per-test SHM root isolation machinery itself. | parallel | none |
| `context_transport_test.rs` | Every runtime-built `NodeContext` carries the HOST's `TransportManager` (`Arc::ptr_eq`). | `#[serial]` | none |
| `dead_node_cleanup_config_test.rs` | Every `TransportManager` config disables iceoryx2 auto dead-node cleanup (the cdylib same-PID reap hazard). | `#[serial]` | none |
| `iceoryx2_version_lockstep_test.rs` | iceoryx2 family exact-pinned + single lockfile version (pure file-parse). | parallel | none |
| `shm_footprint_probe.rs` | SHM pools are lazy/demand-paged; oversizing is latency-free (floor/p50/p99). | hardware-only `#[serial]` | none |
| `shm_guard_madvise_iox2_test.rs` | The MADV_NOHUGEPAGE mitigation applies to every pool mapping (Linux smaps proof). | hardware-only tt1 | none |

### Latches and flood suppression

| Test file | Pins | Serial | Fixtures |
|---|---|---|---|
| `failure_regime_latch_test.rs` | The shared suppression machine: loud/suppressed/decade-ladder decisions, unconditional totals, poisoned-mutex arm, frame-drop reporting layer + field keys. | parallel | none |
| `output_discard_latch_test.rs` | Discard-latch oracle vectors: error → debug(n) → recovery(iff suppressed) → re-arm; unconditional `total_discards`. | parallel | none |
| `drain_latch_test.rs` | Pre-step-drain warn suppression cycle + inversion regressions (pure state machine). | parallel | none |
| `service_test.rs` | Deterministic request-response layer e2e over the one-at-a-time take paths: FIFO order, reply isolation, schema-hash gating, per-client sequence gap/restart diagnostics; flood-latch arms at the production take sites (level-token asserted). | tt1 | none |

### Scheduler, executor, snapshot/hold, backpressure, QoS

| Test file | Pins | Serial | Fixtures |
|---|---|---|---|
| `scheduler_test.rs` | Deterministic scheduler under `VirtualClock`; duration-recording gates; policy rejections. | `#[serial]` | none |
| `graph_test.rs` | YAML graph parsing, validation, runtime stepping. | parallel | none |
| `config_deny_unknown_fields_test.rs` | Graph-config types reject unknown keys: a typo is a loud parse error, never a silent default; round-trip + rejection oracles per config type. | parallel | none |
| `node_test.rs` | `ClosureNodeEntry` + `DylibNodeEntry` loading. | parallel | cdylib fixtures (header) |
| `replay_test.rs` | Replay-equivalence (Replay = Live). | parallel | none |
| `replay_with_shutdown_test.rs` | Replay equivalence for graphs using `request_shutdown` + `shutdown` lifecycle methods. | parallel | none |
| `node_context_runtime_test.rs` / `node_context_adversarial_test.rs` | `NodeContext` surface (`env`, `env_str`, `clock`, `request_shutdown`, `ShutdownSignal`, `run_until_shutdown`): smoke + adversarial. | parallel / tt1 | none |
| `e2e_cli_test.rs` | Full user journey: create workspace → add nodes → build → run, through the CLI surface. | tt1 | built workspace |
| `runtime_shutdown_lifecycle_test.rs` / `chunk_j_pass2_test.rs` | `NodeEntry::shutdown()` invoked on every teardown path; plus cases the first file cannot distinguish (explicit shutdown vs `Drop`). | parallel | none |
| `step_zero_alloc_test.rs` | Zero allocations per `step()` at steady state, incl. multi-node and serial-gated levels. | `#[serial]` tt1 | none |
| `rayon_fire_iox2_test.rs` | Within-level parallel fire: parallel==serial==flat byte-identity; panic isolation; decision-order merge (any other merge order fails it). | `#[serial]` | none |
| `rayon_fire_cdylib_serial_test.rs` | A cdylib with non-trigger inputs routes OFF the rayon path (fired serially). | `#[serial]` | `test_node_macro_period_input_cdylib` |
| `fire_threads_env_test.rs` | `CERULION_FIRE_THREADS` bad-value warn; snapshot-loop desync else-arm. | `#[serial]` | none |
| `snapshot_view_iox2_test.rs` | Accounting-once across snapshot vs in-body drains; frozen-slot byte-identical serves; Err-replay-then-live. | `#[serial]` | none |
| `snapshot_wiring_iox2_test.rs` | Fire-gated step-boundary snapshot: frozen prior value vs same-level publish; block excluded / sample included; flat-vs-level byte-identity; trigger-scoped Sync arm. | tt1 | none |
| `non_trigger_hold_iox2_test.rs` | Cross-step HOLD of non-trigger inputs: pre-delivery wait, held watchdog trips, borrow-floor provisioning, held wire-timestamp freshness accessors. | `#[serial]` | none |
| `backpressure_counters_test.rs` | Pure scheduler backpressure counter registration + visibility. | parallel | none |
| `backpressure_block_iox2_test.rs` | `block` pre-fire defer e2e; degraded-block on mixed topics counts real evictions. | parallel | none |
| `backpressure_sample_iox2_test.rs` | `sample(N)` wire-timestamp decimation + counter + determinism. | parallel | none |
| `backpressure_throttle_iox2_test.rs` | Node-level `throttle_ms` producer rate cap, deterministic. | parallel | none |
| `backpressure_event_iox2_test.rs` | `#[on_event]` fires for drop_oldest (wire-seq gap) and block; once-per-regime warns with unconditional counters, all three policies. | parallel | none |
| `overflow_recovery_iox2_test.rs` | drop_oldest recovery is O(1) in overflow magnitude; eviction count exact vs a hand oracle. | `#[serial]` | none |
| `on_event_test.rs` / `on_event_watchdog_test.rs` / `on_event_multi_handler_test.rs` / `on_event_liveliness_test.rs` | `#[on_event]` routing: per-regime fire, watchdog event types, (port,kind) coexistence + declaration-order dispatch, liveliness transitions. | parallel / parallel / parallel / tt1 | none |
| `expect_within_iox2_test.rs` / `promise_within_iox2_test.rs` / `tick_within_iox2_test.rs` | Input watchdog (non-trigger resets on read); output promise anchor; tick budget counter (liveness-only asserts; wall clock). | parallel | none |
| `expect_within_fifo_iox2_test.rs` | An `expect_within_ms` window that lapses on a per-message FIFO trigger input still carrying unconsumed arrivals is BACKLOG, not silence; the watchdog must not report a live producer dead. | `#[serial]` | none |
| `deadline_qos_test.rs` | Deadline QoS counter/event integration. | tt1 | none |
| `trace_merge_test.rs` | Pure k-way cross-process trace merge; duplicate rank is a loud error. | parallel | none |
| `partition_test.rs` / `auto_partition_test.rs` | Pure cross-process partition derivation/validation; cost-aware fusion + baseline. | `#[serial]` (pure) | none |
| `silent_trigger_validation_test.rs` / `sync_attr_1_trigger_warn_test.rs` / `graph_default_policy_warn_test.rs` | Loud-fail/warn guards on silent-never-fire combos, degenerate sync, and the no-policy default warn. | parallel | none |
| `sync_fire_iox2_test.rs` | Trigger-scoped Sync fire semantics: alignment, stale plain input never gates, starved-trigger watchdog. | `#[serial]` | none |
| `sync_per_set_iox2_test.rs` | PER-SET Sync delivery: one fire per COMPLETE aligned set, in set order, each trigger message consumed by at most one set; members are READ into a sink and compared against hand-written tuples, because a fire COUNT cannot see which frames a fire observed. | `#[serial]` | none |
| `sync_per_set_backpressure_iox2_test.rs` | Per-set Sync composed with the input backpressure policies. | `#[serial]` | none |
| `sync_trigger_wiring_guard_test.rs` | Build-time loud guards for unwired/degenerate/absent sync triggers; anti-tautology clean-build control. | parallel | none |
| `macro_sync_threading_test.rs` | Sync policy threads YAML-wired inputs through the policy conversion. | parallel | none |
| `data_trigger_macro_binding_test.rs` | Runtime synthesizes the data-trigger binding from macro-reported trigger fields. | parallel | none |
| `drain_discipline_seam_test.rs` | `CERULION_DRAIN_DISCIPLINE=separate` seam: exact matching, one warn per build, fan-out provisioning desync pin (a count-site desync fails the build), closure eligibility. | `#[serial]` tt1 | none |
| `collapse_no_publish_test.rs` | A collapsed tick must not arm publish on a fixed-schema output. | `#[serial]` tt1 | none |
| `lazy_loan_iox2_test.rs` | Lazy loan-on-first-write: untouched outputs never loan/publish/discard; partial writes still discard loudly; fieldless `emit()`; read-back-after-write. | `#[serial]` tt1 | none |

### External-trigger nodes and the live loop

| Test file | Pins | Serial | Fixtures |
|---|---|---|---|
| `external_fire_iox2_test.rs` | `trigger_external` + `step` fires an External node. | `#[serial]` | none |
| `external_live_fire_iox2_test.rs` | The external fire seam: fd/Blocking/HostDriven sources, wake coalescing, level-trigger refire, launch refusals, wake-storm no-lost-tail, eventfd arm (Linux). | `#[serial]` | none |
| `external_gating_replay_iox2_test.rs` | Polled path never queries `external_source()`; sticky aggregated launch refusal; replay firewall (source never touched). | `#[serial]` | none |
| `waitset_fd_source_iox2_test.rs` | Mixed listener + raw-fd wake sources; declaration-order reporting; invalid/stale fd guards (host-side EBADF-abort protection). | `#[serial]` | none |
| `waitset_capacity_test.rs` | WaitSet attachment-capacity guard boundary. | parallel | none |
| `waitset_busyspin_test.rs` | The live loop sleeps its heartbeat only when the WaitSet reported nothing (busy-spin guard). | `#[serial]` | none |
| `waitset_reactor_iox2_test.rs` / `waitset_live_loop_iox2_test.rs` / `chunk25b_live_default_iox2_test.rs` | WaitSet reactor wake/dispatch; the live loop seam; live-default behavior. | `#[serial]` | none |
| `wake_set_iox2_test.rs` | Public `WakeSource`/`WakeSet` over real iceoryx2 (data-service-gates-first; fired-index ordering). | `#[serial]` | none |
| `monitor_wait_park_iox2_test.rs` | Park FUNCTIONAL behavior (pure-Period park, doorbell data path, no-data-input degrade); CPU-park latency needs park-capable hardware and is not asserted here. | `#[serial]` | none |
| `unified_stale_wake_park_test.rs` | Stale unified-binding events are drained (timeout-paced idle; re-arm still delivers), the free-run regression pin. | `#[serial]` tt1 | none |
| `live_spin_budget_test.rs` | Scheduled spin-then-block receive path budget semantics. | `#[serial]` | none |
| `polled_vs_live_iox2_test.rs` | Replay=Live firewall: polled seam == live seam == hand oracle; parked live seam identical. | `#[serial]` | none |
| `live_gating_clock_iox2_test.rs` | Deterministic-live gating clock: quantum advancement, wall-independent interleave, watch-clock decoupling. | `#[serial]` tt1 | none |
| `live_pace_anchor_iox2_test.rs` | Live wait sizing in the gating domain: exact pre-anchor quantum cap, anchored wait bound, and a handwritten 16-step clock/fire oracle over real live steps; upper rate bound retained without a lower wall-rate floor that changing CPU load can invalidate. Also covers pace slipping, resume anchoring, and wake-schedule determinism. | parallel (per-test roots) | none |
| `free_run_ctor_iox2_test.rs` | The free-run switch: the two FREE-RUN build ctors thread the stamped `topic_requirements` union (borrow-5-above-floor oracle, single-process ctors as negative controls); the deterministic sibling derives the LOCAL quantum + installs no participant; `place_gating_epoch` re-phases the `Period` deadline + re-seeds the `promise_within` window (no burst, no phantom miss) and refuses a RealClock build / a lockstep participant / a post-step placement; the RealClock arm's inert `set_gating_follows_wall` warn. Further arms: the wall-following clock is REFUSED on a lockstep participant (OFF accepted; the free-run build accepts ON); a placed epoch yields a hand-oracle fixed-quantum trace (`[E+4ms..E+20ms]`, two runs identical); a placed epoch re-seeds the INPUT watchdog window (the quantum IS the window; zero misses); an epoch ARMED for the live anchor is placed when `run_live` takes the anchor (real `run_live`, zero steps) and by the test seam, is one-shot, is refused at arm time with text naming the arm entry point, is refused after a step, and is warned ONCE when stepped past through the polled seam and at `Drop` when never spent. | parallel (per-test roots) | none |
| `lat_probe_env_test.rs` | Latency-probe env → behaviour wiring over a subprocess. | parallel | none |

### Barriers and multi-process

| Test file | Pins | Serial | Fixtures |
|---|---|---|---|
| `barrier_test.rs` | `BarrierShared` + `MappedBarrier` oracle vectors: lockstep, drop/arrive races, orphan reuse, parking `wait()`, wake word, munmap-during-park safety (hardware-only subset `#[ignore]`'d). | `#[serial]` (hermetic) | none |
| `barrier_level_gate_iox2_test.rs` | The barrier firewall: split contexts merge byte-identical to the monolith; stalled-peer terminal poison; build validation errors. | `#[serial]` | none |
| `mid_level_barrier_iox2_test.rs` | The MID-LEVEL barrier: two period nodes at ONE global level joined by a plain non-trigger input, split across contexts: the one edge the DAG does not model. Owns the ORDERING guarantee; the interleave is fixed by the barrier's own signal, never a sleep. | `#[serial]` | none |
| `barrier_level_gate_subprocess_iox2_test.rs` | The real-subprocess (cross-address-space `MAP_SHARED` + real park) firewall replica; single-child block-timeout-poison anti-tautology. | hardware-only (any Unix) | none |
| `credit_block_iox2_test.rs` | The cross-process `block` CREDIT WORD end to end: two transport contexts over one SHM root plus one supervisor-created `MappedCredit` page each side opens separately. The producer plateaus at the FOREIGN consumer's depth, the consumer's drain frees credit across the boundary, every committed frame arrives gap-free, and every plan/topology skew is a loud refusal (depth, policy, mixed topic, both role negatives, the rank-stranger skip). Drives ALL THREE multi-process ctors with a NON-EMPTY binding list (the free-run live one, the lockstep barrier one and the free-run record one), so no ctor is inert-shipped. Per-test SHM roots + pid-scoped credit namespaces; 34 arms, 33 `#[serial]` (the odd one out is a pure `MAX_CONSUMER_DEPTH` drift guard). Includes the flow-mode credit producer-park arms: the wake predicate and its was-full gating, the beyond-mask slot's degradation against an in-mask control, and a LOCKSTEP rank holding a barrier participant while credit-blocked at depth, which must return from the live park rather than wedge. | `#[serial]` | none |
| `barrier_park_wake_iox2_test.rs` | Barrier-arrival park wake predicate: liveness, attribution, correctness, no-false-wakes, park-off routing, wake-word arms. | `#[serial]` | none |
| `supervisor_precreate_iox2_test.rs` | Core seams for supervisor-side pre-creation of owned services before worker spawn. | `#[serial]` tt1 | none |
| `precreate_services_iox2_test.rs` | Graph build pre-creates every owned topic service (startup-race close). | parallel | none |
| `reg_channel_iox2_test.rs` | The cross-process runtime-egress registration channel e2e; the per-process writer-slot arithmetic. | parallel | none |
| `credit_test.rs` | The cross-process block-credit word and its SHM productization `MappedCredit`, over real `MAP_SHARED` POSIX SHM on both shipping platforms (no stub arm on either). | parallel (self-re-exec subprocess arm, NOT `#[ignore]`'d) | none |
| `credit_os_sync_independence_test.rs` | Flow mode: the credit plane's macOS `os_sync` tier is INDEPENDENT of `CERULION_BARRIER_OS_SYNC`. Pins against a coupling: if the availability predicate or `park_wait_credit` itself routes through `barrier::os_sync_active()`, which ANDs in the barrier's switch, disabling the barrier's tier silently disables the credit plane's while `docs/user-api.md` promises independence. Each arm re-execs this binary with the env it wants, because both switches are `OnceLock`-cached and one process can observe exactly one combination; the child reports BOTH the predicate and the runtime park decision, so a defect at either site fails this test. Arm 1 is an anti-vacuity control (no switches set must read AVAILABLE, else the host has no usable backend and the test skips loudly). macOS-only: on Linux the credit word rides a shared futex and the predicate is unconditionally true. | parallel (self-re-exec subprocess arms) | macOS-only (`#![cfg(target_os = "macos")]`) |
| `wedge_alarm_iox2_test.rs` | The wedge alarm's OBSERVABLE half over a real `GraphRuntime`: a node stuck INSIDE a tick stops advancing its progress words while its rank keeps stepping. | parallel | none |
| `wedge_slot_binding_test.rs` | `Scheduler::set_wedge_page` slot binding: the SUPERVISOR owns the node→slot mapping, and a supervisor/worker desync is reported loudly in BOTH directions. | parallel | none |

### Liveliness, liveness, and diagnostics

| Test file | Pins | Serial | Fixtures |
|---|---|---|---|
| `liveliness_sweep_iox2_test.rs` | Graceful-disconnect liveliness transitions across multiple publishers. | `#[serial]` | none |
| `liveliness_crash_iox2_test.rs` | TRUE-crash path: a SIGKILLed forked publisher's lingering port → disconnect event. | `#[serial]` tt1 | none |
| `topic_liveness_iox2_test.rs` | The data-flow liveness observer: dating rule, baselines, cross-clock arms, epoch reset (incl. `a_lull_free_restart_dates_again_within_the_confirmation_window`), budget policy, rate estimate, degradation to UNKNOWN. | parallel | none |
| `gateway_iox2_test.rs` | Gateway plan oracles + demand-driven egress e2e; liveness-over-egress (`*` arms); demand-plane epoch reset (`*`); rate-over-egress (`*`); slot hand-off ordering (reversing release and attach fails it); the rmw borrow-window gap frame forwarded byte-identical. The `TopicBridgeManager` egress posture gate itself (`enable_bridge` / `disable_bridge` / `register_topic`) is unit-tested in `transport/bridge.rs`. | parallel | none |
| `cross_machine_data_plane_harness.rs` | Two-machine data-plane A/B diagnostic (env-driven roles); not run in CI. | hardware-only | none |
| `catalog_query_warn_test.rs` | Loud once-per-robot fallback warn on an undecodable catalog reply. | `#[serial]` | none |
| `node_name_refusal_iox2_test.rs` | An unrepresentable graph identity REFUSES the transport instead of aborting the process, proven at the constructors that really build an iceoryx2 node. | parallel | none |

### Network plane

| Test file | Pins | Serial | Fixtures |
|---|---|---|---|
| `network_test.rs` | Network transport basics over the SHM-backed API. | tt1 | none |
| `network_graph_wiring_test.rs` | `network:` block → `GatewayPlan` derivation; a graph build starts nothing network. | parallel | none |
| `network_ingress_test.rs` | Byte-identical re-injection; schema-mismatch counted-not-delivered; structural loop exclusion; lazy session; teardown release. | `#[serial]` tt1 | none |
| `network_ingress_e2e_test.rs` | Cross-session producer→gateway→zenoh-TCP→re-inject→subscriber, byte-identical vs hand-stamped headers. | `#[serial]` | none |
| `network_ingress_unregister_test.rs` | Ingress teardown: slot freed, self-ingress cleared, full re-register cycle, loud double-unregister. | `#[serial]` | none |
| `network_yaml_e2e_test.rs` | Config-only cross-machine delivery: two YAML graphs, bit-identical payload through the full chain. | `#[serial]` | none |
| `network_tf_e2e_test.rs` | Worst-case wire shape (variable array of nested-with-strings) crosses the link byte-identical, empty-array edge included. | `#[serial]` | none |
| `network_two_gateway_inversion_e2e_test.rs` / `network_two_gateway_reconcile_e2e_test.rs` | Queryable-inversion demand on a strict connect-only peer link; true two-gateway egress + reconciler delivery. | `#[serial]` | none |
| `ingress_injection_seam_test.rs` | The zenoh-free local-SHM ingress injection seam (`create_ingress_injector`). | `#[serial]` tt1 | none |
| `ingress_publish_raw_notify_iox2_test.rs` | A raw-ingress publisher wakes foreign consumers from `publish_raw` itself. | `#[serial]` tt1 | none |
| `mirror_provenance_e2e_test.rs` | Mirror-provenance registration over the control service; gather-back attribution. | `#[serial]` | none |
| `run_registry_iox2_test.rs` / `run_lifetime_iox2_test.rs` | The run registry (one record per run, `Live`→`Ending`); run-watcher lifetime binding. | `#[serial]` | none |
| `runs_serve_iox2_test.rs` | The `runs` verb's SERVE side over real iceoryx2: gather `/__cerulion/runs` at serve time in the process hosting the gateway, then read each run's directory. | parallel | none |
| `runs_query_zenoh_e2e_test.rs` | The `runs` verb over a REAL zenoh hop: a serving gateway answering a desk-side GET with a real run directory underneath. | `#[serial]` | none |
| `ingress_build_iox2_test.rs` | The ingress-build PROGRESS channel over real iceoryx2, the WIRE half; the recorder-side policy is oracle-tested in `cerulion_bagd`. | parallel | none |
| `ingress_hash_preflight_test.rs` | `graph validate` must not pass a graph that `graph run` then REFUSES to build: a network `ingress:` topic consumed by a node whose parsed `InputMeta::schema_hash` is the `0` no-declared-schema sentinel. | parallel | `test_node_macro_data_trigger_cdylib`, `test_node_snapshot_fail_cdylib` |

### Trace rings and recording seams

| Test file | Pins | Serial | Fixtures |
|---|---|---|---|
| `shm_ring_test.rs` | SPSC POSIX-SHM ring: wrap/overrun math, torn-commit truncation, corruption rejection, SPSC guard, cross-process child rendezvous. | `#[serial]` (per-test tags) | none |
| `shm_ring_zero_alloc_test.rs` | Wait-free `push` does zero heap allocations (thread-scoped counter, one-body fold). | `#[serial]` | none |
| `shm_ring_backpressure_test.rs` | The ring's `BACKPRESSURE` overrun mode and `ShmRingProducer::resync_after_fork`: changes to a SHIPPED primitive, so the arms pin what must NOT move as much as what must. | parallel | none |
| `trace_ring_hook_test.rs` / `trace_ring_hook_zero_alloc_test.rs` / `trace_ring_parallel_test.rs` | Scheduler → trace-ring hook e2e; recording-ON zero-alloc; parallel-level merge-ring arm. | `#[serial]` | none |
| `recording_honest_clock_iox2_test.rs` | The replay-grade recording clock model (runtime half). | `#[serial]` tt1 | none |
| `recording_provisioning_iox2_test.rs` | The record-side tap provisioning raise through the real `recorded_topics` build seam. | `#[serial]` | none |
| `publish_trace_integration_test.rs` | The publish trace captures wire-header metadata from real publishes. | parallel | none |
| `read_outcome_capture_iox2_test.rs` | Per-edge READ-OUTCOME capture over real iceoryx2, including the per-site `role_view` arms. | `#[serial]` | none |
| `read_outcome_cdylib_capture_iox2_test.rs` | Read-outcome capture for a **cdylib** node over a real POSIX-SHM trace ring: the capture sites run inside the cdylib's OWN statically linked copy of `cerulion_core`, which no in-process arm reaches. | `#[serial]` | `test_node_macro_period_input_cdylib` |

### State capture, checkpoints and restore

The checkpoint arc: what a node's state looks like on the wire, who is allowed to capture it,
when, and what a restore refuses. Several rows come in PURE/BEHAVIOURAL pairs: the decision
layer is oracle-tested in-module or in a `*_restore_test`, and the row here is the half that
needs a real `GraphRuntime`, a real SHM mapping or a real `fork(2)` to be observable at all.

| Test file | Pins | Serial | Fixtures |
|---|---|---|---|
| `state_derive_test.rs` | `#[derive(CerulionState)]` end to end: every type derived by the production macro, compiled by rustc, driven against a hand-written byte oracle. | parallel | none |
| `state_derive_max_variants_test.rs` | The 256-variant boundary, COMPILED for real: exactly 256 is accepted, and 256 one-byte tags exhaust `u8`, which makes the derive's invalid-tag arm genuinely unreachable. | parallel | none |
| `node_state_test.rs` | The `#[cerulion_node]` fold-in: a node struct that declares NOTHING is capturable, over exactly its non-port fields, through the production attribute macro. | parallel | none |
| `state_prealloc_budget_test.rs` | The decoder's hostile-COUNT pre-allocation cap (`ELEMENT_PREALLOC_CAP`) pinned by an ALLOCATION oracle: a blob-supplied count must not buy a reservation. Own binary (process-global allocator). | `#[serial]` | none |
| `state_ring_test.rs` | The state ring over REAL POSIX SHM: the seams a pure test structurally cannot see (a whole anchor read back, chunked reassembly across a real mapping). | parallel | none |
| `state_arm_test.rs` | The checkpoint ARM WORD over REAL POSIX SHM: the behavioural half of `state_arm.rs`'s in-module cadence/claim-table oracles. | parallel | none |
| `state_arm_clamp_iox2_test.rs` | END TO END: a real `MappedStateArm` on a real `GraphRuntime` clamps a `Period` node's catch-up burst: the only place the whole chain runs. | parallel | none |
| `state_restore_test.rs` | Oracle-vector pins for the PURE restore decision layer that runs before any byte reaches a node; every assertion against a hand-written expected value, never a second call of the function under test. | parallel | none |
| `state_restore_runtime_iox2_test.rs` | The RESTORE seam through a real `GraphRuntime`: recorded bytes actually reach a node, and a refusal actually stops them. | parallel | none |
| `state_anchor_boundary_iox2_test.rs` | The ANCHOR BOUNDARY through a real `GraphRuntime`, a real mapped arm word and a real POSIX-SHM state ring, the INLINE half: what the node thread writes, at which steps, and when it declines. | parallel | none |
| `state_anchor_fork_iox2_test.rs` | The FORK-CARRIER half of that boundary, over a real `fork(2)`. | `#[serial]` | none |
| `state_carrier_seam_test.rs` | The erased carrier seam `NodeEntry::{inline_safe, cer_probe}`: the two answers the boundary walk reads before it touches a node. | parallel | none |
| `state_child_main_fork_test.rs` | What the fork-child module DOES, over a real `fork(2)` child: the fd close really closes, the keep list really keeps, the signal mask really comes back. Its sibling source walk proves what the child does NOT do. | parallel | none |
| `state_breadcrumb_fork_test.rs` | The child BREADCRUMB over a real `fork(2)`: the one property no in-process test can see, that a child's progress stamps reach the PARENT (`MAP_SHARED` and `MAP_PRIVATE` are indistinguishable from one process). | parallel | none |
| `state_fork_panic_hook_test.rs` | The parent-installed, pid-branching panic hook over real `fork(2)` children, including a fork taken while std's hook lock is genuinely READ-HELD, the precondition it exists for. ONE test body, because everything here is process-global. | parallel (ONE test body in its OWN binary; the hook is process-global) | none |
| `state_reaper_fork_test.rs` | The reaper over real children, and the `kill(pid, 0)` liveness predicate driven through the PRODUCTION arm-word sweep. | parallel | none |
| `state_dontfork_test.rs` | An excluded mapping is genuinely ABSENT from a `fork` child, over a real child on both shipping platforms. | parallel | none |
| `node_death_trigger_iox2_test.rs` | The NODE-DEATH mint through a real `GraphRuntime`: the mint site the scheduler's circuit-breaker arms cannot reach. | parallel | none |
| `flashback_producer_fault_warn_test.rs` | The node-death producer's three failure branches (no transport, the trigger channel will not open, the request will not publish), each LOUD, because a robot binary compiles `debug!` out. | parallel | none |

### Latency and flatness gates

| Test file | Pins | Serial | Fixtures |
|---|---|---|---|
| `latency_threshold_test.rs` | Catastrophic-regression backstop (accidental memcpy class). | tt1 + release | none |
| `flat_latency_test.rs` | Full one-way publish→receive flatness: drop-one-outlier floor ratio; interleaved rounds + bounded window-health retries; uniform-stall discriminator. Debug-OK; gates every PR. | tt1 | none |
| `cross_thread_rtt_test.rs` | Round-trip flatness + the ports-are-`Send` runtime proof (build then MOVE across threads). Debug-OK. Per-size interleaving is deliberately NOT applied here (unlike `flat_latency_test`): interleaving would keep every size's ports (including two 16 MiB legs) alive at once and muddy the build-then-MOVE Send proof; the bounded window-health retry re-measures a stalled size instead. Do not "harmonize" it with the sibling gate. | tt1 | none |
| `graph_latency_test.rs` | USER-POV latency through a real 3-node macro graph (p50 backstop + flatness); global warm-up head for DVFS hardware. Needs updating on macro/message API bumps; it exercises the full user-facing dispatch path. | tt1 + release | none |
| `cli_e2e_graph_latency_test.rs` | Full user-journey CLI + cdylib graph latency. | tt1 + release | built workspace |
| `streaming_latency_bench_test.rs` / `period_decompose_bench_test.rs` | Data-driven spin bench; per-hop period decomposition. Manual release benchmarks: no CI job EXECUTES them (whole-file `#![cfg(not(debug_assertions))]`, and the release lanes name other targets); CI type-checks them through the rot guard. They are the only place in the tree that measures a native-graph spin A/B or a per-hop split. | `#[serial]` release (bench-style) | none |

### Clock model

| Test file | Pins | Serial | Fixtures |
|---|---|---|---|
| `clock_sources_test.rs` | Clock-source read matrix at the public API; thread-CPU vs wall under preemption. | `#[serial]` | none |
| `clock_model_e2e_iox2_test.rs` | End-to-end clock-model contracts over real iceoryx2. | `#[serial]` tt1 | none |
| `d2_sim_ns_warn_test.rs` / `ext_ns_warn_test.rs` | `RealClock::virt_ns()` / `ext_ns()` return `None` + one-time warn. | parallel | none |
| `macro_shim_clock_sources_test.rs` | Macro shim clock reads reach the runtime's ACTIVE clock. | `#[serial]` tt1 | none |

### Macro runtime (in-process)

| Test file | Pins | Serial | Fixtures |
|---|---|---|---|
| `macro_test.rs` | `#[cerulion_node]` lifecycle, info, state. | parallel | none |
| `macro_graph_test.rs` | Macro-generated nodes inside `GraphRuntime`. | tt1 | none |
| `macro_lifecycle_test.rs` | User-defined `init`/`shutdown` lifecycle methods are invoked. | parallel | none |
| `macro_shim_methods_test.rs` / `macro_chunks_ef_adversarial_test.rs` | Injected shim methods reach the runtime; auto-`Default` + hidden context fields adversarial. | parallel | none |
| `cerulion_node_impl_test.rs` | The AST-rewriting impl macro end-to-end. | parallel | none |
| `macro_compile_fail_test.rs` | trybuild: blocking own-`compile_error!` diagnostics; `#[ignore]`'d toolchain-fragile rustc-diagnostic groups. | parallel | none |
| `in_code_pipeline_test.rs` | Two multi-node pipelines driven as tests: a macro-defined pipeline builds and steps over the real transport, `step(delta)` advances the clock exactly once, the camera and detector pair publishes fully initialised frames and counts the lit pixels; hand oracles from the period arithmetic. | parallel | none |

### cdylib runtime (FFI parity: the "no inert shipping" family)

| Test file | Pins | Serial | Fixtures (`cargo build -p …` first) |
|---|---|---|---|
| `macro_cdylib_test.rs` | Macro-generated cdylib loads + runs via `DylibNodeEntry`. | tt1 (its data-flow arm takes the global `TransportManager`, and it carries no `#[serial]`) | `test_node_macro_cdylib`, `test_node_cdylib`, `test_node_macro_external_cdylib`, `test_node_failing_cdylib` |
| `macro_cdylib_policy_round_trip_test.rs` | Macro → info-JSON → runtime policy round-trip oracle. | parallel | `test_node_macro_unbounded_sync_cdylib` |
| `macro_cdylib_overflow_test.rs` | Variable-schema spill round-trip across the FFI. | tt1 | cdylib fixture (header) |
| `info_json_key_parity_test.rs` | The cdylib info-JSON emitter and its host parser agree on the KEY SET, read as TEXT on both sides (the emitter lives in a proc-macro crate, the parser is a private cfg-gated fn). It matters because the two tiers fail differently: the OUTER info structs are `#[serde(default)]`, so a key the emitter starts writing that the parser never learned is not an error: it is dropped, the field takes its default, and the node runs with the setting silently absent. The nested `PolicyJson` is the strict exception (`deny_unknown_fields`), so an unknown key THERE fails the parse loudly instead. Either way the emitter can also go wrong by writing nothing: a missing policy key leaves the host with no policy and the node falls back to a plain data trigger, so a node whose policy gates on ALL its inputs fires on any one of them instead. The invariant is therefore two-sided: the key sets must agree, and a key the emitter omits must be one whose parser-side default is the semantic the node declared. The macro-side half, "every advertised attribute reaches the emitter", is `cerulion_macros::codegen::advertised_attribute_parity_tests`; this file is the other half: whatever the emitter writes, the host must have somewhere to put it. | parallel | none |
| `abi_version_mismatch_test.rs` | `DylibNodeEntry::load` rejects an ABI-version-skewed cdylib. | `#[serial]` | cdylib fixture (header) |
| `rustc_fingerprint_mismatch_test.rs` | `DylibNodeEntry::load` rejects a cdylib compiled by a DIFFERENT rustc than the host, and refuses a null fingerprint pointer. The ABI-version and `abi_layout` pins only measure size/align/offset, so they cannot see a rustc release changing the bit pattern `Option<T>::None` writes for a niche-holding `T`: the host then reads a live `Some(..)` where the writer meant `None` and the drop glue frees uninitialised bytes, a libmalloc SIGABRT with no panic text. The fault is induced through the fixture's `CER_RUSTC_FAULT` env var, mirroring `CER_ABI_FAULT`. | `#[serial]` (the arms mutate `CER_RUSTC_FAULT`, a process-global env) | `test_node_cdylib` |
| `dylib_corrupt_info_test.rs` | Corrupted info JSON refuses to load with a rich diagnostic. | `#[serial]` tt1 | `test_node_corrupt_info_cdylib` |
| `node_raw_ffi_template_test.rs` | The raw-FFI scaffolding template's emit loads; LAST_ERROR semantics. | parallel | `test_node_raw_ffi_template_cdylib` |
| `chunk_c_ffi_error_test.rs` / `chunk_c_ffi_codes_3_4_test.rs` | FFI error codes end-to-end (rich message threading; poisoned-registry + missing-handle codes; single-body lifecycle ordering). | `#[serial]` | failing-mode cdylib fixtures (headers) |
| `env_snapshot_isolation_test.rs` | `NodeContext::env_snapshot` isolates nodes from post-build env mutation. | `#[serial]` | none |
| `cdylib_non_trigger_hold_test.rs` | Cross-step hold through the optional snapshot FFI pair; symbol-less back-compat is a safe no-op. | `#[serial]` tt1 | `test_node_macro_period_input_cdylib`, `test_node_cdylib` |
| `cdylib_snapshot_set_failure_test.rs` | A failed snapshot-set is TERMINAL: the per-step snapshot FFI is never called (counter 0 under fault; anti-tautology control). | `#[serial]` tt1 | `test_node_snapshot_fail_cdylib` |
| `cdylib_state_ffi_test.rs` | State capture and restore across the cdylib wall over a real `dlopen` through `DylibNodeEntry`; the file's own header calls it the KEYSTONE of this mechanism. | `#[serial]` | `test_node_macro_state_cdylib`, `test_node_state_liar_cdylib`, `test_node_state_panic_cdylib`, `test_node_cdylib` |
| `cdylib_unified_drain_test.rs` | Optional drain-FFI capability; forced-Separate byte-identity; drain-failure maps to safe no-fire + one latched error. | `#[serial]` tt1 | `test_node_macro_data_trigger_cdylib`, `test_node_cdylib`, `test_node_drain_fail_cdylib` |
| `cdylib_unbounded_sync_fire_test.rs` | UnboundedSync fires only on ALL inputs through the FFI (the policy-degrade regression); retained-input completion. | `#[serial]` tt1 | `test_node_macro_unbounded_sync_cdylib` |
| `cdylib_sync_nontrigger_test.rs` | Trigger-scoped Sync parity across the FFI: per-input trigger flags, held plain input, absent-input body wait, in-process twin identity. | `#[serial]` tt1 | `test_node_macro_sync_nontrigger_cdylib` |
| `cdylib_sync_backpressure_parity_test.rs` | `#[input(backpressure = block \| sample(N))]` on a PER-SET Sync TRIGGER input, through the FFI, the deployment-surface half of `sync_per_set_backpressure_iox2_test.rs`. `block` stops its producer at EXACTLY the declared depth under a starved partner (the block half's only exact-valued oracle; the bounds alone pass under a wrongly-sized gate) and throttles it on the pair stimulus (`published < steps`, the discriminating half; measured, the loss invariants hold under `drop_oldest` too there); `sample(N)` decimates BEFORE matching, pinned by exact membership `[(0,0), (5,4), (10,10)]` against an exact ungated discriminator; per-policy in-process twin parity; determinism; and the `backpressure`/`depth` info-JSON carry into `InputMeta`. | `#[serial]` tt1 | `test_node_macro_sync_block_cdylib`, `test_node_macro_sync_sample_cdylib` |
| `sync_head_op_ffi_test.rs` | The ARGUMENT GUARDS of `cerulion_node_sync_head_op` by RAW libloading: the contract every FFI consumer sees, not only the loader Cerulion ships. | `#[serial]` | `test_node_macro_sync_nontrigger_cdylib` |
| `sync_head_op_fail_cdylib_test.rs` | The align driver's `Failed` policy is FAIL-CLOSED at the cdylib seam, on a real `dlopen`ed node. | `#[serial]` | `test_node_sync_op_fail_cdylib` |
| `cdylib_trigger_semantics_test.rs` | Period-is-a-schedule, Data, Sync, HostDriven fire semantics through loaded cdylibs; no-policy warn; live refusal. | `#[serial]` | `test_node_macro_period_cdylib`, `…_data_trigger_…`, `…_sync_…`, `…_external_…`, `test_node_cdylib` |
| `cdylib_on_event_dispatch_test.rs` | All four `#[on_event]` kinds dispatch INSIDE a loaded cdylib (counters read back across SHM); parity + determinism. | `#[serial]` tt1 | `test_node_macro_onevent_cdylib` |
| `cdylib_portwrite_e2e_test.rs` | Port-write surface over the production cdylib path: hand-built nested oracle; nested-write conflict is a loud tick error; partial staged child discards. | `#[serial]` | `test_node_macro_portwrite_cdylib` |
| `cdylib_qos_ffi_test.rs` | QoS declarations round-trip the info-JSON FFI. | `#[serial]` | cdylib fixture (header) |
| `cdylib_qos_behavioral_test.rs` | expect/promise/tick_within, throttle, sample behave e2e on cdylibs (tick budget load-proofed via slow-step containment). | `#[serial]` | `test_node_macro_qos_cdylib`, `test_node_macro_depth_cdylib` |
| `cdylib_depth_ffi_test.rs` | `#[input(depth = N)]` + declared backpressure are real across the FFI. | `#[serial]` tt1 | `test_node_macro_depth_cdylib` |
| `cdylib_depth_multiprocess_test.rs` | FFI-declared depth survives into the multi-process provisioning harvest (`topic_requirements()`). | `#[serial]` | `test_node_macro_depth_cdylib` |
| `cdylib_block_degrade_test.rs` / `cdylib_dropoldest_overflow_test.rs` / `cdylib_hold_block_excluded_test.rs` | Mixed-topic block degrade; drop_oldest count + drain-to-latest freshness; block excluded from hold, each through a loaded cdylib. | `#[serial]` | cdylib fixtures (headers) |
| `cdylib_clock_accessors_test.rs` / `cdylib_collapse_no_publish_test.rs` / `cdylib_max_slice_len_provisioning_test.rs` | Clock shims through the FFI; collapse-no-publish parity; tier-default `max_slice_len` provisioning from a loaded cdylib. | `#[serial]` | cdylib fixtures (headers) |
| `cdylib_blocking_doorbell_test.rs` | Blocking source → doorbell pipe: classification, live fire + delivery, poisoned-helper EOF UNBIND (no busy-loop), library-leak breadcrumb, registry-poison containment. | `#[serial]` tt1 | `test_node_macro_blocking_cdylib` |
| `cdylib_tracing_stopgap_test.rs` | Cdylib-local stderr tracing subscriber via subprocess stderr capture: env-filter contract, flood-latched discard error, install-once breadcrumb (printed only when `RUST_LOG` is set; a default run prints none, its own pin); an UNPARSEABLE `RUST_LOG` spec falls back to `info` LOUDLY (the fallback breadcrumb names the offending spec, never a silent swallow). | `#[serial]` tt1 | `test_node_discard_probe_cdylib`, built in the SAME profile as the test binary (`cargo build [--release] -p …`; a sibling-profile fixture is refused loudly, never silently used) |
| `debug_count_discipline_test.rs` | The release-safe DEBUG-count discipline as a RULE: a comment-stripped source walk over `cerulion_core` + `rmw_cerulion` (src AND tests) requiring every DEBUG-line count to route through `cerulion_core::testing::debug_lines_expected` (presence: `debug_level_compiled_in` / `trace_level_compiled_in`), a same-marker never-loud sweep beside every gated site, and an attached or bound zero for a silence control; names the offending `file:line` + fn. The counting vocabulary lives in `cerulion_core::testing` too: `line_level` reads the level from a captured line's HEADER, `count_at`/`logged_at`/`never_loud` match it as a whole token, and `count_at_exclusively`/`lines_at_exclusively` are THE form for a positive "exactly N loud lines" claim: the level count AND the level-free total in one call, so a copy of the line at another level cannot pass. Scope is those two crates ONLY; the same shape exists ungated in the test code of `cerulion_bagd`/`cerulion_netd`/`cerulion_viz`/`cerulion_cli_engine` (unit tests inside `src/` as well as `tests/`; see the file header). | parallel | none |
| `cdylib_iox2_log_level_test.rs` | `IOX2_LOG_LEVEL` reaches a cdylib's OWN iceoryx2-log static; repo-walking structural guard on every hand-written init. | `#[serial]` tt1 | `test_node_discard_probe_cdylib`, `test_node_cdylib` |
| `iox2_log_level_test.rs` | `IOX2_LOG_LEVEL` filters iceoryx2's own logger (subprocess; the real production warning line as probe). | parallel | none |

### Structural source-walk gates

Guards whose subject is the SHAPE of the tree rather than a runtime behaviour. Each walks the
file's own source and fails with the evidence it found: a `file:line` per finding where the walk
tracks one (`cfg_audit_test`, `read_site_role_mint_inventory_test`), otherwise the offending token
plus why it is banned (`state_child_discipline_test`, whose ban-list arm reports the token and the
rationale rather than a location). Most strip comments first, because the subject is code shape;
`doc_attachment_discipline_test` deliberately does NOT, because `///` lines are its subject and
stripping would delete them. They live here because a property only a walk can see has no
behavioural home; two of them guard the ABSENCE of code, which no runtime check can observe.
`debug_count_discipline_test.rs` is a further member of this family, listed under *cdylib runtime*
above.

| Test file | Pins | Serial | Fixtures |
|---|---|---|---|
| `serial_discipline_test.rs` | A test that reaches the process-global iceoryx2 singleton must be `#[serial]` unless its whole package runs `-- --test-threads=1`, plus the nextest `default-namespace` fence asserted set-EQUAL to this file's own inventory, in BOTH directions (a stale waiver fails too). | parallel (pure file + source parse) | none |
| `tracing_field_discipline_test.rs` | The structured-logging rule becomes a GATE instead of prose: a datum belongs in a structured field, never spliced into the message text. | parallel | none |
| `doc_attachment_discipline_test.rs` | A `///` block belongs to the item BELOW it. This is the doc-ANNEX class, where inserting an item between prose and its target silently re-parents the prose onto its neighbour. | parallel | none |
| `cfg_audit_test.rs` | Nothing PORTABLE in `cerulion_core` may name a `#[cfg(unix)]`-only module, in code OR in an intra-doc link, so the portable half of the crate keeps compiling for a non-unix target. It is a source walk, and it does NOT stand in for a native Windows toolchain build: a real `--target x86_64-pc-windows-msvc` check cannot run from a mac host (`iceoryx2-pal-posix`'s bindgen dies on a missing `errno.h`), so nothing else covers this. Two arms, because configuring the unix-only MODULES out leaves the `unix` predicate TRUE and so cannot tell a gated reference from an ungated one. | parallel | none |
| `read_site_role_mint_inventory_test.rs` | Every read-outcome MINT names its read-site role as a compile-time constant, and the set of mint sites is a DECLARED inventory. The role's contract is that the CALL SITE declares it; a site that inferred one from state would be the silent inversion the declared constant exists to prevent. A STRUCTURAL floor, not a correctness proof: it proves a mint NAMES a role and that the mint count has not moved unnoticed. Per-site behaviour is pinned by the `role_view` arms in `read_outcome_capture_iox2_test.rs`; a NEW mint arm is covered by no behavioural arm, and the failure mode is silence. | parallel | none |
| `state_child_discipline_test.rs` | The fork-child module's documented ban list: nothing in it may take a lock, allocate, or leave by a path running `Drop` for the parent's live data plane. A fork child has one thread, so a lock any other parent thread held at the fork instant is held forever by nobody: these do not FAIL when wrong, they WEDGE in a process nobody watches and surface five seconds later as `ChildStalled` pointing at an innocent node. Two rules are the ABSENCE of code (e.g. the child never calls `panic::set_hook`), which no runtime check can see. | parallel | none |
| `doc_inventory_discipline_test.rs` | Every depth-1 `*.rs` under `crates/cerulion_core/tests/` (the set cargo builds as a test binary and `ci_test_shard.sh` enumerates) has a row in THIS file; plus the stale-ROW direction and the duplicate check, all three keyed on a row's TARGET PATH rather than its base name (a bare name means this directory, a qualified path means itself) so a sibling crate's same-named test is neither counted as coverage here, nor skipped when it is deleted, nor reported as a duplicate; a two-sided exemption inventory; and hand fixtures for the row reader (18 real rows name 2-4 files in one cell, so a first-name-only reader would report false holes), the stale-row rule, and the same-name collision. Enforces PRESENCE, not substance. | parallel | none |

## Adding a test here

- Behavior names; hand-written oracles, never a run compared against itself. A
  fire-count proves scheduling, not delivery: strong pins assert downstream DELIVERY.
- New real-transport tests default to a per-test SHM root (`build_for_test` /
  `init_for_test`) so they land in the parallel column; only the global-singleton path
  forces a tt1 row. Update this map in the same PR.
- Every macro-runtime semantic proven in-process gets a cdylib-parity twin (the
  `cdylib_*` family); the two paths diverge when only one is tested.
- Mutation-verify load-bearing guards on PURE decision functions only (never live
  syscall/transport paths), against a committed baseline, each variant failing an
  attributable test.
- Multi-mode failure fixtures follow the `CER_FAIL_MODE` env-switch pattern (canonical:
  `crates/test_fixtures/test_node_failing_cdylib`; the drain-fail / snapshot-fail / portwrite
  fixtures follow it): ONE fixture crate, one env var selecting the failure arm, so each
  test drives its mode without minting a fixture per mode.
- A structural guard must never quote the FULL literal it protects: a guard that quotes
  the guarded text dies to one global find/replace: the rename rewrites the expression
  AND the assertion together, and the gate stays green. Assert a property or a shorter
  substring instead of the guarded text verbatim.
- Env-mutating tests: `#[serial]` + an RAII env guard (restore on Drop); declare any
  mutex guard field LAST in fixtures (Rust drops fields in declaration order; a
  first-declared lock releases before the env restores).
- `#[traced_test]` requires the `no-env-filter` feature (already on the dev-dep) or
  production-crate events are silently dropped; assert suppression-latch lines with the
  LEVEL TOKEN, not message text alone.
