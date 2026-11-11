# cerulion_core - agent notes

Core: wire, codegen, iceoryx2, scheduler, graph and gateway; per-binary serial tests.

## Invariants

- ONE general read path: the iceoryx2 queue receive. No bypass/raw-handle read;
  a fused chain may serve a frame it just committed, in-process.
- Any change to a generated-cdylib FFI signature or error-code meaning bumps
  `CERULION_ABI_VERSION`; a missed bump misloads every cdylib.
- Native plugins: check boundary major and complete stable layout; ignore compiler provenance.
- Never `#[cfg(feature)]`-gate a struct FIELD on a type crossing the cdylib boundary (the two
  sides see different layouts); fields stay, only setters gated.
- Forbidden as raw SHM payload: types with internal pointers (`String`, `Vec`, `HashMap`);
  strings and bytes ride variable fields via the offset table, the slot is the backing.
- Wire `sequence` is consumed at COMMIT (`OutputProxy::Drop`), never at loan: a discarded
  loan burns no sequence, so streams stay gap-free.
- A publisher drains its OWN event listener on every notifying path (iceoryx2 delivers
  notifies to self); skipping it saturates the socket, floods logs.
- New flood-suppression sites reuse `transport::failure_regime_latch`, never hand-rolled;
  totals log as `total_failures=`, site keys as `topic=`/`service=`.
- Sync fires ONCE PER COMPLETE ALIGNED SET, each trigger consumed by at most one set; verdicts
  from pure `sync_match.rs`; in-order consumption and arrived-set preservation inviolable.
- Graph YAML denies unknown fields - a typo'd key is a loud parse error, never a silent
  default; a new field needs round-trip + rejection oracle arms.
- Replay = Live: `external_source()` is queried once at `run_live` entry, never under
  polled `step()`; parks/spins/wakes are record-only (change WHEN, never WHAT).
- Wake loops: drain BEFORE waiting, never block on an empty slice; pace `last_wait_blocked()`;
  never drop a `SeqCst` fence in the doorbell ring/park pair (its lost wake is untestable).
- Same-PROCESS REST producers of a `multi_publisher_topics` topic publish in declaration
  order (`RestWalk::InsertionOrder`), never under rayon (replay-unstable, trace-blind).
- iceoryx2 deps: ONE exact-pinned version workspace-wide; a skew silently kills the data plane.
- Hot-path alloc lint sweeps `src/{transport,graph,scheduler}/`; column-0 `#[cfg(test)]` is SKIPPED, so a `mod tests` alloc is invisible.
- `graph/node_plugin_abi/` owns local adapters for the checked C tables. Host
  samples remain behind integer leases. Ordinary receive owners outlive the entire
  callback, including an early view drop; local writers drop before release.
  Adapter oracle tests: `cargo test -p cerulion_core --lib graph::node_plugin_abi`.
  Bootstrap/metadata/refusal fixture: `node_plugin_exports_test` (build
  `test_node_macro_cdylib` first; no transport is created).
  `node_plugin_raw_test` exercises both raw SDK backends with actual SHM delivery;
  it also pins helper clock/raw delivery during an active node execution gate.
  Node diagnostics replace one pending token per instance; create failures
  replace one library token. New failures invalidate old tokens.
  run it alone with `--test-threads=1`.
- Stable node/host tests: `core-transport.md`; readers/writers drop before release.
- Shipped surface (`check_public_surface.sh`): `examples/` here is no user example (in-code
  graphs: tests only); no tracker ids or typographic dashes shipped; never bulk-rewrite inside
  a string literal (a removed dash lowers the file's ledger line).

## Testing

- Binaries taking the global TransportManager singleton run ALONE (`-- --test-threads=1`
  locally, a nextest fence in CI); per-test-SHM-root ones are parallel-safe.
- CI shards 4 ways: `./tools/scripts/ci_test_shard.sh cerulion_core <n> 4` runs one leg
  (`cargo nextest run`, NOT `--test-threads=1`); fence `.config/nextest.toml`, `serial_discipline_test`.
- cdylib tests dlopen prebuilt fixtures (`cargo build -p test_node_*` first); after ANY core
  change rebuild ALL, or a stale one SIGABRTs like a regression. Executor/step/drain changes:
  run EVERY `GraphRuntime`/`step()` binary (subsets missed twice).
- Cross-process credit: `credit_test`, `credit_block_iox2_test`, `free_run_ctor_iox2_test`:
  parallel-safe, no fence. Re-bless trybuild (`TRYBUILD=overwrite`, `macro_compile_fail_test`)
  on drift, from CI's output.

## Gotchas

- `#[traced_test]` needs tracing-test's `no-env-filter` feature (on the dev-dep) or
  events vanish. `GraphRuntime` is `!Debug`: use `match`.
- macOS iceoryx2 select() aborts when any fd NUMBER >= 1024 enters a WaitSet.
- Per-frame byte-identity: publish in lockstep + `try_receive_one` (drain-all keeps newest).

Read first (`docs/internals/`): `core-transport.md` (transport/latch/gateway), `core-scheduler-graph.md`
(scheduler/graph/determinism), `core-testing.md`, `core-dynamic.md`.
