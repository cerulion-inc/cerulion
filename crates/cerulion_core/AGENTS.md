# cerulion_core - agent notes

Core runtime: wire format, codegen, iceoryx2 transport, scheduler/level executor, graph
runtime, gateway plane; 260+ test binaries with per-binary serial rules.

## Invariants

- ONE consumer read path: every read goes through the iceoryx2 queue receive; no
  bypass/raw-handle reads (chain fusion is the latency lever).
- Any change to a generated-cdylib FFI signature or error-code meaning bumps
  `CERULION_ABI_VERSION`; a missed bump misloads every cdylib.
- The loader ALSO refuses a cdylib whose `RUSTC_FINGERPRINT` differs from the host's.
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
- Sync fires ONCE PER COMPLETE ALIGNED SET, in order, each trigger consumed by at most one
  set, never once per alignment on the freshest frames. Verdicts come from pure
  `scheduler/sync_match.rs`; in-order consumption and arrived-set preservation are inviolable.
- Graph YAML denies unknown fields - a typo'd key is a loud parse error, never a silent
  default; a new field needs round-trip + rejection oracle arms.
- Replay = Live: `external_source()` is queried once at `run_live` entry, never under
  polled `step()`; parks/spins/wakes are record-only (change WHEN, never WHAT).
- Wake loops: drain BEFORE waiting, never block on an empty slice; pace on `last_wait_blocked()`.
- Same-PROCESS REST producers of a `multi_publisher_topics` topic publish in declaration
  order (`RestWalk::InsertionOrder`), never under rayon (replay-unstable, trace-blind).
- iceoryx2 deps: ONE exact-pinned version workspace-wide; a skew silently kills the data plane.
- Hot-path alloc lint sweeps `src/{transport,graph,scheduler}/`; column-0 `#[cfg(test)]` is SKIPPED, so a `mod tests` alloc is invisible.
- Shipped surface (`check_public_surface.sh`): `examples/` here is no user example (in-code
  graphs: tests only); no tracker ids or typographic dashes in shipped text; never bulk-rewrite
  inside a string literal (a removed dash lowers the file's ledger line).

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
  production events silently vanish. `GraphRuntime` is `!Debug` - use `match`.
- macOS iceoryx2 select() aborts when any fd NUMBER >= 1024 enters a WaitSet.
- Per-frame byte-identity: publish in lockstep + `try_receive_one` (drain-all keeps newest).

Read first (`docs/internals/`): `core-transport.md` (transport/latch/gateway), `core-scheduler-graph.md`
(scheduler/graph/determinism), `core-testing.md`, `core-dynamic.md`.
