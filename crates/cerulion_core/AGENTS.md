# cerulion_core - agent notes

Wire/codegen, iceoryx2, scheduler/graph and gateways; test isolation varies by binary.

## Invariants
- Excluded producers hold guards before data ports; remote taps, mirrors and announces retain opposite leases until withdrawal. Remote metadata uses `collect_remote_run_entries`: v2 private/v1 public, local accepts both; hide private ids and counts. No frame-path checks.

- ONE general read path: the iceoryx2 queue receive. No bypass/raw-handle read;
  a fused chain may serve a frame it just committed, in-process.
- FFI signature/error-meaning changes bump `CERULION_ABI_VERSION` or misload cdylibs;
  the loader also refuses a differing `RUSTC_FINGERPRINT`.
- Never `#[cfg(feature)]`-gate a struct FIELD on a type crossing the cdylib boundary (the two
  sides see different layouts); fields stay, only setters gated.
- No pointer-bearing raw SHM types (`String`, `Vec`, `HashMap`); strings/bytes use
  variable-field offsets backed by the slot.
- Wire `sequence` is consumed at COMMIT (`OutputProxy::Drop`), never at loan: a discarded
  loan burns no sequence, so streams stay gap-free.
- A publisher drains its OWN event listener on every notifying path (iceoryx2 delivers
  notifies to self); skipping it saturates the socket, floods logs.
- New flood-suppression sites reuse `transport::failure_regime_latch`, never hand-rolled;
  totals log as `total_failures=`, site keys as `topic=`/`service=`.
- Sync fires ONCE PER COMPLETE ALIGNED SET, each trigger consumed by at most one set; verdicts
  from pure `sync_match.rs`; in-order consumption and arrived-set preservation inviolable.
- Graph YAML denies unknown fields loudly; new fields need round-trip and rejection oracles.
- Replay = Live: `external_source()` is queried once at `run_live` entry, never under
  polled `step()`; parks/spins/wakes are record-only (change WHEN, never WHAT).
- Wake loops: drain BEFORE waiting, never block on an empty slice; pace `last_wait_blocked()`;
  never drop a `SeqCst` fence in the doorbell ring/park pair (its lost wake is untestable).
- Same-PROCESS REST producers of a `multi_publisher_topics` topic publish in declaration
  order (`RestWalk::InsertionOrder`), never under rayon (replay-unstable, trace-blind).
- iceoryx2 deps: ONE exact-pinned version workspace-wide; a skew silently kills the data plane.
- Hot-path alloc lint sweeps `src/{transport,graph,scheduler}/`; column-0 `#[cfg(test)]` is SKIPPED, so a `mod tests` alloc is invisible.
- `check_public_surface.sh`: in-code graphs belong in tests, including this crate's examples;
  no shipped tracker ids/typographic dashes. Never bulk-edit strings; removed dashes lower ledgers.

## Testing

- Binaries taking the global TransportManager singleton run ALONE (`-- --test-threads=1`
  locally, a nextest fence in CI); per-test SHM roots isolate data, but process-global counters serialize.
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
