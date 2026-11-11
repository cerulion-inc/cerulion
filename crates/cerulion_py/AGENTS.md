# cerulion_py - agent guide

PyO3 wheel: raw zero-copy Python client over the Cerulion transport.
Standalone workspace: own lock/target; deny uses `tools/release/deny.toml`.
**iceoryx2 must match the root workspace at 0.10.0 from the patched fork, mirrored in this
manifest's `[patch.crates-io]`, and zenoh must stay at 1.8.0** in `Cargo.lock`: after manifest edits run
`cargo update --workspace --offline`, never `cargo generate-lockfile`.

## Build, lint, test

```bash
export RUSTUP_TOOLCHAIN=1.95.0          # everything in this workspace
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo deny --config ../../tools/release/deny.toml check advisories licenses bans sources
maturin develop --release               # in a venv
cargo test -p cerulion_py_fixtures
cargo build --release -p cerulion_py_fixtures
cd /tmp && CERULION_PY_FIXTURE=<abs>/crates/cerulion_py/target/release/cerulion_py_fixture \
  python -m pytest -p no:cacheprovider <abs>/crates/cerulion_py/tests -q
./scripts/e2e_fresh_venv.sh             # builds + installs a real wheel
```

## Invariants

- One process-wide transport: `connect()` wraps the core `TransportManager` singleton. ALL tests run in
  ONE pytest process - never run two pytest processes on the same topics; keep test topics unique per test.
- Zero-copy means: `Frame` views (`raw`/`payload`/`as_numpy`, read-only - the subscriber maps the
  publisher's segment `r--`) and `loan()`'s writable slot view. `to_bytes()`/copying is "materialised
  (copies)"; `publish(buffer)` is "single-copy publish". Use that wording only.
- A published sample lives in the PUBLISHER's SHM segment: an instant-exit publisher
  loses undelivered frames; the fixture's `--linger-ms` (default 1000) exists for this.
- Held frames hold borrowed slots (`max_borrowed_samples`, default 2); `release()`
  defers while views live. `BorrowLimitExceeded` is the symptom of hoarding.
- Error mapping lives in `src/errors.rs`: every `TransportError` variant is enumerated; keep it exhaustive.
- Buffer protocol/poll/assume_init unsafe sites carry SAFETY arguments; new sites need approval.
- Facade (`python/cerulion/_api.py`) owns Python ergonomics; `src/` owns transport semantics.
  The facade rejects non-contiguous/non-byte buffers as `TypeError`.
- Typed layer (`SchemaSet`, `Layout`, `Message`, `Frame.view()`): read-only NumPy views on receive,
  writable views on loans. `string[]` / `Type[]` are pre-framed BYTES only on publish/loan; loan views
  are block-scoped (a live export at `with`-exit discards the loan: `EncodeError`); a set mutation that
  changes a typed publisher's hash fails with `SchemaMismatch`.
- `cerulion.msgs` (`python/cerulion/msgs/`): ONE schema set per workspace (`_set_for`), bound per
  node type by the host (`_bind_workspace`, node dir active, swaps the `cerulion.msgs.<pkg>` modules
  with its `helpers`), else the process default. Views are `_view()`-built `schemas.message_type(name)`
  instances; never construct `Message(...)` directly. Built-in `<pkg>.py`/`.pyi` and `__init__.pyi` are
  GENERATED (`_stubs.write_builtin_stubs`, pinned by `test_msgs.py`): regenerate after corpus/renderer edits.
- Embedded nodes use ONE CPython per process (GIL serialized; no Python threads, fork or
  signal handlers). Rebuild `fixtures/pynodes/*` separate cdylib workspaces after changes.

`src/` (native module: session, publisher, subscriber, frame, typed, bag, errors), `python/cerulion/`
(facade), `fixtures/` (Rust interop binary), `tests/` (pytest), `scripts/` (e2e). User doc:
`../../docs/python.md` - its code blocks are executed by `tests/test_docs.py`.

Stable export details and per-site safety: `../../docs/internals/node-abi.md`.
Python objects stay local; raw leases/spill retain native preflight and publish order.
Metadata retains code/context for process lifetime (CPython caches code/escaped views).
No state/external capabilities; diagnostics preserve exact text plus host classification.
Escaped-view tests pin lifetime; deferred releases flush before Python runs.
