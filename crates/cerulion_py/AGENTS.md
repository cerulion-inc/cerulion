# cerulion_py - agent guide

PyO3 wheel: raw zero-copy Python client over the Cerulion transport.
Standalone cargo workspace (root `Cargo.toml` excludes it; it has its own
`Cargo.lock` and target dir; cargo-deny uses `tools/release/deny.toml`). **iceoryx2 must stay 0.9.1 and
zenoh 1.8.0** in `Cargo.lock`: after manifest edits run
`cargo update --workspace --offline`, never `cargo generate-lockfile`.

## Build, lint, test

```bash
export RUSTUP_TOOLCHAIN=1.93.0          # everything in this workspace
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo deny --config ../../tools/release/deny.toml check advisories licenses bans sources
maturin develop --release               # in a venv with maturin
cargo build --release -p cerulion_py_fixtures
cd /tmp && CERULION_PY_FIXTURE=<abs>/crates/cerulion_py/target/release/cerulion_py_fixture \
  python -m pytest -p no:cacheprovider <abs>/crates/cerulion_py/tests -q
./scripts/e2e_fresh_venv.sh             # builds + installs a real wheel
```

## Invariants

- One process-wide transport: `connect()` wraps the core `TransportManager`
  singleton. ALL tests run in ONE pytest process - never run two pytest
  processes on the same topics, and keep test topics unique per test.
- Zero-copy means: `Frame` views (`raw`/`payload`/`as_numpy`, read-only -
  the subscriber maps the publisher's segment `r--`) and `loan()`'s
  writable slot view. `to_bytes()`/copying is "materialised (copies)";
  `publish(buffer)` is "single-copy publish". Use that wording only.
- A published sample lives in the PUBLISHER's SHM segment: an instant-exit
  publisher loses undelivered frames. The fixture's `--linger-ms`
  (default 1000) exists for this.
- Held frames hold borrowed slots (`max_borrowed_samples`, default 2);
  `release()` defers while views live. `BorrowLimitExceeded` is the
  symptom of hoarding.
- Error mapping lives in `src/errors.rs` - every `TransportError` variant
  is enumerated explicitly; keep it exhaustive.
- PyO3 buffer protocol (`__getbuffer__`/`__releasebuffer__`, `libc::poll`,
  `assume_init`) are the only unsafe blocks; each carries a SAFETY
  comment. New unsafe needs maintainer approval (root `AGENTS.md`: Ask first).
- Facade (`python/cerulion/_api.py`) owns Python ergonomics; `src/` owns
  transport semantics. Non-contiguous or non-byte buffers are rejected as
  `TypeError` at the facade.
- Typed layer (`SchemaSet`, `Layout`, `Message`, `Frame.view()`): read-only
  NumPy views on receive, writable views on loans. `string[]` / `Type[]` are
  pre-framed BYTES only on publish/loan; loan views are block-scoped (a live
  export at `with`-exit discards the loan: `EncodeError`); a set mutation that
  changes a typed publisher's hash fails with `SchemaMismatch`.
- `crates/cerulion_pynode` embeds ONE CPython per node process (GIL-serialized;
  no Python threads, `fork`, or signal handlers). Its fixtures under
  `fixtures/pynodes/` are separate cdylib crates: rebuild them after any change.

## Layout

`src/` (native module: session, publisher, subscriber, frame, typed, bag, errors),
`python/cerulion/` (facade), `fixtures/` (Rust interop binary),
`tests/` (pytest), `scripts/` (e2e). User doc: `../../docs/python.md` -
its code blocks are executed by `tests/test_docs.py`.
