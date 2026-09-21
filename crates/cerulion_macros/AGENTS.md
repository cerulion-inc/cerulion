# cerulion_macros - agent notes

Proc-macro crate: `#[cerulion_node]` (struct side: parse, validate, codegen, cdylib FFI)
and `#[cerulion_node_impl]` (tick-body rewriter). A green `cargo test -p cerulion_macros`
runs only the in-src unit modules - it does NOT validate generated-code behavior. The
integration oracle for every codegen/rewriter change is `cargo test -p cerulion_core`.

## Invariants

- Trigger policy is macro-only; graph YAML carries no policy block. Never add a YAML
  policy path.
- `sync_window_ms` / `unbounded_sync` fire ONCE PER COMPLETE ALIGNED SET, in set order, each
  trigger message consumed by at most one set - a backlog holding k sets yields k fires, not one
  on the freshest frames. Sync means nothing under 2 `#[input(trigger)]` ports: 0 is a compile
  error; exactly 1 is ACCEPTED by the validator and the graph build degrades the node to a data
  trigger with a `warn` (`sync_attr_1_trigger_warn_test.rs`). A plain `#[input]` never gates a fire.
- Any change to a generated cdylib FFI signature or its return-code semantics bumps
  `CERULION_ABI_VERSION` (`crates/cerulion_core/src/lib.rs`) - the loader hard-rejects a
  mismatch, so an unbumped change breaks every deployed cdylib at load. New OPTIONAL
  symbols are additive with no bump; symbol presence is the capability gate.
- A trigger-policy surface change updates ALL of, together: `MacroPolicy` + `PolicyJson`
  (`crates/cerulion_core/src/graph/node.rs`), BOTH emitters (`gen_cdylib`'s `policy_json` arm
  AND `gen_zero_copy_node_entry_impl`'s `.with_policy(...)` chain in `src/codegen.rs`),
  the CLI `--policy` parser, and the user docs - the in-process and cdylib paths
  diverge whenever only one emitter moves.
- Keep internal modules `mod`, never `pub mod` - a `proc-macro = true` crate may export
  only its macros; `pub mod` is a hard compile error.
- Macro error paths re-emit the original struct alongside the `compile_error!` - keep
  that, or one real diagnostic cascades into pages of "cannot find type" noise.

## Testing

- `cargo test -p cerulion_macros` - token-level unit modules (parse/rewriter/determinism).
- Behavior lives in `crates/cerulion_core/tests/`: `macro_test`, `fill_from_rewriter_test`,
  `rewriter_var_field_assign_test` (parallel-safe); `macro_cdylib_test`,
  `macro_graph_test`, `abi_version_mismatch_test`, `lazy_loan_iox2_test` need
  `-- --test-threads=1` (they touch the process-global transport singleton).
- Diagnostics: `cargo test -p cerulion_core --test macro_compile_fail_test` (trybuild).
  Re-bless after an intentional message change:
  `TRYBUILD=overwrite cargo test -p cerulion_core --test macro_compile_fail_test`.
  The rustc-rendered groups (`tests/ui/const_eval/`, `tests/ui/type_error/`) are
  `#[ignore]`d (toolchain-fragile); run them via `-- --ignored`.

## Gotchas

- `Ident::to_string()` keeps the `r#` prefix; feeding it into `format_ident!` for a
  derived name panics at expansion. Strip `r#` before deriving idents; keep the raw
  spelling in user-facing messages. ROS 2 has keyword fields (`Marker.type`).
- Two-phase borrows do not apply to closure captures: wrapping a user RHS in a closure
  over the written port breaks same-port read-backs (E0502). Hoist the RHS to a temp
  first - Rust evaluates `rhs` before `place`, so the hoist is semantics-preserving.
- The struct↔impl port handoff (`Mutex<HashMap>`), the source-order requirement (struct
  expands before its impl), and the one-node-type-per-struct-name-per-crate collision
  limit are documented in `src/registry.rs`'s module doc; read it there before touching either macro.
- Parse integer attrs with `syn::LitInt::base10_parse` - `Literal::to_string()`
  preserves `100_000`/`0xff`/type suffixes, which `str::parse::<u64>()` rejects, and a
  swallowed parse error silently drops the user's policy.

Deep reference: docs/internals/macros.md - read before changing the attribute grammar,
the rewriter, generated FFI symbols, or trybuild fixtures.
