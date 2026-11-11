# cerulion_macros - agent notes

Node-author proc macros; token tests alone do not prove codegen behavior.
Codegen/rewriter changes run core and cerulion tests with root serial discipline.

## Invariants
- Generated runtime paths start at `#__cer_root` (`crate_root::root()`), never at
  `::cerulion_core`; under `quote_spanned!`, at `crate_root::root_at(span)`.
- Trigger policy is macro-only; graph YAML carries no policy block. Never add a YAML
  policy path.
- `sync_window_ms` / `unbounded_sync` fire ONCE PER COMPLETE ALIGNED SET, in set order, each
  trigger message consumed by at most one set - a backlog holding k sets yields k fires, not one
  on the freshest frames. Sync means nothing under 2 `#[input(trigger)]` ports: 0 is a compile
  error; exactly 1 is ACCEPTED by the validator and the graph build degrades the node to a data
  trigger with a `warn` (`sync_attr_1_trigger_warn_test.rs`). A plain `#[input]` never gates a fire.
- Stable table IDs, layouts and status meanings are frozen in `cerulion_node_abi`.
  Generated code uses the same checked boundary as Python; changes to frozen
  shapes require a new boundary major and a loud loader refusal.
- A trigger-policy surface change updates ALL of, together: `MacroPolicy` + `PolicyJson`
  (`crates/cerulion_core/src/graph/node.rs`), BOTH emitters (`gen_cdylib`'s `policy_json` arm
  AND `gen_zero_copy_node_entry_impl`'s `.with_policy(...)` chain in `src/codegen.rs`),
  the CLI `--policy` parser, and the user docs - the in-process and cdylib paths
  diverge whenever only one emitter moves.
- `#[param]` is ONE module, `src/param_attr.rs`: parse, checks and all three emissions come
  from one parsed list, so the in-process and cdylib paths cannot disagree (internals doc).
- Keep internal modules `mod`, never `pub mod` - a `proc-macro = true` crate may export
  only its macros; `pub mod` is a hard compile error.
- Stable adapters share rewritten bodies (`plugin_impl.rs`/`plugin_codegen.rs`).
  Writers drop before lease release, including SPILL_RELEASE after copied spill.
- Macro error paths re-emit the original struct alongside the `compile_error!` - keep
  that, or one real diagnostic cascades into pages of "cannot find type" noise.

## Testing
- `cargo test -p cerulion_macros` - in-src unit modules.
- `cargo test -p cerulion` - the only check of the `::cerulion::core` root.
- Behavior: `crates/cerulion_core/tests/`: `macro_test`, `fill_from_rewriter_test`,
  `rewriter_var_field_assign_test` (parallel-safe); `macro_cdylib_test`,
  `macro_graph_test`, `abi_version_mismatch_test`, `lazy_loan_iox2_test` need
  `-- --test-threads=1` (they touch the process-global transport singleton).
- Diagnostics: `cargo test -p cerulion_core --test macro_compile_fail_test` (trybuild).
  Re-bless after a deliberate message change:
  `TRYBUILD=overwrite cargo test -p cerulion_core --test macro_compile_fail_test`.
  The rustc-rendered groups (`tests/ui/const_eval/`, `tests/ui/type_error/`) are
  `#[ignore]`d (toolchain-fragile); run them via `-- --ignored`.

## Gotchas
- `Ident::to_string()` keeps the `r#` prefix: strip it before `format_ident!` (it panics) and
  for external names (`#[param] r#type` is the param `type`). ROS 2 has keyword fields.
- Two-phase borrows do not apply to closure captures: wrapping a user RHS in a closure
  over the written port breaks same-port read-backs (E0502). Hoist the RHS to a temp
  first - Rust evaluates `rhs` before `place`, so the hoist is semantics-preserving.
- The struct↔impl port handoff (`Mutex<HashMap>`), the source-order rule (struct expands
  before its impl) and the one-node-type-per-struct-name-per-crate collision limit: read
  `src/registry.rs`'s module doc before touching either macro.
- Parse integer attrs with `syn::LitInt::base10_parse` - `Literal::to_string()`
  preserves `100_000`/`0xff`/type suffixes, which `str::parse::<u64>()` rejects, and a
  swallowed parse error silently drops the user's policy.

Deep reference: docs/internals/macros.md (grammar, rewriter, root, FFI, trybuild).
