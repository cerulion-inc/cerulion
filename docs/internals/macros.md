# Macros internals: `#[cerulion_node]` / `#[cerulion_node_impl]`

Scope: the `cerulion_macros` proc-macro crate and the contracts coupling it to
`cerulion_core` (generated-code write shims, the cdylib FFI/ABI, the trybuild
diagnostic suite). The user-facing attribute surface is specified in `docs/user-api.md` and
is a contract: changes need explicit maintainer approval.

## Crate layout and division of labor

| Module | Owns |
|---|---|
| `src/lib.rs` | The two `#[proc_macro_attribute]` entry points; module wiring. |
| `src/parse.rs` | Attribute grammar: node-level `NodeAttr` args + field-level `#[input]`/`#[output]` parsing; loud rejection of the legacy `type_name`/`inputs(...)`/`outputs(...)` args. |
| `src/validate.rs` | Cross-attribute validation: mutual exclusions, bounds, duplicate ports, trigger-inference preconditions. |
| `src/codegen.rs` | Struct-side generation: the `<Name>Entry` wrapper, its `NodeEntry` impl, policy wiring, and the cdylib FFI symbols (behind the `cdylib` feature). |
| `src/impl_macro.rs` | The tick-body AST rewriter, `#[on_event]` handler discovery, and the determinism deny walk. |
| `src/registry.rs` | The cross-macro port handoff (struct macro → impl macro). Its module doc is the authority on the mechanism, the source-order rule, and the collision limit. |
| `src/determinism.rs` | The banned-non-deterministic-symbol table the deny walk matches against, the single source of truth for the determinism lint policy. |

The macro crate is deliberately **schema-blind**. Fixed-vs-variable field resolution
lives in `cerulion_core`'s schema codegen (`crates/cerulion_core/src/codegen/generator/`),
which emits a uniform fallible write shim per schema field
(`__cer_assign_<field>` / `__cer_fill_from_<field>`) on each generated `<Name>Shm`
type. The rewriter targets shims by port name alone and never inspects the schema, so
schema changes never require rewriter changes. Schema field names starting with
`__cer` are reserved.

A `proc-macro = true` crate may export only its macros: every internal module stays
`mod` (never `pub mod`, a hard compile error), and nothing here is importable by other
crates. Anything both the macro and another crate must consume (e.g. the determinism
table) has to be relocated to a shared non-proc-macro crate, not re-exported.

## Attribute grammar contracts

Node-level `#[cerulion_node(...)]`, all optional; at most one trigger-policy hint:

| Arg | Meaning | Constraint (enforced in `validate.rs`) |
|---|---|---|
| `period_ms = N` | Period trigger | excludes `throttle_ms` |
| `sync_window_ms = N` | Bounded sync | requires ≥2 `#[input(trigger)]` fields; excludes `unbounded_sync` |
| `unbounded_sync` | Unbounded sync | requires ≥2 trigger inputs; excludes `sync_window_ms` |
| `external` | Self-triggering ingress node | REQUIRES a user-written `external_source()` method; that method is REJECTED on non-external nodes (gated via the registry's `external` flag) |
| `tick_within_ms = N` | Tick-duration QoS counter | `N > 0`; stacks with any trigger |
| `throttle_ms = N` | Producer rate cap | `N > 0`; mutually exclusive with `period_ms` only |
| `allow_non_deterministic`, `uses_live_io` | Determinism-lint opt-outs | consumed by the deny walk |

With no node-level hint, the trigger is inferred from a field-level `#[input(trigger)]`.
Field-level grammar: `#[input(trigger, depth = N, backpressure =
drop_oldest|block|sample(N), expect_within_ms = N)]` and
`#[output(promise_within_ms = N)]`. Deadlines must be `> 0`; duplicate port names
reject. The `inputs(...)`/`outputs(...)` args are rejected at parse
time with a hint pointing at field attributes (`PORT_ARGS_NOT_ACCEPTED_HINT` in
`parse.rs` centralizes that wording); `type_name` is rejected separately with a
message pointing at the folder name.

Error-emission contract: on any parse or validation failure, the macro re-emits the
ORIGINAL struct alongside the combined `compile_error!`s. Dropping the struct
re-emission turns one real diagnostic into a cascade of "cannot find type" errors on
every use site.

Integer attribute values must be read with `syn::LitInt::base10_parse::<u64>()` (or an
equivalent underscore/suffix/radix-aware parser). `proc_macro2::Literal::to_string()`
preserves source spelling (`100_000`, `100u64`, `0xff`), all of which
`str::parse::<u64>()` rejects; a swallowed parse error there silently drops the user's
policy, and the node compiles but falls back at runtime with only a warn.

## Rewriter rules (`impl_macro.rs`)

`#[cerulion_node_impl]` takes NO arguments. Port names and types come from the
cross-macro registry the sibling `#[cerulion_node]` populated (see below). The rewriter
builds a per-tick stack frame and rewrites every `self.<port>` access in `tick`, and in
every helper method on the impl block, to dispatch through it:

- **Outputs** become lazy-loan slots (`LazyOutput<T>`): the port loans its SHM slot on
  the FIRST write and vends an `OutputProxy<T>`. An output the tick never writes never
  loans, never publishes, never discards.
- **Inputs** become SHM-backed `InputView<T>` reader handles, taken upfront via the
  disjoint-borrow split on `NodeContext` and wrapped in nested `try_view` closures so
  every input view is in scope simultaneously.
- **Helper methods** get per-port parameters injected at the end of their signatures,
  and every call site is rewritten to pass the per-tick locals. Only methods whose
  receiver is `&self`/`&mut self` are walked; other receivers are left untouched and any
  port reference inside them fails to compile, surfacing the misuse.
- **Write shims**: every `self.<port>.<field> = expr` becomes a hoisted-RHS block
  calling the codegen-emitted `__cer_assign_<field>(…)?`; the `fill_from` form routes to
  `__cer_fill_from_<field>`. Because the rewritten write carries `?`, any helper that
  writes port fields must return `Result`, a user-visible contract.
- **`#[on_event(input|output = "...")]` handlers** are discovered on the impl block's
  methods and STRIPPED (there is no real `on_event` attribute; leaving one behind
  produces "cannot find attribute" on the emitted impl). A method may carry at most one;
  the event kind routes from the handler's parameter type; multiple handlers co-firing
  in a tick dispatch in DECLARATION order (pinned; a sort-by-name regression breaks
  `on_event_multi_handler_test`).
- **Determinism deny walk**: every method body is matched against the deny rows of
  `determinism.rs`; each hit emits a `compile_error!`. The opt-out attrs suppress it.
  The macro half emits DENY errors only; stable Rust gives proc-macros no warn-level
  diagnostic API.

Hard rules when changing the rewriter:

1. **Hoist before closure-wrapping.** Two-phase borrows make
   `receiver.method(arg_reading_receiver)` legal only as a direct method call; the same
   dataflow through a closure (or a chain link returning `&mut`) is E0502, because
   closures capture by reference while the receiver's `&mut` is live. Any rewrite that
   wraps a user RHS in a closure over the written target must bind the RHS to a temp
   first, semantics-preserving because Rust evaluates `rhs` before `place` in
   `place = rhs`. The pin is a test whose RHS reads the port being written
   (read-back-after-write in `lazy_loan_iox2_test` and the in-src rewrite-shape tests).
2. **Strip `r#` before deriving identifiers.** `Ident::to_string()` keeps the raw
   prefix, and `format_ident!("__cer_assign_{}", name)` on `"r#type"` panics at macro
   expansion. A prefixed/suffixed derived name is never a keyword, so stripping is
   always safe; user-facing messages keep the `r#` spelling (it is what the user must
   type). Keyword fields are common in ROS 2 schemas (`Marker.type`,
   `JoyFeedback.type`); schema codegen escapes them for field declarations and bare
   readers.
3. **Keep existing emission shapes byte-identical** when adding rewrite arms; the
   in-src unit tests pin exact emitted token shapes, and behavior parity is re-proven in
   `cerulion_core` (see the test map).

## Cross-macro registry (`src/registry.rs`)

The module doc in `src/registry.rs` is the authority; the load-bearing facts:

- `#[cerulion_node]` writes each struct's resolved port list into a process-wide
  `Mutex<HashMap>` keyed by the unqualified struct identifier; `#[cerulion_node_impl]`
  reads it at expand time. Both macros run inside one proc-macro DLL invocation, so the
  static map spans every expansion in a single `cargo build`.
- **Source order matters**: the struct's `#[cerulion_node]` must expand before its impl
  block's `#[cerulion_node_impl]`; the impl macro emits a clear error when the entry is
  missing.
- **Collision limit**: one node type per struct name per crate (the key is the
  unqualified identifier). Cross-crate collisions are impossible; each crate gets its
  own proc-macro instantiation.
- Port types are stored as token strings (a `syn::Type` is not `Send`/`Sync`); the
  consumer re-parses them, infallibly, because they came from a previously-parsed type.
- The registry also carries the determinism opt-out flags and the `external` flag (which
  gates the required-`external_source()` check). It is process-local to the proc-macro
  DLL and gone by the time any CLI runs; never route runtime-visible metadata through
  it; that path is structurally dead.

## Generated cdylib FFI and ABI coupling

`src/codegen.rs` emits the cdylib entry points (behind the `cdylib` feature). The
loader side is `DylibNodeEntry` in `crates/cerulion_core/src/graph/node.rs`.

| Symbol | Role |
|---|---|
| `cerulion_abi_version` | Must return `CERULION_ABI_VERSION`; the loader hard-rejects a mismatch or a missing symbol. |
| `cerulion_rustc_fingerprint` | Must return `RUSTC_FINGERPRINT` (ABI v22); the loader hard-rejects a mismatch or a missing symbol, checked immediately after `cerulion_abi_version`, because two different rustc releases can agree on every struct size and offset the ABI check proves and still encode a niche-holding `Option::None` differently. |
| `cerulion_node_info` | Node metadata JSON (ports, policy, capabilities). Parsed host-side into `PolicyJson`; corrupt JSON refuses the load. |
| `cerulion_node_init` / `cerulion_node_tick` / `cerulion_node_shutdown` / `cerulion_node_pump_history` | Lifecycle. Shared return-code semantics below. |
| `cerulion_node_external_source` | External-trigger source classification (external nodes). |
| `cerulion_node_set_snapshot_inputs` / `cerulion_node_snapshot_inputs` | OPTIONAL input-hold capability pair. |
| `cerulion_node_drain_trigger_input` | OPTIONAL unified trigger-drain capability. |

Return-code semantics (part of the ABI; the host and the replay verdicts classify on
them structurally, never on message text): `0` success; `1` tick returned `Err`; `2`
panic caught by `catch_unwind`; `3` the cdylib's node-registry mutex is poisoned; `4`
handle not found.

**The ABI rule**: any change to a generated FFI signature or to these return-code
semantics MUST bump `CERULION_ABI_VERSION` (`crates/cerulion_core/src/lib.rs`; name the
constant, never its value, in docs and comments). Without the bump, every deployed
cdylib built against the old shape loads against the new host and skews silently; with
it, the loader refuses loudly (`abi_version_mismatch_test`). New OPTIONAL symbols are
additive and need NO bump; **symbol presence is the capability gate**: the loader
probes for the symbol and degrades to the non-capable path when absent, which also
structurally scopes each capability to macro-generated cdylibs (raw-FFI nodes simply
lack the export).

Policy coupling: the two generation paths must stay in lockstep. The in-process path
chains `.with_policy(::…::MacroPolicy::…)` in `gen_zero_copy_node_entry_impl`; the
cdylib path hand-builds the `policy_json` arm in `gen_cdylib`, round-tripped through
`PolicyJson` (`crates/cerulion_core/src/graph/node.rs`) at load. A policy variant added to
only one emitter ships a node that silently degrades to the data-trigger fallback (with
a host warn) on the other path, so a policy change updates BOTH emitters and the
round-trip tests that cover them; the change-surface checklist is in
`crates/cerulion_macros/AGENTS.md`.

Node-side environment: the generated `cerulion_node_init` installs a cdylib-local
stderr `tracing` subscriber and applies `IOX2_LOG_LEVEL` from the node's frozen env
snapshot. A cdylib statically links its OWN copy of the core stack, so host-process
logging/config never reaches it; any hand-written raw-FFI `cerulion_node_init` must do
the same, and a repo-walk test enforces it
(`every_hand_written_cdylib_init_applies_the_iox2_log_level` in
`crates/cerulion_core/tests/cdylib_iox2_log_level_test.rs`).

## Trybuild suite and regen

Compile-fail coverage lives in `crates/cerulion_core/tests/macro_compile_fail_test.rs` over
`crates/cerulion_core/tests/ui/`:

- **Blocking** (every PR): `tests/ui/*.rs`, our own `compile_error!` diagnostics,
  stable across rustc versions, plus compile-pass fixtures in `tests/ui/pass/`.
- **`#[ignore]`d** (toolchain-fragile rustc renderings): `tests/ui/const_eval/`
  (const-eval panics) and `tests/ui/type_error/` (rustc-rendered type errors, e.g. the
  write-only-proxy misuse diagnostics). Run via `-- --ignored`.

Regen after an intentional diagnostic change or a toolchain bump:

```bash
TRYBUILD=overwrite cargo test -p cerulion_core --test macro_compile_fail_test
```

Two regen cautions:

- Prefer re-blessing from the CI job's ACTUAL OUTPUT block over a wholesale local
  overwrite; the local rustc usually differs from CI's, and `TRYBUILD=overwrite`
  clobbers unrelated `.stderr` files with local-toolchain renderings.
- Stacked PRs that each correctly re-bless the SAME `.stderr` fixture can git-auto-merge
  into a snapshot matching neither branch. Before folding a re-blessed snapshot across
  stacked branches, check whether the sibling branch has its own commit on that fixture
  path; if so, each branch needs its own rendering.

## Test map

The macro crate's own suite is token-level only; generated-code behavior is proven in
`cerulion_core`. Fixture cdylibs must be prebuilt (`cargo build -p <fixture>`); each
test file's header names its exact prereqs.

| Test | Pins | Serial? | Prereq fixtures |
|---|---|---|---|
| `cerulion_macros` in-src `#[cfg(test)]` (parse.rs, impl_macro.rs, determinism.rs) | Attribute parsing, emitted rewrite shapes (incl. the hoist-and-`?` nested-chain form), banned-symbol table | no | none |
| `crates/cerulion_core/tests/macro_test.rs` | Macro lifecycle, `info()`, state | no | none |
| `macro_compile_fail_test.rs` | Diagnostics via trybuild (blocking + ignored groups above) | no | none |
| `fill_from_rewriter_test.rs` | Rewriter no-false-positives (`fill_from` forms; `=` rewrite preserved) | no | none |
| `rewriter_var_field_assign_test.rs` | The schema-blind `=` write-shim rewrite, in-process | no | none |
| `macro_graph_test.rs` | Macro nodes inside `GraphRuntime` | yes | none |
| `macro_cdylib_test.rs` | Macro-generated cdylib loads via `DylibNodeEntry` | no | macro fixture cdylibs (see header) |
| `macro_cdylib_policy_round_trip_test.rs` | Policy JSON survives the FFI info round-trip | no | `test_node_macro_unbounded_sync_cdylib` |
| `abi_version_mismatch_test.rs` | Loader rejects an ABI-version mismatch | yes | fixture cdylib (see header) |
| `lazy_loan_iox2_test.rs` | Unwritten output never loans/publishes; incomplete write still discards loudly; same-port read-back-after-write compiles and delivers | yes | none |
| `cdylib_non_trigger_hold_test.rs` | Optional snapshot symbol pair: capability by presence; symbol-less cdylib is a safe no-op | yes | `test_node_macro_period_input_cdylib`, `test_node_cdylib` |
| `cdylib_unified_drain_test.rs` | Optional drain symbol: capability by presence; FFI failure maps to a safe empty drain | yes | see header (3 fixtures) |
| `cdylib_iox2_log_level_test.rs` | Generated + hand-written inits apply `IOX2_LOG_LEVEL` (repo-walk guard) | yes | `test_node_discard_probe_cdylib`, `test_node_cdylib` |

Further siblings (`macro_lifecycle_test`, `macro_shim_methods_test`,
`macro_shim_clock_sources_test`, `macro_sync_threading_test`,
`macro_chunks_ef_adversarial_test`, `macro_cdylib_overflow_test`,
`data_trigger_macro_binding_test`, `fill_from_codegen_test`, `fill_from_e2e_test`, and
the `cdylib_*` policy/QoS/event files) each state their serial requirement and fixture
prereqs in their file header; run serial binaries individually with
`-- --test-threads=1`, never the whole workspace at once.
