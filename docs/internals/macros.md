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
| `src/crate_root.rs` | The crate root every generated runtime path starts with, resolved once per compilation from the consuming package's `Cargo.toml`, and the missing-dependency diagnostic. Its module doc is the authority on the rule. |

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

## The crate root (`src/crate_root.rs`)

Every runtime path the macros emit is absolute, and Rust resolves its first
segment only against a crate the consuming package names itself. So the first
segment is resolved per package: `::cerulion_core` when the package names the
runtime, `::cerulion::core` when it names only the umbrella crate, or names
both with only the umbrella an unconditional dependency (a plain
`[dependencies]` entry without `optional = true`). The module doc carries the
full table, the optional-entry rule and what the resolver cannot see.

Rules for an emitter:

- Bind `let __cer_root = crate_root::root();` in the function and write
  `#__cer_root::graph::...`. A literal `::cerulion_core::...` compiles and
  passes every test in this repository that names the runtime, and breaks
  every package that names only the umbrella.
- Inside `quote_spanned!`, take `crate_root::root_at(span)` at the same span.
  `quote_spanned!` re-spans only its own tokens; an interpolated `root()`
  keeps the call-site span, and rustc then reports an unsatisfied
  `where`-clause predicate at the derive or node attribute instead of at the
  field. The five state predicates in `state_derive.rs` do this.
- Every exported macro returns its expansion through `with_root_diagnostic`
  (or matches on `crate_root::unresolved_diagnostic` at statement level), so a
  package that names neither crate gets a named error. A source-walk unit test
  in `crate_root.rs` enforces it.

The macro crate's own manifest names neither crate, so its unit tests that
render generated code read the `::cerulion_core` fallback; only the
`crate_root.rs` tests render the umbrella spelling, from a `Root` they build.
The umbrella spelling is compiled only by
`crates/cerulion/tests/one_dependency_build_test.rs` (see the test map).

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
drop_oldest|block|sample(N), expect_within_ms = N)]`,
`#[input(accumulate, cap = N)]` and `#[output(promise_within_ms = N)]`. The accumulate
keys parse, validate and generate in `input_accumulate.rs`: the cap is written into
`depth`, `init` registers the input on the entry's `AccumulatorSet`, the entry's freeze
(in-process `snapshot_inputs` and the cdylib `cerulion_node_snapshot_inputs` export)
goes through `AccumulatorSet::snapshot`, the tick serves the port as
`&Accumulated<'_, T>` outside the latest-value `try_view` chain, and both state seams
(`capture_state`/`restore_state` and the cdylib `cerulion_node_{capture,restore}_state`
exports) append `AccumulatorSet::{capture,restore}` after the user's fields (nothing for
a node without one). The CLI mirror of the grammar is `cerulion_cli_engine`'s
`node_metadata_accumulate.rs`. Deadlines must be `> 0`; duplicate port names
reject.

`#[param(default = L, min = L, max = L, description = "..")]` on a PLAIN field (`src/param_attr.rs`
owns parse, checks and emission). The field type is the param type (`bool`, the fixed-width
integers, `f32`, `f64`, `String`); `default` is required and must be a literal of exactly that
type (no int-to-float widening; integers must fit the width); `min`/`max` are numeric-only,
`min <= max`, and the default lies within them; `live` is refused as reserved; a param field
cannot also be a port. The module emits three things from one parsed list: the
`.with_params(vec![ParamSpec{..}])` chain in `info()`, the `,"params":[..]` fragment of the
cdylib info JSON, and `__cer_apply_param_defaults` (called by the wrapper's `new()`) plus
`__cer_apply_params` (called by `init` before the user's `init`, on the values
`NodeEntry::set_params` held). `__cer_param_values` snapshots every param right after that
write and `__cer_changed_param` compares the fields with the snapshot (floats by bit
pattern); the wrapper's `capture_state` and the cdylib `cerulion_node_capture_state`
refuse a capture once a param changed, because an anchor does not carry params. Resolution against a graph's `params:` is core's
`graph::params::resolve_params`. The `inputs(...)`/`outputs(...)` args are rejected at parse
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

`plugin_codegen.rs` also emits the optional checked `cerulion_node_api_v1`
bootstrap with node-local lifecycle, state and external-source tables.
`plugin_impl.rs` adapts the common rewritten init/tick/helper methods to
`graph::node_plugin_abi` ports. Parameter defaults, metadata and trigger policy
share the native emitters; both port directions include checked schema, fixed
size, variable count and alignment facts.

Stable instances hold copied host callbacks and integer port/lease identities.
The host retains transport, clocks, queues and sessions. Local schema writers
publish only after the common success gate arms them. An adaptive spill is an
explicit synchronous payload copy; the original lease stays live until the
writer drops, then receives `SPILL_RELEASE`. Ordinary receive leases stay outside
the callback view until the callback returns, so copied fixed readers remain
valid after an early view drop. Accumulator views borrow the host's frozen fire
batch. State sinks borrow chunks synchronously; accumulator
records use the same canonical encoding as native state.

`src/plugin_codegen.rs` emits the stable tables behind the node crate's `cdylib`
feature. The loader is `DylibNodeEntry` in `graph/node_dylib_abi.rs`.

| Boundary | Role |
|---|---|
| `cerulion_node_api_v1_stabbied_v3` and report | Checked discovery of the complete immutable NodeApiV1 and its frozen Stabby identity. |
| `NodeApiV1` | Create, tick, shutdown, destroy and scoped error-copy callbacks. Metadata carries ports, policy, retention and pre-init inline safety. |
| `StateApiV1` | State shape, capture, restore, inline safety and quiescence probe. Null means the node declares no state table. |
| `ExternalApiV1` | External-source description and history pumping. |
| `HostApiV1` | Host-owned transport loans, terminal releases, spill, state observation, clock and raw publishers. |

The loader ignores compiler provenance and legacy symbols. An older supported
minor loads when it needs only available capabilities; a different boundary
major or complete layout is refused with a rebuild instruction. Native error
kinds and phase labels cross in Status plus scoped error tokens; no Rust owner
or allocator crosses. See [Node ABI](node-abi.md) for the exact contract.

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
`cerulion_core`, and under the umbrella root in `cerulion`. Fixture cdylibs must be
prebuilt (`cargo build -p <fixture>`); each test file's header names its exact prereqs.

| Test | Pins | Serial? | Prereq fixtures |
|---|---|---|---|
| `cerulion_macros` in-src `#[cfg(test)]` (parse.rs, impl_macro.rs, determinism.rs) | Attribute parsing, emitted rewrite shapes (incl. the hoist-and-`?` nested-chain form), banned-symbol table | no | none |
| `crates/cerulion_core/tests/macro_test.rs` | Macro lifecycle, `info()`, state | no | none |
| `crates/cerulion/tests/one_dependency_build_test.rs` | The `::cerulion::core` root: scratch node crates whose only dependency is `cerulion`, built with the `cdylib` feature the scaffold turns on (data-trigger, sync and external nodes; params, accumulate, every state escape); a package naming neither crate gets the named error from all three macros; a field failing a state bound is reported at the field under both roots | no | none (drives `cargo check` on scratch projects; `CERULION_SCRATCH_TARGET_DIR` moves their target dir) |
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
| `macro_params_test.rs` | `#[param]` in process: specs in `info()`, defaults at `new()`, `set_params` values reach `init` and the first tick, unknown name and wrong type refused at `init`, a param written in `init` or `tick` refuses the state capture | no | none |
| `cdylib_params_test.rs` | `#[param]` across the FFI: specs in the info JSON, values through the optional `cerulion_node_init_with_params` symbol, two instances differ, a symbol-less cdylib refuses a handoff, a library that declares params without the symbol is refused when its info is read | yes | `test_node_macro_param_cdylib`, `test_node_cdylib`, `test_node_params_no_symbol_cdylib` |
| `tests/ui/param_*.rs` | The six `#[param]` compile-fail diagnostics (unknown key, wrong-type default, `min` above `max`, default outside bounds, bounds on a `String`, `live` reserved) and `pass/param_fields_pass.rs` | no | none |

Further siblings (`macro_lifecycle_test`, `macro_shim_methods_test`,
`macro_shim_clock_sources_test`, `macro_sync_threading_test`,
`macro_chunks_ef_adversarial_test`, `macro_cdylib_overflow_test`,
`data_trigger_macro_binding_test`, `fill_from_codegen_test`, `fill_from_e2e_test`, and
the `cdylib_*` policy/QoS/event files) each state their serial requirement and fixture
prereqs in their file header; run serial binaries individually with
`-- --test-threads=1`, never the whole workspace at once.

The embedded Python `export_node!` uses the same stable export adapter with
node-local Python context and buffer owners. The native and stable backends
share input validation, touched-output preflight and tick failure cleanup.
Stable frame reads pin host leases, and writable buffer exports retain their
loan until released; dynamic variable frames use the explicit adaptive spill
callback. The host supplies sequence and clock stamps. Python metadata requires
process retention because CPython caches node-local pyclass code and retained
memoryviews may still hold leases after a failed tick. Python nodes advertise
no state or external-source capability and restart from init.
