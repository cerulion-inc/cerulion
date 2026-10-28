# cerulion_cli - agent notes

Thin binary crate: clap parsing + dispatch + exit codes only. ALL command logic lives in
`cerulion_cli_engine` - read that crate's AGENTS.md first; behavior changes go there.

## Invariants
- `clap_complete::CompleteEnv::with_factory(families::cli_command).complete()` is the FIRST
  statement of `main()` - it owns stdout for a completion invocation. Nothing may print
  before it, and completion output must stay bare candidates with zero stderr bytes
  (pinned by `tests/completions_cli_test.rs` under a hostile logging env).
- Never gate `Completions` or `Clean` behind `command_needs_identity` - a shell rc file
  and a signed-out desk must meet no auth prompt (`tests/login_gate_e2e_test.rs`).
- Account robot lookup lives in the engine; `--no-network` and completions never fetch it.
  `viz --robot` resolves BEFORE vizd starts. Hidden `bootstrap-robot` dispatches before
  telemetry, logging and the login gate.
- Multi-value flags use exact `num_args = N`, never a range (clap's Append flattens repeats
  into one Vec). Length guards run BEFORE any positional access.
- Optional-value flags (`--record`, `--run`) use `require_equals`: a bare flag takes its
  default and the following words parse as positionals.
- A `String`-typed path arg completes NOTHING without a `value_hint` (`PathBuf` gets
  `AnyPath`). Every new value arg needs a completer/hint or an inventory entry
  (`src/completion_wiring_tests.rs`); for a path the fix is the hint.
- New top-level verb: a `src/families/` module + a `FAMILIES` entry, not a
  `Commands` variant (cli.md §14)
- Create verbs (`node|graph|schema create`) complete nothing.

## Testing
- Wiring tests are binary-crate unit tests (no lib target): `cargo test -p cerulion_cli --bin cerulion`.
  They move cwd + `HOME` under a file-local mutex declared as the fixture's LAST field (Rust
  drops fields in declaration order).
- Never run a bare `cerulion clean` from a test: the `/tmp/*.shm_state` reclaim is machine wide by
  construction (compile-time directory, no `TMPDIR`), so arms here pass `--report-only` only and the
  destructive proof lives in `cerulion_cli_engine`'s `sweep_dead_nodes_test.rs`
  (`tests/trace_inspect_and_clean_cli_test.rs`).
- `account_offline_dispatch_test` (alone): offline listing and hostile-env completion make
  no account request and no netd spawn, against positive-control sentinels.
- E2E binaries drive the REAL binary; run each `#[serial]` one alone with
  `-- --test-threads=1`. Build fixtures first:
  - `replay_cli_test`: `cargo build -p test_node_macro_period_cdylib -p
    test_node_macro_period_perturbed_cdylib -p test_node_macro_period_panic_cdylib -p test_node_nondeterministic_cdylib`
  - `mp_record`, `mp_execution_mode`, `network_gateway{,_mp}` e2e:
    `cargo build -p test_node_macro_period_cdylib -p test_node_macro_data_trigger_cdylib`;
    `mp_auto_partition`: those two + `..._trigger_block_cdylib`
  - `signal_matrix`: `period` only. `credit_death`: + `..._trigger_block_cdylib`.
    `mp_split_pair`: `period` + `period_input`. `mp_consumer_first_spawn`: those two + `data_trigger`.
    `graph_params_e2e`: `period` + `param`. `graph_launch_args*`: `param` + `data_trigger`.

## Gotchas
- Signal tests assert exit code EXACTLY 0 (a signal kill is `code()==None`). Gateway-teardown
  tests also need the gateway's shutdown log line: a reap alone passes kill-on-drop too.
- `mp_support::ChildGuard` has NO public field: `spawn_group_leader` if the child owns a
  subtree (workers, gateway, ros2 entry), else `single_process`; reap/poll via its METHODS
  (`wait_bounded`, `try_wait_noting`) so it notes live workers first, and it never caches an
  EMPTY note, so a poll at t=0 cannot freeze the verdict. `Drop` only REPORTS a leak; arms
  that ASSERT call `#[must_use]` `finish().assert_clean()`. A walk enforces constructor shape
  and flags bare reaps it can see; await GO (`deployment live`) before a kill, else ABORT path.

Reference: docs/internals/cli.md (contracts, completion, verb families, test maps).
