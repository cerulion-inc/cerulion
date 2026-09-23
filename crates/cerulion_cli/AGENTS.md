# cerulion_cli - agent notes

Thin binary crate: clap parsing + dispatch + exit codes only. ALL command logic lives in
`cerulion_cli_engine` - read that crate's AGENTS.md first; behavior changes go there.

## Invariants

- Account robot lookup stays in the engine. `topic list --no-network` and
  completions never fetch account data. Resolve `viz --robot` before starting
  vizd, retain its daemon hold through command dispatch, and show lookup failures.

- `clap_complete::CompleteEnv::with_factory(Cli::command).complete()` is the FIRST
  statement of `main()` - it owns stdout for a completion invocation. Nothing may print
  before it, and completion output must stay bare candidates with zero stderr bytes
  (pinned by `tests/completions_cli_test.rs` under a hostile logging env).
- Keep `Completions` excluded from `command_needs_identity` - `cerulion completions zsh`
  runs from shell rc files and must never block shell startup on an auth prompt.
- Multi-value flags use exact `num_args = N`, never a range - clap's default Append
  action flattens repeated invocations into one Vec, making two partial invocations
  indistinguishable from one full one. Length guards run BEFORE any positional access.
- Optional-value flags (`--record`, `--run`) use `require_equals` - load-bearing, not
  style: a bare flag takes its default and the following words parse as positionals.
- Path completion: clap auto-derives `ValueHint::AnyPath` for `PathBuf`-typed args; a
  `String`-typed path arg completes NOTHING until given an explicit `value_hint`. Every
  new value-taking arg must get a completer/hint or be classified in the free-form
  inventory - `src/completion_wiring_tests.rs` fails until you do, and the fix for a
  path arg is a `value_hint`, never an inventory entry.
- Create verbs (`node|graph|schema create`) deliberately complete nothing - the existing
  name set is exactly what a create verb rejects.

## Testing

- Wiring tests are binary-crate unit tests (no lib target):
  `cargo test -p cerulion_cli --bin cerulion`. They move the process cwd + `HOME` under
  a file-local mutex declared as the fixture's LAST field - Rust drops fields in
  declaration order, so a mutex declared first releases before the env guards restore.
- `account_offline_dispatch_test` exercises the real binary with positive-control
  HTTP/spawn sentinels. Run it alone with `-- --test-threads=1`; offline topic
  listing and hostile-env robot completion must make no account request or
  netd spawn. Its project-local iceoryx config isolates both root and prefix.
- The e2e binaries drive the REAL binary; run each `#[serial]` one individually with
  `-- --test-threads=1`. Build fixtures first:
  - `replay_cli_test`: `cargo build -p test_node_macro_period_cdylib
    -p test_node_macro_period_perturbed_cdylib -p test_node_macro_period_panic_cdylib
    -p test_node_nondeterministic_cdylib`
  - `mp_record_e2e_test`, `mp_auto_partition_e2e_test`, `network_gateway{,_mp}_e2e_test`:
    `cargo build -p test_node_macro_period_cdylib -p test_node_macro_data_trigger_cdylib`
  - `signal_matrix_e2e_test`: `cargo build -p test_node_macro_period_cdylib`
  - `credit_death_e2e_test`: + `..._trigger_block_cdylib`. `mp_split_pair`:
    `period` + `period_input`. `mp_consumer_first_spawn`: those two + `data_trigger`

## Gotchas

- Signal tests assert exit code EXACTLY 0: a signal-killed process reports
  `code()==None`, so the integer 0 is the proof the handler drove a graceful shutdown
  rather than a default-disposition kill. Don't loosen the discriminator.
- Gateway-teardown tests require the gateway's own shutdown log line, not just
  exit 0 + child reaped - reap alone also passes a kill-on-drop teardown.
- `mp_support::ChildGuard` has NO public field: `spawn_group_leader` if the child owns a
  subtree (workers, gateway, ros2 entry), else `single_process`; reap/poll via its METHODS
  (`wait_bounded`, `try_wait_noting`) so it notes live workers first, and it never caches an
  EMPTY note, so a poll at t=0 cannot freeze the verdict. `Drop` only REPORTS a leak; arms
  that ASSERT call `#[must_use]` `finish().assert_clean()`. A walk enforces constructor shape
  and flags bare reaps it can see; await GO (`deployment live`) before a kill, else ABORT path.

Deep reference: docs/internals/cli.md - command contracts (replay exit codes,
completions design rules, discovery ladder, graph run/profile/partition) and the test
map for both crates.
