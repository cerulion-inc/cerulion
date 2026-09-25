# cerulion_cli_engine - agent notes

All `cerulion` CLI command logic; `cerulion_cli` is its thin clap binary;
the engine is testable without spawning processes.
Every workspace-file writer (`node create/delete/modify`, `node stage`,
`graph create/partition`, `graph run`'s auto-partition persist,
`ros2 attach`'s consent batch, `ros2 migrate --write`, `schema create/delete`) holds
`workspace_lock::WorkspaceLock` (`<root>/.cerulion/workspace.lock`, a kernel
flock shared with `cerulion-wsd`) across check-and-write; nesting on one
thread is reentrant, others wait (one `warn!`) - UNIX only; elsewhere a no-op.
Reads use `acquire_read` (creates nothing; refuses a same-thread re-entry); all
refuse a symlinked `.cerulion`. ONLY `acquire_and_track_gitignore` writes a
TRACKED file (an EXISTING `.gitignore` on creating `.cerulion/`); migrate takes
`acquire_interruptibly(root, interrupted)`, which polls and returns
`AcquireError` (no `From`). Per-ROOT; `--workspace` is the COLCON root.
Stage a node only via `graph_cmd::stage_declared_node` (declared ports).
## Invariants
- Enforce invariants at the engine boundary; CLI checks are UX.
- `node build`'s PATH rustc probe is advisory: Cargo compiler overrides may differ.
  Only the built cdylib's full fingerprint is authoritative, checked at load.
- `node_metadata::parse_node_metadata` is the sole port/trigger source. Raw-FFI markers optional; only `INFO_END` before `INFO_START` is fatal.
- Never hand-compute pinned schema hashes; run `pinned_hashes` after a recipe change
  and copy its values into `topic_cmd.rs`.
- Completions never hang, spawn, or open the network: `run_bounded` +
  `COMPLETION_BUDGET`; pinned by `completions_test`.
- Port spellings are SIZED before acceptance; unverifiable = `CliError::SchemaUnchecked`,
  never warn-and-accept, payload never self-naming (the consumer frames it). A duplicated
  identity refuses iff ANY bearer is un-carryable (`schema info`'s rule); all-healthy passes
  here and there while the fold refuses the key - an asymmetry, not a licence to escalate.
- Rewrite `node modify` sources with `syn` spans, never `str::replace`.
- iceoryx2 namespace prefixes must be prefix-free: fixed-length hashes and
  `Config::global_config().clone()`, never `Config::default()`.
- Blocking waits check `running` after interrupted ERRORS and retry only within
  `MAX_CONSECUTIVE_WAIT_ERRORS`; never sleep-retry an ERROR (a lock wait POLLS -
  that is not this). FOREIGN children Cerulion SIGINTs go via `child_signals`
  (SIG_IGN/blocks survive exec); self_exe excepted.
- After changing `templates.rs`, regenerate its fixture from the repo root:
  `cargo run -p cerulion_cli_engine --example dump_raw_ffi_emit > crates/test_fixtures/test_node_raw_ffi_template_cdylib/src/lib.rs`.
- `orphan_port_tags::reclaim_orphan_port_tags` removes only `.port_tag` files of a provably-dead
  node from a directory re-listed then, holding nothing else; extend its
  refusals, never its acceptance (`docs/internals/cli.md` §10; pin `clean_orphan_port_tag_test`).
## Workspace dependency contract
Workspace dependencies follow the binary, never cwd: checkout paths or exact registry
pins. See `docs/internals/cli.md` §11 for the full contract and compiler checks.
## Testing
- `cargo test -p cerulion_cli_engine` covers most binaries.
- Run `replay_engine_test`, `graph_profile_iox2_test`, `topic_observer_iox2_test`
  individually with `-- --test-threads=1`; the latter two share iceoryx2's ns.
- Build `test_node_macro_period_cdylib` + `test_node_macro_data_trigger_cdylib`
  before `graph_profile_iox2_test`; `mdns_live_test` is hardware-only, ignored.
## Gotchas
- `proc_macro2::Literal::to_string()` preserves `100_000`, `100u64`, `0xff`; use
  `parse_int_literal` or `syn::LitInt::base10_parse`.
- Numeric verification has false-pass classes (NaN folds, >2^53 widening,
  length-derived fields); see the dossier before adding a metric.
See `docs/internals/cli.md` before changing replay exits, completions, topic listing,
Python nodes (§13), or graph run/profile/partition consent.

