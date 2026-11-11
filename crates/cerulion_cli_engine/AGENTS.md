# cerulion_cli_engine - agent notes

CLI logic; `cerulion_cli` is the thin clap binary.
Workspace writers (`node create/delete/modify/stage`, graph create/partition/run
partition persistence, ros2 attach/migrate --write, schema create/delete) hold
`workspace_lock::WorkspaceLock` at `<root>/.cerulion/workspace.lock` across changes:
UNIX flock shared with wsd, thread reentrant, others wait with one warn;
elsewhere a no-op. Reads use `acquire_read`: create nothing, refuse re-entry.
All refuse symlinked `.cerulion`. Only `acquire_and_track_gitignore` edits a tracked
file, an existing `.gitignore` when creating `.cerulion/`. Migration uses polling
`acquire_interruptibly(root, interrupted)` returning `AcquireError` (no From).
Locks are per root; --workspace is the COLCON root. Stage declared ports via
`graph_cmd::stage_declared_node`. Paired skips implicit LAN catalogs; Open,
locators and scan retain them. Broken pairings are diagnostics. Observer channels
are private per source; duplicate sources need selectors; caches bind account/key.

## Invariants
- Enforce invariants at the engine boundary; CLI checks are UX.
- `account_robot_access`: only netd opens WAN. Probes share 4 s, 500 ms per robot,
  never pair/demand. Automatic catalogs share another 4 s; owned/paired coalesce by
  endpoint key. LAN/account merge requires robot ID AND key. Auth keeps
  `/v1/me.account_id`; pairing maps via `account_identity::pairing_account_id`.
  Login validates mapping/challenge/registration/leaf/key before writes;
  `device-chain.json` commits LAST.
- `node build`'s rustc probe is advisory; the cdylib fingerprint, checked at load, counts.
- Android: cdylib lookup prefers `<target>/<triple>/<profile>`, then `<target>/<profile>`;
  `cerulion viz` refuses before connecting or spawning (`docs/android.md`).
- `node_metadata::parse_node_metadata` is the port/trigger authority; `INFO_END`
  before `INFO_START` is fatal.
- Never hand-compute pinned schema hashes: run `pinned_hashes`, copy into `topic_cmd.rs`.
- Completions never hang, spawn, or open the network (`run_bounded`, `completions_test`).
- Port spellings are SIZED before acceptance; unverifiable = `CliError::SchemaUnchecked`,
  never warn-and-accept. A duplicated identity refuses iff ANY bearer is un-carryable.
- Rewrite `node modify` sources with `syn` spans, never `str::replace`.
- iceoryx2 namespace prefixes must be prefix-free: fixed-length hashes and
  `Config::global_config().clone()`, never `Config::default()`.
- Blocking waits check `running` after interrupted ERRORS and retry only within
  `MAX_CONSECUTIVE_WAIT_ERRORS`; never sleep-retry an ERROR. FOREIGN children Cerulion SIGINTs go via `child_signals`
  (SIG_IGN/blocks survive exec); self_exe excepted.
- After changing `templates.rs`, regenerate its fixture from the repo root:
  `cargo run -p cerulion_cli_engine --example dump_raw_ffi_emit > crates/test_fixtures/test_node_raw_ffi_template_cdylib/src/lib.rs`.
## Workspace dependency contract
Workspace dependencies follow the binary, never cwd: checkout paths or exact registry
pins. See `docs/internals/cli.md` §11 for the full contract and compiler checks.
## Testing
- `ci_test_coverage_test`/`ci_doc_pin_walk_test`: no CI step gated on an
  ungrounded or unmarked (`selection:`) selection, no `# doc-pin:`-less doc
  root; no bash-only construct in a `container:` run step lacking `shell: bash`.
- Serial: `replay_engine_test`, `graph_profile_iox2_test`, `topic_observer_iox2_test`,
  `topic_type_iox2_test` (last 3 share iox2 ns), `account_robot_access_e2e_test` (env);
  the three `pairing_*_parity_test` JSON oracles stay byte-identical to the app's. Build
  `test_node_macro_{period,data_trigger}_cdylib` first.
## Gotchas
- `proc_macro2::Literal::to_string()` preserves `100_000`, `100u64`, `0xff`; use
  `parse_int_literal` or `syn::LitInt::base10_parse`.
- Numeric checks have false-pass classes (NaN folds, >2^53 widening, length fields).
Read `docs/internals/cli.md` before changing topics/local scope, starters,
replay, completions, Python nodes (§13), or graph run/profile/partition.
