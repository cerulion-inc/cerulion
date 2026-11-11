# cerulion_cli_engine - agent notes

CLI logic; `cerulion_cli` parses. Tests need no children.
Workspace writers (`node create/delete/modify`, `node stage`,
`graph create/partition`, `graph run`'s auto-partition persist,
`ros2 attach`'s consent batch, `ros2 migrate --write`, `schema create/delete`) holds
`workspace_lock::WorkspaceLock` (`<root>/.cerulion/workspace.lock`, a kernel
flock shared with `cerulion-wsd`) across check/write; same-thread reentrant,
others wait (one `warn!`); UNIX only, elsewhere no-op.
Reads use `acquire_read` (creates nothing; refuses a same-thread re-entry); all
refuse a symlinked `.cerulion`. ONLY `acquire_and_track_gitignore` writes a
TRACKED existing `.gitignore` when creating `.cerulion/`; migrate polls with
`acquire_interruptibly(root, interrupted)` returning `AcquireError` (no `From`). Per-ROOT; `--workspace` is the COLCON root.
Stage nodes only via `graph_cmd::stage_declared_node` (declared ports).
## Invariants
- Graph/node guards precede services/workers, including recording: off/inert excludes every topic, strict excludes outputs outside egress. Drop guards after all ports/workers.
- Engine enforces invariants; CLI checks are UX.
- Account listing/viewer selection: `account_robot_access`; only netd opens WAN. Probes
  share 4 s (500 ms/robot); never pair or demand. LAN and account rows merge only on
  robot ID AND key. Auth keeps `/v1/me.account_id`; pairing maps it
  (`account_identity::pairing_account_id`). Login checks mapping, challenge,
  registration, leaf and key before writing, and commits `device-chain.json` LAST.
- `node build`'s rustc probe is advisory; only the cdylib fingerprint, checked at load, counts.
- Android: cdylib lookup prefers `<target>/<triple>/<profile>`, then `<target>/<profile>`;
  `cerulion viz` refuses before connecting or spawning (`docs/android.md`).
- `node_metadata::parse_node_metadata` is the sole port/trigger source; only `INFO_END`
  before `INFO_START` is fatal.
- Never hand-compute pinned schema hashes: run `pinned_hashes`, copy into `topic_cmd.rs`.
- Completions never hang, spawn, or open the network (`run_bounded`, `completions_test`).
- Size port spellings before acceptance; unverifiable = `CliError::SchemaUnchecked`.
  ANY un-carryable duplicate bearer refuses the identity, never warn-and-accept.
- Rewrite `node modify` sources with `syn` spans, never `str::replace`.
- iceoryx2 namespace prefixes must be prefix-free: fixed-length hashes and
  `Config::global_config().clone()`, never `Config::default()`.
- Waits check `running` after interrupted errors; retry <= `MAX_CONSECUTIVE_WAIT_ERRORS`, never sleep-retry errors. FOREIGN children Cerulion SIGINTs use `child_signals` (SIG_IGN/blocks survive exec); self_exe excepted.
- After changing `templates.rs`, regenerate its fixture from the repo root:
  `cargo run -p cerulion_cli_engine --example dump_raw_ffi_emit > crates/test_fixtures/test_node_raw_ffi_template_cdylib/src/lib.rs`.
## Workspace dependencies
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
