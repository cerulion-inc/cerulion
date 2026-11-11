# cerulion_cli_engine

Parser: `cerulion_cli`. Writers hold `workspace_lock::WorkspaceLock` at `<root>/.cerulion/workspace.lock` through check/write: UNIX flock shared with wsd, reentrant, others wait (one warn); elsewhere no-op.
Reads use `acquire_read` (creates nothing; refuses same-thread re-entry); all
refuse a symlinked `.cerulion`. Only `acquire_and_track_gitignore` edits
a tracked `.gitignore` when creating `.cerulion/`; migrate polls with
`acquire_interruptibly(root, interrupted)` returning `AcquireError` (no `From`); per-root; `--workspace` selects COLCON.
Stage ports via `graph_cmd::stage_declared_node`.
## Invariants
- Account/pair catalogs bind identity; selectors resolve collisions. Paired skips implicit LAN; Open/locators/scan retain it; broken pairs diagnose. Refresh/enroll <=2s; enroll once/process, existing keys, no browser. Cache debug. Unread catalogs prove no absence; unpaired account keys need login; run warns on registration failure.
- Scope precedes ports: off/inert excludes all, strict non-egress. Drop after ports/workers; recorder I/O absolute; output/status keep spelling.
- Paired graph producers need serving_authority; canonical ingress before GO retains pinned demands; refuse schema/QoS/writer mismatches.
- Observers pin source/hash/channel; probes use demand-free clients and busy is unknown. Loss re-demands; graph inputs revalidate identity.
- Paired starts are quiet; Open warns for every plan using the daemon's posture.
- `account_robot_access`: only netd opens WAN. Probes: 4 s (500 ms/robot), never pair/demand. Catalogs: another 4 s; coalesce by endpoint key. LAN/account rows merge on robot ID AND key. Auth keeps `/v1/me.account_id`; pairing maps it
  (`account_identity::pairing_account_id`). Login checks mapping, challenge,
  registration, leaf and key before writing, and commits `device-chain.json` LAST.
- node build rustc advises; load-time cdylib fingerprint counts.
- Android libs: <target>/<triple>/<profile>, then <target>/<profile>; viz refuses before connect/spawn (docs/android.md).
- Ports/triggers: only `node_metadata::parse_node_metadata`; INFO_END before INFO_START is fatal.
- Pinned hashes: run `pinned_hashes`, copy into `topic_cmd.rs`; never hand-compute.
- Completions never hang/spawn/network (`run_bounded`, `completions_test`).
- Size port spellings before acceptance; unverifiable = `CliError::SchemaUnchecked`.
  Any un-carryable duplicate bearer refuses, never warn-and-accept.
- Rewrite `node modify` sources with `syn` spans, never `str::replace`.
- iceoryx2 namespace prefixes must be prefix-free: fixed-length hashes and
  `Config::global_config().clone()`, never `Config::default()`.
- Interrupted waits check running; retry <= MAX_CONSECUTIVE_WAIT_ERRORS, never sleep-retry errors. SIGINT foreign children via child_signals (exec inherits ignore/blocks), self_exe excepted.
- After changing `templates.rs`, regenerate its fixture from the repo root:
  `cargo run -p cerulion_cli_engine --example dump_raw_ffi_emit > crates/test_fixtures/test_node_raw_ffi_template_cdylib/src/lib.rs`.
## Dependencies
Dependencies follow binary, never cwd: checkout paths or exact registry pins.
Checks: `docs/internals/cli.md` §11.
## Testing
- `ci_test_coverage_test`/`ci_doc_pin_walk_test`: no CI step gated on an
  ungrounded or unmarked (`selection:`) selection, no `# doc-pin:`-less doc
  root; no bash-only construct in a `container:` run step lacking `shell: bash`.
- Serial: `replay_engine_test`, `graph_profile_iox2_test`, `topic_observer_iox2_test`,
  `topic_type_iox2_test` (last 3 share iox2 ns), `account_robot_access_e2e_test` (env);
  the three `pairing_*_parity_test` JSON oracles stay byte-identical to the app's. Build
  `test_node_macro_{period,data_trigger}_cdylib` first.
## Gotchas
- `proc_macro2::Literal::to_string()` keeps `100_000`, `100u64`, `0xff`; use
  `parse_int_literal` or `syn::LitInt::base10_parse`.
- Guard NaN folds, >2^53 widening and length fields.
`docs/internals/cli.md`: topics/local scope/starters,
replay, completions, Python nodes (§13), or graph run/profile/partition.
