# cerulion_cli_engine

Parser: `cerulion_cli`. Workspace writers (node/graph/schema edits,
partition and ROS 2 writes) hold
`workspace_lock::WorkspaceLock` (`<root>/.cerulion/workspace.lock`, kernel flock shared with wsd) across check/write; same-thread reentrant, others wait (one warn); UNIX only, elsewhere no-op.
Reads use `acquire_read` (creates nothing; refuses a same-thread re-entry); all
refuse a symlinked `.cerulion`. ONLY `acquire_and_track_gitignore` writes a
TRACKED existing `.gitignore` when creating `.cerulion/`; migrate polls with
`acquire_interruptibly(root, interrupted)` returning `AcquireError` (no `From`). Per-ROOT; `--workspace` is the COLCON root.
Stage via `graph_cmd::stage_declared_node` (declared ports).
## Invariants
- Bounded account/pair catalogs; selectors resolve collisions; caches fence identity. Paired skips implicit LAN; Open/locators/scan retain it; broken pairings report diagnostics.
- Graph/node scope precedes services/workers, also recording: off/inert excludes all topics, strict outputs outside egress. Drop guards after ports/workers.
- Paired graphs gate producers with serving_authority; resolve canonical ingress before GO, retain pinned demands, refuse schema/QoS/writer mismatches.
- Engine enforces invariants; CLI aids users.
- Paired starts are quiet; Open warns for every plan using the daemon's posture.
- `account_robot_access`: only netd opens WAN. Probes share 4 s (500 ms/robot), never pair/demand. Automatic catalogs share another 4 s deadline, coalesce by endpoint key. LAN/account rows merge on robot ID AND key. Auth keeps `/v1/me.account_id`; pairing maps it
  (`account_identity::pairing_account_id`). Login checks mapping, challenge,
  registration, leaf and key before writing, and commits `device-chain.json` LAST.
- node build rustc probe is advisory; only load-time cdylib fingerprint counts.
- Android cdylibs prefer <target>/<triple>/<profile>, then <target>/<profile>; viz refuses before connect/spawn (docs/android.md).
- Only `node_metadata::parse_node_metadata` supplies ports/triggers; INFO_END before INFO_START is fatal.
- Never hand-compute pinned schema hashes: run `pinned_hashes`, copy into `topic_cmd.rs`.
- Completions never hang/spawn/network (`run_bounded`, `completions_test`).
- Size port spellings before acceptance; unverifiable = `CliError::SchemaUnchecked`.
  ANY un-carryable duplicate bearer refuses the identity, never warn-and-accept.
- Rewrite `node modify` sources with `syn` spans, never `str::replace`.
- iceoryx2 namespace prefixes must be prefix-free: fixed-length hashes and
  `Config::global_config().clone()`, never `Config::default()`.
- Interrupted waits check running; retry <= MAX_CONSECUTIVE_WAIT_ERRORS, never sleep-retry errors. SIGINT foreign children via child_signals (exec inherits ignore/blocks), self_exe excepted.
- After changing `templates.rs`, regenerate its fixture from the repo root:
  `cargo run -p cerulion_cli_engine --example dump_raw_ffi_emit > crates/test_fixtures/test_node_raw_ffi_template_cdylib/src/lib.rs`.
## Workspace dependencies
Dependencies follow the binary, never cwd: checkout paths or exact registry pins.
Contract/compiler checks: `docs/internals/cli.md` §11.
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
- Numeric false-pass classes: NaN folds, >2^53 widening, length fields.
Read `docs/internals/cli.md` before changing topics/local scope, starters,
replay, completions, Python nodes (§13), or graph run/profile/partition.
