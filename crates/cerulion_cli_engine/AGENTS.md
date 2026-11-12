# cerulion_cli_engine

Clap delegates here; tests never spawn processes. Writers hold `workspace_lock::WorkspaceLock` at `<root>/.cerulion/workspace.lock` through check/write: UNIX flock shared with wsd, reentrant, others wait (one warn); elsewhere no-op.
Reads: `acquire_read` (creates nothing; refuses same-thread re-entry); all
refuse a symlinked `.cerulion`. Only `acquire_and_track_gitignore` edits
tracked `.gitignore` when creating `.cerulion/`; migrate polls with
`acquire_interruptibly(root, interrupted)` returning `AcquireError` (no `From`); per-root; `--workspace` selects COLCON.
Ports: `graph_cmd::stage_declared_node`.
## Rules
- Engine enforces invariants; CLI checks are UX.
- Catalogs bind account/key; resolve collisions. Paired skips implicit LAN; Open/locators/scan keep it; diagnose broken pairs. Refresh<=2s, save<=1s; enroll<=2s once/process, existing keys, no browser. Cache debug. Unread catalogs prove no absence; unpaired account keys need login; warn on failed registration.
- Scope precedes ports: off/inert excludes all, strict non-egress. Drop after ports/workers; recorder I/O absolute; output/status keep spelling.
- Paired graph producers need serving_authority; canonical ingress before GO retains pinned demands; refuse schema/QoS/writer mismatches.
- Observers pin source/hash/channel; busy=unknown. Loss re-demands, graph rechecks identity. Epoch errors retain trust and route latches on reconnect.
- Paired starts are quiet; Open warns for every plan using the daemon's posture.
- `account_robot_access`: only netd opens WAN. Probes 4s/500ms per robot, no pair/demand; catalogs 4s, coalesce keys. LAN/account merge only on robot ID AND key. Auth keeps `/v1/me.account_id`; pairing maps via `account_identity::pairing_account_id`. Login checks mapping/challenge/registration/leaf/key before writing; `device-chain.json` commits LAST.
- `node build`: Cargo's rustc/MSRV, no pin; load checks the frozen boundary, never compiler provenance.
- Android libs: <target>/<triple>/<profile>, then <target>/<profile>; viz refuses before connect/spawn; account TLS: system roots, no JVM.
- Ports/triggers: only `node_metadata::parse_node_metadata`; INFO_END before INFO_START is fatal.
- Pinned hashes: run `pinned_hashes`, copy into `topic_cmd.rs`; never hand-compute.
- Completions: no hangs/spawns/network (`run_bounded`, `completions_test`).
- Size ports before acceptance; unverifiable: `CliError::SchemaUnchecked`.
  Un-carryable duplicate bearers refuse.
- Rewrite `node modify` with `syn` spans, never `str::replace`. Raw INFO has PORT_SCHEMAS names and typed wire facts.
- iceoryx2 namespace prefixes must be prefix-free: fixed-length hashes and
  `Config::global_config().clone()`, never `Config::default()`.
- Interrupted waits check running; retry<=MAX_CONSECUTIVE_WAIT_ERRORS, never sleep-retry errors. Foreign SIGINT: child_signals (exec inherits ignore/blocks), self_exe excepted.
- After `templates.rs` changes, regenerate its fixture at repo root:
  `cargo run -p cerulion_cli_engine --example dump_raw_ffi_emit > crates/test_fixtures/test_node_raw_ffi_template_cdylib/src/lib.rs`.
## Dependencies
Deps follow binary: checkout/exact registry pins.
Checks: cli dossier §11.
## Tests: lib env `test_env::env_lock`
- `ci_test_coverage_test`/`ci_doc_pin_walk_test`: no CI step gated on an
  ungrounded or unmarked (`selection:`) selection, no `# doc-pin:`-less doc
  root; no bash-only construct in a `container:` run step lacking `shell: bash`.
- Serial: `replay_engine_test`, `graph_profile_iox2_test`, `topic_observer_iox2_test`,
  `topic_type_iox2_test` (last 3 share namespace), `account_robot_access_e2e_test` (env);
  `pairing_*_parity_test` JSON oracles match the app byte-for-byte. Build
  `test_node_macro_{period,data_trigger}_cdylib` first.
## Gotchas
- `proc_macro2::Literal::to_string()` keeps `100_000`, `100u64`, `0xff`; use
  Use `parse_int_literal`/`syn::LitInt::base10_parse`.
- Guard NaN, >2^53 widening and lengths.
`docs/internals/cli.md`: topics/scope/starters/replay/completions/Python (§13)/graph run/profile/partition.
