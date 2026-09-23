# cerulion_cli_engine - agent notes

All `cerulion` command logic; `cerulion_cli` is its thin clap binary, and the
engine is testable without spawning processes.
EVERY workspace-file writer holds `workspace_lock::WorkspaceLock` (a flock shared
with `cerulion-wsd`) across check-and-write: reentrant on one thread, others wait
with one `warn!`, UNIX only. Reads use `acquire_read`; all refuse a symlinked
`.cerulion`. ONLY `acquire_and_track_gitignore` writes a TRACKED file, migrate takes
`acquire_interruptibly`, and `--workspace` is the COLCON root. Stage a node only via
`graph_cmd::stage_declared_node` (`docs/internals/cli.md`).
## Invariants
- Account listing and viewer selection live in `account_robot_access`; only netd
  opens WAN endpoints. Probes share a 4 s budget, 500 ms per robot, never pair or
  demand, and a row is not presence. LAN and account rows stay distinct until robot
  ID AND key match; a chosen account target never retries as LAN.
- Network-serving runs and non-dry-run ROS attach need a persisted prior login,
  expired tokens included; network-off and inert clocks are exempt. The reader lives
  in `cerulion_netd::serving_login` and never prompts or refreshes at startup.
- Auth keeps `/v1/me.account_id`; pairing maps it through
  `account_identity::pairing_account_id`, and an opaque label is never a pairing ID.
  Cert login checks mapping, challenge, registration, leaf and key before writing.
- Login stages `device-chain.json` and commits it LAST under the auth-store lock;
  `owner_certificate::load` wants its leaf to match `device.cert`, the login account
  and `desk.key`, so a partial login cannot reuse an old chain.
- `node build`'s rustc probe is advisory: only the cdylib's full fingerprint is
  authoritative, checked at load.
- `node_metadata::parse_node_metadata` is the sole port/trigger source; raw-FFI
  markers are optional, only `INFO_END` before `INFO_START` is fatal.
- Never hand-compute pinned schema hashes: run `pinned_hashes` after a recipe bump
  and copy its values into `topic_cmd.rs`. Completions never hang, spawn or open a
  socket (`run_bounded` + `COMPLETION_BUDGET`, `completions_test`).
- Port spellings are SIZED before acceptance; unverifiable = `SchemaUnchecked`, never
  warn-and-accept. A duplicate identity refuses iff ANY bearer is un-carryable.
- Rewrite `node modify` sources with `syn` spans, never `str::replace`. iceoryx2
  prefixes must be prefix-free: fixed-length hashes and
  `Config::global_config().clone()`, never `Config::default()`.
- Blocking waits check `running` after interrupted ERRORS, retry only within
  `MAX_CONSECUTIVE_WAIT_ERRORS`, never sleep-retry one. FOREIGN children get SIGINT
  via `child_signals` (SIG_IGN survives exec).
- `orphan_port_tags::reclaim_orphan_port_tags` removes only `.port_tag` files of a
  provably-dead node, from a directory re-listed then: extend its refusals, never its
  acceptance. Workspace deps follow the binary, never cwd (`cli.md` §10, §11). After
  editing `templates.rs`, regenerate its fixture with the `dump_raw_ffi_emit` example.
## Testing
- `cargo test -p cerulion_cli_engine` covers most binaries. Run `replay_engine_test`,
  `graph_profile_iox2_test`, `topic_observer_iox2_test` and `account_robot_access_e2e_test`
  alone with `-- --test-threads=1`: two share iceoryx2's ns, one mutates env.
- Build `test_node_macro_period_cdylib` + `test_node_macro_data_trigger_cdylib` first;
  `mdns_live_test` is hardware-only, ignored.
- The pairing, chain and grants JSON oracles stay byte-identical to the app fixtures:
  run the three `pairing_*_parity_test` binaries alone, never generating expected
  bytes from the code under test. `owner_certificate::tests` pins the flock interop
## Gotchas
- `proc_macro2::Literal::to_string()` preserves `100_000`, `100u64`, `0xff`: use
  `parse_int_literal` or `syn::LitInt::base10_parse`. Numeric checks have false-pass
  classes (NaN folds, >2^53 widening, length-derived fields).
Read `docs/internals/cli.md` before changing replay exits, completions, topic
listing, or graph run/profile/partition consent.
