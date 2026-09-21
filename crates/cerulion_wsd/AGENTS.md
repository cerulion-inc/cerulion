# cerulion_wsd - workspace engine daemon

`cerulion-wsd` exposes `cerulion_cli_engine` verbs to Studio over a private
Unix-socket NDJSON protocol (`protocol.rs`; hello line
`{"hello":"cerulion-wsd","protocol":1}`). Adapt engine functions, never
re-implement engine rules: a staged node's outputs come from
`graph_cmd::stage_declared_node` (declared ports), the same fn the CLI uses.

## Invariants
- Every request type is `deny_unknown_fields`; `VERBS` and the `Request`
  variants are one set (a unit test pins it). Bump `PROTOCOL_VERSION` for any
  change a v1 client could misread; adding a verb is not one.
- Response `id` is `null` only when the line had no parseable id. Codes:
  `bad_request`, `unknown_verb`, `workspace_not_found`, `not_found`,
  `invalid_request` (the engine's own refusal text), `version_conflict`,
  `engine_error` - map through `engine_failure`, never a bare string.
- Mutations hold `WorkspaceLock::acquire_and_track_gitignore` across read-hash
  → engine call → post-hash; reads take `acquire_read`, which never creates the
  lock file and refuses a re-entry from a thread already holding the write lock.
  `expect_version` is a compare-and-swap under that lock.
- The accept loop never exits on an accept error (flood-latched log + pause);
  finished connection tasks are reaped; an over-long line (>
  `MAX_REQUEST_LINE_BYTES`) gets a `bad_request` with `id: null`, then the
  connection closes. `shutdown` aborts in-flight work after
  `CERULION_WSD_HARD_EXIT_MS` (default 5000).
- `hygiene.rs` is a thin binding of `cerulion_hygiene::WSD` (shared with netd
  and vizd): flock singleton before any socket surgery, per-user last rung,
  the socket-directory rule - see `cerulion_hygiene`'s module docs and
  USER_API's socket-path cell (`docs/user-api.md`); do not restate it here - pidfile `O_NOFOLLOW`.
  Change it THERE. (Both this file and `src/hygiene.rs` used to carry a copy
  of the rule, and both still described the pre-fix shape after the shared
  crate changed it; `no_wsd_doc_restates_the_socket_directory_rule` fails if a
  copy comes back.)
- `graph.validate` never loads a node cdylib into the daemon: it goes through
  `graph_cmd::graph_validate_with_inspector` with a `SubprocessInspector` that
  runs `cerulion-wsd --inspect-node <lib>` in its own process group, output
  capped, document = the child's whole stdout (`inspect.rs` moves stdout to a
  close-on-exec fd and dup2s fd 2 over fd 1 before loading; measured: an
  aborting library killed the daemon; a descendant holding the pipe bypassed
  the deadline).
- Library code never prints; `main.rs` installs the stderr tracing
  subscriber (RUST_LOG, default `info`) and owns `--help`.

## Testing
- `cargo test -p cerulion_wsd` (unit + `tests/daemon.rs`, parallel-safe:
  per-test sockets and workspaces under the temp dir).
- `tests/inspect_channel_test.rs` - the `--inspect-node` channel isolation: a
  self-re-exec probe plus FOUR arms that spawn the production binary. The
  `#[ignore]`d `inspect_channel_*_child` arms are the probe's own children
  (env-gated on `CER_WSD_INSPECT_CHANNEL_CHILD`; never run them standalone).
  Two production arms need a real cdylib: they REQUIRE the fixture under the
  test binary's OWN profile (no sibling-profile copy) and PANIC naming
  `cargo build -p test_node_cdylib` without it - a skip is INVISIBLE (libtest
  discards a passing test's stderr), so CI's `crate-tests` builds the fixture
  right before `cargo test -p cerulion_wsd`. The not-a-library arm always
  runs; the `ulimit -n` sweep needs no fixture and PANICS under `CI` (loud
  line on a desk) if no budget reaches the surgery refusal.
