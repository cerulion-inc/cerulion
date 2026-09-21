# Cerulion - agent guide

Cerulion is a zero-copy, deterministic communication framework for real-time robotics:
iceoryx2 shared memory locally, zenoh across machines, ROS 2 interop via `rmw_cerulion`.
A Rust workspace (authoritative crate list: root `Cargo.toml`). The user-facing API - the
`#[cerulion_node]` macro, the `cerulion` CLI, graph/schema YAML (`docs/user-api.md`) - is a contract.
Users build a WORKSPACE: ONE node type per `nodes/<type>/` crate, wiring ONLY in `graphs/*.yaml`,
run by `cerulion` verbs. Examples, docs, rustdoc, README and the scaffold must show nothing else:
multi-node files, `main`, in-code graphs, runtime-API construction (`GraphRuntime`) are for tests ONLY.

## Critical invariants (bugs, not style - each has an enforcing gate)
Code comments cite these as "Principle #N" (`docs/internals/core-scheduler-graph.md`).

- **Zero-copy hot path**: no heap allocation on publish/receive paths. Enforced by
  `zero_alloc_test`, `zero_copy_hot_path_test` and `./tools/scripts/check_hot_path_allocs.sh`;
  a justified cold-path alloc needs a `// hot-path-alloc-ok: <reason>` line.
- **Replay = Live**: re-executing a recording is byte-identical to the live run. Never add
  wall-clock reads, hash-order iteration or randomness to execution paths - use
  `VirtualClock` and `IndexMap`. Enforced by `replay_test` + the resim CLI's exit gates.
- **Data is truth / observable state**: no callback encodes meaning; state is observable
  independently of execution (counters and wire stamps, not log text).
- **One session per process**: ONE iceoryx2 node, ONE zenoh session. Graph topics are
  single-writer unless listed in `multi_publisher_topics`.
- **Bags are MCAP**: recording bags are standard `.mcap` (`cerulion_bag`), not JSONL; the
  only JSONL artifact is the legacy publish trace (`cerulion trace inspect`).
- **No fake data** in tests, benchmarks or reports - ever. Tests assert against
  hand-written oracle vectors, never a run against itself.
- **NEVER run `cargo test --workspace`** - ~100 test programs at once, and the shared-memory
  suites can deadlock (`#[serial]` protects only within one program). Every CI test step names
  its packages. `cargo clippy/build --workspace` are fine: no tests run.

## Build, lint, test

```bash
cargo build  # default members only
cargo fmt --all  # CI fails on unformatted code, in every workspace
cargo clippy --workspace --all-targets -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps  # docs gate
```

Testing ladder (long builds are normal - don't kill them):

1. Per-crate, parallel-safe: `cargo test -p <crate>`. `cerulion_core` is
   SHARDED in CI - one leg: `./tools/scripts/ci_test_shard.sh cerulion_core <n> 4`.
2. Shared-memory suites: run each named binary individually with `-- --test-threads=1`;
   the per-crate `AGENTS.md` lists which ones are serial.
3. Fixture tests `dlopen` prebuilt cdylibs - build them first; each header names its prereqs.
4. Release-only latency gates (main CI, not PRs): the `--release` `latency_threshold_test`.
5. Hardware-in-the-loop (robots, lab boxes): never in agent sessions.

Lints live ONCE: `[workspace.lints]` in the root `Cargo.toml`, inherited via
`[lints] workspace = true` (a manifest gate names any member that does not; an extra lint goes in
the crate's own source, never a forked table). `dead_code`/`unused_imports`/`unused_variables` =
**deny** - delete dead code at once. CI's stable is newer than most local ones: write the form the
strictest clippy accepts (`if let` over `is_some()`+`unwrap()`). Detail: `docs/internals/ci-and-gates.md`.

## Conventions

- Logging: `tracing` with structured fields (`topic = %name`); levels error=unrecoverable,
  warn=recoverable, info=lifecycle, debug=operational, trace=hot path. Standard fields:
  `topic`, `schema`, `node_id`, `graph`, `size_bytes`, `seq`, `latency_us`, `error`,
  `total_failures`. GATED, not advice: library crates deny `clippy::print_stdout`/`print_stderr`
  outside tests (escape = a targeted `#[allow]` + reason), `clippy.toml` bans `dbg!` and
  `process::exit`, and a walk fails any message interpolating a runtime value.
- High-frequency failure paths flood-suppress (loud first-of-regime, counted repeats,
  recovery line): reuse `FailureRegimeLatch`, never a new hand-rolled one.
- Errors: one `thiserror` enum per crate; propagate with `?`; exit codes only at the CLI.
- Tests: descriptive behavior names; a fire-count proves scheduling, not delivery - strong pins
  assert downstream DELIVERY. Five categories per new path: happy, edge, adversarial,
  determinism (two runs bit-identical), every error arm.
- Mutation checks: mutate PURE decision functions only (never live syscall/transport paths),
  against a COMMITTED baseline; each variant must fail a test.
- Some docs are pinned by oracle tests (tutorial YAML, completion hints): a doc edit that
  fails a test is the gate working - update both together. A regression test per finding.
- Prefer compile-time prevention (types, exhaustive matches, `compile_error!`) over runtime
  checks; a loud `warn!` at the inference site over a silent "smart" default; killing a
  misleading name over documenting one.

## Git & PRs

- Branches: `<type>/<kebab-slug>`, `type` from the PR-title set (`feat` `fix` `docs` `test`
  `refactor` `ci` `chore` `perf` `style` `build`); target `main`. NO personal prefix, NO tracker
  id in the name (it goes in the PR body); gated by `tools/scripts/check_pr_title.sh`.
- PR titles: `<type>(<scope>)!: <description>`. PRs are SQUASH-merged, so the title becomes
  the commit message. Commit bodies carry the WHY.
- PR bodies are self-contained: summary, what changed, how to test (copy-pasteable
  commands), actual test output. Keep diffs reviewable (~800 lines; split above).
  Self-review first; call out breaking changes with migration steps.
- Stacked PRs (B depends on unmerged A): base B on A's branch; merge in dependency
  order. GitHub retargets dependents when a merged base is deleted; a rename closes them.
- Plan gate: read the issue in full, check dependencies (an unlanded one => confirm the
  base with the user), produce the plan (files, tests, risks, chunks) + its questions, get
  the go-ahead. Never skip it.
- Chunked implementation: 3-5 logical chunks, one commit each, nothing unrelated bundled;
  gates after EVERY chunk (fmt, clippy `-D warnings`, affected tests incl. serial): a kill
  or rollback then loses one chunk, not the branch.
- Final gate before merge: affected parallel + serial tests green; fmt + clippy clean; local
  review clean of HIGH/MEDIUM-actionable; bench within ~10% of baseline (a regression = stop);
  log + memory updated. MERGE RULE for the EXTERNAL read (the local two-pass review runs before
  every push and fixes HIGH/MEDIUM there): ONE read per head; SEVERE findings (bug, correctness,
  data loss, security, broken contract) fixed in-PR in ONE swept push, the rest FILED as a
  separate issue (one per PR); merge on CI green once every thread is fixed, refuted with
  evidence, or filed - never wait for an empty read; no PR absorbs another's work.
- Review discipline: (1) after every major chunk run the two-pass review - pass 1 (correctness,
  silent failures, type design, test coverage) finds; fix HIGH + MEDIUM-actionable; pass 2
  (comment accuracy in place of type design) validates the fixes and catches regressions they
  introduced - never skip it. (2) Push for the external bots only when the gates AND pass 2 are
  clean. (3) A bot finding is a CLASS, not a line: before pushing, sweep the WHOLE diff for
  every sibling and fix them all in ONE push. (4) Bots review the latest commit only: after a
  substantive push confirm a fresh review ran; read ALL of it, including reviews you did not
  trigger: inline threads (`gh api --paginate repos/<owner>/<repo>/pulls/<PR#>/comments`) and
  `gh pr view <PR#> --comments`.
- A behavior change updates its docs (`docs/user-api.md`, `docs/`, the affected `AGENTS.md`)
  in the same PR. Never commit, push, force-push or run destructive git unasked.

## Working agreements (mandatory for every agent here)

- Findings surfaced during active work: SEVERE fixed in the CURRENT PR, the rest FILED
  (merge rule above); decide by severity, never offer "now vs. later". DROPPING a finding
  needs maintainer buy-in. Ledgers burn DOWN, never up. No drive-by or review-driven growth.
- Be terse; never collapse the rules that produce a default into the value they produce.
- End substantial turns with two tables - Decisions
  (`# | change beyond the ask | why | risk & undo | alternatives considered`) and Deferrals
  (`# | item | where it lives | why deferred | cost`); write "none serious" rather than
  padding. "Substantial" = >=2 substantive changes, >=1 commit, or edits beyond a one-line
  tweak; skip trivia (fmt, lint fixes in your own new code). The bar: would it surprise a
  reviewer reading the diff? Put them in the reply, never only in a log/PR.
- The user-facing surface (macro attributes, CLI flags, YAML keys, error text, defaults) is
  the contract: changes need explicit maintainer approval, and every special case in a
  defaulting rule is a future semantic-flip bug - prefer unification.
- The bar for `unsafe` is high; modest perf wins don't clear it.

## Workspace map (* = has a scoped `AGENTS.md`)

| Area | Crates |
|---|---|
| Core runtime (wire, transport, scheduler, graph, codegen) | `cerulion_core`* |
| Node-author macros | `cerulion_macros`* |
| Generated ROS 2 message types | `native_ros2_messages`* |
| CLI binary / logic / TUI | `cerulion_cli`*, `_cli_engine`*, `_cli_tui` |
| Recording: MCAP writer/reader + daemon | `cerulion_bag`*, `cerulion_bagd`* |
| ROS 2 RMW layer (via `cerulion ros2 run/launch`) | `rmw_cerulion`* |
| Network & discovery daemons | `cerulion_netd`*, `_dds`*, `_discovery`, `_mdns` |
| Local workspace engine daemon | `cerulion_wsd`* |
| Remote access / pairing / accounts | `cerulion_{connectd,remoted*,pairing,accountd,wire,wireclient,link}`, `cerud` |
| Visualization (not in default-members) | `cerulion_viz`* (`lib/*`, `bin/cerulion_vizd`) |
| Fixtures, benches, examples | `crates/test_fixtures/*`, `benches/`*, `examples/`* |

Unmarked crates have no scoped file: use this page plus the area dossier
(`network-daemons.md`: `_discovery`/`_mdns`; `remote-access.md` + the
`cerulion_remoted` file: the remote-access row; fixtures: `crates/cerulion_core/AGENTS.md`).

## Shipped surface (gate: `tools/scripts/check_public_surface.sh`; the calls a script cannot make)

- A user-facing example is a WORKSPACE (the rule above); a single-file multi-node program or
  runtime-API construction belongs under `tests/` only, never in `examples/` or a doc.
- No number on a user-facing page without a shipped package behind it: print a figure exactly as
  the package under `docs/benchmarks/results/` prints it (10.45, never 10.4), or ship the package.
- Plain English: no tracker ids, no typographic dashes in shipped text (a dash you remove lowers
  that file's line in `tools/scripts/public_surface_dash_ledger.txt`; never raise one).
- A bulk edit never rewrites the inside of a string literal: fix each by hand, run the tests that read it.
- Shipped text (docs, examples, comments, test comments, strings) tells the user what works, what
  is experimental, what is not supported and what to do; never how the project was built. Gate
  class `work-state` refuses that by key: hardware-status, who-decided, plan-step, review-round,
  session-id, deferred-work, proof-tag, our-machines, candour-voice (patterns:
  `tools/scripts/public_surface_workstate.txt`). Its ledger burns down; a user-facing page reads zero.
- Every removal of shipped content is a maintainer ruling: propose it, never decide it in a lane.
- Docs name only verbs, flags, paths and files that exist: links resolve, `cerulion` verbs parse,
  paths are post-move (`crates/...`, `tools/scripts/...`, `docs/user-api.md`).

## Boundaries

- **Always**: fmt/clippy/doc gates green before any push; docs ride the same PR.
- **Ask first**: new dependencies (license + real-time fit; `deny.toml` gates CI), any
  user-API surface change, anything `unsafe`.
- **Never**: hand-edit generated sources (`native_ros2_messages` types are `build.rs` output in `OUT_DIR`;
  vendored `.msg` edits go via `tools/scripts/refresh_upstream_msg_manifest.sh`); commit
  credentials; name a machine, address, path, login or person (public;
  `docs/leak_guard.md`); fabricate data; leave dead code.

## Deeper context

- `docs/user-api.md` - the user API reference (CLI, macros, YAML, env vars).
- `crates/<crate>/AGENTS.md` - scoped invariants, serial-test lists, gotchas.
- `docs/internals/*.md` - contributor dossiers (test maps, module contracts); each crate
  names its own; `ci-and-gates.md`: the repo-wide gates.
- `docs/` - user guides (networking, multi-process, recording/replay, tutorials).
- https://docs.cerulion.com - hosted docs; index at `/llms.txt`.
