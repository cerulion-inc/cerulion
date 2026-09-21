# CLI internals: `cerulion_cli_engine` + `cerulion_cli`

Contributor dossier for the CLI layer. `cerulion_cli_engine` is a library holding ALL
command logic; `cerulion_cli` is the thin clap binary over it (parse + dispatch + exit
codes). The engine is independently testable without spawning processes, which is why
almost every contract below is pinned by an engine test and only the process-level
behavior (exit codes, signals, completion stdout protocol) by a binary e2e.

Related user-facing docs: `docs/cli_completions.md`, `docs/schema_resolution.md`,
`docs/multi_process.md`, `docs/auto_partitioning.md`, `docs/networking.md`,
`docs/replay_determinism_footguns.md`, `docs/user-api.md`.

---

## 1. `cerulion bag play --resim`: exit-code contract

The surface is `cerulion bag play <bag> --resim all [--verify] [-u|--duration D]
[--strict-state] [--report FILE] [--tolerance YAML]`. `resim_cmd.rs` owns flag legality and
the neutral renderer, `replay_cmd.rs` the entry gates and the stable exit-code surface, and
`replay_engine.rs` the deterministic re-execution itself. The bag is the golden: it carries
the graph, env snapshot, recorded frames, and scheduler trace, and the run re-executes the
workspace's CURRENT cdylibs against it.

There is no replay verb under `cerulion` and nothing hidden behind it: no behaviour and
no stub variant that diagnoses the spelling. Typing `replay` after `cerulion` gets clap's plain
`unrecognized subcommand`, exit 2. There is no `Replay` variant to intercept, above the
login gate or anywhere else (`crates/cerulion_cli/src/cli.rs`, pinned by
`removed_verb_tests::removed_replay_verb_is_an_unknown_subcommand`). A bare `--resim all`
is NEUTRAL: it re-executes and REPORTS, claims nothing about
matching the recording, and exits 0 on any completed run. `--verify` is the verdict opt-in
and carries the whole contract below. Neutrality scopes EXACTLY the two comparison codes
(1 and 6); 2, 3 and 5 mean the re-execution could not be performed and stay loud in both
modes. `--duration D` bounds the run in SECONDS OF BAG TIME and is legal in both modes (a
per-rank resim has k step axes and no shared step number, which is why there is no tick
bound); `--report` and `--tolerance` REQUIRE `--verify`. Every misuse exits 2, never 1:
under `--verify` 1 means "your code diverged", so a malformed invocation reported as 1
would make CI announce a regression that does not exist.

| Code | Meaning |
|---|---|
| 0 | pass |
| 1 | data violation (including a per-field tolerance-exceeded) |
| 2 | bag I/O / not replay-grade / bag↔graph mismatch / schema-drift preflight |
| 3 | node failure: cdylib load error, OR panic-class execution failure mid-replay |
| 4 | tolerance-YAML validation error (pre-flight, before any cdylib loads) |
| 5 | internal error (panic, transport, scheduler) |
| 6 | structural trace divergence |

Code 7 is NOT a resim outcome: it is the CLI-wide "authentication required" refusal
from `login_cmd`'s gate in `main`, listed with the replay codes only so the whole
exit-code space stays visible in one place.

Rules that must not drift:

- **Precedence 3 > 6 > 1**: root cause over symptoms (a crashed candidate explains both
  a diverged schedule and missing frames). The verdict output still renders every block.
- **Exit-3 detection is structural** (the catch site / FFI failure codes), never message
  text. A deterministic tick `Err` is normal execution and replays to exit 0.
- **A `--report` write failure never masks a failing verdict**: a failing run keeps its
  code and emits one loud `error!`; only a PASSING run converts the write failure into
  exit 5.
- **Exit 4 is a pre-flight gate**: unknown/misspelled key, out-of-range threshold,
  unresolvable topic/field name (with ranked suggestions), or a non-`bit_exact` metric on
  a publisher-opaque field; all refused before any cdylib loads. When both an exit-4 and
  an exit-2 problem exist, exit 4 wins.
- **Schema-drift preflight (exit 2)**: each produced topic's recorded first-frame schema
  hash (only when the bag channel's recorded hash recipe matches the current one) is
  compared against the current workspace's BEFORE any cdylib loads; drift refuses with
  per-topic recorded/current hashes plus both remediations (re-record, or check out
  recording-era schemas). Legacy-recipe channels deliberately skip the check.
- **Discovered-topic carve-out**: a recorded topic outside the bag's embedded graph that
  is marked `source: discovered` lands in the unmodelled class and is SKIPPED with one
  warn, never a bag↔graph mismatch. A malformed `record_coverage.json` is treated as
  absent plus a loud warn.
- **The frame-vs-boundary membership check forgives a multi-rank shutdown tail ONLY,
  and only by range.** Frames stamp their step's advanced clock (the exact value the
  step's boundary record carries), so on a healthy recording the last frame sits at the
  last boundary with ZERO margin. The "frames and trace share one writer thread, one
  batch" premise holds for SINGLE-RANK bags only: a multi-process recording's frames
  (SHM taps) and its k per-rank trace rings are cut independently at shutdown, a
  departing rank's barrier participant `Drop` opens the barrier so a survivor can
  commit one more step's frames past the last boundary the authoritative rank-0 stream
  carries (unboundedly many when a survivor outlives a dead rank 0 under
  `--peer-loss continue`), and the boundary gate itself tolerates that peer-longer
  stream. So a multi-rank LOCKSTEP bag that declares no covered range DERIVES one at
  rank 0's last boundary target (`resolve_covered_range`): a PEER-OWNED topic's frames
  past it are TRAILING: skipped by the membership check, deducted from the diff's
  expected counts, reported (`derived: true` on the report's `covered_range` + a
  cause-naming NOTE), never refused, never silent. The forgiveness is scoped to
  peer-owned topics because only a peer can legitimately out-run the authoritative
  stream: a rank-0-owned frame past rank 0's own last boundary is unreachable on a
  finalized bag (the worker banks the boundary in its ring before any publish, the
  recorder's writer drains rings after each batch's tap frames, finalize tail-drains
  the rings, and a ring overrun un-finalizes), so such a frame is a hand-edit and
  still refuses, as does a topic whose producer no manifest names. NOTE: that scoping
  DEPENDS on bagd's batch discipline; see the breadcrumb at bagd's finalize tail
  drain. Everything the boundary stream can still judge is judged at full strength: a
  mid-bag stamp matching no target still refuses exit 2; single-rank bags never derive
  (a tail frame there is genuine corruption); free-run bags are untouched (each rank
  replays to its OWN recorded end, so the class does not arise); and a DECLARED range
  always wins over the derivation, clamp and overstatement refusal included, and
  keeps the declared-capture contract's rank-agnostic forgiveness. A derived range
  with nothing beyond it
  is dropped, not reported, which keeps every pre-existing multi-rank bag's outcome
  and report byte-identical.
- **A refused read log DECLINES the frame comparison it steered; it never fails
  it.** The read log is report-only, and on a multi-process bag it also steers
  the cross-rank injection window. When it is refused for **a rank whose
  read-log-steered injection fell back**, that fallback can hand an injected
  frame to a fire early or late, so the rank's producers legitimately emit
  different bytes. Those topics' divergences are WITHHELD from the verdict and
  reported under `declined_frame_comparisons` (topic, rank(s), cause code, the
  outcome, and the withheld `Violation`s verbatim), with a terminal NOTE and one
  line per topic: `warn!` where divergences were withheld, `info!` where the
  topic matched under the fallback and was merely not credited; `passed`,
  `violations` and `divergence_classes` never see them, and such a topic counts
  in `topics_checked` but not `topics_passed`. The granularity is per topic
  selected by RANK. For the whole-rank census decline that is the cause's own
  granularity; for a PER-TOPIC read-log-derived refusal (`truncated_read_log`,
  `unresolved_producer`) it is deliberately WIDER than the finest truth (which
  would be "the produced topics downstream of the stood-down consumed topic")
  because the rank is the granularity the fallback is known at here, and the
  wider set errs away from accusing the candidate. The wider set is deliberate,
  not a defect. `no_read_log_coverage` is NOT in the set: it says the
  recording never carried read records for the edge, so there was no steering to
  lose, and declining on it would silently stop crediting every pre-annotation
  bag. Listing is UNCONDITIONAL: a topic that MATCHED under the fallback is
  listed too, with `matched_under_fallback: true`, and is still not credited: a
  match there is evidence the fallback reproduced the frames, not evidence the
  candidate is correct.
  **The bound on all of this:** the rule rests on a FALLBACK, so a refused read
  log on a rank with
  **nothing injected declines nothing; there is no fallback to withhold**,
  and no row is minted for it (the whole-rank warn says the same thing, per
  rank, in its `declines` field). One residual bounds that, the
  multi-publisher one, and it is stated in full in `read_log_forensics.md`'s
  decline row.
  **The unknown-ownership residual:** a produced topic whose OWNER rank the map
  does not name is NOT declined; it is judged and credited the ordinary way
  even when a declining rank produced it. Treating unknown ownership as
  declining would decline topics no refusal touched. A SINGLE-RANK replay has no
  owner map at all, and there every produced topic belongs to the one rank that
  ran, so the whole rank declines together.
- **The ragged-tail window opens after the last step a rank's recording PROVES ran,
  not after its last boundary.** `Scheduler::begin_step` banks step *k*'s
  STEP_BOUNDARY before any of step *k*'s fires (the same ring-before-SHM ordering the
  derived range's unreachability proof rests on), so a worker exiting in that window
  leaves a trailing boundary with no fires behind it. The resim runs that step (rank
  0's boundary stream is its clock) and re-fires there, while a ragged-tail
  rule keyed on each rank's last BOUNDARY would give rank 0 a window of
  width ZERO by construction: exit 6 plus an ExtraMessages on the rank-0-owned topic.
  The window key is `TestifiedEnd`, the last boundary step, stepped back by ONE
  **iff** that step banked no fires for that rank, and the frame diff's per-rank width
  is DERIVED from the same value, so the three gates (structural, frame, read log) still
  read one window. The step-back is exactly one and never "back to the last fire-bearing
  step": a boundary for step *k+1* proves step *k* completed, so every non-final step is
  proven and a replay fire in one is a real divergence. Fire-LESS only: a final step
  that recorded SOME fires (a SIGKILL landing mid-fires) earns nothing and still
  refuses: a partial fire set is bounded by nothing the recording carries, so the
  refusal is the tripwire. Both tolerated shapes are reported, in wording that names which
  SHAPE the bag has, but the fire-less note states its CAUSE as an either/or ("either the
  step ran and fired nothing, or the worker shut down between the step boundary and the
  step's fires — the recording cannot tell which"), because the recording genuinely cannot
  distinguish them and asserting the shutdown would be wrong on every graceful stop of a
  graph with a sub-rate node. Unlike the derived range, this half is NOT inert under free-run:
  the recorder's cut is not an artefact of the single-clock model, and a free-run rank
  is driven across its own last boundary identically. Note the rule keys on the
  RECORDING, not on the cause: a rank whose slowest node is SUB-RATE ends its stream on a
  naturally quiet step just as often as a cut worker does, and the two are
  indistinguishable in the bag (only a successor boundary can prove a step completed), so
  such a rank gets the fire-less wording and the one-step-wider window too. That is the
  cost, bounded and announced: an over-fire at exactly that one step is not caught,
  because the recording cannot say whether the step ran.
- **The bag's `graph.yaml` attachment is the EFFECTIVE in-memory config the run
  executed** (rendered by `graph_cmd::render_effective_graph_yaml` through the shared
  `prepare_recording_inputs` seam), never a copy of the on-disk graph file. The same
  renderer writes the run directory's `graph.yaml`, so run dir and bag agree by
  construction; changing one means checking the other.
- **Cross-host bags warn, never refuse**: bags carry a `__cerulion/recorder.json`
  host-identity attachment; replay warns on a cross-arch/os bag (float ULP-skew
  advisory). Absent = silent back-compat; malformed = loud warn.
- Nondeterministic node code (wall clock / RNG in a tick) replays as an exit-1 byte
  mismatch, provably NOT a trace divergence. The footgun catalog is
  `docs/replay_determinism_footguns.md`.

### Tolerance engine

`--tolerance YAML` RELAXES the diff. Metrics: `max_abs`, `max_rel`, `rmse`, `bbox_iou`
(feasibility-matched: PASS iff a perfect matching exists using only pairs at or above
`min_iou`; permutation-invariant; the reported worst value is the bottleneck pair),
`set_equal`, `set_subset`, `ordered_list_equal`. Resolution precedence per schema field:
`fields[path]` > topic-wide `metric:` > `default_metric`. Everything resolving to
`bit_exact` and every untargeted topic stay byte-exact, so a drifting sibling field is
still caught. A topic-wide or default metric requires EVERY covered field to be
metric-decodable, else exit 4. An all-`bit_exact` document is a loudly-warned no-op.
Worst-value tracking is O(1) running state per `(topic, field)`; frames are never
buffered; replay memory stays bounded by advise-behind eviction in the bag reader.

### Numeric false-pass classes (hunt these in any comparison/tolerance code)

1. **NaN-discarding folds**: `f64::max`/`min` silently drop NaN: a mid-computation NaN
   folds the running worst to 0.0 and passes divergent data. Gate on `!x.is_finite()`,
   not `is_nan()`; sentinel infinities get byte-exact compare.
2. **i64/u64 → f64 widening**: integers above 2^53 collapse: two distinct ns timestamps
   widen to the same f64 and diff to 0.0. Use integer-domain math for 64-bit int fields
   and reject precision-lossy metrics on them at validation time.
3. **Length-derived envelope fields**: when payload lengths may legitimately differ,
   any header field derived from length byte-fails the whole frame; compare envelopes
   field-wise excluding exactly the length-derived fields.
4. **Max-sum assignment is the wrong objective for a floor verdict**: it can false-FAIL
   when an all-above-floor matching exists. Check FEASIBILITY and report the bottleneck
   (this is why `bbox_iou` is feasibility-matched).

---

## 2. `graph run` / `graph levels` / `graph profile` / `graph partition`

### `graph run`

- **Multi-process by default** on Unix under the real clock: an unpartitioned graph
  derives a partition (cost-fused via `graphs/<name>.costs.yaml` when present, else
  process-per-node) and runs supervisor + one worker per group. A `process_groups:`
  block is respected as written. Non-Unix falls back to the monolith with a loud notice.
- **Consent ladder (the never-mutate floor)**: persisting a derived partition into the
  YAML needs consent. `--yes` writes it (with a `.bak`); a TTY previews and asks y/N
  (N = run the derived groups in-memory, file untouched); no-TTY runs in-memory with a
  loud notice naming `--yes` and `--single-process`. A refused/failed run never mutates
  the graph file and never leaves a `.bak`.
- `--single-process` opts out entirely; `--auto-partition` re-derives over an existing
  block and conflicts with `--single-process`; virtual/external clocks keep the monolith
  routing. The in-memory derived plan and the written plan must be equal, pinned by a
  JSON-equality test in `graph_run_preflight_test.rs`.
- **Network decision** (`resolve_run_network` → `Off | Inert | Strict | Permissive`,
  pure, pinned by `network_run_gate_test.rs`): no/disabled `network:` block ⇒ Permissive
  (on Unix the shared `cerulion-netd` daemon hosts the plane; graph/worker processes
  stay network-free); an enabled block ⇒ Strict (verbatim locators, egress allow-list,
  declared ingress), routed to a per-run `graph run-gateway` child because the shared
  permissive session cannot represent verbatim locators or a per-run allow-list;
  `--network off` / `CERULION_NETWORK=off` ⇒ Off with a loud kill-switch warn;
  virtual/external clocks ⇒ Inert. Run SHAPE is not an input; networked multi-process
  is first-class. `--record` KEEPS the network; only record plus a declared `ingress:`
  block is refused, naming both workarounds.
- **Run directory**: every `graph run` (recorded or not) writes
  `~/.cerulion/runs/<sanitized-graph>-<run_id>/` (`run.json` + `graph.yaml`/`env.json`/
  `recorder.json`) and announces it on the run registry. Never fatal: any bookkeeping
  failure is exactly one `warn!`. The write sits after the partition pre-flight and
  before deployment dispatch, and the recorded `process_groups` comes from the RESOLVED
  deployment, never from the config's own flag (the config may not reflect an in-memory
  derived partition).

### `ros2:` graph entries (spawn + supervise only)

A graph entry with a `ros2:` block instead of `type:` is a stock ROS 2 process the run
SPAWNS beside the native graph, on the identical child environment the `cerulion ros2`
pass-through verbs stage (`ros2_cmd::stage_base_child_env` is the one shared seam:
`RMW_IMPLEMENTATION=rmw_cerulion`, `LD_LIBRARY_PATH` + a staged ament prefix on
`AMENT_PREFIX_PATH`, and the preload matrix: auto-inject `libcerulion_heaphook.so` when
present beside the rmw lib, `CERULION_ROS2_PRELOAD` as an ADDITION that stacks ahead of the hook (`user.so : hook :
ambient`, user's first) or the `off`/`none` kill switch;
`decide_preload` is the pure oracle). The verbs themselves are VERBATIM pass-throughs
dispatched by `main`'s raw-argv intercept BEFORE clap (a leading `--prefix` must reach
ros2 untouched; the clap variants exist for help + completions only). Design rules, each
with a validation pin in `crates/cerulion_core/tests/graph_test.rs`:

- **Validate whole, then split.** `validate_graph` runs on the FULL file (the entry's
  shape rules: `type:` XOR `ros2:`, no ports, `package`+`executable` XOR `launch`,
  scalar `params`, never the only node kind, never in `process_groups:` /
  `level_assignments:`). `graph_run` then lifts the entries out with
  `GraphConfig::take_ros2_nodes` BEFORE the partition preflight, deployment planning,
  the recording's effective-graph attachment and the runtime build, so every seam below
  it sees a plain native graph. BOTH embeds (the run directory's and the recording's
  `graph.yaml`) render the effective config CLONED BEFORE the split (`embed_config`,
  threaded through `RecordingRun` / `SupervisorRecordingStart` / `RecordingInputSpec`):
  the whole run, in authored node order, interleaving included (the earlier re-append
  helper is gone). The resim preflight lifts them back
  out (one warn + `ReplayOutcome::ros2_entries_skipped`; never a BagGraphMismatch, never
  a respawn; pinned structurally by `ros2_resim_no_respawn_test.rs` and behaviorally by
  the record→resim e2e arm), and `bag info` renders them as their own section.
  `GraphRuntime::build` refuses a config that still carries one, by name.
- **Core derivations are native-only** (`GraphTopology::build`, `build_trigger_edges`,
  `validate_process_groups`, the partition derivations): `graph levels` and
  `graph partition` need no extraction and stay accurate; `graph profile` extracts (it
  builds a runtime) and says the snapshot covers native nodes only.
- **Supervision** lives in `crates/cerulion_cli_engine/src/ros2_graph.rs`: children spawn in
  their own process group (the recorder precedent: the graph process drives the
  teardown), a death-watch thread polls `try_wait`, and the ONE pure decision
  `classify_child_exit` maps an exit to the run's `--peer-loss`: exit 0 ⇒ warn + continue
  (a sidecar finishing must not stop the robot), non-zero/lost ⇒ `continue` warns and
  runs degraded, `fail` flips the run's `running` flag (the graph drains exactly as on
  Ctrl-C) and `finish` turns the drained `Ok` into the `Err` naming the entry. Teardown
  is per child freeze → peek → SIGINT: SIGSTOP (uncatchable; a stopped child cannot
  exit), a `waitid(WEXITED | WSTOPPED | WNOWAIT)` peek so an exit already in flight is
  judged as the child's OWN death by the run's policy and never as this teardown's stop, then
  SIGINT + SIGCONT; the 10 s `ROS2_CHILD_SHUTDOWN_GRACE` for `ros2 launch` to wind its
  nodes down; then a SIGKILL backstop to the child's WHOLE process group (each entry is
  spawned as a group leader, so the launched nodes go with it). A leader that dies on its
  own takes its group with it before it is reaped, on every path that reaps one, with two
  stated residuals: a group kill the kernel refuses is logged loudly and the leader alone
  reaped; a leader whose peek the kernel refuses is judged from `try_wait` alone with its
  group untouched. `Drop` is the never-orphan floor on error paths: no grace, no SIGINT,
  the group SIGKILLed at once. Spawn
  happens once, before the deployment dispatch, so every run shape (supervisor, record,
  both monolith arms) gets the same children, and each exit folds through `finish`.
- **No product seam for tests.** The spawner resolves `ros2` through `PATH` and the rmw
  lib through `CERULION_LIB_DIR`, so the e2e puts a fixture `ros2` script first on the
  child's PATH and an empty `librmw_cerulion.so` in a fixture dir; zero test-only surface
  in the YAML or the CLI.

### `cerulion ros2 migrate`: loaned-message rewrite

Two halves with a versioned JSON seam between them. The AST prover/rewriter is a
standalone clang LibTooling tool (`tools/ros2_migrate/cerulion_ros2_migrate_clang.cpp`)
built and gated ONLY in the ROS 2 toolchain container; the Rust workspace never links
libclang; the tool consumes `compile_commands.json`, proves the safe publish pattern
from the AST, and emits byte-range edits + refusal candidates (fixed reason vocabulary,
documented in its header) as sorted, deterministic JSON. The Rust half
(`cerulion_cli_engine::ros2_migrate`) orchestrates: per-TU engine runs through the
`MigrateEngine` seam, merge (header-site dedupe; overlapping proposals demote to
`conflicting-analyses` candidates), edit application only after re-verifying the
original bytes on disk, and ONE `MigrationPlan` serving both modes: the dry-run's
printed diff and `--write`'s patch file are the same bytes by construction. Design
rules:

- **Dry-run is the default and writes exactly one thing**: the machine-readable
  manifest at `.cerulion/ros2-migrate-manifest.json` (atomic temp+rename). The manifest
  carries the workspace git key (HEAD + tracked-dirty), candidate/manual/rclpy lists,
  counts, and a consumer-owned `decision` slot this verb writes as `null` and preserves
  VERBATIM on refresh: the launch-time offer flow records a declined offer there, and
  a manual dry-run must not forget it. `--write` clears the candidates it consumed and
  re-keys to the post-commit HEAD.
- **The diff renderer is derived from the edit spans, not a heuristic diff**: change
  runs are per-edit, so the fill lines between the decl edit and the publish edit
  render as CONTEXT (the chosen 3-line diff shape). Reversibility is pinned with REAL
  `git apply -R` restoring the original bytes.
- **`--write` gates**: clean tracked tree (untracked files tolerated EXCEPT one the
  migration would edit; no committed baseline means no one-commit undo; untracked
  files are enumerated INDIVIDUALLY via `--untracked-files=all`, because git's default
  collapses an entirely untracked package to one `?? dir/` entry that matches no
  planned file and would let the gate commit a file `git revert` then deletes), consent
  (TTY y/N or `--yes`; non-TTY without `--yes` refuses naming both), then ONE commit +
  `cerulion-ros2-migration.patch`, then the automatic
  `colcon build --packages-select <affected>` (affected set via the public
  `derive_affected_packages` walk; the launch-time offer flow reuses it). A build
  failure exits 1 naming `git revert <sha>` + the patch; the commit stays. The write
  window polls an injected `interrupted` seam (the CLI arms a flag-flipping Ctrl-C
  handler for `--write` only) at three safepoints (pre-write, between file writes,
  pre-commit), and a trip rolls back what was written and refuses; anything the
  rollback cannot restore (a write failure) or deliberately preserves (foreign
  content) is reported for hand repair, never silently dropped.
- **The write batch holds the workspace lock, in the variant whose wait the user can
  end.** `--write` serializes against every other Cerulion
  writer on the SAME root (a second `migrate`, and `ros2 attach`/`node`/`graph`/
  `schema`/`cerulion-wsd` where the colcon and Cerulion workspace roots coincide) on
  `<ws>/.cerulion/workspace.lock`. The lock is per-ROOT and `--workspace` is the COLCON
  root, so two migrations of different workspaces inside one git repository are NOT
  serialized against each other even though they share an index (the in-lock
  `staged_all`-vs-`staged_ours` refusal catches the common case of that, but it is a
  TOCTOU check, not a backstop: `git commit` takes the whole index, so two
  migrations in one repository can each pass it before the other stages). Scope is exactly the
  write batch: taken AFTER consent (holding it across the analysis and an interactive
  prompt would block every other writer for as long as a human reads a diff), covering
  the TOCTOU re-verify through the commit and the manifest refresh, released BEFORE the
  colcon build (a compile, not a mutation). It is
  `WorkspaceLock::acquire_interruptibly` and neither blocking constructor, for a reason
  per rejected constructor. (1) `acquire_and_track_gitignore` appends `.cerulion/` to an
  existing `.gitignore` the first time it creates the lock directory, a TRACKED
  modification, and this verb promises a dry-run modifies nothing and a rolled-back
  `--write` leaves a pristine tree (MEASURED, when the tracked write still lived in the
  default constructor: swapping it in fails `write_rolls_back_when_the_commit_fails` and
  `a_failing_source_write_rolls_back_files_already_written` on `M .gitignore`). (2) BOTH
  blocking constructors block in the kernel, and `ctrlc` installs its handler with
  `SA_RESTART`, so that `flock` is auto-restarted and answers no signal; a wait entered
  AFTER the user consented, in the one verb whose interrupt safety is a documented
  contract, must be endable; the variant polls (on the kernel `flock` AND on the
  reentrancy registry's condvar, so a same-process peer's hold is interruptible too) and
  reports `AcquireError::Interrupted` when `deps.interrupted` trips. That enum
  deliberately has no `From` into `CliError`, so `?` cannot skip it and migrate must match
  every arm; it turns the interrupt into a DISTINCT refusal, `interrupted while waiting
  for the workspace lock at '<path>'`, not the bare `interrupted — …` the write
  safepoints use. The two have different causes and different remedies (you were queued
  behind another `cerulion` and gave up, versus your Ctrl-C landed mid-write), and the
  separate wording is also what lets a test tell the two paths apart. A THIRD arm,
  `AcquireError::Internal`, carries the lock's own internal-bug refusals (it re-entered
  its own acquire; its reservation vanished mid-acquire) and says so; folding them into
  the failure arm would send a user to audit a `.cerulion` that is perfectly fine, for an internal bug.
  Since
  `git commit` runs inside the lock, a `pre-commit` hook that shells out to a Cerulion
  WRITER verb on the same root deadlocks against this run; hooks are the user's
  responsibility, and the colcon build is outside the lock for the adjacent reason. On a
  non-Unix build the guard is a declared-weaker no-op that locks nothing, like every
  other `WorkspaceLock` caller. The dry-run takes no lock at all, not because it cannot
  (`acquire_read` creates nothing) but because its one product is HEAD-keyed, so a
  manifest written by the loser of a race is discarded by its consumer; the accepted
  cost is that a dry-run running beside a `--write` can report a half-migrated tree.
- **Rollback destroys only what it can PROVE is the migration's** (the fd-bound
  guarantees in this bullet are UNIX-SCOPED: the non-Unix build carries
  declared-weaker fallback arms (path-joined I/O, a unit `FileIdentity` that always
  compares equal, path-level type gates) per the `#[cfg(not(unix))]` arm's own
  comments in `ros2_migrate.rs`; a path or symlink swap is not defended there): every
  source restore is bound to the fd it validated (`anchored::RestoreFile`: one open,
  read + fstat identity + disposition + rewrite through the SAME descriptor, so an
  atomic replace landing after validation retargets the path, never the restore), and
  the
  rewrite RE-VERIFIES the content through that descriptor immediately before
  truncating, so an in-place same-inode edit landing after validation WITHDRAWS the
  restore (reported; residual: the re-verify narrows that window to the
  adjacent-syscall gap between the confirming read and the truncate; closing it
  outright would need mandatory locking no portable API offers). A vanished file is
  recreated strictly into absence (`O_EXCL`); the patch teardown captures whatever is
  at the patch path by ATOMIC RENAME to a private NONCE-UNIQUE quarantine name
  (pid + monotonic counter + 64 random bits, verified absent before use;
  a fixed pid-scoped name could collide with, and the rename silently replace, a
  stale quarantine left by a crashed prior run) and verifies + destroys
  only the quarantined object (ours → unlinked, prior patch restored into absence via
  `create_new`; foreign → returned in place, or preserved at the reported quarantine
  name when the path re-occupied; nothing is ever deleted unverified). The patch
  INSTALL holds the same rule: create-new FIRST, and on
  `AlreadyExists` the occupant is captured by the same nonce-unique atomic rename
  and destroyed only when it is verifiably THE FILE the preflight snapshotted:
  filesystem identity ((dev, ino) + mtime, fstat'd off the reading fd) plus the byte
  compare, never byte equality alone, since an identical-bytes file another actor
  creates after the preflight is a different file (inode recycling makes the
  identity strong evidence rather than proof; combined with the mtime and the
  unpredictable capture name there is no constructible collision); a file that
  appeared at the path after the preflight check is refused and given back
  byte-untouched, never unlinked. Foreign state
  (replaced bytes, in-place edits, planted links, hook-staged index entries) is
  preserved and reported. Stated residuals: the index unstage's `git show`-to-`git
  reset` gap (git has no per-entry compare-and-swap; the harm is re-derivable: the
  worktree is untouched and a discarded entry is one `git add` away), and the in-place
  rewrite window above.
- **No inert shipping**: the engine binary resolves via `CERULION_ROS2_MIGRATE_TOOL`
  (must exist when set), beside the `cerulion` binary, then `PATH`; absent means exit
  69 with the container build instructions, never a silently-empty analysis.
- **rclpy is report-only** (no loaned-message API upstream); the report and manifest
  state the true floor.

### `graph levels`

Read-only derived DAG levelization view: nodes + trigger policies per level, triggering
edges, the `process_groups:` band mapping, and a spawner-consumability verdict. An
invalid partition prints in full, then exits nonzero. `ros2:` entries are listed apart
from the DAG as spawned, unscheduled processes, and the `nodes:` count excludes them.

### `graph profile`

Profiles LIVE (real clock only; network-inert) and writes per-node p50, per-edge rates,
and a FROZEN default budget (`derived_budget_ns` / `profile_cores`) to
`graphs/<name>.costs.yaml`, the auto-partitioner's input; the `hop:` block is
user-editable. Per-node fire targets are auto-derived from a warm-up projection;
profiling stops when every warm-up-active node meets its own target, at `--duration`,
or on Ctrl+C (harvest isolation re-projects to the actual window, so an early Ctrl+C
never over-projects). `--fires N` is the uniform-target override. Under-sampled nodes
are ISOLATED: no fabricated cost, a loud warn (silent-through-warm-up marker +
starved-trigger hint where applicable), and the command still exits 0.

The e2e (`graph_profile_iox2_test.rs`) separates load-insensitive claims (asserted
unconditionally) from rate-target claims (routed through a pure starved-run classifier),
so a loaded machine degrades the test visibly instead of flaking it. Keep that split
when adding arms; the load-bearing assertions are counters and artifact shape, never CPU
percentages or wall-clock bands.

### `graph partition`

Derives and WRITES the `process_groups:` partition. Cost-aware fused when a costs
snapshot exists; an explicit `--costs` path must exist and parse; malformed is a hard
Err, never a silent fallback to the baseline; no snapshot ⇒ process-per-node baseline.
`--budget-ns` defaults to the artifact's frozen `derived_budget_ns` (readers never
re-derive it; `graph run` resolves its default through the same point). The rewrite is
SURGICAL: only the `process_groups:` block changes, a stale `process_group_order:` key
is removed (listing order IS rank order), comments are byte-preserved, and a `.bak` is
written (`rewrite_process_groups_block` + `write_yaml_atomically`, pinned by
`partition_emit_test.rs`). Consent floor: `--dry-run` wins over `--yes`; TTY previews
and asks; non-TTY requires `--yes`. Validation is replace-scoped, which makes the verb
the recovery tool for stale/broken partition blocks.

---

## 3. `topic list`: discovery ladder and remote topics

Local topics list first and instantly; remote discovery runs by default (`--no-network`
skips the whole remote half; `--connect`/`--listen` are repeatable and additive; the
opt-in `--scan` subnet sweep is a separate rung that must stay opt-in). The remote half
is best-effort: a session/query failure is a loud note plus exit 0, never silently
empty, never a hang.

- **The ladder produces candidates, not sessions.** Rungs: mDNS browse (primary), the
  peer cache `~/.cerulion/peers.json` (TTL-bounded), the hostname convention
  (`CERULION_PEERS` / config `peers` / `<name>.local`), and opt-in `--scan`. No rung
  opens a zenoh session; found gateway locators fold into ONE bounded-connect query
  session (`bounded_connect: true` on the ephemeral session config: a positive connect
  timeout, single inline retry attempt, no exit-on-failure).
- **TCP pre-filter is half the defense.** zenoh 1.8 tries connect endpoints sequentially
  and propagates a connect timeout unconditionally, so one black-hole locator would sink
  every reachable robot. `query_remote_topics_with_candidates` therefore TCP-probes every
  endpoint first, in two concurrent batches (ladder candidates at
  `LADDER_PROBE_TIMEOUT`, explicit locators at `EXPLICIT_PROBE_TIMEOUT`) via the pure
  `plan_connect_set` planner (lives in `cerulion_discovery`; the probe primitive is
  shared with the subnet sweep). A dead ladder candidate is dropped and debug-logged; an
  unreachable EXPLICIT locator is dropped with a LOUD warn. On open failure the query
  retries once with the reachable-explicit set only.
- **Presence is verification.** A robot rows in `ROBOTS` only from announce presence or
  an mDNS browse answer; non-mDNS ladder finds render on a labeled
  `candidates (unverified):` line: not rows, not reachability evidence. No remote
  TOPIC collapses the `REMOTE TOPICS` section to ONE
  `remote: none discovered in N ms (...)` line (`render_remote_none_discovered`), after
  a blank line when a `ROBOTS` section rendered: the `--connect tcp/<host>:7683` hint
  when nothing was reachable, `retry` when a peer was (a given locator, a robot row, an
  mDNS answer). With no robot and no candidate there is no `ROBOTS` section either. A
  populated gather keeps the header and its rows unchanged.
- **Internal topics are hidden from the LOCAL section by default.**
  `topic_cmd::is_internal_topic` is the ONE predicate for the framework's own
  channels: the prefix list `topic_cmd::INTERNAL_TOPIC_PREFIXES` (`/__cerulion/`, its
  slashless twin, `/bagd/`) PLUS the bare `/__cerulion` token (the gateway's reserved
  rule), which no prefix can spell. `render_local_topics_section` skips them unless
  `--all`, ends the section with a count line naming `--all` when it hid any, and marks
  a shown one `internal` in the trailing column (the path stays the row's first token).
  REMOTE rows are NOT filtered: neither `render_remote_topics_section` nor the mirror
  fold consults the predicate, so a robot's own channel that reaches the announce or
  demand space, or a local mirror of one, prints as a plain REMOTE row with or without
  `--all`. `bag record --all` / `--regex` CALL the same predicate
  (`bag_cmd::auto_selectable`), so a row the listing hides is never auto-selected.
  `bag_cmd::AUTO_SELECT_EXCLUDED_PREFIXES` is an alias of the prefix slice and
  `cerulion_bagd::EXCLUDED_TOPIC_PREFIXES` is equality-pinned to it (bagd's
  live-discovery planner walks its prefix copy alone; the bare token is the one name
  where it would differ, and no producer creates a data service under it today). Extend
  `topic_cmd::INTERNAL_TOPIC_PREFIXES` (and mirror it in `cerulion_bagd`), never a
  call site.
- **The remote-query FAILURE note is engine-rendered.** A failed query prints ONE
  stderr line of the same shape as the empty notice,
  `remote: discovery unavailable (<error>; pass --no-network to skip it)`, from
  `topic_cmd::render_remote_discovery_unavailable` (exact oracle; a multi-line error
  folds onto the one line, control characters are neutralized). The binary prints the
  returned string verbatim, like every other `topic list` line.
- **Peer-cache write-back** goes through the `resolve_write_back` gate: only robots
  confirmed live write back; an announce-only row has no verified locator and is never
  cached (its old entries age toward the TTL).
- **Mirror fold**: a local service that is really a re-injected mirror of a remote
  robot's topic folds OUT of LOCAL and INTO REMOTE, attributed to its origin robot. The
  fold logic lives in `cerulion_core::transport::mirror_registry` (shared with the viz
  daemon); `topic_cmd` keeps only a thin adapter; do not re-implement the fold here.

### Observer verbs (`topic echo` / `info` / `hz`)

- `classify_observed_topic` routes each verb: a genuine local producer is read directly;
  a mirror or an absent topic goes through the network daemon's demand plane (one shared
  mirror per topic; the demand is released when the command exits).
- **Convergence wait**: schema/catalog resolution polls the daemon via the converged
  query verbs. Posture is PER SEAM: a seam that makes no absence claim must pass
  `ResolveWait::no_wait()`, or every LOCAL topic stalls the full first-contact ceiling.
  The wait loop must use the non-reconnecting `_once` client verbs (a reconnect re-runs
  connect-or-spawn and falsifies the wall). A Ctrl-C'd wait is an INTERRUPTION, never an
  absence claim: no-claim message, exit 0. All progress lines go to stderr through the
  single `write_convergence_line` seam.
- **SIGINT during a blocking transport wait** can surface as a transport `Err` before
  the ctrl-c handler flips the running flag. On such an `Err`: check the flag, retry
  bounded by attempt count (`MAX_CONSECUTIVE_WAIT_ERRORS`), never sleep-retry.
- **An event is not a message.** The observer loop paces itself when event wakes yield
  no deliverable frame (a notify flood must not spin a core), and every empty report
  names the frame source and counts "decodable" frames; the observers count only frames
  that survive decoding, so a bare "0 frames" claim would be wrong under a
  malformed-frame storm. The load-bearing test assertion is the pacing counter
  (`observer_pacing_engaged_count`), never a CPU percentage; CPU gates fail open on a
  loaded runner.

---

## 4. Schema resolution

- `schema info` resolves workspace schemas (`schemas/*.yaml`, by schema ENTRY name first,
  then by file stem: `schema_cmd::workspace_lookup`, the ONE lookup the existence probe,
  the resolver's workspace arm and the validator's identity share, so an entry `Foo`
  declared in any file and a `Foo.yaml` whose entries are named otherwise are ONE
  collision, refused naming both sources, the decision below) AND built-in ROS 2 types (`pkg/Type` or
  `pkg::Type`) from the embedded registry `native_ros2_messages::BUILTIN_MSGS`. On a name
  collision the workspace wins, with a loud stderr shadow warning. `schema list` shows
  both, shadowed built-ins marked inline. The claim's own state space (entry declarers ×
  stem file × store twin × spelling) is the doc of `schema_cmd::WorkspaceLookup` and the
  table of `the_workspace_claim_state_space`. A
  spelling that names MORE THAN ONE definition is REFUSED by the lookup itself: a loud
  `Validation` error naming every source in one format, `'<spelling>' is ambiguous in this
  workspace — defined by: schemas/a.yaml (entry Foo), schemas/Foo.yaml (file stem; entries:
  Bar), schemas/nav/msg/Foo.msg (store).`, for all three shapes: an entry declared by
  several files, a file stem beside an entry of that name in another file, and a YAML
  definition the `.msg` store also spells: a bare entry or stem beside any
  `schemas/<pkg>/msg/<Name>.msg`, a nested `schemas/<pkg>/<Type>.yaml` beside
  `schemas/<pkg>/msg/<Type>.msg` for the qualified spelling (the twin, which the
  store-parity lane refuses with the same `defined by:` shape), and a bare name
  one nested sole-entry `schemas/<pkg>/<Name>.yaml` and the store, or a second nested file,
  also spell; refused by the probe, the resolver, `schema info` and the identity's own fast
  path BEFORE the store may answer "present"; an unreadable nested stem file beside another
  declarer is unchecked, naming the file. A nested file beside a TOP-LEVEL definition is NOT
  a twin: the bare spelling names the top-level one and the nested file is `<pkg>/<Name>`;
  the cfg-decided fixtures model exactly that pair as the two builds' schemas.
  `schema info`, the existence probe, the resolver and the validator's identity all render
  that one error, never "first wins" or "entry wins". The two verbs that EXECUTE a workspace
  graph apply the same lookup to
  every output's `schema:` before the graph is built: `graph run` (with the report, which
  then names it first, or without it under `--no-validate`; `node run` and a `ros2 attach`
  bridge graph run through it) and `graph profile`, which never runs the report
  (`graph_cmd::refuse_ambiguous_output_schemas`, one private fn, two call sites; a bare name
  the `.msg` store defines in two packages, and a schema file the lookup cannot read or
  parse, are refused the same way, the latter naming the file), so an ambiguous graph cannot
  run on a last-wins pick. Outside it, stated: the hidden `graph run-worker` verb is only ever
  GIVEN a plan a gated `graph run` mints, and `bag play --resim` runs the BAG's graph while
  still folding this workspace's schema-hash map last-wins (advisory), a residual. The resolver's qualified arm
  consults the lookup too, so `node create -o pkg/Foo` / `node modify` refuse a qualified
  twin exactly as the run does (and an unreadable `schemas/pkg/Foo.yaml` refuses them, naming
  the file). Precedence exists only where nothing is spelled:
  the `schema info` tree resolves a NESTED reference over built-ins → `.msg` store →
  TOP-LEVEL workspace YAML, last-wins per qualified name
  (`schema_cmd::resolution_schema_set`), so a store twin never overwrites a top-level
  workspace definition the verb selected (a nested `schemas/<pkg>/<Type>.yaml` is reachable
  by spelling, never by that fold, so a nested REFERENCE to a name it and the store both
  spell renders the store's; the served catalog (`schema_serve::build_schema_docs`, what a
  gateway answers and a bag records) keeps the STORE first, deliberately: it carries the
  `schema_hash → name` binding a runtime-registered `ros2 attach` route and `bag record` are
  named by, and the bridge encodes with the store's definitions, so for a nested twin the
  tree shows the workspace's definition while the desk decodes with the store's, a stated
  divergence). The identity's
  verdict (`graph_cmd::spelling_verdict`: `Same` / `Different` / `Refused(reason)`) carries
  the reason into the failing check, so both the producer and the consumer check name the
  sources; an unreadable claim is `Refused` as "could not be checked", never absent, and a
  refused spelling does not agree with itself; the equal-spelling fast path consults the
  claim before answering. The stem `schema info` reopens is built from real path
  components, so a `\` in a Unix filename is a filename character, not a separator.
- **One renderer for every source**: `schema_cmd::render_schema_entry` prints the same
  tree for built-in, workspace YAML, `.msg`-store, and network-resolved schemas: a
  header block, then a recursive field tree (non-primitive fields expand inline; arrays
  expand the element type once; repeats read "expanded above"; an unresolvable ref is
  flagged). `hash` and `wire fixed size` are top-level only (the message's wire
  identity); every field line keeps its `(fixed|variable)` class; `source:` is a
  one-line provenance. Network-served docs are parsed and seeded beside the built-in
  registry so nested built-in refs expand too; an unparseable root falls back to the
  verbatim served text.
- Remote resolution routes through the network daemon's shared query plane; a consumer
  opens its own transient session only when the daemon is not applicable (explicit
  locators, non-Unix) or unreachable: a loud degrade, never a silent fallback.
- `ros2 attach` type acquisition (workspace `.msg` store + built-in corpus first, then
  the wire-native type-description service, then the local ament harvest) is documented
  in `docs/schema_resolution.md`; materialized `.msg` files ride the same consent gate
  as the generated graph.

### Pinned schema-hash constants

`topic_cmd.rs` pins `STD_MSGS_STRING_SCHEMA_HASH` and `SENSOR_MSGS_IMAGE_SCHEMA_HASH`
(the TUI reuses them). After an INTENTIONAL hash-recipe change: run
`cargo test -p cerulion_cli_engine --test integration_test pinned_hashes` and copy the
actual values from the assertion failure into the constants. Never hand-compute them.

---

## 5. Node metadata: source code is truth

Port and trigger metadata lives in `nodes/<type>/src/lib.rs`; there is no sidecar file.
`node_metadata::parse_node_metadata` derives it at runtime (walks the macro-form
struct's `#[input]`/`#[output]` field attributes; for raw-FFI nodes, reads the
`// CERULION:INFO_START` JSON marker block). `node info` and `node list` are direct
callers. Contracts:

- **Detection is fall-through, not hard-required**: `try_parse_raw_ffi_node` returns
  `Ok(None)` when EITHER marker is absent (macro parsing is tried next); a missing
  marker is NOT an error. The one hard ordering error is `INFO_END` appearing before
  `INFO_START`.
- New metadata extends `parse_node_metadata`; never add a parallel metadata file (a
  second source of truth is how metadata drift happens).
- **A port's schema NAME is derived from the `use` that brings its type in, and it is a
  schema name, not a Rust path.** `use native_ros2_messages::pkg::T;` → `pkg/T` (the
  codegen crate is stripped). An EXTERNAL message crate keeps its crate name as the
  package (`use unitree_go::T;` → `unitree_go/T`, what the `.msg` store and the graph
  YAML both spell). A path rooted at a module declared in the SAME scope as the `use`, or
  at `crate`/`self`/`super`, is crate-internal and yields the BARE leaf
  (`mod detections { include!(…) } use detections::DetectionArray;` → `DetectionArray`),
  because a workspace schema's canonical name IS its bare entry name
  (`schema_cmd::resolve_port_schema`'s `Workspace` arm returns it unchanged) and the
  codegen-into-a-module shape is the only way a `schemas/*.yaml` becomes a Rust type.
  Only the node struct's ancestor scope chain contributes (nearest scope wins; a sibling
  module's `use` never decides a port), and a leading `::` is always external. A port's
  type ident is resolved PER LEAF over that chain (`node_metadata::resolve_leaf`,
  whose doc is the state-space table and whose test is the same table, row for row,
  `the_use_walk_state_space`): the scope's own `use` items binding the leaf
  first (a cfg-exclusive pair carries every name; two plain imports, the later shadows),
  then what its DIRECT globs make visible: `use super::*` the parent's bindings, `use
  super::super::*` the grandparent's, `use crate::*` the root's; a narrow glob (`use
  super::helpers::*`) reaches no ancestor. A `use` path is resolved by its root (`::x`
  external; `crate`/`self`/`super`×k the corresponding scope; a module this scope
  declares, or one an ancestor declares and a direct glob makes visible, INLINE or
  OUT-OF-LINE (`mod schema_types;` is crate-internal, never the package
  `schema_types/…`); else the external crate), walks inline sub-modules, and resolves the
  leaf in the reached scope by the same rules (its full bindings on the chain, its `pub
  use` re-exports otherwise), so a local RE-EXPORT names its target before any
  qualification is stripped (`mod aliases { pub use native_ros2_messages::sensor_msgs::
  Image; } use aliases::Image;` → `sensor_msgs/Image`; a vendor crate's type keeps its
  package; a rename inside or at the node is keyed by the name the field sees; a
  cfg-gated re-export module keeps the crate spelling as its alternative) and a chain
  through sibling modules (`mod first { pub use …::Image; } mod aliases { pub use
  super::first::Image; }`) resolves to its target, with a per-(scope, leaf) cycle guard.
  A module with no `pub use` of the leaf is the codegen shape, bare. UNDECIDABLE (a
  glob re-export, a cyclic chain, or either behind a hop) is said loudly and propagated
  to whoever imports it: no schema recorded, never a bare guess, and never an older
  glob-visible binding of the same leaf; a port type no explicit `use` names while any
  glob is in scope is reported the same way. An ancestor's import is a cfg alternative of
  a nearer conditional one only when a direct glob makes it visible (without one the
  cfg-off configuration has no such name in scope: E0425). A
  `#[cfg]`-gated same-scope module is local unless its predicate is statically FALSE
  (`#[cfg(any())]` declares nothing, so the import is the crate); it is a GATE (the
  bare name reported, the crate spelling recorded as an alternative, and a warn said by
  the port that binds it) only when the predicate is undecidable (a feature, a target,
  a custom key; a `cfg_attr`-emitted `cfg` likewise; an always-present one is a plain
  local module). Measured: a same-scope `mod X` shadows an extern crate `X` for a bare
  `use X::T` while the cfg is on, and with it off the import reaches the crate instead:
  two configurations, two schema names, and a source parser cannot know the active set.
  For that shape, spell `use ::X::T;` (crate) or `use self::X::T;` (module) to make it
  unambiguous to rustc and the parser alike. A `cfg` on the `use` itself is decided the
  same way, and gives a port more names in four more shapes (each a shape whose OTHER
  configuration BUILDS and publishes the other name): an ancestor's module
  reached through a `#[cfg]`-gated glob (or a gated ancestor module reached through a glob:
  spell the module by path, `use super::X::T;`, or the crate, `use ::X::T;`), a
  `#[cfg]`-gated `use` of the MODULE a type's path starts with (`#[cfg] use vendor::foo;
  use foo::T;`: with every such import off the path starts at whatever is in scope without
  it, an ancestor's glob-reached module or the extern crate; spell the type by its crate
  under each cfg), cfg-exclusive imports of one leaf, or of the module a path
  starts with (two or more; every further one is a further name), and
  a conditional import shadowing an ancestor scope's plain one, the last two of which have
  no import spelling that ends the ambiguity (the parser's warning names the shape, and the
  node type). A `#[cfg]`-gated glob that is the ONLY path to a type is NOT one of them
  because with the cfg off the type is unbound (rustc's E0425, a configuration that does
  not build and so cannot be validated against), so the port carries ONE name, the
  configuration in which the glob exists, records NO alternative, and the parser says so for
  the port (`a #[cfg]-gated glob is the only path the parser sees to this port's type …`,
  with `node_type=` / `port=` / `glob=` / `schema=`); `graph validate` describes that
  configuration only and refuses any other label as a plain disagreement, never a
  pass-with-warning; the buildable configuration is never refused for the sake of one that
  cannot build (pinned by `an_only_path_gated_glob_port_carries_one_name_at_validation`).
  Until
  the ambiguity is gone the port carries EVERY name (`PortDef::schema_alternatives`,
  spelled by the external arm's rule, nested segments kept) and `graph validate` accepts
  any of them as a pass-with-warning on the producer and consumer checks, never a
  refusal. The names are ONE map entry per leaf ident, so a nearer UNCONDITIONAL import
  shadowing that leaf replaces them all; an ancestor scope's alternative never outlives
  the import it belonged to (a nearer conditional import keeps the ancestor's name as an
  alternative instead: that is the shadowing shape). The use-root rules are measured
  rustc behaviour (2018 uniform paths), and each rejected simplification (crate-root
  modules only, any `mod` anywhere in the file, one flat map across scopes, ignoring the
  leading colon, dropping a cfg-gated module from the local set) fails its own
  `node_metadata` unit arm; the cfg classification of input rustc rejects outright
  (`#[cfg(not())]`, a `cfg_attr` the parser cannot read) is this parser's fail-open
  convention (`Conditional`, the permissive tier), not a rustc fact.
- **`graph validate`'s schema-disagreement remedy names a value only when it resolves.**
  The node's declared name is judged ONCE (`graph_cmd::declared_resolvability`, over
  `schema_cmd::port_schema_exists` and the nested sole-entry rule) and both checks render
  that verdict: resolvable → the literal `schema:` edit, naming the port and the node id;
  a bare name that is a NESTED workspace file's sole entry → the QUALIFIED edit
  (`schema: <pkg>/<Type>`), never "absent" of a type declared there; unresolvable → the
  remedy points at the node side (declare the type under `schemas/`, or pick one from
  `cerulion schema list`), never at a value the sibling resolvability check would refuse
  on the next attempt; resolver error → the reason is printed verbatim and nothing is
  claimed about resolvability. The producer check renders through the pure
  `graph_cmd::schema_disagreement_detail`; the consumer match appends the same verdict
  (`declared_verdict_suffix`, nothing when the name resolves; the label already names
  both sides). Both are oracle-tested without a workspace, over identical inputs.
- `proc_macro2::Literal::to_string()` preserves exact source formatting (`100_000`,
  `100u64`, `0xff`) and `str::parse::<u64>()` rejects all of them; use the in-module
  `parse_int_literal` helper, or `syn::LitInt::base10_parse` when a `LitInt` is in hand.
- `node modify` source rewriting is AST-based: `syn::parse_file` → collect
  `(start, end, replacement)` edits from `Span::byte_range()` → apply LAST-to-FIRST.
  Never `str::replace` (it also matches doc comments and string literals). Item walks
  must recurse into `Item::Mod`; field splices must check for a trailing comma.

---

## 6. Shell completions: design rules

`completions.rs` (value sources) + `crates/cerulion_cli/src/completion.rs` (shell adapters).

- **One mechanism**: `clap_complete`'s dynamic `CompleteEnv`, wired as the FIRST
  statement of `main()` from the same `Cli::command` factory the binary parses with;
  the whole tree completes for free and cannot drift. No static script is ever
  generated (a frozen snapshot cannot complete a live name, and sourcing both would
  leave whichever loaded last in charge).
- **Hard rule: a TAB press never hangs, never spawns a process, never opens the
  network.** Every source runs through `run_bounded` under `COMPLETION_BUDGET`; a zero
  remaining budget refuses to START work (an exhausted budget cannot buy a later source
  a fresh ceiling). Abandoning the helper thread at the deadline is sound only because a
  completion process exits immediately after. The exclusions (process spawn, daemon
  client, zenoh, mDNS, the expensive list/parse helpers) are enforced by a
  comment-stripped structural source walk in `tests/completions_test.rs`; it names the
  forbidden tokens with reasons and carries an anti-tautology arm.
- **Value sources**: topics from the local shared-memory service directory; node types
  from a `nodes/*` scan; graph names via `graph_cmd::graph_list` (so the offered set IS
  what `graph_read` resolves, `.yaml` only); schemas from workspace YAML stems + the
  `.msg` store walk + `BUILTIN_MSGS`, first-wins so a workspace copy shadows a built-in;
  robots from `~/.cerulion/robots.toml` + the TTL-bounded peer cache.
- **Wire-safety filter**: candidate values arrive from remote-supplied records and are
  rejected on newline (forges a second candidate on zsh's wire), whitespace (breaks the
  word bash inserts unquoted), `*`/`?`/`[` (re-globbed by bash's unquoted `COMPREPLY`
  command substitution), and `:` (bash's default `COMP_WORDBREAKS` makes readline
  re-insert the prefix). The rejected set is the UNION over both shells: one candidate
  stream feeds both, so a value only one shell mangles is still unsafe. HELP text keeps
  its colons (help reaches only zsh, where the renderer handles them; values are
  inserted by both shells and get the stricter rule).
- **Path args**: clap auto-derives `ValueHint::AnyPath` for `PathBuf`-typed args; a
  `String`-typed path arg completes nothing until given an explicit `value_hint`. The
  free-form inventory walk in `crates/cerulion_cli/src/completion_wiring_tests.rs` enumerates
  every value-taking arg that completes nothing and fails on any unclassified newcomer;
  the fix for a path arg is a `value_hint`, never an inventory entry.
- **Create verbs complete nothing** (`node|graph|schema create`): the existing name set
  is exactly what a create verb rejects.
- The zsh install hint carries a `compinit` guard (the generated script ends in
  `compdef`, a function `compinit` defines, and a stock macOS zshrc never calls it);
  the hint text is byte-pinned by a test.
- The `Completions` subcommand is excluded from the identity gate: it runs from shell
  rc files and must never block shell startup on an auth prompt.
- Accepted costs (deliberate, do not "fix"): a mirrored topic carries no help text
  (better than an affirmatively wrong `local` label), and a remote robot's topics
  complete only while something holds a mirror (demands are process-scoped).

---

## 7. Signal handling: `cerulion_cli`

One `setup_ctrlc_handler` closure drives graceful shutdown for every long-running verb,
and SIGTERM/SIGHUP reach it alongside SIGINT via the ctrlc crate's `termination`
feature, an EXPLICIT `cerulion_cli` dependency. That Cargo line is the declared
contract, not the enforcement: the same feature also arrives through the `cfg(unix)`
`cerulion_bagd` dependency's feature unification, so reverting only the explicit line
stays green, including `signal_matrix_e2e_test.rs`, which is the END-TO-END behavioral
floor (each signal → graceful exit EXACTLY 0), NOT a Cargo-line guard. The matrix
regresses only if BOTH sources of the feature vanish; if they do, SIGTERM/SIGHUP
silently fall back to their default disposition (terminate with a signal, never a clean
exit 0) on every deployment. Treat the explicit feature line and the bagd link as two
legs of one contract: dropping either is safe only while the other stands, so a
dependency-trimming change touching either must re-check the pair.

---

## 8. Test map: `cerulion_cli_engine`

All paths under `crates/cerulion_cli_engine/tests/`. "Serial" means run that binary
individually with `-- --test-threads=1`; everything else is parallel-safe within its
own binary.

| Test file | What it pins | Serial? | Prereq fixtures |
|---|---|---|---|
| `replay_engine_test.rs` | Replay engine e2e: every test crafts a bag from a reference run, replays with injected node factories, asserts against hand oracles (byte-identity via recorded-clock re-advance; violation taxonomy; the `--duration` bag-time bound; `--report` JSON; an rmw borrow-window gap-frame recording byte-diffs clean) | whole binary `#[serial]` (env mutation + empirically racy transport) | none |
| `replay_gates_test.rs` | Exit-4 pre-flight loudness (opaque-field tolerance refusal) | no | none |
| `integration_test.rs` | Engine integration incl. `pinned_hashes_match_generated_constants` (the hash-regen seam) | no | none |
| `graph_profile_iox2_test.rs` | `graph profile` e2e: artifact writes on happy/cap-hit/Ctrl-C paths, auto-derived targets, starved-node isolation, load-degrade classifier | yes (global iceoryx2 namespace) | `test_node_macro_period_cdylib`, `test_node_macro_data_trigger_cdylib` |
| `partition_emit_test.rs` | Surgical `process_groups:` splice, block scanner, `process_group_order` removal, atomic write + `.bak`; `node stage` over the untouched `graph create` scaffold takes no `.bak` and no warn, any other byte keeps both | no | none |
| `graph_partition_test.rs` | `graph partition` verb: cost-mode selection, consent ladder, replace-scoped validation | no | none |
| `graph_run_preflight_test.rs` | `resolve_partition_intent` matrix, consent ladder, lenient-costs degrade, in-memory==written plan equality | no | none |
| `network_run_gate_test.rs` | `resolve_run_network` decision matrix + gateway-port parse (env-mutating; file-local mutex inside) | no | none |
| `completions_test.rs` | Completion value sources vs hand-written candidate lists; wire-safety filter; structural no-side-effect walk | no | none |
| `discovery_ladder_test.rs` | Hermetic injected-rung ladder pins | no | none |
| `topic_network_live_test.rs` | Live loopback remote-topic e2e (real zenoh sessions, no shared memory) | no | none |
| `mdns_live_test.rs` | Real-multicast mDNS loopback | `#[ignore]` (hardware only) | none |
| `topic_observer_iox2_test.rs` | `topic echo`/`hz` observer loops: multiplicity oracle, timeout drain, rate-line math, notify-flood pacing counter | yes (process-global transport singleton) | none |
| `viz_attach_convergence_test.rs` | Viz attach convergence against a scripted fake daemon (served-request count is the oracle; deadline arm asserts the error KIND, never a wall band) | no | none |
| `convergence_adoption_test.rs` | Structural walk: the observer seams adopt the converged query verbs | no | none |
| `bag_record_run_attach_test.rs` | `bag record --run` mid-run attach over a real lapped live trace ring | no | none |
| `clean_orphan_port_tag_test.rs` | The orphan port-tag reclaim over a REAL shape (self-re-exec child on an isolated root): first sweep refuses, selector picks the node, reclaim removes the one tag, second sweep converges; stray-entry refusal, live-pid death guard, dry run; each anti-tautology arm paired with the regression it catches | yes (`#[serial]`; the diagnostics sweep pins the process-global iceoryx2 log level) | none |
| `ros2_cmd_test.rs` | The `cerulion ros2` pass-through plan builder: verbatim argv (never parsed or validated), exact env pairs (prepend order + separator), missing-rmw exit 69, the heap-hook preload matrix (`decide_preload` + composition: auto / off / none / explicit), ament-prefix staging (symlink, idempotence, stale refresh, installed layout), exec `NotFound` → 127; and the `--adopt-take` decision: both verbs REFUSE it (exit 69, before any file is inspected, on every host), the refusal is the LEADING flag only (a non-leading one still forwards verbatim), the direct-launch recipe it prints PREPENDS each path var to the sourced value (shell-quoted, with the expansion outside the quotes, asserted through a real `sh`) and is WITHHELD entirely for a path the loader cannot carry or that is not valid UTF-8, and the RETAINED gate keeps its own arms through a `test-seams`-gated seam | env-touching tests `#[serial]` | none |
| `ros2_migrate_test.rs` | `ros2 migrate` orchestration over an injected fixture engine + a synthetic colcon ws in a real temp git repo: dry-run byte-determinism, manifest lifecycle (decision-slot preservation; write consumes candidates), consent ladder, dirty refusal, `git apply -R` reversibility oracle, build-failure revert text, changed-file refusal | no | none |
| `ros2_resim_no_respawn_test.rs` | Structural walk (comment-stripped): no replay-path module references the ros2 spawner or builds a `ros2` command; a resim classifies-and-skips `ros2:` entries, never respawns them | no | none |
| `system_deps_manifest_test.rs` | Structural walk: every REAL `[package.metadata.cerulion.optional-system-deps]` block in the repo parses and is complete. A walk rather than a hand list on purpose: a hand list reproduces a failure this repo has already had, where a sweep missed the one node that mattered because nothing enumerated the set being swept; a node that adds a declaration is covered here by construction, with no test edit. A failure here means `cerulion node build <that node>` would refuse: a malformed declaration is a hard error by design. The decision core is oracle-tested in `cerulion_cli_engine::system_deps` against hand-written manifests | no | none |

`ros2_graph.rs` carries its own unit oracles (the `classify_child_exit` matrix, relative
launch/params resolution + missing-file refusal). Metric math + decodability
classification live as unit tests inside `tolerance_metrics.rs` /
`replay_field_registry.rs`.

## 9. Test map: `cerulion_cli`

| Test file | What it pins | Serial? | Prereq fixtures |
|---|---|---|---|
| `tests/replay_cli_test.rs` | Exit-code contract over the real binary (exit-6 mapping; the exit-3 execution arm via a panicking twin cdylib) | yes | `test_node_macro_period_cdylib`, `test_node_macro_period_perturbed_cdylib`, `test_node_macro_period_panic_cdylib`, `test_node_nondeterministic_cdylib` |
| `tests/mp_record_e2e_test.rs` | Multi-process `--record` bag contracts (one bag, per-rank manifests, departure sentinel, ring sweep) | yes | `test_node_macro_period_cdylib`, `test_node_macro_data_trigger_cdylib` |
| `tests/mp_auto_partition_e2e_test.rs` | Multi-process-by-default consent ladder over the real binary (no-TTY floor, persist, opt-out, refusal never mutates the file) | yes | same two |
| `tests/mp_split_pair_e2e_test.rs` | The mid-level barrier's PLUMBING over the real binary: classify -> stamp -> serialise -> install, read off each worker's own build line. Deliberately NOT the ordering discriminator (that is deterministic only in-process); what it buys is that the extra generation neither desynchronises a real deployment nor loses frames, and that two live runs record byte-identical frames. | yes | `test_node_macro_period_cdylib`, `test_node_macro_period_input_cdylib` |
| `tests/network_gateway_e2e_test.rs` | Permissive gateway lifecycle: notice exactly once, child reaped on SIGINT/SIGTERM, graceful-forward discriminator (the gateway's own shutdown line, not just exit 0 + reap) | yes | same two |
| `tests/network_gateway_mp_e2e_test.rs` | Strict networked multi-process acceptance (worker → SHM → gateway → zenoh → external) | yes | same two |
| `tests/signal_matrix_e2e_test.rs` | SIGINT/SIGTERM/SIGHUP each exit EXACTLY 0 (`code()==None` would mean a default-disposition kill) + repeat-SIGINT idempotency, the behavioral floor for the §7 signal contract | yes | `test_node_macro_period_cdylib` |
| `tests/viz_interrupt_e2e_test.rs` | A SIGINT'd viz attach exits 0 with no absence-claim vocabulary | no | none |
| `tests/ros2_run_e2e_test.rs` | Both `cerulion ros2` verbs over the real binary with a fixture `ros2` first on PATH: exit-code INHERITANCE through `exec()` (42 → 42) on EACH verb, LEADING-hyphen verbatim forwarding (the raw-argv intercept pin), the heap-hook auto-inject + kill switch + explicit preload, the staged env + ament symlink, and the 69 / 127 / 2 contract; plus the `--adopt-take` refusal over the real binary (both verbs, exit 69, `ros2` never exec'd, the cause and the direct-launch recipe on stderr, with a flagless control that still execs; and that the refusal is decided BEFORE any staging, so it creates nothing) and the pin that `cerulion ros2 --help` says the flag is REFUSED rather than that it arms the child; no ROS 2 needed | no (per-child env only) | none |
| `tests/ros2_graph_e2e_test.rs` | `ros2:` graph entries over the real binary: spawn on `rmw_cerulion` with the declared argv, graceful SIGINT fan-out recorded by the child + no orphan, `--peer-loss fail` stops the run / `continue` runs degraded, missing rmw refuses before spawning, `validate` + `levels` accept the mixed graph, the multi-process default arm; the record→resim round trip (a mixed bag resims exit-0, the skip warn + report JSON name the entry, no respawn); plus an `#[ignore]`d real-`demo_nodes_cpp` machine arm with its recipe | yes | `test_node_macro_period_cdylib` |
| `tests/ros2_migrate_cli_test.rs` | `ros2 migrate` over the real binary with a stub engine script + stub `colcon` on the child PATH: the production `ClangToolEngine` spawn path, write/commit/patch + the exact colcon argv, failing-build exit 1 naming the revert commit, engine-absent exit 69, `--yes`-without-`--write` usage error | no (per-child env only) | none |
| `tests/completions_cli_test.rs` | Zero-stderr completion protocol under a hostile env (trace-level logging, dead daemon socket); bare separator-delimited candidates only | no | none |
| `src/completion_wiring_tests.rs` | Wired-completer inventory (set equality + spelled-out create-verb guard), free-form inventory walk, `.mcap` path filter; run via `cargo test -p cerulion_cli --bin cerulion` | file-local mutex | none |
| `src/clean_diagnostic_tests.rs` | `cerulion clean`'s wiring as a source walk (the real thing deletes from the developer's `/tmp`): the state-file diagnostic is called, sweep-before-diagnostic order, the convergence gate as a whole expression, the refusal listing between breakdown and unclassified arm, the orphan port-tag reclaim between exactly two sweeps with `--report-only` as its dry-run bit and the SECOND sweep's convergence handed to the gate, plus hand-oracle pins of the two pure renderers. Run via `cargo test -p cerulion_cli --bin cerulion` | no | none |

## 10. `cerulion clean`: dead-node sweep, orphan port-tag reclaim, state-file gate

`cerulion clean` runs iceoryx2's dead-node sweep with its trace lines captured
(`ipc_cleanup::cleanup_dead_iceoryx2_nodes_with_diagnostics`), attributes every
refusal to its node with the sub-causes iceoryx2 logged, and reclaims
`.shm_state` name mappings only when the sweep left the registry CONVERGED: a
still-registered dead node needs its mappings, and one removed underneath it
can never be reaped again.

One refusal shape is healed rather than reported. A publisher destroyed while
one of its loaned samples had been leaked deregisters its port but leaves the
port's `.port_tag` under `<root>/nodes/<id>/` (the tag is owned by the
publisher's shared state, which every forgotten sample keeps alive until the
process dies). The sweep then reclaims the port's resources, never deletes the
tag, removes the `.details` storage, and fails the final `rmdir`, every sweep,
forever. `orphan_port_tags::orphan_port_tag_candidates` selects a refused node ONLY
when its variant is `InternalError` and its sub-causes are exactly that
four-line chain with the quoted directory equal to the registry's own
`<node_dir>/<id>`; `orphan_port_tags::reclaim_orphan_port_tags` then removes the tags
only if the recorded pid is provably gone (`shm_state::creator_verdict`, the one
`kill(pid, 0)` predicate the state-file reclamation trusts; the reclaim names no
liveness type of its own) and the directory (re-listed at that instant, never
from the sweep's memory of it) holds nothing but regular files named
`<prefix><port id><port-tag suffix>`. Anything else refuses the whole directory
and names the offenders. The verb prints one line per node, runs ONE more
sweep, prints its summary, and hands the SECOND sweep's convergence to the
state-file gate. `--report-only` still runs the FIRST dead-node sweep (iceoryx2's
own reclaim of a dead node's resources), prints the
candidates the reclaim WOULD act on, removes no port tag, and skips the second
sweep; "removes nothing" is true of the reclaim, not of the sweep. The reclaimer heals a machine that already carries the shape; the
rmw destroy path is what stops it being minted. Extend the
refusals, never the acceptance.

## 11. Workspace dependencies and compiler compatibility

`workspace create` writes root `[workspace.dependencies]` by the BINARY's location
(`find_cerulion_base` from `current_exe`, then baked `CARGO_MANIFEST_DIR`), never
cwd: checkout builds use absolute `path` deps, others exact registry pins. Exposed
as `CerulionWorkspace::dependency_source`; nodes inherit `{ workspace = true }`,
user overrides rewritten on recreation.

New workspaces pin the CLI's stable `RUSTC_RELEASE` in `rust-toolchain.toml`
with the minimal profile only after installed-only `rustup run` verifies its
full `RUSTC_FINGERPRINT`. Missing or mismatching compilers warn and preserve
ambient selection; workspace creation never downloads a toolchain. Preserve
existing TOML or legacy toolchain files and symlinks without probing; warn when
preserving a choice or when nightly or beta metadata cannot name a toolchain.
Never change rustup defaults.

Write and sync the toolchain manifest to a sibling staging file before publishing
it with a no-replace hard link. Failed writes leave no partial final manifest;
concurrent choices win. Clean up the staging file and propagate filesystem errors,
including filesystems without hard-link support; never fall back to overwriting.

`node build` probes PATH `rustc` only for an advisory warning. Cargo owns compiler
selection, including environment and project configuration overrides. The built
cdylib must match the host's full compiler fingerprint, checked at load before init.

## 12. The login gate

Every command runs under a logged-in-ever identity. `command_needs_identity` in
`crates/cerulion_cli/src/main.rs` exempts `login`, `completions` and the two
internal `graph run-worker` / `run-gateway` subprocess verbs; clap's `--help` and
`--version` and the usage refusals `main` performs before the gate call answer
above it and need no exemption. `ensure_login_gate` in
`crates/cerulion_cli_engine/src/login_cmd.rs` reads local state only: a machine
that signed in once proceeds with zero network, offline and on an expired
session. A machine that never signed in runs the device-code flow inline when
stderr and stdin are both terminals, and otherwise refuses at once with exit 7
rather than starting a ten minute poll nobody is watching.

The gate is on in every build, released or built from source. One escape exists
for this repository's own runs: `CERULION_LOGIN_GATE` set to exactly `off`. The
match is byte for byte, so `0`, `false` and `OFF` leave the gate on. It is set
in two places and nowhere else: the workspace `.cargo/config.toml`, which covers
everything cargo starts including the binaries tests spawn, and a workflow `env:`
in the few jobs that run the binary outside cargo. The handful of shell and
python harnesses that launch the binary themselves export it at the top. No user
documentation names it.

A test whose subject is the gate removes the variable on the command it spawns
and points `CERULION_ACCOUNT_SERVICE` at a closed loopback port, so it meets the
gate the way a user's machine does and reaches no network:
`crates/cerulion_cli/tests/login_gate_e2e_test.rs` is the whole contract over the
real binary, and `a_malformed_resim_is_refused_before_the_login_gate` in
`replay_cli_test.rs` pins the one ordering property. A test that needs an
identity rather than an absent one provisions it:
`cerulion_cli_engine::auth::seed_logged_in_at` from Rust, or
`tools/ci/seed_test_login.sh <dir>` from a shell, both writing the `auth.json`
a real sign-in writes, with `CERULION_HOME` pointed at the directory.
