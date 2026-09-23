# Repo-wide gates: CI jobs, lint policy, and the source walks

Contracts for the checks that apply to the whole tree rather than to one crate. Read
before changing `.github/workflows/ci.yml`, the root `Cargo.toml` lint table, `clippy.toml`,
or anything under `tools/scripts/`.

## Lint job budget

The `Lint` job has a 45-minute limit covering runner setup, cache restore,
compilation, the checks themselves, and the cache save. A cold cache restore can
take a quarter of an hour on its own, so a tighter budget expires inside Clippy
and the job dies before it saves a cache, which makes the next run pay the same
restore again. The limit bounds this job alone; which checks are required is set
elsewhere in the workflow.

## Lint policy: one table, inherited

The lint levels live in exactly one place: `[workspace.lints]` in the root `Cargo.toml`.
Every workspace member inherits them with:

```toml
[lints]
workspace = true
```

Cargo refuses to mix `workspace = true` with crate-local lint keys, so a crate either
inherits the whole table or opts out visibly. A crate that needs one extra lint declares it
in its own source (`#![deny(unsafe_code)]` at the crate root, as the macro crate does)
rather than forking the table.

`crates/cerulion_cli_engine/tests/workspace_lints_manifest_test.rs` fails, naming the crate and
the fix, if a member does not inherit, and pins the table's levels so they cannot be
weakened silently. It walks THREE universes, because no one of them sees the others:

- the root's DECLARED `[workspace] members`;
- what cargo actually RESOLVES (a path dependency is folded into the workspace whether or
  not the members list names it, so the resolved set can be wider than the declared one,
  and a crate reachable only that way is still subject to the gates);
- each MIRRORED workspace's own members (`examples/go2` carries a byte-equivalent table,
  because `workspace = true` resolves against whichever workspace is LOADING).

## The print ban

`#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]` is carried by
every library crate cargo resolves into the root workspace, by the library members of each
mirrored workspace (`examples/go2` node crates ship to a robot, which is the class the rule is
written for), and by both node scaffold generators, so a crate created tomorrow inherits
it. Pinned by `crates/cerulion_cli_engine/tests/library_print_ban_test.rs`.

Binaries and test binaries are exempt: printing is a bin's job, and a test printing evidence
under `--nocapture` is normal. A genuine exception takes a targeted
`#[allow(clippy::print_stdout)]` (or `print_stderr`) with a written reason: the pre-tracing
bootstrap paths, machine-parseable state lines that are a command's output protocol, and
probe markers a parent test process reads.

`clippy.toml` also bans `dbg!` and `std::process::exit` repo-wide, each with a reason text
an offender reads. Only a process entrypoint may exit.

## The tracing-message walk

`crates/cerulion_core/tests/tracing_field_discipline_test.rs` walks every library `src/` tree in
the workspace and fails on a message that interpolates a runtime value, and on a
near-spelling of a standard field name. Contracts worth knowing before editing a log line:

- The `%field` / `?field` Display/Debug shorthand IS the field declaration; the walk reads
  it as one. A raw identifier field (`r#type = %ty`) is read under the name tracing records
  (`type`).
- `tracing::event!(Level::X, ..)` is covered alongside the five level macros; the LEVEL
  argument is skipped so it is not read as a field.
- `target:` / `parent:` / `name:` are DIRECTIVES, not messages; the field behind them is
  still read.
- A `{SCREAMING_CONST}` capture is exempt on EVIDENCE, not on casing: the name must resolve
  to a `const`/`static` declared in the walked tree, and a `static` with interior mutability
  (`Atomic*`, `Mutex`, `RwLock`, `Cell`, `OnceLock`, `LazyLock`, `OnceCell`) does not count
  (an atomic rendered as `{X:?}` is runtime data wearing a constant's spelling). The
  declaration's type is read from that declaration only.
- Declared debt is pinned per file by MESSAGE (whitespace-collapsed, truncated, compared as
  a multiset), never by a count and never by a line: a count is blind to REPLACEMENT, and a
  message survives the line drift a line number churns on.
- Two documented blind spots, each with its own arm: a `macro_rules!` TEMPLATE's message is
  not read (what runs is the expansion), and a non-literal message (`warn!(concat!(..), v)`)
  is REPORTED rather than guessed at.

## The hot-path allocation lint

`./tools/scripts/check_hot_path_allocs.sh` runs in the `lint` job alongside its own `--self-test`
step. It scans `crates/cerulion_core/src/transport/`, `.../graph/` and `.../scheduler/`, each minus
a reasoned exclusion list; a new file in a scanned directory is linted BY DEFAULT.

The pattern set covers collection BUILDING as well as the obvious constructors:
`.collect(` / `.collect::<`, `.to_owned()`, the map/set/deque constructors (`HashMap::new`,
`String::new`, …), `.extend(` and `.reserve(`, alongside `vec!` / `Vec::new` / `format!` /
`.clone()`. It is an over-approximation on purpose; the answer to a cold hit is an
annotation with a reason.

`.push(` is DELIBERATELY absent, because a push into a PREALLOCATED buffer allocates
nothing. The scanned files push into buffers this codebase CLEARS AND REUSES
(`drain_scratch`, `scratch_heads`, `fired_idx`, the per-level `decision_slots`), which is
the exact shape the level executor USES to stay zero-alloc, and into
`TraceRingProducer::push`, whose wait-free zero-alloc path has its own regression test.
Matching `.push(` would therefore demand a wall of annotations that all say "this buffer is
warm", which is the reflexive annotating this gate exists to avoid. `.extend(`/`.reserve(`
do not have that property (`reserve` is written precisely when the author INTENDS to
allocate), which is why they are matched and `.push(` is not, pinned by three self-test
arms, including the anti-tautology half that a `.push(` into a reused buffer is NOT a
finding.

Three annotations, and the `<reason>` is ENFORCED rather than conventional: a bare marker
is a hard error, not a silent pass:

| Annotation | Scope | Meaning |
|---|---|---|
| `// hot-path-alloc-ok: <reason>` | one line | that line is cold |
| `// hot-path-alloc-ok-fn: <reason>` | one `fn` | the whole function is cold, for one shared reason |
| `// hot-path-alloc-known: <reason>` | one line | a REAL hot-path allocation |

A `-known:` does not fail the build but is printed on every run under a
`KNOWN HOT-PATH ALLOCATIONS` banner carrying a count; a new hot allocation is
a finding to report, not to annotate away. A second banner, `ANNOTATIONS ON UNMATCHED
LINES`, lists annotations sitting on a line the pattern set does not match, where the
marker arms NOTHING, so the annotation is either STALE or a genuine blind spot. Run the
script to read both counts for the tree as it stands: an entry under the second banner
means one of those two things. A `-known:` is never swallowed by an enclosing `-ok-fn:`;
the function form exists for a function whose allocations are cold for one shared reason, so
a `-known:` inside it is the author saying that reason does not hold at that line.

Two structural rules on the `-ok-fn:` scope, both LITERAL-AWARE (a line that begins inside a
multi-line string is text, not code):

- the terminator is a comment-stripped byte-compare, so a trailing comment (`} // end of
  foo`, which `cargo fmt` preserves) cannot close the scope on the next sibling's brace;
- a `fn` signature at the annotated function's own indentation while its body is still open
  is a loud OVERRUN error, and a bodiless `fn` declaration is a hard error.

Scope is decided by literal-aware INDENTATION rather than brace counting: this tree's
multi-line string literals desynchronize a per-line brace counter. The literal mask drives
neither the pattern scan nor the annotation grammar, so it cannot manufacture a finding; a
false "inside" only defers a scope decision, and an unclosed scope is still a hard error at
EOF.

The complete argument list is validated, so no malformed invocation can read as a passed
self-test: an unrecognized argument exits 2, and so does anything after `--self-test`.

## The serialisation fence (`.config/nextest.toml`)

Tests run under `cargo nextest run`, which gives each test its OWN PROCESS, and the shard
script passes no `--test-threads` flag. Process-per-test is why a package-wide
serialisation is not needed: a fresh process carries its own cdylib node registry,
environment, global `tracing` subscriber and allocator probe, and the large majority of
these binaries mint per-test SHM roots (`init_for_test` / `generate_isolated_config` /
`build_for_test`) and share no namespace with anything. iceoryx2's shared-memory singleton
constrains only the binaries that touch the DEFAULT namespace.

What genuinely cannot run beside a sibling is fenced BY NAME, in two parts that use two
different mechanisms ON PURPOSE:

| Fence | Mechanism | Members | Why this mechanism |
|---|---|---|---|
| `default-namespace` | a test GROUP with `max-threads = 1` | binaries that create or enumerate services on the DEFAULT iceoryx2 namespace | they collide with each other, and only with each other |
| wall-clock timing gates | `threads-required = "num-cpus"` | the latency gates | they need a QUIET MACHINE, not protection from one particular sibling; a mutex group would let the rest of the suite run alongside and skew the measurement |

**Membership is GATED, not trusted.** `crates/cerulion_core/tests/serial_discipline_test.rs` reads
the config and asserts fence membership equals a declared inventory in BOTH directions, and
separately requires every singleton-CREATING test file in a nextest package to be a member,
so a new one fails until it is classified. It also checks that every fenced binary names a
test file that exists.

**Doctests are the hole this closes.** nextest does not run doctests at all, so they still
run under `cargo test --doc`, where a doctest can take neither `#[serial]` nor a fence
entry. The same gate therefore walks every `src/` doctest and fails if an executing one could
reach the singleton.

Per-package steps outside the shard still carry `-- --test-threads=1` individually where the
package needs it.

## The naming gate

`tools/scripts/check_pr_title.sh` runs in the `lint` job on every pull request (locally:
`--title` / `--branch`, or `--self-test` to drive its oracle table). It enforces
`<type>(<scope>)!: <description>` for the title (comma-separated scopes accepted) and
`<type>/<kebab-slug>` for the branch: no personal prefix, and no tracker id baked into a
branch name: a branch name is permanent and public, so the id belongs in the pull request body, where
it links and stays editable. The same rule applies to the title by CONVENTION, which the
script does not check. The script can exempt a head branch that cannot be renamed
(`GRANDFATHERED_BRANCHES`), as exact `(PR number, name)` pairs: renaming a pull request's head
branch closes it, so a listed branch is accepted verbatim until it merges (the list is
empty); the pair is required because a head ref is fork-controlled text
while the PR number is minted here once, so the same name on any other PR, or with no
`--pr` at all, still fails.

## The agent-docs gate

`tools/scripts/check_agents_md.sh` runs in the `lint` job. Context files are loaded by coding
agents with SILENT truncation and nearest-file-wins chaining, so the repo must be the alarm.
It enforces: per-file size budgets (root `AGENTS.md` <= 170 lines and <= 11500 bytes; every
other <= 60 lines and <= 4096 bytes), a whole-chain byte budget, a `CLAUDE.md` shim beside
every `AGENTS.md` (crate shims contain exactly `@AGENTS.md`), no symlinked context file (a
symlink dodges every check while tools happily read the target), and a banned-token scan
over every `AGENTS.md`, the root `CLAUDE.md` addendum and `docs/internals/*.md`.

The scan has three tiers. Generic shape-based patterns ship in the script. Name-class tokens
(people, machines, internal tools, private repos) load from an untracked file
(`$AGENTS_TOKENS_FILE`, else `.agents-tokens.local`): the list is itself internal
vocabulary, so it never ships in the script; when the file is absent that tier is skipped
with an INFO line and public CI enforces the generic tier only. `grep` exiting >= 2 is a
VIOLATION naming the pattern, so a malformed pattern cannot report a file as clean it never
searched. The third tier holds the names themselves: machine names, logins and retired
personal branch prefixes come from the leak guard's private pattern list, read by the leak
guard's own loader (`LEAK_PATTERNS`, then `LEAK_PATTERNS_FILE`, then the default file under
the user's config directory), and the gate's host-identity rule scans the operator files
(`tools/scripts/*.sh`, `.github/workflows/*.yml`) for the same machine names plus two shipped
address shapes. When no list is loaded the gate prints a NOTE and skips that tier; the leak
guard's private tier covers the same content where it runs. An unreadable list or an entry
that does not compile is a VIOLATION.

## What the shipped tree may carry

Three properties of the tree are shapes no vocabulary pattern can see, so they are tests
rather than gate classes. All three are in
`crates/cerulion_hygiene/tests/shipped_surface_structure_test.rs` (pure: a walk over the
tree, no cargo, no network), and each names the document it is the machine half of.

| Test | What fails it |
|---|---|
| `no_published_crate_can_package_an_agent_instruction_file` | A publishable member whose `include` list reaches `AGENTS.md`, `CLAUDE.md`, `.claude/`, `notes/` or a design note or running log left at a crate root, one of those names sitting inside a packaged directory, or a member that declares no `include` at all (cargo then packages whatever the directory holds). The positive control is that `README.md` and `src/lib.rs` ARE packaged, so a crate cannot pass by including nothing. |
| `no_source_or_test_file_is_named_after_a_plan_step` | A file under a `src/` or `tests/` directory whose name carries the step of the work that produced it: a `chunk` segment, an indexed `pass2` / `stage3` / `phase1` / `wave2` / `round4`, or a leading lane label like `d2_`. An index is what separates a step from a word, so the `node stage` verb and a `round_trip` are silent, and `e2e_` is not a lane. Seven names the tree still carries are declared in the test; the list may only shrink, and a name that leaves the tree has to leave the list. |
| `the_documented_login_exemptions_are_the_ones_the_code_exempts` | `docs/user-api.md`'s "Exempt: `login` …" sentence and `command_needs_identity` in `crates/cerulion_cli/src/main.rs` naming different verbs. The code side is read from a comment-stripped view of the function, so a variant mentioned in a comment beside the list does not count as a member of it. `--help` and `--version` are declared separately: clap answers them above the gate, so they are asserted PRESENT in the sentence rather than compared against the code. |

The fourth doc-versus-code property, that every `cerulion <verb>` spelled in `README.md`
and `docs/user-api.md` exists in the CLI, is enforced by the public-surface gate's
`docs-refs` class, which walks the verb tree out of the clap definitions; its self-test
carries a markdown TABLE row, because a table cell is where those two pages spell their
verbs.

## The leak guard

`tools/scripts/leak_scan.py` keeps machine names, addresses, home paths, logins, people and
location metadata out of everything the repository publishes: file contents, path and branch
names, commit messages and identities, pull request text, and media containers. It runs in
three places. The `lint` job runs its self-test and then a whole-tree scan with the GENERIC
classes only (`--no-private`, because that job holds no secret); both steps block CI.
`.github/workflows/leak-guard.yml` runs with the private tier as a repository secret mapped
at step level:
changed files and added names, commit messages plus pull request title and body (through
environment variable names, never interpolated), and media metadata, on every pull request,
merge queue batch, push to `main` and manual run. The git hooks (`tools/hooks`, installed by
`tools/scripts/install_hooks.sh`) run the same scanner before a commit exists.

The two-tier shape is the agent-docs gate's, with one stricter output contract: a private hit
prints the file, the line number and the pattern INDEX and nothing else, a generic hit on a line
a private pattern touches in any view prints no matched text, every printed path is redacted, and
every printed line is stripped of line breaks and escapes, so a CI log does not publish a name
the secret knows and no path can start a workflow command (the forge masks a secret only as a
whole string; a regex match is never a registered secret). The scanner's own source assembles every
matchable literal from fragments and scans itself to zero with no allowlist entry, a self-test
arm pinned both ways. "Found something" and "could not run" never share an exit code, a
built-in control per class must hit before any scan, and an allowlist entry that matches no
file or excused nothing fails a full-tree run. Contributor-facing detail: `docs/leak_guard.md`.

## The public-surface review (`.github/workflows/public-surface-review.yml`)

`tools/scripts/check_public_surface.sh` refuses the wording it can NAME. A regex cannot
see narrative, a stale claim, or a sentence that contradicts another page, and an
independent audit of this tree found all three in files every pattern passed. This job
reads the pull request's DIFF for those.

It runs the Claude Code action on the same terms as the Claude Code Review workflow: the
same pinned action, the same three gates (a maintainer author, a head branch in this
repository so a fork never holds the token, and not a draft), `show_full_output: false`,
and no transcript artifact. It is smaller in every other way. The prompt is fixed and in
the tree at `tools/review/public-surface-review.md` (the eleven families of tell, the
vocabulary that is legitimate and must be judged rather than reported, and the four
severities LEAK, EMBARRASSING, CONFUSING, COSMETIC). The diff is computed by a step and
handed over as a file, so the model is allowed `Read`, `Grep`, `Glob` and `Write` and no
shell at all. It writes ONE JSON verdict and posts nothing.

`tools/review/render_public_surface_verdict.py` reads that verdict, renders the one pull
request comment, and decides the exit, so the outcome never depends on the session
choosing to call a tool: clean is a silent pass, a LEAK or EMBARRASSING finding FAILS,
advisory findings pass with the comment, and a missing or unparseable verdict FAILS,
because a semantic review that did not happen looks exactly like a clean one. Its
`--self-test` drives every arm, including a quoted pipe that must not add a table column
and a blocking finding past the table's row cap.

ADVISORY until the maintainers add it to the required checks on `main`: a red here is a
review to read, not a block. COST: one ubuntu job and one model session per push, over
the diff rather than the tree, capped at 20 minutes, billed to the same OAuth account as
the Claude Code Review workflow, and cancelled when a new push supersedes it.

## The docs gate

`RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps` is a blocking job, and the
failure it actually catches is a broken intra-doc link: linking a `pub(crate)` or
submodule item from a `pub` item's docs is the recurring one. It is not obvious locally,
so run it before pushing any `src/` doc-comment change.

`cargo doc --no-deps` does NOT process `tests/*.rs` module docs, so editing a test file's
`//!` header cannot break this gate, a useful thing to know before being cautious about
the long doc comments this repo's test files carry.

## The login gate in CI

Every `cerulion` command runs under an account, in every build. CI machines have
no account, so this repository's own runs set `CERULION_LOGIN_GATE=off`, matched
byte for byte. It lives in the workspace `.cargo/config.toml`, which covers every
job that goes through cargo, and in a workflow level `env:` for the jobs that run
the binary outside cargo: the release install smoke and the perception replay
demo. A container gets it from the script that runs inside the container, because
a workflow `env:` does not cross `docker run`. Same for the shell and python
harnesses under `tools/` and `benches/` that launch the binary themselves.

That list used to live only in this paragraph.
`crates/cerulion_hygiene/tests/shipped_surface_structure_test.rs` now holds it as
`GATE_CARRIERS` and asserts each entry really sets the variable to `off`, and it walks
`.github/workflows/`, `tools/` and `benches/` for a command whose head is the binary: a
file that runs it and is in neither list fails, naming this section. A match the detector
cannot tell from a command (a sentence inside a multi-line string, the public-surface
gate's own fixture) is recorded in `NOT_AN_INVOCATION` with its reason, and a line there
that stops matching fails too. `cerulion --version` is not an invocation for this purpose:
clap answers it above the gate.

A job that wants a machine WITH an account instead seeds one:
`tools/ci/seed_test_login.sh <dir>` writes the `auth.json` a real sign-in writes,
and `CERULION_HOME=<dir>` points the gate at it. The file's shape is pinned
against the type it has to parse as by a unit test in
`crates/cerulion_cli_engine/src/auth.rs` that runs the script.

## CI job map (`.github/workflows/ci.yml`)

`lint` runs: `cargo fmt --all --check` plus a workspace-root WALK
that fmt-checks the workspaces outside the root (`examples/go2`, every `benches/*`, every
`examples/*`, the fuzz workspace); `cargo clippy --workspace --all-targets -- -D warnings`;
the hot-path alloc lint and its self-test; the agent-docs gate; the leak guard's self-test
and generic-class tree scan; the naming gate (pull requests only); `shellcheck` over
`tools/scripts/**` recursively and over the extensionless hooks in `tools/hooks` (`-type f`
deduplicates a symlink into a scanned subdirectory); and `actionlint` over every workflow.

EVERY job runs on a GitHub-hosted runner, and the macOS jobs run on `pull_request` and
`merge_group` events like everything else: there is no cost gate, no routing expression
and no stub job standing in for a skipped required check.

`lint` gates the jobs that do NOT set the wall (`docs`, `netd-wan`, `crate-tests`,
`viz-tests`, and the push-only `fuzz`, `miri` and latency jobs), so a red `lint` still
saves their runner minutes. It does NOT gate the three that do: `test-archive`,
`test-linux` and `test-macos` start at t=0. A `lint` verdict was never a data dependency
for them, and while it gated them the wall was `lint` plus the longest test job instead of
the longest test job. `test-linux` keeps `needs: [test-archive]`, which IS a data
dependency: it runs the binaries that job builds.

`test-linux` is 4-way SHARDED (`strategy.matrix.shard: [0,1,2,3]`) and `test-macos` is
3-way (`[0,1,2]`); both `fail-fast: false`. The macOS count is set from per-step
measurement: under the earlier 2-way split the legs ran 28.3 and 46.0 min with a warm
cargo cache and 57.4 and 55.1 with none, so one leg set the wall of the whole workflow
while the other idled, and the skew INVERTED with the cache state (the pinned trybuild
tail costs 6 min warm against 18 cold, so a hand tilt tuned on either column is wrong in
the other). A third leg divides the variable work by 3 while the fixed per-leg cost
(`cargo build --workspace`, toolchain, nextest install) is paid once more, which is
better in both cache states: 26.6 min warm and 42.4 cold for the longest leg. Each leg
runs `./tools/scripts/ci_test_shard.sh cerulion_core <shard> <count>`, which ENUMERATES
`crates/cerulion_core/tests/*.rs` at depth 1 and takes every file whose position is
`index mod count`, GENERATED, never hand-listed, save for ONE pinned name
(`macro_compile_fail_test`, the serial trybuild tail (see `PINNED_TEST` in that script for the
per-run measurement), which must not relocate every time a test
file is added; it lands on `PINNED_SHARD % count`, shard 2 of 4 on Linux, shard 2 of 3 on
macOS, and `--check` proves the pin), and execs `cargo nextest run --profile ci`
(install via `tools/scripts/install_nextest.sh`). It does NOT pass `--test-threads=1`; see the
serialisation fence below. The split across runners is legal because each VM has its own
`/dev/shm`.
`--lib` and the doctests ride shard 0; the non-core packages are distributed across the
shards (one per shard on Linux; a hand-balanced 3-way tilt on macOS), with the iroh-tree
packages kept together so that large tree compiles once. On macOS, shard 2 also carries the
six viz steps (`cerulion_viz`, `go2_tf`, the serial `cerulion-vizd` suite and the three
OpenH264 steps) beside the trybuild tail, which is why it takes the lightest package set;
there is NO macOS `viz-tests` leg. Linux shard 0 is the cache SAVER and
shards 1-3 restore only; the macOS job's shard 0 saves and shards 1-2 restore, for the same
reason. Each sharded job's `shard:` matrix is held to the count its shard step passes by
`ci_test_coverage_test.rs`.

`test-linux` does not build the `cerulion_core` test binaries four times: `test-archive`
builds them once with `cargo nextest archive`, uploads the archive as a run-scoped
artifact with its `sha256` as a job output, and each shard downloads it, verifies the hash
before extracting, and RUNS the archive rather than compiling one.

`crate-tests` is the catch-all lane: one `cargo test -p <package>` step for each package
no other job runs, which is what keeps every package inside a blocking job. `viz-tests`
runs the visualization tree in two lanes, Linux only: the library lane PARALLEL (every
real-iceoryx2 file claims an isolated per-test SHM root, so parallel is the stronger
gate), the daemon lane SERIAL. `machete` (unused-manifest sweep) and `fuzz` are non-blocking
(`continue-on-error: true`), which is why the coverage walk refuses to count a package named
only there (`miri` is a blocking job; it has no `continue-on-error`). Also: `examples`,
`demos-go2`, `netd-wan`, `iroh-leanness`, `rerun-leanness`, `docs`, `deps` (cargo-deny,
blocking), and the release-mode latency jobs (push to main + `workflow_dispatch` only, never
on PRs).

Six Linux jobs run on push / `workflow_dispatch` only and never
on a `pull_request` event: `deb-smoke`, `cross-aarch64-linux`, `msrv`, `fuzz`, `miri` and
`machete` each carry a job-level `if: github.event_name != 'pull_request' && github.event_name
!= 'merge_group'`; the second
conjunct is required because a bare `!= 'pull_request'` ADMITS a merge-queue batch,
which would run the same work a second time over the same commits (`main`'s push run is the
control), with their `needs: [lint]` (`fuzz`, `miri`) and
`continue-on-error: true` (`fuzz`, `machete`) untouched. None of the six is a required
status context on `main`, so a skipped one is simply absent from a pull request's checks.
They run on every merge to `main` (the push run is where their breakage
surfaces, revert-on-red), and the coverage walk drops any job behind a job-level `if:`
from its PR-blocking view, so none of the six can credit pull-request coverage it does not
provide.

EVERY test step names its PACKAGES explicitly; there is no blanket `cargo test --workspace`
on the root workspace, which makes coverage a hand list.
`crates/cerulion_cli_engine/tests/ci_test_coverage_test.rs` is the walk that holds it, counting only
steps and jobs that can actually fail a pull request, and it drives
`tools/scripts/ci_test_shard.sh --check` to prove the shard partition is TOTAL and DISJOINT, so a
broken split is a local red rather than a silent gap.

Vendored in-repo bash is deliberate over marketplace actions: a third-party action is code
this repo executes with its own token, and the checks here are a regex and a few `wc` calls.

Release tags must set `CITATION.cff`'s `date-released` to a valid UTC date in the
inclusive range from the tagged commit's UTC date through the release run's UTC date.
The release workflows reject missing, malformed, impossible-calendar, pre-tag, and
future dates before publishing artifacts. The shared implementation is
`tools/scripts/check_citation_release.sh`, which is also exercised by the release-gate
regression script.

## Test map

| Test | What it pins | Serial? |
|---|---|---|
| `tools/scripts/check_citation_release.sh` | citation version, calendar, and release-date window validation | n/a |
| `crates/cerulion_cli_engine/tests/workspace_lints_manifest_test.rs` | every member inherits the one lint table; the table's levels | no |
| `crates/cerulion_cli_engine/tests/library_print_ban_test.rs` | every library crate carries the print ban | no |
| `crates/cerulion_cli_engine/tests/ci_test_coverage_test.rs` | every package runs in a blocking job; the shard partition is total and disjoint | no |
| `crates/cerulion_core/tests/tracing_field_discipline_test.rs` | no interpolated log message; no near-spelled field name | no |
| `crates/cerulion_core/tests/serial_discipline_test.rs` | nextest fence membership equals its declared inventory both ways; every singleton-creating file is fenced; no executing doctest reaches the singleton | no |
| `tools/scripts/check_hot_path_allocs.sh --self-test` | the annotation grammar and scope rules | n/a |
| `tools/scripts/check_pr_title.sh --self-test` | the title/branch oracle table | n/a |
| `tools/scripts/check_agents_md.sh` | context-file budgets, shims, banned tokens | n/a |
| `tools/scripts/leak_scan.py --self-test` | every generic class on every surface, the redacted private output contract, exit codes, allowlist and pragma rules, the self-scan | n/a |
| `tools/scripts/install_hooks.sh --self-test` | the hooks refuse a planted leak and a planted message, pass a clean commit, cover a worktree without `tools/hooks`, and uninstall cleanly | n/a |
