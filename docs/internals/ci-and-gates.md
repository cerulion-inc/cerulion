# Repo-wide gates: CI jobs, lint policy, and the source walks

Contracts for the checks that apply to the whole tree rather than to one crate. Read
before changing `.github/workflows/ci.yml`, the root `Cargo.toml` lint table, `clippy.toml`,
or anything under `tools/scripts/`.

## Lint job budget

The `Lint` job has a 45-minute limit covering runner setup, cache restore,
compilation, the checks themselves, and the cache save (restore-only under the
default save policy below; the save runs when `CACHE_SAVE_NAMESPACES` names
`Linux-lint` on a default-branch run). A cold cache restore can take a quarter of an hour on its own, so
a tighter budget expires inside Clippy and the job dies before it saves a cache,
which makes the next run pay the same restore again. The limit bounds this job alone; which checks are required is set
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
It enforces: per-file size budgets (root `AGENTS.md` <= 190 lines and <= 13000 bytes; every
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

The fourth doc-versus-code property, that every `cerulion <verb>` spelled in `README.md` and
`docs/user-api.md` exists in the CLI, is enforced by the public-surface gate's `docs-refs` class,
which walks the verb tree out of the clap definitions; its self-test carries a markdown TABLE row,
because a table cell is where those two pages spell their verbs. That gate's `agent-file-ref`
class refuses `AGENTS.md` and `CLAUDE.md` on a user-facing page, excepting files so named.

## The dependency-architecture rules

Which crate may depend on what is an architecture decision, and each one was written as a
sentence: in `.github/workflows/ci.yml`, in the root `Cargo.toml`, in a crate manifest
header, in a crate `AGENTS.md`, or in `docs/internals/`. The machine half of every sentence
is `crates/cerulion_hygiene/tests/dependency_rules_test.rs`, so an optional, renamed or
transitive edge that breaks one fails `cargo test`. Each test quotes the sentence it
enforces.

| Test | What fails it |
|---|---|
| `default_member_build_is_iroh_free` | A `cargo build` with no `-p` reaches the iroh tree. `cerulion_netd`, `cerulion_link`, `cerulion_wireclient`, `cerulion_remoted`, `cerulion_connectd` and `cerulion_accountd` are workspace members and not default members for this reason, and lean consumers of netd depend with `default-features = false`. |
| `netd_per_package_build_pulls_the_iroh_wan_plane` | `cargo build -p cerulion_netd` stops reaching iroh, which means the shipped daemon lost the WAN plane that its `wan` feature carries by default. Also the positive control for the rule above: the two differ only in the root set. |
| `a_plain_cargo_test_compiles_no_iroh_and_no_rerun` | A `cargo test` with no `-p` reaches either tree. It walks each default member's own `[dev-dependencies]` as well, which is the other half of the sentence the root `Cargo.toml` states. |
| `default_member_build_is_rerun_free` | A `cargo build` with no `-p` reaches the Rerun SDK. Rasterization is desk-side, so `cerulion_viz` and `cerulion_vizd` build with `-p`. |
| `desk_viz_still_pulls_rerun` | `cargo build -p cerulion_viz` stops reaching rerun, which means either the desk viz stack lost its render edge or the rule above probes nothing. |
| `the_robot_demo_workspace_is_rerun_free` | Any `examples/go2` crate takes a rerun dependency of any kind. Read from that workspace's committed manifests and lockfile, so dev edges, build edges and every platform count. |
| `the_robot_workspace_reader_sees_normal_and_dev_packages` | The lockfile reader stops seeing `go2_tf`, or the manifest reader stops seeing `tempfile`, which `dds_bridge` declares only under `[dev-dependencies]`. A dev edge is how the Rerun SDK reached the robot the one time it did, so that is the arm worth a control of its own. |
| `the_dds_stack_is_confined_to_cerulion_dds` | A second workspace member declares a DDS dependency, or the default build reaches the DDS stack along a route that avoids `cerulion_dds`. |
| `the_lean_crates_declare_exactly_their_allowed_dependencies` | `cerulion_discovery` declares anything but serde, serde_json, dirs and tracing, or `cerulion_hygiene` anything but libc and tracing. Exact in both directions: an allowance nobody removed pre-authorises the next edge. |
| `the_confined_crates_reach_nothing_they_forbid` | `cerulion_pairing` reaches iroh or rerun, `cerud` reaches iceoryx2 or zenoh, or `cerulion_link` reaches `cerulion_core`. |
| `the_heavy_members_stay_out_of_default_members` | One of the eleven deliberate `default-members` exclusions is back in the default set, or a member left the default set with no row saying why. Checked in both directions, and each row names what a plain build would gain. |
| `the_forbidden_families_exist_in_this_workspace` | A family pattern matches no package in the resolve at all, which would let every rule written over it pass while proving nothing. |
| `the_checker_reports_a_forbidden_crate_when_one_is_present` | The one checker every rule above calls stops reporting a violation under roots that carry one. |
| `the_resolver_covers_every_package_cargo_tree_reports` | The dependency closure the rules are stated over misses a package `cargo tree -e normal` reports, which is a place a forbidden crate could sit unseen. A floor on the reported count keeps a broken `cargo tree` invocation from satisfying it with silence. |

Thirteen of the rules are stated over `cargo metadata --format-version 1`, run once per test
binary; the fourteenth runs `cargo tree` as its oracle. The
emitted `resolve.nodes` graph is not walked directly: it carries one feature set per package,
unified across every member that selects it, so netd appears there with `wan` on and iroh
attached even though the default build reaches netd through an edge that says
`default-features = false`. The file runs cargo's feature algorithm itself from a chosen set
of roots instead, and uses `resolve.nodes` only to map a manifest dependency onto the package
id cargo picked for it. No target filtering is applied, so a `cfg(windows)` edge counts and
the verdict is the same on every machine. The default-build rules walk normal AND build
edges, because a build dependency on a forbidden crate compiles that crate during a
`cargo build` exactly as a normal one does.

Two `cargo tree` jobs in `.github/workflows/ci.yml` used to assert five of these rules and
are retired, because each test that replaces one is strictly stronger: the tests walk build
edges as well as normal ones, apply no target filter, and read the robot workspace off its
committed lockfile and manifests, which covers every edge kind and every crate the demo
reaches by `path`.

## The leak guard

`tools/scripts/leak_scan.py` keeps machine names, addresses, home paths, logins, people and
location metadata out of everything the repository publishes: file contents, path and branch
names, commit messages and identities, pull request text, and media containers. It runs in
four places. The `lint` job runs its self-test and then a whole-tree scan with the GENERIC
classes only (`--no-private`, because that job holds no secret); both steps block CI.
`.github/workflows/leak-guard.yml` runs with the private tier as a repository secret mapped
at step level:
changed files and added names, commit messages plus pull request title and body (through
environment variable names, never interpolated), and media metadata, on every pull request,
merge queue batch and push to `main`. Its three job names are required contexts and every
run writes a check run under each of them, so it triggers on those three events and on no
other. `.github/workflows/leak-guard-conversation.yml` runs the
`Leak guard (issue and comment bodies)` job on `issues`, `issue_comment` and
`pull_request_review_comment`, scanning the body the event carries; that name is required
by nothing, which is what lets it run on those events. The git hooks (`tools/hooks`,
installed by `tools/scripts/install_hooks.sh`) run the same scanner before a commit exists.

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

`lint` gates the four push-only jobs that depend on it (`fuzz`, `miri`, `test-latency` and
`cli-e2e-latency`), which no pull request and no queued batch runs; a red `lint` saves their
runner minutes, and none of the four reports a required context. It gates none of the four
dependants that DO report one (`docs`, `netd-wan`, `crate-tests`, `viz-tests`): each carries
the `!cancelled()` guard, which replaces the implicit `success()` over the whole `needs:` set;
GitHub offers no per-dependency form, so all four run and report through a red `lint`, and the
dependency buys the ordering alone. A skipped required context counts as satisfied, which is
what the guard is there to stop. It does NOT gate the two that set the wall: `test-linux` and
`test-macos`. Both list `changes` and nothing else in `needs:`, so both start after the
classifier, which is a checkout and a path classification, about a minute, and no build. No
REBUILD waits on it: each shard builds the `cerulion_core` test binaries it runs, so nothing
in front of either job is a data dependency for compilation. A `lint` verdict was never one
either, and while it gated them the wall was `lint` plus the longest test job instead of the
longest test job.

`test-linux` is 4-way SHARDED (`strategy.matrix.shard: [0,1,2,3]`) and `test-macos` is
3-way (`[0,1,2]`); both `fail-fast: false`. The macOS count is set from per-step
measurement: under the earlier 2-way split the legs ran 28.3 and 46.0 min with a cache hit
on the default-branch run 35666419690, so one leg set the wall of the whole workflow while
the other idled, and the skew turns over when the cache misses (the pinned trybuild tail
costs 6 min with a cache hit and is the largest single step of a cold leg, so a hand tilt
tuned on the cache-hit column is wrong on a miss; the cache-miss measurement of that split
was taken but its run id was not kept, so its numbers are not cited). A third leg divides
the variable work by 3 while the fixed per-leg cost (`cargo build --workspace`, toolchain,
nextest install) is paid once more, which is better in both cache states: a longest leg
PROJECTED from those per-step costs at 26.6 min warm; the eight default-branch runs of
2026-09-27 put the observed macOS shard walls at 29 to 44 min, cache state not recorded. Each leg
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

Each `test-linux` shard builds the `cerulion_core` test binaries it runs: the shard step
compiles exactly the quarter `tools/scripts/ci_test_shard.sh` assigns to it and then runs
what it built. There is no shared archive of test binaries and no job that builds one.

`crate-tests` is the catch-all lane: one `cargo test -p <package>` step for each package
no other job runs, which is what keeps every package inside a blocking job. `viz-tests`
runs the visualization tree in two lanes, Linux only: the library lane PARALLEL (every
real-iceoryx2 file claims an isolated per-test SHM root, so parallel is the stronger
gate), the daemon lane SERIAL. `machete` (unused-manifest sweep) and `fuzz` are non-blocking
(`continue-on-error: true`), which is why the coverage walk refuses to count a package named
only there (`miri` is a blocking job; it has no `continue-on-error`). Also: `examples`,
`demos-go2`, `netd-wan`, `docs`, `deps` (cargo-deny,
blocking), and the release-mode latency jobs (push to main + `workflow_dispatch` only, never
on PRs).

Six Linux jobs run on push / `workflow_dispatch` only and never
on a `pull_request` event: `deb-smoke`, `cross-aarch64-linux`, `msrv`, `fuzz`, `miri` and
`machete` each carry a job-level `if: github.event_name != 'pull_request' && github.event_name
!= 'merge_group'`; `deb-smoke` carries one exception, below; the second
conjunct is required because a bare `!= 'pull_request'` ADMITS a merge-queue batch,
which would run the same work a second time over the same commits (`main`'s push run is the
control), with their `needs: [lint]` (`fuzz`, `miri`) and
`continue-on-error: true` (`fuzz`, `machete`) untouched. None of the six is a required
status context on `main`, so a skipped one is simply absent from a pull request's checks.
They run on every merge to `main` (the push run is where their breakage
surfaces, revert-on-red), and the coverage walk drops any job behind a job-level `if:`
from its PR-blocking view, so none of the six can credit pull-request coverage it does not
provide.

The `changes` job classifies a pull request's changed paths (rules and a
`--self-test` table in `tools/scripts/ci_changed_paths.sh`, executed by `lint`) into four
outputs. `packaging` only ever makes a job RUN that would otherwise skip: a pull request
that touches the packaging inputs themselves runs the 22-minute Debian and APT smoke,
because those are the only pull requests that can break it and "caught on the merge to
main" means a revert rather than a red check. EVERY job that `needs:` the classifier opens
its job-level `if:` with `!cancelled()`, not `deb-smoke` alone: `needs:` by itself lets a
failed classifier skip a dependant, and a skipped required context reads as satisfied.
`test-linux`, `test-macos`, `crate-tests` and `viz-tests` carry the bare call; `deb-smoke`
carries it in front of its own event gate, so it keeps its `push` run whatever the
classifier did. `cerulion_cli_engine::ci_test_coverage_test` holds the rule over every
dependant, however the job consumes the outputs, rather than over the jobs with a one-line
selection gate alone. The `changes` job probes the base it resolved before the diff reads
it: an empty base turns `$BASE...HEAD` into a range over HEAD alone, which lists no path and
selects nothing, so the probe refuses it and fails the job;
`the_selection_job_probes_the_base_before_the_diff_reads_it` in the same test binary pins the
probe, its refusal and their order ahead of the diff in the script text.

`code`, `docs` and `pkgs` are the test-impact selection, and they run in the other
direction: they SKIP test steps. Four rules bound them.

* PULL REQUESTS ONLY. On `push`, `merge_group` and `workflow_dispatch` every package is
  selected. The queue run is the last gate before `main` and the one place a miss has no
  later catch.
* ONE OFF SWITCH. The repository variable `CI_SELECTION` reaches the classifier through
  the workflow-level `env:` block; `off`, and any value the classifier does not know,
  selects every package. Nothing else may read it: a step is gated on the classifier's
  OUTPUT, never on the variable, and `ci_test_coverage_test` refuses any other shape.
* STEPS, NEVER JOBS. Every job still runs and still reports its own required context. A
  gated step carries the one condition the coverage walk credits,
  `contains(fromJSON(needs.changes.outputs.pkgs), '<package>')`, and a companion step
  under the exact negation of that condition prints one line beginning `selection:`, so
  the log says what was skipped and why.
* THE SELECTION IS WIDER THAN CARGO. `pkgs` is the reverse cargo dependency closure over
  normal, build and dev edges UNIONED with the observation edges in
  `tools/ci/observation_edges.tsv`: a test that reads another package's tree, walks the
  repository, or loads an artifact another package builds reaches it without a manifest
  edge. That table is derived from the sources by
  `crates/cerulion_cli_engine/tests/ci_doc_pin_walk_test.rs`, which fails on a missing row
  and on a stale one. A read the walk cannot place on one package, an unattributable literal or
  a walk over a tree holding more than one member, records the observing package as observing
  `all`; such a package rides every selection that names a package, and no step of it is gated.
  Four packages are in that state today, `cerulion_core` among them, which is why the
  `cerulion_core` shard steps carry no condition of their own: the shard runner reads the
  selection itself and prints the `selection:` line when it skips.

The supported subset, stated plainly: the selection narrows PER-PACKAGE test steps on pull
requests. It does NOT narrow the workspace build, it does not gate a job, it does not apply
to any event but `pull_request`, and it never removes a required status context.

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

## The cache save policy (`CACHE_SAVE_*` in `ci.yml`)

The Actions cache store has a 10 GB free allowance; storage above it is billed, and while the
account carries a failed payment the service refuses every save into a store above 10 GB
(measured 2026-09-28: 13 refusals into a 15.8 GB store, then one accepted save into an emptied
one). One generation of the eight namespaces measured is 17.1 GB (the rmw lanes and the smaller
tool caches are not in that figure), so the workflows save only what fits and pays: the macOS
test shards' archive (4.27 GB, shard 0 of `test-macos`; shard 0 ran 28.3 min with a cache hit on
run 35666419690, against 29 to 44 min across eight 2026-09-27 runs whose cache state is not
recorded) and the Linux test shards' archive (3.18 GB, shard 0 of `test-linux`; 19 to 28 min
across runs 36284081039, 36295731007 and 36357164260), 7.45 GB together, on `main` only. Every other `actions/cache/save` step is gated off by default and its job is
restore-only.

The policy is two workflow-level `env` values, each defaulting from a repository variable:

* `CACHE_SAVE_NAMESPACES` (`${{ vars.CACHE_SAVE_NAMESPACES || 'macOS Linux' }}`): the
  space-separated namespace tokens that may save, or `all`. A token is the key text between
  `cargo-` and the scope segment, with any `${{ steps.*.outputs.* }}` segment removed: `macOS`,
  `Linux`, `crates-Linux`, `viz-vizd-Linux`, `Linux-lint`, `examples-replay-Linux`, and
  `rmw-distros-jazzy` for `cargo-rmw-distros-jazzy-<header hash>-<scope>-<lockhash>`, whose
  header hash is a cache generation rather than part of the namespace. The default is the
  expression's literal, so a repository with no variable saves the measured frontier and a
  misspelt variable matches nothing.
* `CACHE_SAVE_ON_PULL_REQUEST` (`${{ vars.CACHE_SAVE_ON_PULL_REQUEST }}`): empty means saves
  run on `main` only; any value lets pull-request runs save into their own `-pr-` scoped keys
  as well, and only from a branch of this repository. A fork pull request's token cannot hold
  `actions: write`, so the prune's delete would fail the job; the clause tests
  `github.event.pull_request.head.repo.full_name == github.repository`.

Before every save of a lockfile-keyed archive (the machete tool cache is exempt, below)
`tools/scripts/ci_cache_prune.sh "<the save key>"` runs: it keeps exactly that key and deletes
every other entry of the namespace (stale lockfile generations, `pr` scoped entries, legacy
unqualified keys), so the store never holds two archives of one namespace and a lockfile change
costs one generation, never two. The jobs that prune carry
`permissions: {contents: read, actions: write}`; nothing else does.

The prune's gate is WIDER than the save's, deliberately: the branch clause and the merge-queue
clause, never the namespace list. Every default-branch run of a job that has a prune step
prunes, whether or not its namespace may save, and keeps exactly that job's current key even
when no entry under that key exists -- so a restore-only namespace is emptied rather than left
holding archives nothing will ever replace. Nothing else reclaims them: a pull request's restore
refreshes an entry's last-access time, so GitHub's seven-day idle eviction never fires on a
stale archive that pull requests keep reading.

Three rules inside the script keep a prune from deleting what another run still needs.

* **The tip check.** On `refs/heads/main` the script reads the branch tip once and deletes
  nothing unless the tip is this run's own commit. Two runs on `main` overlap all the time, and
  the older one would otherwise delete the generation the newer one had just saved -- and then
  skip its own save as an exact hit, leaving the namespace empty. A run behind the tip deletes
  nothing and saves nothing: the prune step writes `proceed=false` to its `$GITHUB_OUTPUT` and
  every save step tests `steps.cache-prune.outputs.proceed == 'true'`, because a run that skipped
  the prune and uploaded anyway would leave two generations of its namespace resident (14.90 GB
  across the two shard namespaces, over the allowance, with the next save refused). The next tip
  run prunes what it left.
* **The clock guard.** The prune records the UTC second it began -- before the tip check, which
  is a network round trip a newer run can save during -- and deletes only entries created strictly
  earlier than that. An entry that appeared while the prune was
  running belongs to a run ahead of this one; it is kept and named in the log.
* **The scope rule.** A keep key scoped `-pr-` may delete only `-pr-` entries of its namespace:
  a pull-request prune must never take the `-main-` archive or the legacy unqualified entries,
  which are what every other run restores. A `-main-` or unscoped keep key deletes every
  shape-matching entry of the namespace, which is the point of the prune. Identity is (key,
  ref), because GitHub stores one entry per pair and under the pull-request override every open
  branch holds its own entry under the same key string.

`lint`, `docs` and `netd-wan` build far smaller targets than the shards and used to share the
shards' `cargo-Linux-` key, so whichever finished first saved it: on 2026-09-28 the entry under
the shards' key was a 0.63 GB archive saved by one of the smaller jobs, not the shards' 3.18 GB
one. They now have their own namespaces (`cargo-Linux-lint-`, `cargo-Linux-docs-`,
`cargo-Linux-netd-wan-`). Under the default all three are restore-only and fall through their
`restore-keys` to `cargo-Linux-main-`, the shards' archive, a superset of what they need;
`test-linux` shard 0 restores and saves its own key; the shards 1 to 3 restore it.

Two exemptions, both narrow. The machete binary cache (`~/.cargo/bin/cargo-machete`, keyed on
the pinned version and not on a lockfile hash) is a TOOL cache: a few MB with no lockfile
generation for the prune to work from, so it carries no namespace gate and no prune step, only
the cache-hit test and the merge-queue test. Gating it into a namespace the default never names
meant the binary was never cached and `cargo install cargo-machete` ran on every push.
`release.yml` restores and never saves: a tag run's entry is restorable by no other ref, so
saving one would only add to the store.

`tools/scripts/ci_cache_policy_check.py` holds every workflow in `.github/workflows` to this
contract -- the directory, not a hand list of the files known to cache, because two workflows
were writing into the same store with the combined `actions/cache` action while a two-file
invocation reported clean. The rules: the gate names the key's namespace (R1); it carries the
main-only clause with its fork condition (R2); the prune step directly precedes the save with
the byte-identical key under `id: cache-prune`, an explicit sweep prefix is exactly the key text
before the generation segment, and the save's condition is the prune's own plus the cache-hit test
in front and the prune's `proceed` output and the namespace gate behind (R3); the job permission is declared (R4); the default literal is
unchanged (R5); a save is never reachable from a merge-queue run (R6); the save is the job's
last step (R7); every gate is a conjunction of clauses from a closed set, each at most once,
with no top-level `||` (R8 -- which is what stops `... || true` from turning a gate off while
every other rule still passes); a container job installs `gh` and `jq` in the step before its
prune, under the prune's own condition (R9); nothing shadows the policy in a job- or step-level
`env` (R10); a prune whose keep key carries no `-main-`/`-pr-` scope segment -- one the scope rule
cannot narrow, so it clears the whole namespace -- lives only in a job whose `if:` can never be
true on a pull request, either a `github.event_name != 'pull_request'` conjunct or an allowlist of
events naming none (R11); a key with no lockfile hash is a tool cache or a violation (TOOL_CACHE);
the
combined `actions/cache` action appears nowhere (NO_COMBINED_CACHE_ACTION); and a job whose
steps the reader cannot enumerate -- a reusable-workflow call, or one carrying no `steps:` -- is
refused rather than passed in silence. The `Lint` step "Cache save policy" runs the prune
script's self-test and the checker's self-test, both of which flip every rule from both sides,
before the checker reads the real files.

## Test map

| Test | What it pins | Serial? |
|---|---|---|
| `tools/scripts/check_citation_release.sh` | citation version, calendar, and release-date window validation | n/a |
| `crates/cerulion_cli_engine/tests/workspace_lints_manifest_test.rs` | every member inherits the one lint table; the table's levels | no |
| `crates/cerulion_cli_engine/tests/library_print_ban_test.rs` | every library crate carries the print ban | no |
| `crates/cerulion_cli_engine/tests/ci_test_coverage_test.rs` | every package runs in a blocking job; the shard partition is total and disjoint; a step gated on a changed-path selection still runs on the change that selects only its own package. A selection condition counts only where it is GROUNDED: the job `needs:` the classifier, the classifier declares the output, and that declaration is exactly `${{ steps.<id>.outputs.<name> }}` naming a step of it that can set an output OF THAT NAME: a `run:` step whose script writes `<name>=` into `$GITHUB_OUTPUT`, or a `uses:` step, whose action's outputs are not in the file to read. A literal value, an expression carrying another operand, a step that writes no output, and a step that writes some other output's name each ground nothing; the `changes` job probes its resolved base, with a refusal that fails the job, ahead of the diff that lists the changed paths; and a step in a `container:` job whose `run:` script uses a bash-only construct (`pipefail`, another `set -o` option, or the `[[ ... ]]` conditional, never a POSIX class `[[:...]]`) declares `shell: bash`, because a container step with no `shell:` runs the image's `/bin/sh`, which is dash on the ROS base images. The walk prints the count of container `run:` steps it judged and fails on zero. And a job whose name reports a required status context skips on no event its workflow triggers on, the steps that split its work by event together admit every such event, it carries `!cancelled()` whenever it lists `needs:`, and its workflow triggers on no event outside the allowed set. The required names are read from `tools/ci/required_contexts.txt` | no |
| `crates/cerulion_cli_engine/tests/ci_doc_pin_walk_test.rs` | the `# doc-pin:` markers in `ci.yml` equal, both ways, the shared-root reads derived from every workspace member's `tests/*.rs` and `src/**/*.rs`: a string literal rooted at `docs`, `tools`, `.github`, `benches` or `examples`, or a root markdown file name, that the surrounding code opens or joins as a path, never one it only names, writes, or joins onto its own crate directory. A `src/` read is attributed to the library test binary (`<package>::<package>`). A path assembled at run time, or reached through a helper in the crate's library, is NOT seen: that is a stated limitation, and `cerulion_core::serial_discipline_test`'s shell-script reads are the known case | no |
| `crates/cerulion_core/tests/tracing_field_discipline_test.rs` | no interpolated log message; no near-spelled field name | no |
| `crates/cerulion_core/tests/serial_discipline_test.rs` | nextest fence membership equals its declared inventory both ways; every singleton-creating file is fenced; no executing doctest reaches the singleton | no |
| `tools/scripts/check_hot_path_allocs.sh --self-test` | the annotation grammar and scope rules | n/a |
| `tools/scripts/check_pr_title.sh --self-test` | the title/branch oracle table | n/a |
| `tools/scripts/check_agents_md.sh` | context-file budgets, shims, banned tokens | n/a |
| `tools/scripts/ci_selected_packages.py --self-test` | the reverse CARGO-DEPENDENCY closure of a set of touched packages, over hand-built metadata documents and over this workspace, with normal, build and dev edges followed, a renamed dependency keyed by its package name, and an unknown name refused. A document that is not an object carrying `packages` (a list), `workspace_members` (a list) and `version` is refused with exit 2 and one line naming the field, in every mode including `--all`, so no caller reads an empty selection as the answer. It does NOT prove that the selected set is everything a change can break: see below | n/a |
| `tools/scripts/leak_scan.py --self-test` | every generic class on every surface, the redacted private output contract, exit codes, allowlist and pragma rules, the self-scan | n/a |
| `tools/scripts/install_hooks.sh --self-test` | the hooks refuse a planted leak and a planted message, pass a clean commit, cover a worktree without `tools/hooks`, and uninstall cleanly | n/a |

A `# doc-pin:` marker is a YAML comment in `ci.yml`, of the form
`# doc-pin: <package>::<test binary> reads <root>, <root>`, recording that the
named test binary opens a path outside its own crate; it changes no step and no
condition, and it sits beside the step that runs its package so a rule deciding
which test steps a change needs can find it there.

## What the selection proofs cover, and what they do not

`tools/scripts/ci_selected_packages.py` is the reverse CARGO-DEPENDENCY closure:
given the packages a change touches, it prints those packages plus every
workspace member that depends on one of them through a normal, build or
dev-dependency edge. That is what it proves. It validates the metadata document
before any mode, `--all` included, and refuses one that is not an object
carrying `packages` (a list), `workspace_members` (a list) and `version` with
exit 2 and one line naming the field. The refusal is about the DOCUMENT, not the
answer: a well-formed document whose workspace has no members legitimately
prints `[]` at exit 0, and what no caller can get is an empty selection read out
of a document the script could not parse. A test can observe another package
with no dependency edge at all: by opening a path literal into that package's
tree, by walking the whole repository, or by loading an artifact built from that
package at run time (`dlopen`). The closure sees none of those.

The doc-pin walk covers one of those classes: every test binary that opens the
shared documentation and tool trees (`docs`, `tools`, `.github`, `benches`,
`examples`) or a root markdown file is pinned, both ways, against the markers in
`ci.yml`. Cross-crate source literals and dlopen fixtures are an open class. They
have to be pinned before any CI test step is gated on the selection.
