# Repo-wide gates: CI jobs, lint policy, and the source walks

Contracts for the checks that apply to the whole tree rather than to one crate. Read
before changing `.github/workflows/ci.yml`, the root `Cargo.toml` lint table, `clippy.toml`,
or anything under `tools/scripts/`.

## Android compile check

`Android check (arm64-v8a)` builds (compiles and links) the 17 runtime crates for
API 30 with NDK 27.2.12479018 and cargo-ndk 4.1.2, on pull requests and
merge-queue runs. The job is selected when the path classifier
includes any of those packages and builds all 17 in one Cargo invocation. It is
not a required context: `tools/ci/required_contexts.txt`
mirrors the externally configured branch-protection and merge-queue checks, which
this workflow does not change. Its cache namespace is `android-arm64`; the default
save policy is restore-only until that namespace is opted in.

## Android emulator e2e

`Android emulator e2e (x86_64, API 30)` builds the CLI, netd, the basic timer
workspace, and the Android app, then runs them on a real API 30 Google APIs
emulator. It decodes values from device and host topic echo, checks recorded
MCAP frame counts and schemas, and checks paired and open app access. Pull
requests and pushes to `main` run when Android-affecting paths change. The
workflow also runs nightly at 06:13 UTC because a path filter cannot see a
regression introduced by a shared dependency outside those paths. It is not a
required context. Its `android-emulator` Cargo cache restores by default and
saves only when the cache policy opts that namespace in.

## Period park lanes

`Period park (x86_64 Linux)` and `Period park (aarch64 Linux)` build, lint and
test the period-learned park of `rmw_cerulion` (cfg `cerulion_period_park`),
one on `ubuntu-latest` and one on the hosted `ubuntu-24.04-arm` runner, on every
run of the CI workflow: pull requests, merge-queue runs, pushes to `main` and
manual runs. Every Linux aarch64 build carries the park; on x86_64 only
`--features period-park` does. Each leg runs clippy with `-D warnings` over all
targets with the feature on and off, then over the bare lib with and without it
(the only build of the lib without `test-seams`), then the crate's tests
serially with the feature on. On the aarch64 leg the feature on and off compile
the same park, so there those steps differ only in the cfg rule's own test. The aarch64 leg then runs the tests again with
default features and `CERULION_RMW_PERIOD_PARK=0`, which turns the learned park
off at run time, since no other job tests the one-rung idle park that the
switch restores on aarch64 Linux. The steps run when the path classifier
selects `rmw_cerulion`. This is the only job that runs clippy on Linux aarch64,
where `c_char` is `u8` rather than `i8`, so an `as` cast between `c_char` and
`u8` lints there and nowhere else. Both legs are required contexts (see
`tools/ci/required_contexts.txt`). Its cache namespaces are `period-park-X64`
and `period-park-ARM64`; the default save policy is restore-only until they are
opted in.

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
a reasoned exclusion list, and all of `crates/cerulion_tf/src/` (the frame math a node tick calls);
a new file in a scanned directory is linted BY DEFAULT.

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
`--pr` at all, still fails. The second exemption is the forge's own update namespace: a
head branch of the shape `dependabot/<ecosystem>/<name>` is accepted when the pull
request author login is exactly `dependabot[bot]`, which `ci.yml` passes from the event
payload as `github.event.pull_request.user.login` (set when the pull request opens;
`github.actor` is whoever pushed the run). The script's author list is matched whole-line
and fixed-string, so a login containing it or contained by it grants nothing, and the branch
name is what the exemption covers: the title is still checked.

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
because a table cell is where those two pages spell their verbs. The same class resolves every
relative link before it judges it: a target that resolves to nothing is a finding whatever its
spelling, and one of the `ALL_CAPS_WITH_UNDERSCORE` placeholder shape says in its own message
that no file in the tree resolves the token. A target held by a maintainer ruling is named line
by line in `tools/scripts/public_surface_allow.txt`, and its row is deleted by the change that
replaces the token, because a row that excuses nothing fails the run. That gate's `agent-file-ref`
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
| `the_lean_crates_declare_exactly_their_allowed_dependencies` | `cerulion_discovery` declares anything but serde, serde_json, dirs and tracing, `cerulion_ops_proto` anything but serde, serde_json, thiserror, base64 and sha2, or `cerulion_hygiene` anything but libc and tracing. Exact in both directions: an allowance nobody removed pre-authorises the next edge. |
| `the_confined_crates_reach_nothing_they_forbid` | `cerulion_pairing` reaches iroh or rerun, `cerud` reaches iceoryx2 or zenoh, or `cerulion_link` reaches `cerulion_core`. |
| `the_permissive_crates_reach_no_agpl_crate` | A workspace crate licensed `MIT OR Apache-2.0` reaches an AGPL workspace crate over normal or build edges, or one of the four documented permissive crates stops declaring that license. A control requires the same check to report `cerud` under `cerulion_remoted`. |
| `the_heavy_members_stay_out_of_default_members` | One of the eleven deliberate `default-members` exclusions is back in the default set, or a member left the default set with no row saying why. Checked in both directions, and each row names what a plain build would gain. |
| `the_forbidden_families_exist_in_this_workspace` | A family pattern matches no package in the resolve at all, which would let every rule written over it pass while proving nothing. |
| `the_checker_reports_a_forbidden_crate_when_one_is_present` | The one checker every rule above calls stops reporting a violation under roots that carry one. |
| `the_resolver_covers_every_package_cargo_tree_reports` | The dependency closure the rules are stated over misses a package `cargo tree -e normal` reports, which is a place a forbidden crate could sit unseen. A floor on the reported count keeps a broken `cargo tree` invocation from satisfying it with silence. |

Fourteen of the rules are stated over `cargo metadata --format-version 1`, run once per test
binary; the fifteenth runs `cargo tree` as its oracle. The
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
file or excused nothing fails a full-tree run. The reference classes carry that separation
onto the network: the `HEAD` they send to the forge's repository endpoint, up to three per
reference, is a verdict only when it is answered (200 clean, 404 or 410 a `ref-unopenable`
finding), and a throttle, a server error, a transport error or a timeout is retried, three
attempts with waits of 1.5 s then 3.0 s between them, and then recorded as NOT QUERIED
with the status or the error kind the last attempt saw. The hits and the summary print
first either way: a run that otherwise reads OK exits `3` and asserts no leak, and a run
already reading `FAIL` on its own evidence keeps that exit beside those lines.
`leak-guard.yml` and
`leak-guard-conversation.yml` each set `GITHUB_TOKEN` as a workflow-level env, which
reaches every step of every job in them and lifts the probe's hourly budget off the 60 an
anonymous runner address shares; the scanner's reference probe is the only reader of that
variable under `tools/`. WHO ASKS DECIDES THE ANSWER: a job token is scoped to the
repository the workflow runs in, so for every other repository it is a stranger and a 404
there is the not-found verdict, while a personal token used locally is not a stranger and
can read clean what the job reads as a leak. Contributor-facing detail:
`docs/leak_guard.md`.

## The docs gate

`RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps` is a blocking job, and the
failure it actually catches is a broken intra-doc link: linking a `pub(crate)` or
submodule item from a `pub` item's docs is the recurring one. It is not obvious locally,
so run it before pushing any `src/` doc-comment change.

`cargo doc --no-deps` does NOT process `tests/*.rs` module docs, so editing a test file's
`//!` header cannot break this gate, a useful thing to know before being cautious about
the long doc comments this repo's test files carry.

The `cerulion` binary of `cerulion_cli` sets `doc = false`. The umbrella library crate has the
same name, rustdoc writes both to `target/doc/cerulion`, and the two outputs race under
`cargo doc --workspace` (cargo issue 6313): the gate then fails at random with missing files.

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
`GATE_CARRIERS` and asserts each entry has a line of its own that sets the variable to
`off` for its own process (the same words inside a quoted command run on another host do
not count), and it walks `.github/workflows/`, `tools/` and `benches/` for a command whose
head is the binary: a file that runs it and is in neither list fails, naming this
section. A match the detector cannot tell from a command (a sentence inside a multi-line
string, the public-surface gate's own fixture) is recorded in `NOT_AN_INVOCATION` with
its reason, and a line there that stops matching fails too. `cerulion --version` is not
an invocation for this purpose: clap answers it above the gate.

`CERULION_LOGIN_GATE=off` switches off the command-wide check and nothing else.
Serving the network is a second rule the variable does not reach. These need a
saved login on the machine (an expired one counts):

- a real-clock `graph run` or `node run` that offers topics to the network: under
  the permissive default (no `network:` block, or one with `mode: disabled` or no
  `mode:`), any graph with a producer or a `ros2:` node; under an enabled
  `network:` block (`mode: peer` or `mode: client`), one that lists egress topics;
- a `ros2 attach` that is not a dry run;
- a `cerulion-netd` started with `CERULION_NETD_LISTEN`.

Without one they refuse with
`` serving the network requires a prior login; run `cerulion login` ``. The runs
and `ros2 attach` also accept a saved robot pairing; the netd listener does not.
A job with no account keeps a `graph run` or `node run` on the runner with
`--network off` (also spelled `--local`) or `CERULION_NETWORK=off`, and a
`ros2 attach` with `CERULION_NETWORK=off` (it has no `--network` flag); a
`graph run` under `--time-source virtual` or `external` is inert too. The release
install smoke and the perception replay demo both pass `--network off`. The seeded
login below satisfies the serving rule as well, and a run under it then serves for
real. The workspace latency harness runs its real-clock legs under the permissive
default (its backtoback leg passes `--time-source virtual` and stays inert), and the
cross-host harness's `cerulion-tcp` and `cerulion-quic` stacks give each machine a
`network:` block that lists egress topics. Both harnesses therefore need a
signed-in machine for those legs, and the cross-host harness needs a login on each
of its two machines.

A job that wants a machine WITH an account instead seeds one:
`tools/ci/seed_test_login.sh <dir>` writes the `auth.json` a real sign-in writes,
and `CERULION_HOME=<dir>` points the gate at it. The file's shape is pinned
against the type it has to parse as by a unit test in
`crates/cerulion_cli_engine/src/auth.rs` that runs the script.

## CI job map (`.github/workflows/ci.yml`)

`lint` runs: `cargo fmt --all --check` plus a workspace-root WALK
that fmt-checks the workspaces outside the root (`examples/go2`, every `benches/*`, every
`examples/*`, the fuzz workspace); `cargo clippy --workspace --all-targets -- -D warnings`;
the hot-path alloc lint and its self-test; the advisory issue and nightly full run
bookkeeping self-tests;
the agent-docs gate; the leak guard's self-test and generic-class tree scan; the three
crates.io script self-tests (the index wait, the publish retry, and the post-publish
archive reader `release.yml` runs after an upload); the nextest, actionlint and cargo-deny
installer self-tests, the tool tarball cache key self-test and the tool download check
(`tools/scripts/check_tool_downloads.py`: every curl download in a workflow or a tool
fetcher is retried, bounded and written to a file, and every fetcher but
`install_hooks.sh` holds one); the
naming gate (pull requests only);
`shellcheck` over
`tools/scripts/**` recursively and over the extensionless hooks in `tools/hooks` (`-type f`
deduplicates a symlink into a scanned subdirectory); and `actionlint` over every workflow.

EVERY job runs on a GitHub-hosted runner, and the macOS jobs run on `pull_request` and
`merge_group` events like everything else: there is no cost gate, no routing expression
and no stub job standing in for a skipped required check.

`lint` gates the four push-only jobs that depend on it (`fuzz`, `miri`, `test-latency` and
`cli-e2e-latency`), which no pull request and no queued batch runs: their `if:` carries no
status function, so a red `lint` skips them and saves their runner minutes. `python-client`
also lists it, but carries `!cancelled()` because it needs `changes`, so it runs through a red
`lint` and the dependency only orders it; it reports no required context. No job that reports
a required context waits for `lint`. `docs` and `netd-wan` list no `needs:` at all and start at once;
`test-linux`, `test-macos`, `crate-tests` and `viz-tests` list `changes` and nothing else, so
they start after the classifier, which is a checkout and a path classification, about a
minute, and no build. No REBUILD waits on `lint`: each job builds what it runs, so nothing in
front of any of them is a data dependency for compilation. A `lint` verdict was never one
either, and while it gated a job that job started a whole `lint` run late. `Lint` is its own
required context, so a red one still blocks the pull request.

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
(`macro_compile_fail_test`, held out of the rotation so the other files keep their shards
(see `PINNED_TEST` in that script); it lands on `PINNED_SHARD % count`, shard 2 of 4 on
Linux, shard 2 of 3 on macOS, and `--check` proves the pin; the blocking trybuild suite it
held runs as the `node_macro_ui` package on that same shard), and execs `cargo nextest run --profile ci`
(install via `tools/scripts/install_nextest.sh`). It does NOT pass `--test-threads=1`; see the
serialisation fence below. The split across runners is legal because each VM has its own
`/dev/shm`.
`--lib` and the doctests ride shard 0; the non-core packages are distributed across the
shards (one per shard on Linux; a hand-balanced 3-way tilt on macOS), with the iroh-tree
packages kept together so that large tree compiles once. On macOS, shard 2 also carries the
six viz steps (`cerulion_viz`, `go2_tf`, the serial `cerulion-vizd` suite and the three
OpenH264 steps) beside the trybuild suite, which is why it takes the lightest package set;
there is NO macOS `viz-tests` leg. Every shard of both jobs has a cache key of its own: shard 0
`cargo-<os>-<scope>-<lock hash>`, shard k above 0 `cargo-<os>-shard<k>-<scope>-<lock hash>`,
starting from shard 0's archive until it has one. Shards above 0 save only when the cache save
policy below names their namespaces, which the default does not. Each sharded job's `shard:` matrix is held to the count its shard step passes by
`ci_test_coverage_test.rs`.

`--with <package>` adds another package's integration tests to the same nextest
invocation: `-p <package> --test <name>` for each of its `tests/*.rs` at depth 1. Shard 2
of both lanes runs its shard in a step of its own (`if: matrix.shard == 2`, so the coverage
walk sees the leg) that passes `--with node_macro_ui --with test_node_macro_cdylib`, so the
`#[cerulion_node]` diagnostics suite and the node context helper suite build in the
shard's cargo invocation and their trybuild tests run beside the core tests. nextest starts
tests by priority, then in binary order, and its queue does not start a test past the one
at its head, where a test that claims every slot waits for every running test to end; so
`.config/nextest.toml` gives the timing tests that claim every slot priority 100 and the
two suites' test binaries 99 (at the default priority they sort after every
`cerulion_core` binary and would start last), in a `trybuild` test group of one slot: the
suites share `target/tests/trybuild`, and each re-checks its fixtures when the other has
run there since, so at the same time they took several times as long as one after the
other. On the other shards the timing tests run first, each alone, which costs no slot
they would not have waited for anyway.
Cargo applies `--lib` and each `--test` to every package one command selects, so the
script refuses `--with` on shard 0, which carries `--lib`, and an extra package that holds
a test attribute, a doctest source or a test target outside `tests/*.rs` (see `check_extra`
in the script), or a `tests/*.rs` named like one of `cerulion_core`'s; each suite's
compile-pass programs are binaries of its package, which cargo builds for an integration
test even when targets are named. `node_macro_ui`'s dev-dependencies ask for
`test-helpers` on `cerulion_core` and `native_ros2_messages` and `test_node_macro_cdylib`'s
for `trybuild` alone, all of which `cerulion_core`'s own test build already turns on, so
every unit the shard compiles for the core tests keeps its features (the shard-2 unit
graphs at both counts, with and without the two packages, agree on every unit they share).
`test_node_macro_cdylib`'s programs build in every build of that package that does not
filter its targets too, so a build without dev-dependencies, the workspace build among
them, compiles them without `cerulion_core`'s `test-helpers`. Cargo copies a cdylib into
`target/debug` only when the command asks for its library, as `cargo build -p` and
`--workspace` do, not when it builds the library as a dependency of a test; so during a
shard run the fixture the core tests load from there is the one the leg's workspace build
left. The
selection is read for each package on its own and the script prints a `selection:` line
for each package it leaves out. In CI every selection that names a package names
`cerulion_core`, which observes the whole tree, so the step sees `[]` (every package skips)
or `cerulion_core` with or without each extra; a selection of an extra alone, which the
script's hand table pins, would run that extra and none of the core files. `--list` applies the same selection and
prints the argument list the run would hand nextest, which is how to check a change to the
step without running it. The coverage walk credits each `--with` package on the line; the
package has no step or selection marker of its own in either job.

Each `test-linux` shard builds the `cerulion_core` test binaries it runs: the shard step
compiles exactly the quarter `tools/scripts/ci_test_shard.sh` assigns to it and then runs
what it built. There is no shared archive of test binaries and no job that builds one.

`netd-owner-pair-transition-repeat` runs `cerulion_netd`'s owner pair serving transition
test 20 times in a row on one runner, on every event but a merge-queue batch, with every
step past the checkout gated on `cerulion_netd`'s selection; it reports no required context,
and the `cerulion_netd` step of `test-linux` shard 3 is the one a merge waits for.

### Build once per shard (`tools/scripts/ci_built_tests.py`)

A `cargo test -p X` per package resolves features for X alone, so a leg that runs several
packages that way rebuilds every crate they share (`cerulion_core` and the third-party tree
under it, up to a dozen times per leg). The runner builds a group of packages in ONE cargo
invocation and runs the prebuilt test binaries the way `cargo test` runs them.

* **Build.** `build --group NAME SPEC...` runs `cargo test --no-run --timings` over the
  group's packages, so the job's timing report covers the grouped compile, then
  `cargo test` over the same packages with the script as cargo's target runner. That
  recording run compiles nothing; for each test binary it records the environment and
  working directory cargo gives it, in cargo's run order. Cargo then runs
  the group's doctests against the libraries the build compiled; rustdoc hands each doctest
  it runs to the same runner, which runs any binary outside the target directory. They run
  without a step's `-- --test-threads=1` or env. A package builds
  when `CI_SELECTED_PACKAGES` names it, the same rule as a step's
  `contains(fromJSON(...))` condition; a `+` package builds on every change, for a step with
  no selection condition.
* **Run.** A package step keeps its command and prefixes it:
  `python3 -B tools/scripts/ci_built_tests.py run -- cargo test -p X [--lib] [--bins]
  [--test T] [--features F] [--no-fail-fast] [FILTER] [-- ARGS]` (`--locked` and `--package`
  are accepted too); any other flag is refused.
  The script runs the prebuilt binaries that command selects, each as one libtest process as
  under cargo, with `FILTER ARGS` as their arguments, the recorded environment (on macOS a
  system interpreter cannot see `DYLD_FALLBACK_LIBRARY_PATH`, so the script sets it as cargo
  does), cargo's order and cargo's fail-fast rule. A serial package keeps
  `-- --test-threads=1`, so its tests still run one at a time inside that one process, which
  the tests that read process state rely on (a leaked global subscriber, a leftover-manager
  check). Because the line still carries `cargo test -p X`, the coverage walk and
  `serial_discipline_test` read it as before.
* **Audit.** `audit` fails when a built binary had no whole run (one with no test name
  filter and no libtest argument but `--test-threads=N`, `--nocapture`, `--show-output`,
  `--include-ignored` or `--quiet`), unless `--not-run` names it; when `--not-run` names a
  binary the leg does not build; when a leg that kept every package builds fewer than its
  floor of test binaries; or when cargo compiled a unit or ran a build script outside a
  declared build step. Every compiling step is wrapped in `ci_built_tests.py phase NAME --
  CMD`, and an `invoked.timestamp` under `target/debug/.fingerprint` or `target/debug/build`
  dates each compile and build-script run, so a package step that went back to plain
  `cargo test -p` is a red audit. It prints the units each build step compiled and the
  crates built more than once. A test that builds into the job's target directory declares
  each of its builds itself, as a `phase` run from inside its `run` step, when
  `$RUNNER_TEMP/ci-built-tests/groups.json` exists: `node_links_as_library_test` declares
  `node-links app` and `node-links node`, and fails if either compiled a crate from
  outside the workspace.
* **Wiring.** `check-workflow FILE` refuses a job whose legs do not build exactly the
  packages their steps run, a `+` anywhere but on a package whose step carries no selection
  condition, a run whose features the build lacks, a run before the build, a job with no
  unconditioned audit step after its last run, and a `cargo test`, `build`, `nextest` or
  `check` command outside `run` or `phase`. It reads the workflow only, so the phases a
  test declares from inside a `run` step are not in its view.
* **Reclaim.** `reclaim` deletes the executables in `target/debug/deps` except the prebuilt
  binaries a later step still runs, libraries (`.so`, `.so.N`, `.dylib`, `.dll`, `.rlib`, `.a`)
  and cargo's `.rmeta` and `.d` files.

Every `test-linux` and `test-macos` leg uses it: the step `Build this shard's package tests
once and run their doctests` builds the leg's packages, every package step runs through
`run`, and the leg ends with the audit. `cerulion_core` keeps its own build in its shard step,
unchanged, and the `--with` packages build there too: they are in no group and have no
`run` step, so the audit, which dates every compile, attributes theirs to
`phase cerulion_core`. A Lint step runs `check-workflow .github/workflows/ci.yml`.

* **Groups.** One unified build changes some packages' features: a package's tests link a
  shared crate with the union of what every package of the group enables. The unions in
  these legs are additive APIs or test seams that stay inert until a test arms them: among
  them `test-helpers` and `test-seams`, `jazzy` and `live` on `cerulion_dds`, `default` and
  `wan` on `cerulion_netd`'s library (macOS shard 1 links it into the `cerulion_cli_engine`,
  `cerulion_cli_tui` and `cerulion_connectd` tests), `test-util` on tokio, and a second
  rustls provider beside the ring one every client installs. The one that switches
  behavior is `cerulion_viz`'s `decoder-from-source`, so `cerulion_vizd` builds in a group
  of its own on macOS shard 2.
* **Doctests here.** The executing doctests of these legs (`native_ros2_messages`,
  `cerulion_bag`, `cerulion_cli_engine`) belong to packages whose steps pass no
  `--test-threads=1`, so they run as they did.
* **Disk.** The pre-wan reclaim on Linux shard 3 runs `reclaim`.

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

The repeat legs run one test binary or module 20 times in a row on one runner, on every event
but a merge-queue batch, so a change that makes that test fail now and then shows as a red even when a single
run passes: `rcpd-eye-host-repeat` runs `cerulion_rcpd`'s `eye_host_test`, and `remoted-rcp-repeat`
runs `cerulion_remoted`'s `rcp::tests::` module and fails a run that passed no test. Every step past
the checkout is gated on the package's selection, and none reports a required context; the
package's own step in its test job is the one a merge waits for.

`examples` opens with a lockfile WALK: `cargo metadata --locked` over every tracked
`Cargo.lock` outside the root (the benches, the examples, the Python wheel, the Android sample
runtime; floor 13), ahead of any build step that could rewrite one, so a dependency change under
`crates/` that is not carried into those lockfiles fails there. Refresh one with
`cargo metadata --format-version 1 --manifest-path <dir>/Cargo.toml > /dev/null`. That is not a
minimal edit: it adds every crate the manifests now require at its newest compatible version, and
where a path crate's dependencies changed it can re-point a dependency edge of a crate already
locked (onto another locked major version of `syn`, for example). Read its diff: keep each added
package and each move the new requirements force (an exact `=` pin or a raised minimum in a
crate manifest, directly or through the moved crate's own dependencies); restore a changed line
of an already-locked package only when its old version still satisfies every requirement (an
edge re-pointed onto another locked version, or a newer version cargo picked on its own); then
confirm with `cargo metadata --locked`. The
Android sample runtime's lockfile, seeded from the root one, takes each added registry crate at
the version the root `Cargo.lock` ships (the Python wheel's does the same for the iceoryx2
family). `cargo update -p <crate> --precise <root version>` sets that version, but it re-resolves
the same way and can re-point more edges, so diff the result against the stale file and the root
`Cargo.lock` again.

Five Linux jobs run on push / `workflow_dispatch` only and never
on a `pull_request` event: `deb-smoke`, `cross-aarch64-linux`, `fuzz`, `miri` and
`machete` each carry a job-level `if: github.event_name != 'pull_request' && github.event_name
!= 'merge_group'`; `deb-smoke` carries one exception, below; the second
conjunct is required because a bare `!= 'pull_request'` ADMITS a merge-queue batch,
which would run the same work a second time over the same commits (`main`'s push run is the
control), with their `needs: [lint]` (`fuzz`, `miri`) and
`continue-on-error: true` (`fuzz`, `machete`) untouched. None of the five is a required
status context on `main`, so a skipped one is simply absent from a pull request's checks.
They run on every merge to `main` (the push run is where their breakage
surfaces, revert-on-red), and the coverage walk drops any job behind a job-level `if:`
from its PR-blocking view, so none of the five can credit pull-request coverage it does not
provide. `tool-tarballs` runs on push and `workflow_dispatch` only as well, by an `if:` that
lists those two events; it fills and saves the pinned tool tarball cache (below) and runs no
test.

The MSRV check (`msrv`) runs on every pull request and every push to `main`, and carries
`if: github.event_name != 'merge_group'` alone: a compiler-floor break found after the merge
costs a revert on a branch that already landed, and a queue run would do the same check a
second time over the same commits. It is not a required status context, and its job-level
`if:` keeps the coverage walk dropping it from the PR-blocking view, which is correct: it
type-checks the workspace on the floor and runs no test. Its cache key carries the
`-main-`/`-pr-` scope segment, so a prune reached from a pull request deletes only `-pr-`
entries of that namespace; the namespace token `msrv-1.95` is absent from the
`CACHE_SAVE_NAMESPACES` default, so nothing fills it and the check compiles the workspace from
scratch on every run. It lists no `needs:`, so a pull request starts it beside `lint` rather
than after it.

`core-repeat` (`Repeat runs (cerulion_core, macOS)`) runs three `cerulion_core` test binaries,
`gateway_egress_congestion_iox2_test`, `liveliness_sweep_iox2_test` and
`free_run_ctor_iox2_test`, 20 times each on
`macos-latest`, where thread scheduling varies most. Their tests wait on counters the product
exposes, so a failed run here is a wait that still depends on how a loaded host schedules
threads. Each run is the whole binary under `--test-threads=1`; it passes only when cargo exits
0 AND libtest reports at least one passed test and none failed. Every run goes ahead whatever an
earlier one did, and the step fails at the end on any failed run, with the count printed per
binary. The job carries `if: ${{ !cancelled() && github.event_name != 'merge_group' &&
github.event_name != 'pull_request' }}`: pushes to `main` and `workflow_dispatch` runs run it, pull
requests and merge-queue batches do not, since it would hold one more macOS runner that a pull
request's own macOS test shards wait for, and the coverage walk drops it from the PR-blocking
view. It is not a required status context,
so a red reports a flake and blocks no merge. Its cache is restore-only (the macOS test shards'
archive).

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
direction: they SKIP test steps. Five rules bound them.

* PULL REQUESTS, AND HARNESS-ONLY MERGE GROUPS. On `push` and `workflow_dispatch` every
  package is selected. A `merge_group` run diffs the group against the commit where it leaves
  its target branch, so every group queued ahead of it that has not merged is in the diff,
  and selects every package unless every changed path is under `harness/`. The queue run is
  the last gate before `main` and the one place a miss has no later catch.
* THE HARNESS TREE. A change whose every path is under `harness/` selects only the packages
  whose doc-pin markers read `harness` (none today), on a pull request and on a merge group:
  the folder is Python, shell and data that no Rust package builds. A `harness/` path beside
  any other path selects every package. The folder's own checks are the `harness` job
  ("Harness (Linux)"), which no classifier output reaches: no `needs:`, no job or step
  condition, so it runs on every event. It shellchecks every tracked `harness/**/*.sh` and
  runs every tracked `harness/**/test_*.py`, one module per file, fetching nothing. With no
  tracked file under `harness/` it passes saying so; a tracked test file holding no test, and
  a run that judged no script and no test, fail it.
* ONE OFF SWITCH. The repository variable `CI_SELECTION` reaches the classifier through
  the workflow-level `env:` block; `off`, and any value the classifier does not know,
  selects every package. Nothing else may read it: a step is gated on the classifier's
  OUTPUT, never on the variable, and `ci_test_coverage_test` refuses any other shape.
* STEPS, NEVER A REQUIRED JOB. Every job that reports a required context still runs and
  still reports it; `android-check`, which reports none, is the one job gated whole. A
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
requests and on harness-only merge groups. It does NOT narrow the workspace build, it gates
one job whole and no other, `android-check`, which reports no required status context, it
does not apply to `push` or `workflow_dispatch`, and it never removes a required status
context.

EVERY test step names its PACKAGES explicitly; there is no blanket `cargo test --workspace`
on the root workspace, which makes coverage a hand list.
`crates/cerulion_cli_engine/tests/ci_test_coverage_test.rs` is the walk that holds it, counting only
steps and jobs that can actually fail a pull request. A step credits the packages a
`cargo test` or `cargo nextest run` command names with `-p`, or the one `ci_test_shard.sh` takes;
a command whose cargo flags (those before a bare `--`) carry `--doc` or `--no-run` credits none,
because it runs none of the package's unit or integration tests. Each shell command of a line is
read on its own, a `\` continuation joins the line it continues, and a step's `name:` never
counts. `ci_doc_pin_walk_test` reads workflow lines through the same reader. The walk drives
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

## Cargo timing reports and runner hardware

Every `ci.yml` job that compiles with cargo commands written in the workflow passes
`--timings` to each cargo build, check, clippy, doc and test command (the `cerulion_core`
shards through `CI_TEST_SHARD_EXTRA`), prints the runner's architecture, CPU model, core
count and memory right after checkout, and uploads one artifact per job and matrix leg named
`cargo-timings-<job id>[-<matrix values>]-attempt-<run attempt>`, kept 7 days. It holds
`runner.txt` (the hardware lines, present even when the path selection skipped every cargo
step) and one `cargo-timing-<timestamp>-<hash>.html` report per cargo command, under the path
of the target directory it came from (`target/cargo-timings/`,
`examples/go2/target/cargo-timings/`). Download it from the run's summary page or with
`gh run download <run id> --pattern 'cargo-timings-*'`. The collect and upload steps run
under `always()`, so a red or timed-out job still uploads. The collect step moves the reports
out of the job's target directories before any cache save and deletes one it cannot move; it
fails the job only when it can neither move nor delete a report, or passes its 2 minute step
timeout, and a failed job saves no cache, so no cache archive carries a report. Only the upload
carries `continue-on-error: true` (with a 5 minute step timeout). Artifact names spell a shard
leg as `shard-<n>`, for example `cargo-timings-test-linux-shard-0-attempt-1`.
`ci_cache_policy_check.py` holds the steps in place (TIMINGS_REPORTS, below).
`deb-smoke` and `stable-node-compiler-skew` build inside scripts and `fuzz` and `miri` through
cargo plugins, so those four carry none of these steps, and a `cargo install` tool build
writes no report.

## Scheduled workflows

Seven workflows run on a clock, and none of them reports a required status context.
Five of the seven run on nothing but the clock and a manual dispatch:
`.github/workflows/nightly.yml` at 03:30 UTC runs the nightly full run (below),
`.github/workflows/nightly-alarm.yml` every six hours at minute 47 checks that it
still runs, `.github/workflows/moveit-hero.yml` at 03:43 UTC builds the ROS 2 Jazzy
MoveIt image and runs the hero demo, `.github/workflows/advisories.yml` at 04:37 UTC
runs the dependency advisory check, and `.github/workflows/stale.yml` at 07:17 UTC
labels and closes `needs-info` items. The sixth, `.github/workflows/examples-replay.yml`, runs
the replay-tolerance demo at 07:30 UTC and ALSO on a push to `main` and on a pull
request, both path-filtered to `examples/perception/**` and its own file, so for that
one the clock is the second trigger rather than the only one: it is what notices a
change outside the filter breaking the demo. Every minute is off the hour, where
GitHub queues the world's hourly crons at once. The seventh,
`.github/workflows/android-e2e.yml`, runs at 06:13 UTC and also on
Android-affecting pushes and pull requests.

`nightly.yml` dispatches `ci.yml` and `rmw-distros.yml` on `main` with the job's own token,
so every job a push runs also runs once a day, and so does every distro lane, which a push
outside `rmw-distros.yml`'s paths never starts. The clock is not in `ci.yml` itself because a
workflow that reports a required context may trigger only on a pull request, a merge-queue
batch, a push or a dispatch (`ci_test_coverage_test`). `tools/scripts/nightly_full_run.sh run`
takes each run's id from the dispatch's answer, or, when the answer carries none, finds the one
new dispatch run whose actor or triggering actor is the token's bot account; a dispatch that
errors is sent again only when four reads of the listing show no new run. It waits for both
runs, reading each attempt by its number so that a rerun someone else starts never stands in for
it, reruns the failed jobs (as the attempt after the newest, which it then waits on) of a red run once (a failed step skips every later step of its
job, and the rerun shows whether the failure repeats), and records the result on ONE issue
labelled `bug`, found by the exact title `Nightly full run of main failed or did not run`. A
red run, or one not finished after 330 minutes, opens that issue or comments on it, naming the
failed jobs of each attempt; every run green on its first attempt comments on an open one and
closes it. A job that passed only on the rerun is reported as failed on the first attempt, and
no rerun is requested past the deadline. The script exits 0 once the result is recorded, red
or green, or every run passed with no issue open to close, so a successful run of
`nightly.yml` means a result was reached.

`nightly-alarm.yml` runs `nightly_full_run.sh alarm` every six hours. It writes to the same
issue when `nightly.yml` is not active, or when no run of it is in progress and none reached a
result on `main` in the last 26 hours (counted from the workflow's creation until one has),
naming the newest run when it did not succeed; it writes at most once a day. It lives in a
file of its own, with a concurrency group of its own, so that a disabled or broken
`nightly.yml` does not take the alarm with it and a queued alarm never replaces a queued
nightly. Two open issues carrying the title hold both modes at exit 2 until one is closed. The
script's oracle is `tools/scripts/test_nightly_full_run.sh`, cases over a `gh` shim with answer
files per call, run as a step of `lint`.

`advisories.yml` runs `cargo deny --format json --config tools/release/deny.toml
check advisories` and nothing else. Licences, bans and sources are decided by
`Cargo.lock` and `tools/release/deny.toml` together, and an edit to either is a push
the per-change `Dependency Audit (cargo-deny)` job reads. Advisories are decided by
the RustSec database, which `tools/release/deny.toml` does not pin, so an advisory
published after the last push matches a lockfile nobody has edited and no event
`ci.yml` triggers on fires.

`tools/scripts/advisory_issue.sh` turns the result into one write on ONE issue,
found among the open issues labelled `security` by the exact title `Dependency
advisory audit reports a finding on the committed lockfile`. A finding opens that
issue or rewrites its body; a clean run comments the commit the check passed on and
closes it; a clean run with no such issue open writes nothing.

The script acts on a diagnostic in the captured JSON log, at error or warning
severity, whose class is one of `vulnerability`, `unmaintained`, `unsound`, `notice`
and `yanked`. cargo-deny 0.20.2 checks each of those classes over a different set of
crates, and `tools/release/deny.toml` is what fixes which:

* `vulnerability` and `notice`, over every crate in the resolved graph, as
  `error[<class>]` on a nonzero exit.
* `unmaintained`, a scope key the file leaves at the default `all`, so over every
  crate, as `error[unmaintained]` on a nonzero exit.
* `unsound`, the same kind of scope key, which the file leaves at the default
  `workspace`: a crate is checked only when at least one of its DIRECT dependents is
  a workspace member. An unsound advisory on a crate reached only through
  third-party edges emits no diagnostic and the check exits 0, so it reaches neither
  this run nor the per-change job. Measured 2026-10-02 against the committed
  lockfile: the same run with `unsound = "all"` exits 1 on seven such advisories, on
  `anyhow` 1.0.102, `event-listener` 5.4.1, `lru` 0.12.5, `lru` 0.18.1, `rand` 0.8.5
  and `scc` 2.4.0, every one of them silent at `workspace` scope.
* `yanked`, which `yanked = "warn"` renders as `warning[yanked]` on exit 0.

`unmaintained` and `unsound` are the two live keys there, each taking `all`,
`workspace`, `transitive` or `none`. `version = 2` drives none of the above:
cargo-deny 0.20.2 parses that key and discards the value, as it does the lint level
on the removed `vulnerability` and `notice` keys.

An advisory accepted in that file's `ignore` table drops to `note`, which the default
`--log-level warn` keeps out of the log. So the exit status alone would overlook a
yanked crate, which is why the log is read, and the severity bound is what keeps an
accepted advisory from being reported. The body names the crate and version behind
every diagnostic beside any `RUSTSEC-` id on it: the yanked class carries no advisory
id, so the crate name is the only handle on it.

Two shapes route to exit 3, each writing nothing, each leaving an open issue exactly
as it stands, each naming the run URL so the Actions run goes red rather than a
verdict on the lockfile resting on a check that did not fully run. One is
`warning[index-failure]`
in the log at any exit status: cargo-deny emits it per crate whose registry index
query failed, while `yanked` is not `allow`, at warning severity and exit 0, so the
registry query failed and the yanked half of the check did not run for the crates it
names. The other is a nonzero exit carrying no advisory diagnostic: `cargo metadata`
failed, the advisory database would not fetch, the index cache would not load. The
script refuses with exit 2 and writes nothing when `jq` is absent or cannot read the
log, when `gh` is unauthenticated, when the repository carries no `security` label,
when `gh` refuses a listing, and when two open issues carry the title. Its oracle is
`tools/scripts/test_advisory_issue.sh`: thirteen cases over JSON log fixtures against
a `gh` shim that records every call, run as a step of `lint`.

A `schedule` trigger fires only from the default branch, so a pull request branch's
copy of one of these workflows never runs on the clock. `workflow_dispatch` is not
the same: the button appears only once the file is on the default branch, and from
there a dispatch runs the copy on whichever ref it names, a pull request branch
included. Until the file lands, the pre-merge proof of a scheduled workflow is
actionlint, the cache save policy checker and the script self-test; the first run on
the clock is on `main`. GitHub also disables scheduled workflows on a repository
after 60 days with no commit.

## The queue run watchdog

`.github/workflows/queue-watchdog.yml` runs on every merge-queue batch, beside the `ci.yml` run
of the same queue commit, and cancels that run at its first failed required job, or once the
batch has left the queue without merging. A failed required context fails the batch by itself,
so the rest of the run changes no verdict; what it holds is its runner minutes and its
concurrency group, which `ci.yml` keys on the queue branch, and a pull request queued again on
the same `main` gets the same branch, so its new run waits in that group until the failed one
ends. `tools/scripts/ci_queue_watchdog.sh` finds the run as the one `ci.yml` run with event
`merge_group` and the batch's head commit, then once a minute reads its jobs and cancels it
when a job named in `tools/ci/required_contexts.txt` ends `failure`, `timed_out` or
`cancelled`. Once no job is left running it reads the run as well, and stops when the run has
completed: one jobs read a minute while jobs run, from the job token's hourly budget that every
job of the repository shares. A failed job that is not required, and a skipped required one,
cancel nothing; the three Leak guard contexts are reported by `leak-guard.yml`, so their
failures cancel nothing through the jobs read; such a failure takes the batch out of the queue,
and the dead batch check below cancels the run two polls later. The cost is that a failed batch
shows only the failures that ended before the cancel: a pull request run, which the watchdog
never touches, shows them all.

GitHub does not cancel a merge-queue run when its batch leaves the queue: a pull request
removed or queued again, or a failure ahead of it, invalidates the batch, and its run fails
nothing and holds its runners to its end, while the live batches wait for those runners. So
each poll that reads again also reads the merge queue of the base branch
(`github.event.merge_group.base_ref`), every page of `repository.mergeQueue(branch:).entries`
over GraphQL: the batch is live while its commit is the `headCommit` of an entry. When it is no
entry's head commit, `compare/<base>...<commit>` says whether it merged (`identical` or
`behind`: the queue is not read again) or is dead (`ahead` or `diverged`). Dead on two polls in
a row cancels the run with the same cancel and refusal handling as a failed job. One dead poll
alone cancels nothing, because the queue pages and the compare are separate reads that a batch
moving or merging can fall between. A read that fails three times, or an answer missing a field
the decision reads, decides nothing for that poll and starts the count again, so failed reads
never add up to a cancel. An answer refusing the job token (`Resource not accessible`, or a
GraphQL error of type `FORBIDDEN` or `INSUFFICIENT_SCOPES`) is not retried: the watchdog says
so once and stops reading the queue for that run, cancelling nothing for it, while a failed
required job still cancels. The first read that finds the commit in the queue says so once, so
a batch's log shows whether the job token may read the queue. The workflow grants
`contents: read` for the queue read and the compare; no GitHub page names the permission the
job token needs for `repository.mergeQueue`, and a refusal turns only the dead batch check off.
The queue reads come out of the job token's hourly GraphQL budget, per repository like its REST
one; until the batch merges or a read is refused a poll reads one page per 100 entries, and the
compare (REST) runs only while the commit is no entry's head commit.

It lives in a file of its own because no `ci.yml` job takes `actions: write` except to prune
its cache (`ci_cache_policy_check.py` R4), and cancelling needs it; a job inside `ci.yml` would
also sit in the run it cancels. It reports no required context, so a watchdog that cannot find
the run, cannot read it or has its cancel refused exits nonzero, cancels nothing, and leaves
the batch to finish as it would have; a cancel refused because the run completed first exits 0.
Its permissions are `contents: read` (granted for the merge queue and the compare) and
`actions: write` (the jobs and the cancel). Its oracle,
`tools/scripts/test_ci_queue_watchdog.sh`, runs as a step of `lint`.

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
  `Linux` (shard 0 of each test job), `macOS-shard1`, `macOS-shard2` and `Linux-shard1` to
  `Linux-shard3` (the other shards, which the default does not name), `crates-Linux`, `viz-vizd-Linux`, `Linux-lint`, `examples-replay-Linux`, and
  `rmw-distros-jazzy` for `cargo-rmw-distros-jazzy-<header hash>-<scope>-<lockhash>`, whose
  header hash is a cache generation rather than part of the namespace. The default is the
  expression's literal, so a repository with no variable saves the measured frontier and a
  misspelt variable matches nothing.
* `CACHE_SAVE_ON_PULL_REQUEST` (`${{ vars.CACHE_SAVE_ON_PULL_REQUEST }}`): empty means saves
  run on `main` only; any value lets pull-request runs save into their own `-pr-` scoped keys
  as well, and only from a branch of this repository. A fork pull request's token cannot hold
  `actions: write`, so the prune's delete would fail the job; the clause tests
  `github.event.pull_request.head.repo.full_name == github.repository`.

Before every save of a lockfile-keyed archive (the tool caches are exempt, below)
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

In `ci.yml` the jobs that cache the root workspace's `target/` and build no other tracked
workspace hash the root `Cargo.lock` only (`hashFiles('Cargo.lock')`); the bench, example,
Python and Android lockfiles describe other workspaces and do not change that tree.
`android-check`, the example jobs and `python-client` keep their wider hashes. A save happens only on a key miss, whose
run restored the previous generation through `restore-keys`, and cargo writes the units whose
hash changed next to the old ones without deleting those. So in `test-linux` and `test-macos`
each shard whose namespace may save runs `tools/scripts/ci_cache_trim_target.sh` twice around
its build: `age` right after
the restore sets the access time of every restored `.fingerprint` file to 2000-01-01, and `trim`
before the prune deletes from every profile directory under `target/` (the scaffold target dirs
under `target/tmp/` included) each `deps/`, `build/`, `examples/` and `.fingerprint/` entry whose
unit hash no cargo command in the job read, plus `incremental/`. Cargo reads the fingerprint of
every unit in a build graph, fresh or not, so a unit the job used stays. The trim deletes nothing
when a read leaves an aged access time unchanged, when `target/debug` holds no unit read during
the run, or (per profile) when no `deps/` entry carries a live unit hash, and it prints the
target size before and after. It runs on `main` only, under the save's namespace gate. Each
shard's archive then holds only the units its own cargo commands read, so a unit that only a
fall-through job (`lint`, `docs`, `netd-wan`) builds is compiled by that job every time, and so
is a unit only shards 1 to 3 build while the policy does not name their namespaces.

`lint`, `docs` and `netd-wan` build far smaller targets than the shards and used to share the
shards' `cargo-Linux-` key, so whichever finished first saved it: on 2026-09-28 the entry under
the shards' key was a 0.63 GB archive saved by one of the smaller jobs, not the shards' 3.18 GB
one. They now have their own namespaces (`cargo-Linux-lint-`, `cargo-Linux-docs-`,
`cargo-Linux-netd-wan-`). Under the default all three are restore-only and fall through their
`restore-keys` to `cargo-Linux-main-`, shard 0's archive; every `test-linux` shard restores its
own key and saves it when the policy names its namespace.

Two exemptions, both narrow. The TOOL caches carry no lockfile generation for the prune to
work from, so they carry no namespace gate and no prune step, only the cache-hit test and the
merge-queue test. There are two. The machete binary cache (`~/.cargo/bin/cargo-machete`, keyed
on the pinned version) is a few MB; gating it into a namespace the default never names meant the
binary was never cached and `cargo install cargo-machete` ran on every push. The pinned tool
tarball cache (`~/.cache/cerulion-tool-tarballs`, about 33 MB) holds the nextest tarballs for
Linux x86_64 and macOS and the cargo-deny tarball for Linux x86_64, under the key
`tools/scripts/tool_tarball_cache.sh key` derives from both installers' pins and the platform
lists its `fill` fetches, so a moved pin is a new key. Only the `tool-tarballs` job saves it.
`test-linux`, `test-macos`, `deps` and `advisories.yml` restore it and point
`CERULION_TOOL_TARBALLS` at it; `install_nextest.sh` and `install_cargo_deny.sh` use a cached
tarball only after the pinned checksum test a download faces, and a cached file that fails it
is an `::error::` annotation, is deleted and is downloaded again.
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
events naming none (R11); a lockfile-keyed save in a job whose matrix declares `shard` (as a
key or in an `include` entry) runs on shard 0 alone or carries the shard in its key in one of
two forms, `${{ matrix.shard }}` or nothing on shard 0 and `-shard<k>` on shard k, so shards
never race for one archive; any other key expression naming `matrix.shard` is refused (R12); a
key with no lockfile hash is a tool cache or a violation (TOOL_CACHE);
a job that passes `--timings` in a step carries `Collect the cargo timing reports` under
`if: always()` with no `continue-on-error`, running the one script the checker holds, directly
followed by `Upload the cargo timing reports` under `if: always()` with `continue-on-error: true`,
both after its last `--timings` step and before its save, the collect list naming every target
directory its cache steps archive, and a job with neither the flag nor the steps carries neither;
no job- or workflow-level `env` passes the flag (TIMINGS_REPORTS); the
combined `actions/cache` action appears nowhere (NO_COMBINED_CACHE_ACTION); and a job whose
steps the reader cannot enumerate -- a reusable-workflow call, or one carrying no `steps:` -- is
refused rather than passed in silence. The `Lint` step "Cache save policy" runs the prune
script's self-test and the checker's self-test, both of which flip every rule from both sides,
and the target trim's self-test, before the checker reads the real files.

## Test map

| Test | What it pins | Serial? |
|---|---|---|
| `tools/scripts/check_citation_release.sh` | citation version, calendar, and release-date window validation | n/a |
| `crates/cerulion_cli_engine/tests/workspace_lints_manifest_test.rs` | every member inherits the one lint table; the table's levels | no |
| `crates/cerulion_cli_engine/tests/library_print_ban_test.rs` | every library crate carries the print ban | no |
| `crates/cerulion_cli_engine/tests/ci_test_coverage_test.rs` | every package runs in a blocking job; the shard partition is total and disjoint; each sharded job's legs run each package's shard step exactly once, and the jobs that run the shard are exactly `test-linux` and `test-macos`; a `ci_test_shard.sh` line credits its package and each `--with` package; the script's `--with` arms (each selection, the order of two extras, an empty own shard, the archive filterset, every refusal) on a synthetic checkout under `bash` and `/bin/bash`; each workflow `--with` line names every test file of its packages; each `binary_id` in `.config/nextest.toml` names a real test file; a step gated on a changed-path selection still runs on the change that selects only its own package. A selection condition counts only where it is GROUNDED: the job `needs:` the classifier, the classifier declares the output, and that declaration is exactly `${{ steps.<id>.outputs.<name> }}` naming a step of it that can set an output OF THAT NAME: a `run:` step whose script writes `<name>=` into `$GITHUB_OUTPUT`, or a `uses:` step, whose action's outputs are not in the file to read. A literal value, an expression carrying another operand, a step that writes no output, and a step that writes some other output's name each ground nothing; the `changes` job probes its resolved base, with a refusal that fails the job, ahead of the diff that lists the changed paths; and a step in a `container:` job whose `run:` script uses a bash-only construct (`pipefail`, another `set -o` option, or the `[[ ... ]]` conditional, never a POSIX class `[[:...]]`) declares `shell: bash`, because a container step with no `shell:` runs the image's `/bin/sh`, which is dash on the ROS base images. The walk prints the count of container `run:` steps it judged and fails on zero. And a job whose name reports a required status context skips on no event its workflow triggers on, the steps that split its work by event together admit every such event, it carries `!cancelled()` whenever it lists `needs:`, and its workflow triggers on no event outside the allowed set. The required names are read from `tools/ci/required_contexts.txt`. The classifier itself, run on hand-written rows per path and per event, answers the harness rule: a harness-only change selects no package on a pull request and on a merge group, and a harness path beside any other selects every package; the `harness` job carries no `needs:`, no job or step condition and no matrix | no |
| `crates/cerulion_cli_engine/tests/ci_doc_pin_walk_test.rs` | the `# doc-pin:` markers in `ci.yml` equal, both ways, the shared-root reads derived from every workspace member's `tests/*.rs` and `src/**/*.rs`: a string literal rooted at `docs`, `tools`, `.github`, `benches`, `examples` or `harness`, or a root markdown file name, that the surrounding code opens or joins as a path, never one it only names, writes, or joins onto its own crate directory. A `src/` read is attributed to the library test binary (`<package>::<package>`). A path assembled at run time, or reached through a helper in the crate's library, is NOT seen: that is a stated limitation, and `cerulion_core::serial_discipline_test`'s shell-script reads are the known case | no |
| `crates/cerulion_core/tests/tracing_field_discipline_test.rs` | no interpolated log message; no near-spelled field name | no |
| `crates/cerulion_core/tests/serial_discipline_test.rs` | nextest fence membership equals its declared inventory both ways; every singleton-creating file is fenced; no executing doctest reaches the singleton | no |
| `tools/scripts/check_hot_path_allocs.sh --self-test` | the annotation grammar and scope rules | n/a |
| `tools/scripts/ci_cache_trim_target.sh --self-test` | a synthetic target with a read and an unread generation: the unread units' artifacts, build dirs and fingerprints go, the read ones and unhashed files stay, an unread scaffold target and scratch files under `tmp/` go; nothing is deleted when nothing was read, when `age` never ran, or when no `deps/` entry carries a live hash; fails when the filesystem does not update access times | n/a |
| `tools/scripts/check_pr_title.sh --self-test` | the title/branch oracle table | n/a |
| `tools/scripts/check_agents_md.sh` | context-file budgets, shims, banned tokens | n/a |
| `tools/scripts/ci_built_tests.py --self-test` | the build-once runner: the selection rule against the classifier's list (a longer name never matches a shorter one), the `cargo test` command line it accepts and the ones it refuses, which prebuilt binaries a selection runs and in what order, the environment replay (a cargo `[env]` key without `force` yields to the step's own value), the macOS library path in cargo's order, the attribution of compiled units to declared build steps, the wiring check over a hand-written job and nine broken ones, and the modes end to end over a fake cargo: the recording runs no test and a doctest that reaches the runner runs (and fails the build when it fails), a run gets cargo's env and working directory, fail-fast, a missing feature refused, the reclaim keep set (shared libraries kept whatever their mode), a selection naming no package auditing clean, comma feature lists compared as sets, and the audit's refusals (an unrun binary, a binary run only under a filter or `--ignored`, a floor, an unknown `--not-run`, a unit compiled after every step, a floor that is no number) | n/a |
| `tools/scripts/ci_selected_packages.py --self-test` | the reverse CARGO-DEPENDENCY closure of a set of touched packages, over hand-built metadata documents and over this workspace, with normal, build and dev edges followed, a renamed dependency keyed by its package name, and an unknown name refused. A document that is not an object carrying `packages` (a list), `workspace_members` (a list) and `version` is refused with exit 2 and one line naming the field, in every mode including `--all`, so no caller reads an empty selection as the answer. It does NOT prove that the selected set is everything a change can break: see below | n/a |
| `tools/scripts/ci_test_router.py --self-test` | a hand-built workspace with every selection written out beside it: own sources through `mod`, inline and `cfg(test)` modules and `include_str!`; a dev-dependency source selecting only the binaries that name it, and its manifest selecting every binary of the package that dev-depends on it; a daemon spawned from library code through a `const` reaching every linker transitively, and a thread named like a daemon reaching none; a proc-macro consumer's manifest; a cdylib found by its table row and by a literal; a one-file crate-path read; build-script inputs and a file the script does not name; an unresolvable `mod` widening its target; the doc-pin markers; `Cargo.lock` narrowed through a normal and through a dev-dependency; the root manifest's tables and its comments; every global input; the empty change; and every row run twice, in reverse order and under two hash seeds, byte-identical. It reads no file of this checkout; `--live-check`, run in `router-shadow`, pins shapes the real tree keeps. The run ends on a case-count floor | n/a |
| `tools/scripts/ci_router_escapes.py --self-test` | hand-written nextest and libtest logs and TSV extracts: a failure in a skipped binary is an escape and in a selected one is caught, a failure that passed on a rerun at the same head or that a flake issue names (by binary, target or failing test) is a flake and nothing else is, a retry that passed is no failure, a library that failed to compile is an escape only when no selected binary builds it, a failure no binary owns is unplaced, a skipped binary with no verdict is unverified, and two runs give the same document | n/a |
| `tools/scripts/leak_scan.py --self-test` | every generic class on every surface, the redacted private output contract, exit codes, allowlist and pragma rules, the self-scan | n/a |
| `tools/scripts/install_hooks.sh --self-test` | the hooks refuse a planted leak and a planted message, pass a clean commit, cover a worktree without `tools/hooks`, and uninstall cleanly | n/a |
| `tools/scripts/test_nightly_full_run.sh` | the oracle table for `tools/scripts/nightly_full_run.sh`: every issue write of both modes (none, open, comment, comment and close) with its exact text; the run id taken from the dispatch's answer, and found by listing when the answer has none, late, by either actor field, and never a person's dispatch; a dispatch that errors after starting a run is not sent again, one that starts nothing is, and three that start nothing are refused; attempt 1 read by number when someone reran the run by hand, and the nightly's own rerun read at the attempt it started after a rerun by hand; a rerun's attempt answering 404 until it exists; a rerun that reads `completed` on the old attempt for a moment, a refused rerun, a rerun unfinished at the 330-minute deadline and none requested past it; a neutral job and a jobs page cut at 100; the alarm's findings, its silence while a nightly is in progress or in the workflow's first 26 hours, and its once-a-day limit; every refusal; and the same fixtures giving the same calls and text twice. The run ends on a case-count floor | n/a |
| `tools/scripts/test_ci_queue_watchdog.sh` | the oracle table for `tools/scripts/ci_queue_watchdog.sh`: a failed, timed-out or cancelled required job cancels the run once under its rendered matrix name, read through the committed required list too; a failed job that is not required and a skipped required job cancel nothing; the first of two failed required jobs is the one named; the run itself is read only once no job is left running, and the watch ends when it has completed, with jobs or without any, and keeps going while jobs are still to be listed; the run is found by its commit, after a listing that holds only a run of another commit, and two runs or none are refused; a jobs read, a run read and a run listing each failing three times; a cancel retried, refused while the run runs, and refused because it had completed; a jobs page cut at 100; every argument and environment refusal, a missing `jq` and a base ref naming no branch among them, before any call; the exact waits between reads; the same answers giving the same calls twice. The batch liveness read, over a stubbed merge queue and compare: a live queue entry keeps the watch to the run's end and is never compared; dead on one poll then live cancels nothing; dead on two polls in a row (`ahead`, then `diverged`) cancels once, naming the commit, and a cancel refused because the run completed exits 0; off the queue once and then `behind` main, and `identical` with main on the first poll, never cancel and stop the queue reads; dead, live, dead cancels nothing; a queue read failing on every poll (a timeout), a failed read between two dead polls, a compare failing on every poll, a queue answer missing its entry list, an entry's head commit field or oid, its page flag or its next cursor, an oid that is not a string, a queue or compare body that is not JSON, and an unknown compare status each cancel nothing; a queue read or compare refused to the job token (`Resource not accessible`, a `FORBIDDEN` or an `INSUFFICIENT_SCOPES` error body) is read once, said, and turns only the dead check off, a failed required job still cancelling after either, while a rate-limit error body is retried; the first live read said once; the entry found on the second page through the first page's cursor, and a commit on no page read as dead; pages that never end read ten deep; the base branch read off the event for both the queue and the compare; the exact calls of each, the query's fields, and the same dead answers giving the same calls and output twice. The run ends on a case-count floor | n/a |
| `tools/scripts/test_android_e2e_probe_verdict.sh` | the oracle table for `tools/scripts/android_e2e_probe_verdict.sh`, the verdict the Android emulator e2e applies to each run of the direct data demand probe: a passing report and every failing rule for each of the four expectations (`paired`, `open`, `schema-mismatch`, where every frame on the wire must die on the schema, and `connect`, the open rules over a link the app dialed), the `dialer=` field (missing or not `probe` or `app` is refused; `connect` needs `dialer=app`, every other expectation `dialer=probe`), every rule that holds for all four (every counted ack from the app's robot identity and none named without an ack, no ack from a peer the probe did not name because its peer set was full or the name too long, no more acks than demand passes, no more frames delivered or dropped on the schema than crossed the wire, no wire-header refusal, no decode, re-injection or LAN-posture drop), a report line that is absent, repeated, missing a field, carrying a field twice or a non-number or an `ack_peers=` that is not a list of robot names, `wire_frames=` never read as `frames=`, the three usage refusals (an unknown expectation, a missing argument, an app identity that is not a robot name), and the same report giving the same bytes twice. The run ends on a case-count floor | n/a |
| `tools/scripts/test_android_release_docs.py` | the Android app copy step in `docs/android.md` against the root `Cargo.toml`: the app runtime example plus the root table's `iceoryx2*` entries parse as one `[patch.crates-io]` table, the same entries pasted under a second header do not parse, the root table holds entries beyond the `iceoryx2*` family (`re_grpc_server`), the family `cerulion workspace create` writes is the root table's family, and the step text says entries under the existing header and not a complete block; and the `CHANGELOG.md` entry for the empty trust-store MAC key, which appears once and names no key length | n/a |
| `tools/scripts/test_package_android_java.py` | the oracle table for `tools/scripts/package_android_java.py`, which builds the `cerulion-android-java-<version>.tar.gz` release asset in the `android-java` job of `release-artifacts.yml`: the asset from this checkout carries exactly `CerulionRuntime.java` and the crate's `LICENSE` under its versioned directory, byte for byte; over a hand-written crate, the member order, owner, mode and time, the `sha256sum` line, a gzip header with no time and no name, and two runs byte-identical after the source times change; a prerelease identifier with a hyphen accepted; refusals for a version that is not `X.Y.Z` or `X.Y.Z-<pre>`, a crate with no `java/` tree, a tree with no `.java` file, a missing `LICENSE`, a `java/` directory or `LICENSE` that is a symbolic link, a symbolic link or a special file in the tree and missing arguments, each writing nothing | n/a |
| `tools/scripts/test_verify_published.sh` | the oracle table for `tools/scripts/verify_published.sh`, the post-publish reader the `verify` job of `release.yml` runs: a fixture registry on PATH shims (`cargo`, `curl`, `git`, `sleep`) serves sparse-index entries, version metadata and `.crate` archives the test builds, over two crates that cover both rows of the licence table. The `git` shim answers `rev-parse` for one ref spelling and exits 128 on any other, and one arm asserts the ref it recorded, so a change to the ref the provenance check asks for reds the suite instead of downgrading it to a warning. One arm per check passing, and a failing arm for every check except `extract`: a spent index budget, version metadata that answers nothing, an archive the registry does not serve, a digest the archive does not have, the two registry surfaces disagreeing on it, a `crate_size` mismatch, an archive that is not a readable gzip tar, a member outside the archive prefix, a member name ending in a parent-directory segment, `AGENTS.md`, `CLAUDE.md`, a root `tests/` tree, a missing licence text on each row of the table, a licence expression no row knows, a metadata licence the manifest does not declare, a tracker id in the packaged description, a README link outside the archive, and a packaging commit that is not the tag's. `extract`'s own two refusals -- a listing that reads and then fails to unpack, and an archive carrying no packaged `Cargo.toml` -- have no arm. Warning rather than failure, each with an arm: docs.rs unreachable and docs.rs 404, an archive with no README, a checkout without the tag. Also `--skip-docs` asking docs.rs nothing, the bounded index poll in both directions, `--crate` and `--index-url`, and every refusal the argument parsing, the version shape and the member list raise, `--index-url` and `--wait-seconds` with no value included; the two tool-presence refusals have no arm, because the shims put every tool on PATH. The clean arm asserts the exact `PASS`/`WARN`/`FAIL` counts, and the run ends on a case-count floor | n/a |

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
`examples`, `harness`) or a root markdown file is pinned, both ways, against the markers in
`ci.yml`. Cross-crate source literals and dlopen fixtures are an open class. They
have to be pinned before any CI test step is gated on the selection.

## The per-binary test router (shadow mode)

`router-shadow` (`Test router (shadow)`) runs on every pull request and merge-queue run.
It computes, with `tools/scripts/ci_test_router.py`, which test binaries of the root
workspace a change can affect, prints one line per binary, and uploads the answer as the
`router-shadow-<run id>-<attempt>` artifact. It ROUTES NOTHING: no job `needs:` it, no step
reads its output, it is not a required context, and every job runs exactly the tests it
runs without it. On a pull request it diffs against the base the `changes` job resolves; in
the merge queue against `merge_group.base_sha`, so an entry is scored on everything its run
built on top of `main`.

A binary is named the way nextest names it (`pkg`, `pkg::test`, `pkg::bin/name`,
`pkg::example/name`, and `pkg::doctest/<lib>` for the doctests). It is selected when a
global input changed, or when the diff touches one of its inputs:

* its own sources: the target file, every module a `mod` reaches (`#[path]`, inline modules
  and `#[cfg(test)]` tracked), and every `include!`, `include_str!` or `include_bytes!` with a
  literal or manifest-directory path. This is a static reading of the source text, not rustc
  dep-info, so the job needs no build. A `mod` or include it cannot resolve widens that
  target to its whole package directory and prints a warning;
* its crate closure: the normal and build closure of its package, plus the closure of each
  dev-dependency its own sources name (every dev-dependency for a `trybuild` driver, whose
  cases name crates the driver never does). A library's build inputs are its build script,
  what the script names in `rerun-if-changed` (the whole package when it names none or
  computes one), and the literal paths it opens;
* every binary it spawns, cdylib it loads and nested build it drives, transitively, and
  from LIBRARY code too: `crates/cerulion_netd/src/client.rs` spawning `cerulion-netd`,
  `crates/cerulion_cli_engine/src/viz_client.rs` spawning `cerulion-vizd`, and
  `crates/cerulion_cli_engine/src/doctor/daemons.rs` looking for `cerulion-remoted` beside the
  running executable bind every binary that links those libraries, and a spawned binary
  brings its own sources (`crates/cerulion_netd/src/main.rs`) and its own spawns.
  `crates/cerulion_android/src/runtime.rs` runs its recorder, gateway and graph as threads named
  like daemons: that is library code the crate closure already carries, not a spawn;
* what it reads at run time: the rows of `tools/ci/observation_edges.tsv` and the
  `# doc-pin:` markers, narrowed to the binaries whose sources carry the row's shape (the
  whole package when the narrowing cannot see the row's own witness), its own package tree
  when its sources name a path, every file inside its package a literal names (a test runs
  with its package as the working directory), and the files a path anchored at
  `CARGO_MANIFEST_DIR` in library code names;
* whole-tree walkers, unattributable readers and binaries that run cargo on a scaffold
  with no lock file run on every change that touches anything.

A MEMBER MANIFEST selects every binary of every package whose test build includes that
member (its normal, build and EVERY dev-dependency closure, not the per-binary narrowing:
`cerulion_core` dev-depends on `cerulion_cli_engine`, whose features merge into the core
library every core test links), every package that shares a `cargo test` command with
one, and every binary that spawns or loads a workspace artifact, because `cargo build
--workspace` unifies features across the members. A comment-only manifest change is no
exception: `crates/cerulion_macros/src/crate_root.rs` reads the consuming crate's `Cargo.toml`
while it expands, a read rustc's dep-info does not record.

GLOBAL INPUTS select every binary: `.github/workflows/ci.yml`, `.github/actions/**`,
`.cargo/**`, `.config/**`, `rust-toolchain*`, the router itself,
`tools/ci/observation_edges.tsv`, the path classifier, its selector and the shard runner,
every `tools/` path a job that runs tests names, a change to any table of the root
`Cargo.toml` (a change to its comments alone selects the binaries that read its text),
`Cargo.lock` when it cannot be read (otherwise it is narrowed: a member whose normal or
build dependency reaches a changed entry has a changed library, and every binary that
spawns or loads a workspace artifact runs too; a member whose dev-dependency does has
every binary selected), metadata that disagrees with the head tree in either direction (a
target file the head lacks, or a file cargo would make a target that the metadata does not
list), and any path no rule claims. The separate cargo workspaces (`examples/*`, `benches/*`,
`crates/cerulion_py`, the fuzz and Android runtime workspaces) are one unit each, selected
by a path inside them, a changed workspace library or binary, or a changed shared tree.

THE ESCAPE CRITERION. Run by hand on the artifact and the run's job logs,
`tools/scripts/ci_router_escapes.py --selection <artifact> <job logs>` reads the same run's
verdicts (nextest status lines, libtest blocks, cargo's rerun
and compile-failure lines, or the `per_test.tsv` and `per_binary.tsv` extracts) and lists
every binary that failed while the selection skipped it, as one of two kinds. It is a
FLAKE when it passed on a later attempt at the same head (`--reruns`, a JSON object of the
selection's `head` commit, the merge commit on a pull request, to binary ids) or when a
flake issue, open or closed, names it (`--issues`, a JSON object of key to issue numbers;
a key is a binary id, a target name only one binary carries, or a binary id and a full
failing test path separated by a space, and an ambiguous key is refused); the two maps are
files, so a score is offline and repeatable. Every other one is an ESCAPE. A libtest binary whose
`Running` line has no `test result:` line before its step ends died, hung or was cancelled,
and counts as failed, and so does a nextest test whose last status is a word the scorer
does not know (a `SLOW` in the list after `Summary` is a slow pass). A nextest run with no
`Summary`, a test list nextest could not build, and a step that failed with no binary named
are unplaced. A failure it cannot place on a binary is UNPLACED and fails the score
like an escape (exit 1). A skipped binary with no verdict in the logs it was given is
UNVERIFIED (exit 3 when nothing else is wrong), because only a job's own log shows its
tests ran: read every job's log by its name, never the aggregate result of a matrix job,
which cannot show a leg that never ran. Shadow escapes must be 0 over the shadow period
before any job is routed, and a maintainer reviews the shadow data, flakes included,
before any routing lands. A run that left no `router-shadow` artifact counts against the
period: its selection cannot be scored.

The router's self-test in `lint` reads only its fixture. `router-shadow` first runs
`ci_test_router.py --live-check`, which pins shapes the real tree keeps (the vizd binary
reached through `viz_client.rs`, the empty change selecting nothing, a head other than the
checkout refused), so a change to the tree reds that unrequired job and never `lint`. The
selection is computed and uploaded even when the live check fails.
