# Contributing to Cerulion

Thanks for your interest in Cerulion. We're genuinely glad you're here.

Cerulion is a deterministic, replayable, zero-copy alternative to ROS 2 for
real-time robotics. It's an ambitious project, and it gets better every time
someone new shows up. **No contribution is too small, and every contribution
is valued.** It does not matter whether you're a weathered systems programmer
or writing your first lines of Rust: if you build robots, you can help make
this better.

This guide walks you from "curious" to "merged." If anything here is unclear,
that's a bug in this document: open a Discussion and tell us.

## Code of Conduct

Participation in the Cerulion community is governed by our
[Code of Conduct](CODE_OF_CONDUCT.md). It describes the minimum behavior we
expect so this stays a place people want to be. Report concerns to
conduct@cerulion.com.

## Ways to contribute

You do **not** need to write Rust to help:

- **File a good bug report**: a clear reproduction is worth a lot.
- **Try the ROS 2 on-ramp.** Cerulion ships an experimental rmw layer
  (`RMW_IMPLEMENTATION=rmw_cerulion`) for running existing ROS 2 / MoveIt 2
  stacks on the Cerulion transport. It's still maturing: running yours and
  telling us what worked and what didn't is one of the most valuable
  contributions we get.
- **Improve docs or examples**: typos, unclear sections, a missing example
  node.
- **Reproduce or triage** an open issue.
- **Write code**: bug fixes, new schemas, CLI polish, transport work.

## Talk before you build

Two simple rules keep everyone's time well spent:

- **Bug fixes and small doc fixes are welcome cold.** If something is broken
  and you can fix it, just open the PR, no ceremony needed.
- **Features and behavior changes need a conversation first.** Before building
  one, open an issue or start a thread in
  [GitHub Discussions](https://github.com/cerulion-inc/cerulion/discussions)
  describing what you want to change and why. A two-line "here's what I want
  to do" saves you from building something we'd have steered differently, and
  it lets us point you at the right corner of the code. Design conversations
  belong before the code, not after.

Either way, if you're new, please introduce yourself in Discussions. We like
knowing who we're building with, and we'll happily help you find a good place
to start.

## Setting up your development environment

### Prerequisites

- **Rust 1.93 or newer**: this is our MSRV (minimum supported Rust version),
  declared as `rust-version` in the workspace `Cargo.toml`. Install via
  [rustup](https://rustup.rs/).
- **A supported platform:** Linux or macOS (x86_64 and aarch64 / Apple
  Silicon). Windows is **not** supported natively; WSL2 is untested.
- **iceoryx2 prerequisites.** Cerulion's local transport uses
  [iceoryx2](https://github.com/eclipse-iceoryx/iceoryx2) shared memory. On the
  platforms above a stable Rust toolchain is enough to build and run the test
  suite; no extra system packages are required for a standard `cargo build`.

### Build and verify your checkout

```bash
# Clone your fork
git clone https://github.com/<your-username>/cerulion.git
cd cerulion

# Build everything (this also builds the cdylib fixtures the test suite loads)
cargo build --workspace

# Run the core test suite: single-threaded, see the warning below
cargo test -p cerulion_core --tests --lib -- --test-threads=1
```

> **Do not run `cargo test --workspace`.** Cerulion's transport tests share an
> iceoryx2 shared-memory singleton, and one big parallel invocation makes them
> collide (hangs and spurious failures). Run the per-crate commands instead;
> the full list is in [The Quality Contract](#the-quality-contract) below and
> matches CI exactly.

If the build and core tests succeed, your environment is healthy. To see
Cerulion run end to end, install the CLI and follow the quickstart in the
[README](../README.md):

```bash
cargo build --release --locked -p cerulion_cli -p cerulion_netd -p cerulion_connectd
mkdir -p "$HOME/.cargo/bin"
install -m 0755 target/release/cerulion target/release/cerulion-netd \
  target/release/cerulion-connectd "$HOME/.cargo/bin/"
```

The quickstart's commands run under a Cerulion account: sign in once on that
machine with `cerulion login` before the first one; later commands read the
result locally and work offline.

The `cerulion ros2 run` and `cerulion ros2 launch` verbs also need
`librmw_cerulion.so` beside the CLI. The release packages carry it on Linux;
a source build gets it from `cargo build --release --locked -p rmw_cerulion`
run in a ROS 2 Jazzy environment (the build script generates the C ABI
bindings from the installed headers; without them it falls back to a snapshot
that is not deployable), then `install -m 0644 target/release/librmw_cerulion.so
"$HOME/.cargo/bin/"`. The heap hook the same verbs preload is plain Rust:
`cargo build --release --locked -p cerulion_heaphook` on Linux, then install
`target/release/libcerulion_heaphook.so` beside it. See
[ROS 2 compatibility](../docs/ros2_compatibility.md).

### Finding your way around

- [`docs/user-api.md`](../docs/user-api.md): the ground truth for the user-facing API:
  node macros, schemas, graph YAML, and the CLI.
- [`docs/`](../docs/): design and operations docs, including
  [multi-process execution](../docs/multi_process.md),
  [networking](../docs/networking.md), and
  [performance](../docs/PERFORMANCE.md).
- [`README.md`](../README.md): the quickstart and project overview.

## Where things go: Issues, Discussions, and chat

Please use the right channel: it keeps our issue tracker a clean, actionable
work queue.

| Channel | Use it for |
|---|---|
| **GitHub Discussions** | Introducing yourself, design proposals, "should we…?", open-ended Q&A |
| **GitHub Issues** | Concrete, actionable bugs and tasks |
| **Discord** | Real-time questions (*coming soon*) |

**Support questions don't belong in Issues**: please ask in Discussions. We
keep Issues for things with a clear, actionable definition of done.

When filing an issue, use the provided templates and include reproduction
steps, expected vs. actual behavior, and platform details.

### Issue labels

The repo's label taxonomy is defined in
[`.github/labels.yml`](labels.yml) (priority, contributor, workflow,
housekeeping, domain, `area/*`, and type categories). It is the source of
truth: edit that file and the `labels` workflow syncs it to GitHub (run it
from the Actions tab, or it runs automatically when `labels.yml` lands on
`main`). The sync **prunes**: any label not listed in the file is deleted from
the repo. Don't create labels by hand in the UI (the next sync removes them),
and make sure any label referenced by an issue template or workflow has an
entry in the file.

Maintainers: applying `needs-info` to an issue or PR asks the author for more
detail and starts a response clock: after ~7 quiet days the needs-info closer
workflow marks it, and after ~7 more it closes it (14 days total). Any author
reply resets the clock; remove `needs-info` once the question is answered.
Nothing is ever auto-closed unless a maintainer applied `needs-info` first.

## Your first contribution

Look for issues labeled
[**`good first issue`**](https://github.com/cerulion-inc/cerulion/labels/good%20first%20issue).
We keep these stocked and add a short note in each one about where the relevant
code lives.

**You don't need permission, and issues aren't assigned.** Just start working
and open a **draft PR** early: a draft is a great way to get feedback before
you've polished everything. If you're unsure whether your approach fits, say so
on the issue or in Discussions.

Good places to start: adding a new schema, writing an example node, improving a
CLI subcommand, or fixing a docs gap you hit during setup.

## The contribution flow

1. **Fork** `cerulion-inc/cerulion`.
2. **Branch** off `main` as `<type>/<kebab-slug>`: the type is one of
   `feat` `fix` `docs` `test` `refactor` `ci` `chore` `perf` `style` `build`,
   and the slug is lowercase letters, digits and hyphens:
   ```bash
   git checkout -b fix/topic-hz-panic
   ```
   CI checks this (`tools/scripts/check_pr_title.sh`, in the `lint` job). You can
   run it yourself first:
   ```bash
   ./tools/scripts/check_pr_title.sh --title "fix(cli): stop topic hz panicking" \
                                    --branch "fix/topic-hz-panic"
   ```
3. **Make your change**, with tests (see the Quality Contract below).
4. **Open a PR targeting `main`.** Draft PRs are welcome.

## The Contributor License Agreement (CLA)

Before your first PR can be merged, we ask you to sign our
[Individual CLA](CLA/individual.md). The CLA bot posts a signing link
on your first pull request; signing is one comment, and it never blocks the
review conversation, only the final merge. One signature covers all your
future contributions.

**Why we ask:** signing lets us offer Cerulion's own code under both the
open-source AGPL-3.0 license *and* a commercial license. That dual model is what
funds full-time work on the project.

**What signing means:** you **keep the copyright** to your contribution. You
grant Cerulion Inc. a broad license to use and relicense your contribution,
including under a commercial license. Signing is a license *grant*, not a
copyright *assignment*: it does **not** transfer ownership of your code, and it
does **not** take away any of your rights to it. The full text is
[`.github/CLA/individual.md`](CLA/individual.md).

**Contributing on behalf of a company?** If your employer owns the rights to
your work, an authorized representative should execute the
[Corporate CLA](CLA/corporate.md), which is handled manually via
[licensing@cerulion.com](mailto:licensing@cerulion.com); the instructions are
at the top of that document.

## The Quality Contract

Cerulion holds a high quality bar, not as bureaucracy, but because robots
depend on this code being correct, fast, and predictable. Here's the contract, and
here's exactly how to satisfy it before you ask for review. **These mirror what
CI runs**, with one difference called out below the block.

```bash
# 1. Formatting
cargo fmt --all -- --check        # auto-fix with: cargo fmt --all

# 2. Linting: warnings are errors
cargo clippy --workspace --all-targets -- -D warnings

# 3. Tests, one crate at a time (never `cargo test --workspace`)
cargo build --workspace           # builds the cdylib fixtures the suite loads
cargo test -p cerulion_core --tests --lib -- --test-threads=1
cargo test -p rmw_cerulion -- --test-threads=1
cargo test -p native_ros2_messages
cargo test -p cerulion_macros
cargo test -p cerulion_cli_engine
cargo test -p cerulion_cli
cargo test -p cerulion_cli_tui
```

The one difference: CI does not run `cerulion_core` serially. It splits that
suite into shards and runs each under `cargo nextest`
(`./tools/scripts/ci_test_shard.sh cerulion_core <shard> 4`, one process per
test), then runs the doctests with `cargo test -p cerulion_core --doc`. The
serial command above covers the same integration and unit tests on one machine.

CI additionally gates documentation
(`RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps`) and the
hot-path allocation lint described below. The exact CI commands live in
[`.github/workflows/ci.yml`](workflows/ci.yml); when in doubt,
mirror those.

### Code style

- **No dead code.** `dead_code`, `unused_imports`, and `unused_variables` are
  set to `deny`. Delete dead code rather than leaving it.
- **No allocations on the hot path.** The transport hot path must stay
  allocation-free; a CI lint (`tools/scripts/check_hot_path_allocs.sh`) enforces this.
  A genuine cold-path allocation needs a `// hot-path-alloc-ok: <reason>`
  annotation.
- **Structured logging.** Use `tracing` with structured fields, never
  `println!` in library code.
- **Error handling.** Use `thiserror` enums and propagate with `?`.
- **Test placement.** Integration tests live in
  `{crate}/tests/{module}_test.rs`; unit tests for private functions live in
  inline `#[cfg(test)]` modules; fuzz targets live in
  `crates/cerulion_core/fuzz/fuzz_targets/` (nightly toolchain).

### The iceoryx2 shared-memory tests

Cerulion's local transport is iceoryx2 shared memory, which is a
process-global singleton, so run the `cerulion_core` and `rmw_cerulion` test
suites **single-threaded** (`-- --test-threads=1`) locally. CI runs
`rmw_cerulion` the same way and gives each `cerulion_core` test its own
process instead. When iterating on a single test binary, keep the flag:

```bash
cargo test -p cerulion_core --test transport_test -- --test-threads=1
```

### Running benchmarks

Benchmarks live in [`benches/`](../benches/) (each is its own standalone project,
not part of the workspace). The public latency suite is `benches/latency/`:

```bash
python3 benches/latency/bench.py list-cells   # cell inventory, runs nothing
python3 benches/latency/bench.py smoke        # low-n gate vs this host's baseline
```

The CI regression gate runs `latency_threshold_test` in release mode:

```bash
cargo test -p cerulion_core --test latency_threshold_test --release -- --test-threads=1
```

### No fabricated data, ever

Cerulion's whole value is that its numbers are *real*. We never commit fake,
mocked, or simulated data in benchmarks, tests, or docs (this is core
Principle #13). If a test needs data, it uses real, measured, or
deterministically generated data, never an invented number. This is how we keep
our performance claims trustworthy, and we'd love your help holding that line.

**A red CI check is never you failing.** It's just CI telling you something needs
a tweak: push again, no problem. If you're stuck on a check, ask in Discussions
and we'll help.

## Commit and PR conventions

- **PR titles must be [Conventional Commits](https://www.conventionalcommits.org/)**
  in the form `<type>(<scope>): <description>`, e.g. `fix(transport): handle empty payload`
  or `docs: clarify setup steps`. This is checked by
  `tools/scripts/check_pr_title.sh` in CI's `lint` job, not by hand: we squash-merge,
  so the PR title becomes the commit message on `main`. The same script checks
  the branch name (step 2 of the flow above), and both are runnable locally.
  Individual commit titles inside the PR are not gated, only the PR title is.
- **Keep internal tracker ids out of branch names** (the gate rejects them),
  and out of PR titles by convention. If your change relates to an issue,
  reference it in the PR BODY (`Fixes #123`), where it links and stays
  editable; a branch name is permanent and quoted in every merge commit.
- In your PR description, tell us **what** changed, **why**, and **how to test**
  it.
- Add tests for new behavior.
- Reference related issues with `Fixes #123` or `Closes #123`.
- Run the Quality Contract checks before requesting review.

### Issue identifiers in comments

Comments and commit messages do not carry identifiers from the maintainers' internal issue tracker; a change is judged by the comment's own text. Link the related GitHub issue from the pull request body instead.

## Contributing with AI agents (optional)

Cerulion is an AI-native codebase, and you're welcome to use coding agents of
any kind. Whatever tool you use, the same bar applies:

- **You own your PR.** Review and *understand* everything you submit: you're
  accountable for it in review, not the tool.
- **The Quality Contract must pass**, exactly as it does for hand-written code.
- **Principle #13 is absolute.** Never let an agent invent benchmark numbers,
  test fixtures, or measured data.

## What to expect from review

A maintainer will take a look as soon as they can. Review is a conversation, not
a verdict: expect questions and suggestions, and don't hesitate to push back or
ask for clarification. Open early as a draft if you'd like feedback before things
are final.

Pull requests from maintainers also get an automated first-pass review: an
AI-driven review workflow runs on those pull requests and flags likely issues
shortly after they open or update. Its findings are advisory, not gates: a
maintainer always makes the final call. Pull requests from forks are reviewed
by a maintainer directly, because the workflow cannot run against a fork.

## Security issues

**Please do not report security vulnerabilities in public Issues.** See
[SECURITY.md](SECURITY.md) for our responsible-disclosure process and the
security contact address.

## License

Cerulion is distributed under the **GNU Affero General Public License v3.0 only**
([`AGPL-3.0-only`](../LICENSE)), an OSI-approved open-source license.

Contributions are accepted under our CLA (a broad license grant: **you keep
your copyright**) so that Cerulion's own code can be offered under both AGPL-3.0
and a commercial license. Third-party dependencies (for example iceoryx2 and
zenoh) keep their own licenses.

Commercial licenses, for shipping closed-source robots without AGPL obligations
on Cerulion's own code, are available; see [COMMERCIAL.md](../docs/legal/COMMERCIAL.md) or
contact licensing@cerulion.com.

---

Thank you for helping build a better foundation for robotics. We're glad you're
here.
