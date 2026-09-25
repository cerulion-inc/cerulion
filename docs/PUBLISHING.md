# Publishing Cerulion Crates to crates.io

This document records the **verified** publishing order and the pre-publish
checklist for the Cerulion workspace.

Before publishing this workspace, complete the manual DDS fork prerequisites in
[`docs/packaging/dds-forks.md`](packaging/dds-forks.md). The release workflow
publishes only this workspace; it does not publish `cerulion-rustdds` or
`cerulion-ros2-client`.

## Publishing Order

The order is derived from the intra-workspace **normal + build** dependency
graph (dev-dependencies do not constrain publish order: path-only
dev-dependencies are stripped by `cargo package`):

| Crate | Internal normal/build deps |
|---|---|
| `cerulion_macros` | (none; external only: proc-macro2, quote, syn) |
| `cerulion_core` | `cerulion_macros` |
| `native_ros2_messages` | `cerulion_core` (as dep **and** build-dep) |
| `cerulion_dds` | (none; published DDS forks are external dependencies) |
| `cerulion_discovery` | (none) |
| `cerulion_hygiene` | (none) |
| `cerulion_mdns` | (none) |
| `cerulion_pairing` | (none) |
| `cerulion_link` | (none) |
| `cerulion_telemetry` | (none) |
| `cerulion_wireclient` | `cerulion_link`, `cerulion_core`, `cerulion_pairing` |
| `cerulion_bag` | `cerulion_core` |
| `cerulion_netd` | `cerulion_core`, `cerulion_discovery`, `cerulion_hygiene`, `cerulion_mdns`, `native_ros2_messages`, `cerulion_link`, `cerulion_wireclient`, `cerulion_pairing` |
| `cerulion_bagd` | `cerulion_core`, `cerulion_bag`, `native_ros2_messages`, `cerulion_netd` |
| `cerulion_wsd` | `cerulion_core`, `cerulion_cli_engine`, `cerulion_hygiene` |
| `cerulion_cli_engine` | `cerulion_core`, `native_ros2_messages`, `cerulion_dds`, `cerulion_discovery`, `cerulion_hygiene`, `cerulion_mdns`, `cerulion_pairing`, `cerulion_bag`, `cerulion_bagd`, `cerulion_netd` |
| `cerulion_cli_tui` | `cerulion_cli_engine`, `cerulion_core` |
| `cerulion_cli` | `cerulion_cli_engine`, `cerulion_cli_tui`, `cerulion_core`, `cerulion_dds`, `cerulion_bagd`, `cerulion_bag`, `cerulion_netd`, `cerulion_telemetry`, `native_ros2_messages` |

Publish in this order:

1. **`cerulion_macros`**
2. **`cerulion_core`**
3. **`native_ros2_messages`**
4. **`cerulion_dds`**, **`cerulion_discovery`**, **`cerulion_hygiene`**, **`cerulion_mdns`**,
   **`cerulion_pairing`**, **`cerulion_link`**, **`cerulion_bag`**, and
   **`cerulion_telemetry`** (independent of one another after their listed
   prerequisites; `cerulion_telemetry` has no internal dependency)
5. **`cerulion_wireclient`**
6. **`cerulion_netd`**
7. **`cerulion_bagd`**
8. **`cerulion_cli_engine`**
9. **`cerulion_cli_tui`** and **`cerulion_wsd`** (independent of each other)
10. **`cerulion_cli`**

> Note: the intuitive order `cerulion_core → cerulion_macros
> → cerulion_cli` is impossible: `cerulion_core` has a normal
> dependency on `cerulion_macros`, so the macros crate must be published
> first. `native_ros2_messages`, the DDS platform crate, and the transport and
> recording crates must also be published before `cerulion_cli_engine`; all
> CLI crates are last.

### Dev-dependency cycles (intentional, do not "fix")

`cerulion_core` has **dev**-dependencies on `native_ros2_messages` and
`cerulion_cli_engine`, both of which depend on `cerulion_core` normally.
These dev-deps are deliberately **path-only** (no `version`): `cargo package`
strips path-only dev-dependencies, which is what breaks the cycle at publish
time. **Never add a `version` key to them**: doing so would make
`cerulion_core` unpublishable until crates that depend on it are published.

## Version Management

All publishable crates inherit `version` from `[workspace.package]` in the
root `Cargo.toml` and are released in **lockstep**. To bump:

1. Update `[workspace.package].version` in the root `Cargo.toml`.
2. Update the `version` on every internal path dependency entry under
   `[workspace.dependencies]` that has an explicit `version`, so each matches
   `[workspace.package].version`.
3. Run `bash tools/scripts/check_version_sync.sh` and confirm it exits 0.
   This script checks every `[workspace.dependencies]` entry containing both
   `path =` and `version =`, including multiline entries and dependency
   subtables, with either TOML quote style, and fails if none are found or any
   version mismatches.

## Release tags

Before tagging a release, set `CITATION.cff`'s `date-released` to a UTC
calendar date on or after the tagged commit's date and on or before the
release run date. The release workflows accept the inclusive range
`[tagged-commit-UTC-date, release-run-UTC-date]`, reject malformed or
impossible calendar dates, and reject dates in the future. This avoids making a
tag race unrelated merges whose commit dates are outside the operator's control.
Both workflows call
`tools/scripts/check_citation_release.sh` for this shared validation.

For a release being tagged today:

1. Compute the tagged commit's UTC date.
2. Set `date-released` to today's UTC date, or another valid date in the
   inclusive range beginning at the tagged commit's date.
3. Push the tag and confirm the workflow log reports the tag, version,
   tagged-commit date, release-run date, and accepted citation date.

A delayed re-tag may retain the original citation date only when that date
remains within the new tag and release-run date range.

## Pre-Publish Checklist

Run from the workspace root, in order:

- [ ] Versions bumped in lockstep (see above) and `CHANGELOG.md` updated.
- [ ] `cargo metadata --format-version 1 > /dev/null`: manifests parse and
      workspace inheritance resolves.
- [ ] `cargo fmt --all -- --check`
- [ ] `cargo clippy --workspace --all-targets -- -D warnings`
- [ ] The per-package test steps in `.github/workflows/ci.yml`, for every
      crate the release touches (`./tools/scripts/ci_test_shard.sh cerulion_core
      <shard> 4` for the four `cerulion_core` shards), plus the serial
      iceoryx2 test lists in the per-crate `AGENTS.md` files. **Not `cargo test --workspace`**:
      it deadlocks on iceoryx2's shared-memory singleton, which is why every
      test step in `ci.yml` enumerates its packages explicitly.
- [ ] For each publishable crate:
      `cargo package --list -p <crate> --allow-dirty` and verify:
  - `README.md` is present in every listing.
  - `cerulion_core`: no `fuzz/` entries (the cargo-fuzz sub-crate is
    excluded via `exclude = ["/fuzz"]`).
  - `native_ros2_messages`: all `msg/**/*.msg` files present. `build.rs`
    scans `msg/` at compile time, so a package without them builds an **empty**
    crate. To verify: `find crates/native_ros2_messages/msg -name '*.msg' | wc -l` gives the
    expected `.msg` file count; cross-check that
    `cargo package --list -p native_ros2_messages | grep -c '\.msg$'` matches.
- [ ] `cargo publish --dry-run -p <crate>` in the publish order above.
      Note: dry-run for a downstream crate fails until its upstream deps
      exist on crates.io, so the full dry-run chain can only be validated
      during an actual ordered release.
- [ ] `cerulion_dds`: confirm the crates.io owner/team before release and run
      `cargo publish --dry-run -p cerulion_dds --locked` after the DDS fork
      prerequisites are available.
- [ ] Test fixtures (`crates/test_fixtures/*`) and `crates/cerulion_core/fuzz` all carry
      `publish = false`: they must never reach crates.io.
- [ ] crates.io ownership: confirm publishing account / `cargo owner` team
      access for all eighteen publishable crates listed above.
- [ ] After each `cargo publish`, wait for the crates.io index to pick up
      the new version before publishing the next crate in the order.

## Known Caveats

- **License files**: `LICENSE-APACHE` and `LICENSE-MIT` live under `docs/legal/`
  and are not copied into each crate package. The SPDX `license` field
  (`AGPL-3.0-only`, workspace-inherited; `cerulion_link` and `cerulion_pairing`
  override it with `MIT OR Apache-2.0`) is what
  crates.io requires; per-crate license file copies can be added later if
  desired (cargo only auto-copies `readme`/`license-file`, and `license-file`
  is mutually exclusive with `license`).
- **MSRV** (`rust-version = "1.93"`, workspace-inherited): Cerulion's own code needs
  only 1.87 (`usize::is_multiple_of` in `crates/cerulion_core/src/wire.rs` and in
  codegen-emitted code), and the tree without the viz members floors at 1.88
  (darling/time/time-core/home/instability declare rust-version 1.88).
  The binding constraint is the `rerun` 0.34 SDK pulled by the
  `cerulion_viz` / `cerulion-vizd` members: it declares 1.92, but its transitive
  `fixed` 1.31.0 declares 1.93. ENFORCED: the `msrv` CI job runs
  `cargo +1.93.0 check --workspace --all-targets` on every push to `main`
  (and on `workflow_dispatch`; it is skipped on
  `pull_request`; see `docs/internals/ci-and-gates.md` § "CI job map"), so
  this floor is build-proven, not a survey.
- **Minimal-versions floor**: macro-emitted code calls
  `IndexMap::get_disjoint_mut` (added in indexmap 2.8); manifests
  require `indexmap = "2.8"` to match. A consumer who pins
  an older 2.x will be constrained to ≥ 2.8 by Cargo's semver resolver,
  which is the correct behavior.

## First publication: new-crate rate limit and partial publishes

Seventeen of the eighteen publishable crates exist on crates.io; `cerulion_telemetry` is
new and is published for the first time with the release that adds it. The `0.1.0` release
published eleven of them for the first time (`cerulion_bag`, `cerulion_bagd`,
`cerulion_dds`, `cerulion_discovery`, `cerulion_hygiene`, `cerulion_link`,
`cerulion_mdns`, `cerulion_netd`, `cerulion_pairing`, `cerulion_wireclient` and
`cerulion_wsd`). The new-crate limit below therefore applies again only when a
release adds a crate name; recheck the sparse index before publishing. Two
things follow from a first publication:

- crates.io rate-limits **new** crate names per account (a small burst, then
  one every ten minutes by default). `cargo publish --workspace` does not wait
  for that window; if it is hit, the run stops mid-sequence with the crates
  before it already published. `release.yml` therefore publishes through
  `tools/scripts/publish_crates.sh`, which sleeps until the instant crates.io names
  in its 429 (plus 10 s; 300 s when that does not parse) and retries, at most
  8 attempts within 90 minutes. Asking <help@crates.io> to raise the limit
  BEFORE cutting the tag still shortens the run; confirm the
  `CRATES_IO_TOKEN` secret carries the publish-new scope either way.
- A stopped `cargo publish --workspace` cannot be re-run as a whole: cargo
  refuses the whole invocation once any selected crate's version already
  exists. The script recovers on its own: every attempt asks the sparse index
  which crates already exist at their version and passes them as
  `--exclude`, so cargo publishes only the rest, still in dependency order.
  Re-running the failed `Publish to crates.io` job resumes the same way (a
  fully published workspace is a no-op); publishing by hand with
  `cargo publish -p <crate>` in the order above is the fallback only if the
  script itself cannot run.

`release.yml` and `release-artifacts.yml` fire independently on one tag push;
both refuse for the same citation/version reasons before touching
anything, so a stale `CITATION.cff` cannot publish crates.io while
producing no release.

`release-artifacts.yml` reads release assets through the GitHub API
(`gh release download` with the workflow token), never from
`releases/download/...` (404 on a non-public repository even with a token)
nor an unauthenticated `raw.githubusercontent.com` fetch; the install smoke
serves them to `install.sh` from a loopback mirror. To re-run only that smoke
against an existing release (no build, no publish), dispatch the workflow with
`smoke_tag=vX.Y.Z`.

A stable tag also publishes a `stable.txt` asset holding that tag on one line,
uploaded after the archives. That is the release channel: with no `--version`,
`install.sh` reads
`https://github.com/cerulion-inc/cerulion/releases/latest/download/stable.txt`
and installs the tag it names. GitHub never marks a prerelease "latest", and a
prerelease uploads no `stable.txt`, so the redirect always names the newest
stable release and a prerelease is never installed by default. The reader is
strict: a body over 64 bytes, a body that is not exactly one line, or a line
that is not a `vX.Y.Z` tag is refused with the remedy rather than installed.
`--base-url` moves that read along with the archive downloads, so a directory
holding a `stable.txt` beside the archives serves both, which is how the
installer self-test exercises the channel offline.

Every platform archive `release-artifacts.yml` builds carries a copy of the
`tools/scripts/install_rust.sh` helper and a `rustc-version.txt` beside the
binaries.
The metadata is the output of the same `rustc -vV` that built them, never a
compiler version written by hand into the mutable installer, and the archive
checksum covers both files. Before activating any binary, `install.sh` runs that
helper while it holds the install lock: the helper provisions the release
compiler and its Cargo through rustup and verifies the full commit hash, so a
failed setup leaves the previously installed binaries in place, though a partly
installed Rust toolchain can remain under rustup's own management. Existing rustup
defaults, profiles and shell startup files are preserved, and a rustup found
only on `PATH` is reused by adding the missing command proxies to
`CARGO_HOME/bin` without replacing a path that already exists. An archive that
predates both files still installs, with a warning that it cannot set Rust up;
an archive carrying only one of them is refused. A compiler is not enough on its
own: Cargo links through the system C toolchain, so when no `cc` is on `PATH` the
installer names the command that installs one before the user reaches a node
build, on either archive shape.
`tools/scripts/test_install_rust.sh` drives the helper and the installer
transaction against a hand-written rustup fixture, and CI runs it beside the
installer self-test.

After every binary is activated and verified, `install.sh` writes the PATH setup
that makes the install one command: `<cerulion home>/env` and a fish twin, each
prepending the install directory, and Cargo's `bin` when that run provisioned
Rust, guarded so a second source adds nothing; then one marked, idempotent line
sourcing it in `~/.profile`, in the bash startup files that already exist, in
`.zshenv` where zsh is present and in fish's `conf.d`. A profile that is a
symlink out of the home directory is left alone. `--no-modify-path` and
`CERULION_NO_MODIFY_PATH` skip the whole step, and so does an unset `HOME`; then
the PATH to set by hand is printed instead. The step runs last and its failures
are warnings, so a PATH that could not be written never fails an install whose
binaries are already in place. The self-test covers the first write, the second
run, both opt-outs, a symlinked profile, an unset `HOME` and fish.

## The Homebrew formula

Homebrew installs Cerulion from the tap repository `cerulion-inc/homebrew-cerulion`:
`brew tap cerulion-inc/cerulion` adds it and `brew install
cerulion-inc/cerulion/cerulion` installs the formula Homebrew reads there, `Formula/cerulion.rb` on its `main`. Homebrew reads the
default branch only, so a formula that has not reached that `main` reaches
nobody.

A release cannot carry its own formula in the tagged commit: the formula pins
the sha256 of archives that are built from that commit. So the `homebrew` job
of `release-artifacts.yml` runs after the release is published and proposes the
formula as a change to `main`:

1. It skips prerelease tags, and on a stable tag it stops with an `::error::`
   when `HOMEBREW_TAP_TOKEN` is absent, rather than passing green while
   `brew install` still serves the previous version.
2. It downloads the release's `SHA256SUMS` and renders the formula with
   `tools/scripts/render_homebrew_formula.sh`. An archive with no checksum in
   that file is refused; the formula never ships an unverified download.
3. It clones the tap repository with the token, puts the formula on a branch named
   `chore/homebrew-formula-<tag>`, commits it as `cerulion-release[bot]`, pushes,
   opens a pull request against `main`, prints its URL and merges it. Re-running the job
   updates that branch and prints the open pull request instead of opening a
   second one; the formula file itself is re-rendered each time, so an edit made
   to it by hand on that branch is replaced, while other files on the branch
   stay.

**A merge that fails fails the job loudly**, and the pull request then waits for a
hand merge. Until the rendered formula is on `main`, `brew install` serves the
formula already published there, so merging the pull request is what makes a
new release installable through Homebrew.

`HOMEBREW_TAP_TOKEN` is a fine-grained token scoped to the tap repository alone,
with contents read and write and pull requests read and write. The job keeps
`permissions: contents: read`, so the workflow token itself can write nothing:
the branch push and the pull request are the token's two jobs, and the tap's
`main` keeps its protection.

The rendered formula installs the three binaries, plus the ROS 2 Jazzy rmw and
the heap hook on Linux, generates shell completions, and carries the release's
own `install_rust.sh` and `rustc-version.txt` in `libexec` behind a
`cerulion-install-rust` command. Homebrew runs installs with `HOME` pointing at
a directory it deletes, so a formula cannot provision a Rust toolchain itself;
the caveat names that one command and the `PATH` line that follows it, and it
prints unconditionally, so `brew info cerulion` carries both before anyone
installs anything. `tools/scripts/test_render_homebrew_formula.sh`
renders the formula against a hand-written checksum file and checks it, so the
shape that reaches users can be checked without cutting a release; CI runs it
beside the installer self-test.

The formula points at `releases/download` URLs, which Homebrew fetches
unauthenticated, so a green job does not mean `brew install` works until the
repository is public.
