# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.0.0] - 2026-09-21

The first release of Cerulion. Its crates, binaries and Debian packages come from this tree.

### Changed
- The crates live under `crates/`, the tooling under `tools/`, and the user API reference under `docs/user-api.md`.
- The always-on window recorder is pinned and niced, and the dead-node walk left the live loop, so the multi-process round trip stays flat with the recorder on.
- A credit-blocked producer parks on the credit word instead of polling.
- A Flashback capture reads its frames after it snapshots its trace, and its coverage manifest marks every exit that left the close's final drain incomplete.
- The network posture pair and the service drain-all take halves were removed (ABI 21); services take one message at a time by design.
- Comments and documentation no longer carry internal tracker identifiers; the repository root holds the license, the readme, the changelog and the build manifests, with community files under `.github/` and legal files under `docs/legal/`.
- The default log filter is quieter: the per-topic producer reconciliation is one compact line of counters, the recorder's clean finalize keeps its one completion line, and startup, worker drain, exit hygiene and clean-finalize bookkeeping moved to `debug`, so `recording written to <path>` is the last line of a recorded run and a verification's `replay PASS` is no longer preceded by paragraphs of accounting. The long-running verbs still print their `info` lifecycle lines (worker spawn, trace ring and ready-file names, topic provisioning, daemon boot). The daemon's first-use spawn is reported at `info` (a boot still waiting after two seconds is reported at `warn`, so the quiet verbs are never silent for the whole readiness wait), the zenoh no-listener line is filtered by default, and a node cdylib announces its log filter only when `RUST_LOG` is set. The multi-process supervisor's planning build no longer prints a reconciliation line for a graph that never ran. Release builds compile `debug` out (`tracing`'s `release_max_level_info`), so the demoted lines are absent from a release build of `cerulion` (which hosts the recorder, `cerulion bagd`) and of `cerulion-netd` at any `RUST_LOG`; binaries built without cargo's `--release` keep them. The `debug-logging` cargo feature cannot lift that ceiling, and the documentation no longer says it can.

### Added
- Every `cerulion` command runs under an account. `cerulion login` signs a machine in once, with a short code to approve from a browser on any machine; later commands read the result locally and keep working offline. `login`, `completions`, `--help` and `--version` need no account.
- The release installer provisions the compiler that built the release. Every platform archive now carries the compiler metadata of its own build beside the binaries, and `install.sh` installs that exact rustc and its Cargo through rustup, verifies the full commit hash, and only then activates the binaries, so the compiler that builds your nodes is the one that built the CLI loading them. A failed setup leaves the previously installed binaries in place, though a partly installed Rust toolchain can remain under rustup's own management. Existing rustup defaults, profiles and shell startup files are left alone, `CARGO_HOME` and `RUSTUP_HOME` are honored, and an installation that carries Rust without rustup is refused rather than replaced. Archives from before the metadata still install their binaries, with a warning that they cannot set Rust up.
- The launch readme with the benchmark evidence packages, the benchmark charts on three platforms, the ROS 2 compatibility page and the second architecture panel.
- New workspaces select the compiler that built the CLI. `cerulion workspace create` and `cerulion workspace init` write a `rust-toolchain.toml` naming that release with the minimal profile, so `cerulion node build` inside the workspace uses it without an environment variable. The file is written only when a rustup toolchain whose release and full commit hash match the CLI is already installed; the check is offline, installs nothing, and leaves rustup's default alone. A missing, custom, nightly or beta compiler produces a warning instead, and an existing `rust-toolchain` or `rust-toolchain.toml`, symlinks included, is preserved. Workspaces created before this keep using an explicit `RUSTUP_TOOLCHAIN=`.

### Fixed
- A graph could abort with no panic text and no backtrace when the `cerulion` host and a node cdylib had been compiled by different rustc releases. A node crosses that boundary carrying Rust types whose layout the compiler chooses, and two releases can agree on every struct size and offset and still encode one of those types differently at run time, so the existing ABI-version check passed and the host then read a value the node never wrote and freed memory that was never initialized. A node cdylib now reports the compiler that built it, and the loader refuses a node whose compiler differs from the host's, naming both compilers and the two ways out: rebuild the node with the host's compiler, or reinstall the CLI with yours. `cerulion node build` also warns when the `rustc` on `PATH` reports a different release, while leaving Cargo's own compiler selection alone. This takes the node cdylib ABI from 21 to 22, so every prebuilt node cdylib must be rebuilt (`cerulion node build <type>`) before it will load.
- The examples cargo-check job skips the Go2 workspace and manifest-less directories; the fuzz job enters the crate at its new path.
