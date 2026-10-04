# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- `cerulion bag play <BAG> --resim all --record-out <PATH>` writes the frames the re-executed graph published to a new bag: one channel per graph-produced topic, labelled with the schema the re-executed graph publishes, with its name and provisioning copied from the input bag's channel where it has one. It works with and without `--verify`. The path must not exist, so an existing file is refused with exit 2 and never overwritten, and a run that does not finish removes the partial file. The `--report` JSON gains a `record_out` field naming the written path; reports from runs without the flag are unchanged, and `report_version` is unchanged.
- `cerulion-wsd` creates and builds. `graph.create`, `node.create` and `schema.create` do what `cerulion graph create`, `node create` and `schema create` do and answer with the `version` of the file they wrote, which a graph or node edit can send as `expect_version`. `node.build` runs `cerulion node build` and streams cargo's compiler messages as structured `diagnostic` events (file, line, column, level, message, code), then a `done` event; a client that closes its connection cancels the build. The protocol version is unchanged: an older daemon answers the new verbs with `unknown_verb`, and every existing verb is as it was. `cerulion_cli_engine` gains `node_cmd::node_build_streaming`, and `node create`'s trigger-policy defaulting moved from the `cerulion` binary into `node_cmd::resolve_create_policy` so the daemon applies the same rules.
- `cerulion-vizd` answers a new `sample` request: the newest 1 to 20 messages of an attached topic, each as `seq`, `ts_ns`, `size`, the decoded `fields` where the schema is known (or `null`) and a one line `summary`. The daemon keeps frames only for a topic that is being sampled, drops the ring five seconds after the last request, keeps at most eight rings, and keeps no body larger than 16 KiB, so sampling adds no subscription and a bounded amount of memory. The control protocol version in the banner is unchanged; a daemon without the verb answers it with the structured unknown-method error. See `docs/user-api.md`.

### Fixed
- `cerulion node create` refuses two outputs of one name, and (outside `--raw-ffi`) an output named like an input, with a validation error. Both used to scaffold a node whose source could not build.
- A graph could die with no panic text when the `cerulion` host and a node cdylib had been built against different layouts of a transport type. A type whose field types change is re-packed by the compiler, which can leave its size, its alignment and its field names identical and still move its fields, so a node built earlier reads a field at an offset the host no longer writes. This takes the node cdylib ABI from 24 to 25, so every prebuilt node cdylib must be rebuilt (`cerulion node build <type>`) before it will load.

### Changed
- An input that declares no trigger no longer creates an event listener. Such an input is read on its own node's fire by the step's snapshot and is woken by nothing, so the port that would sit in every publisher's notify path for it is not created at all; a topic's live listener count and its expected in-process total both drop by one per such input. `CerulionSubscriber::wait_for_message`, and `AnySubscriber::wait_for_message` which forwards to it and is the method a node body can reach through the prelude, now return an error naming the topic ("this subscriber was built with no event listener, so there is nothing here to wait on") instead of waiting out its timeout and returning `Ok(0)`. That entry serves subscribers a tool builds for itself, the `cerulion topic` observer among them; a node reads a declared input through its own tick or has the step drain it.
- The minimum supported Rust version is 1.95 (it was 1.93), and the release workflow builds with Rust 1.95.0. `cerulion_core` calls `try_update` on its atomics, the name Rust 1.95 stabilised and Rust 1.99 uses in place of the deprecated `fetch_update`; a workspace that denies warnings builds again under the current stable.
- The shared memory transport moves to iceoryx2 0.10.0. Every event service is sized to the event
  ids the transport actually mints rather than the library default, so a listener no longer walks
  256 shared memory counters on every wait.

### Known issues
- On macOS, a process can run one graph containing plugin nodes. After that graph, creating
  further topics in the same process fails. Run one plugin graph per process on macOS, or run on
  Linux, where the limit does not apply. The cause is an upstream defect in iceoryx2 0.10.0
  (eclipse-iceoryx/iceoryx2 issue 2034) and this note goes away when it is fixed.
- Publishing a message allocates once, on every platform. The allocation is inside iceoryx2
  0.10.0, which builds a small shared cell per loan; 0.9.1 did not, and there is no way to avoid
  it through the library's API. It costs 14 to 19 nanoseconds and it is on the publish path, so a
  program with a hard real time budget should know it is there. The cause is
  eclipse-iceoryx/iceoryx2 issue 2035 and this note goes away when it is fixed.

### Added
- `cerulion-wsd` can wire and unwire a graph and remove a node from it. `graph.wire` and `graph.unwire` add or delete one `inputs:` entry, and `graph.unstage` deletes a node entry. Each takes an optional `expect_version`, edits the file in place so comments and layout survive, and refuses a wire between different schemas (`schema_mismatch`) or removing a node other nodes read from (`would_break`, with the wires listed) unless `force` is set. The protocol version is unchanged.

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
