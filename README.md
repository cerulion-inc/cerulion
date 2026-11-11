<p align="center">
  <a href="https://cerulion.com">
    <picture>
      <source media="(prefers-color-scheme: dark)" srcset="docs/media/cerulion-logo-dark.svg">
      <source media="(prefers-color-scheme: light)" srcset="docs/media/cerulion-logo-light.svg">
      <img src="docs/media/cerulion-logo.svg" alt="Cerulion" width="420">
    </picture>
  </a>
</p>

**Cerulion (YC S24) is the operating system for physical AI: an open-source, zero-copy robot runtime that records runs, replays failures against new code, and reaches your robots from anywhere. It's ROS 2 compatible.**

[![License: AGPL-3.0-only](https://img.shields.io/badge/license-AGPL--3.0--only-blue.svg)](LICENSE)
[![crates.io: cerulion_cli 2.0.0](https://img.shields.io/badge/crates.io-cerulion__cli%202.0.0-brightgreen.svg)](https://crates.io/crates/cerulion_cli)
[![Rust 1.95+](https://img.shields.io/badge/rust-1.95%2B-orange.svg)](https://www.rust-lang.org/)
[![Docs](https://img.shields.io/badge/docs-docs.cerulion.com-blue.svg)](https://docs.cerulion.com)

## What ships today

- **Record and replay.** Runs record to standard MCAP. `cerulion bag play --resim all --verify` re-runs a supported native recording against new code and compares the outputs. Exit 1 names the topic that changed. ROS 2 nodes get analysis, not re-execution.
- **Hindsight, in early access.** Recording-analysis tools your agent calls over MCP. Hindsight investigates MCAP recordings and cites topics and timestamps. Hindsight and the hosted MCP server are closed source. [Request access](https://docs.cerulion.com/cerulion/hindsight).
- **Open source.** The runtime is AGPL-3.0-only, with a [commercial license](docs/legal/COMMERCIAL.md) available. The client-side wire crates are MIT OR Apache-2.0.
- **ROS 2.** `rmw_cerulion` runs ROS 2 nodes on Cerulion's transport and is validated on Jazzy. `cerulion ros2 attach` bridges a running robot's ROS 2 topics. It is validated on Jazzy, and on Foxy and Humble through local type resolution only. See [ROS 2 compatibility](docs/ros2_compatibility.md).

## Why it's fast

- **Zero-copy shared memory.** Nodes on one machine exchange messages through [iceoryx2](https://github.com/eclipse-iceoryx/iceoryx2) shared memory. A publisher writes into a loaned slot and other local processes read the same bytes.
- **Measured round trips.** On one x86-64 Linux desktop (24-core Intel Core Ultra 9 285K, stock settings, recording on, nominal 100 Hz load, 2026-09-16), a 1 MiB round trip has a median of 4.08 µs with the default multi-process layout and 10,952 µs on ROS 2 Jazzy with `rmw_fastrtps_cpp` defaults: about 2,700× faster than ROS 2 Jazzy defaults at 1 MiB. One machine, not a guarantee.
- **Recording stays on.** The Cerulion figures above include the capture plane; the ROS 2 figures do not include a recorder.
- **Platforms.** Release binaries run on Linux (x86_64, aarch64) and macOS.

The [Performance](#performance) section has the full table, the p99 figures and the timing boundaries. The raw results are in [`docs/benchmarks/results/8a84baf25d5d1710-2026-09-16-fixed100-heroes`](docs/benchmarks/results/8a84baf25d5d1710-2026-09-16-fixed100-heroes/HEADLINE.md).

![Record a run, verify it passes, change the controller, and watch the same recording fail verification.](docs/media/quickstart-record-verify.gif)

Install, sign in, and once you have a recording, check your current code against it:

```bash
curl -fsSL https://raw.githubusercontent.com/cerulion-inc/cerulion/main/tools/scripts/install.sh | sh && . "$HOME/.cerulion/env"
cerulion login
cerulion bag play "recordings/obstacle_avoidance_<timestamp>.mcap" --resim all --verify
```

The [quickstart](#quickstart-run-record-and-verify) builds the example graph and makes that recording with `cerulion graph run --record`.

**[Docs](https://docs.cerulion.com) · [Guides](https://docs.cerulion.com/cerulion/guides) · [Talk to the founders](https://cerulion.com/demo)**

If Cerulion is useful to you, a star helps other robotics engineers find it.

## The runtime

Cerulion combines [iceoryx2](https://github.com/eclipse-iceoryx/iceoryx2) zero-copy shared memory, a scheduler that follows your node graph, ROS 2 interoperability and secure access to remote robots. The runtime records the data and execution context that re-execution needs.

**[Install](#install) · [Architecture](#how-it-fits-together) · [Run the example](#quickstart-run-record-and-verify) · [Performance](#performance) · [ROS 2](#bring-your-ros-2-stack)**

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/media/development-loop-dark.svg">
  <source media="(prefers-color-scheme: light)" srcset="docs/media/development-loop-light.svg">
  <img src="docs/media/development-loop.svg" alt="Cerulion connects a running robot, live inspection, failure capture and replay verification in one development loop.">
</picture>

## Why Cerulion

- **Zero-copy, beyond fixed-size messages.** Write camera frames, scans and schema-defined variable-length fields directly into shared memory. Other local processes read the same bytes.
- **A scheduler that knows your graph.** Dependency levels are computed ahead of time; trigger policies decide when nodes can run. Eligible nodes in the same level can run in parallel. Event-driven waits, reusable buffers and placement by measured cost reduce overhead; recorded scheduling and input decisions support deterministic replay.
- **Low latency, including the tail.** A 1 MiB native round trip measures 4.08 µs median and 7.38 µs p99 in the Linux multi-process benchmark, with recording enabled. [See the measurements and configurations](#performance).
- **Execution you can test.** Periodic, data-triggered, synchronized and external nodes; explicit backpressure and deadlines; recorded execution traces and a virtual clock for deterministic re-execution.
- **Recordings you can check code against.** Standard MCAP, byte-exact output comparisons, per-field tolerances and machine-readable verdicts for CI. Test your changed code against the inputs that actually reached the robot.
- **Flashback when you forgot to record.** Save the moments before and after a failure from a rolling in-memory history, then inspect the incident or replay its recorded topics.
- **Your robots, wherever they are.** Automatic account-based pairing connects owned and shared robots, with access grants controlling authenticated pub/sub. Discover nearby robots with mDNS; reach remote ones through Iroh hole punching or a relay.
- **A path from ROS 2.** Run an existing stack through `rmw_cerulion`, attach to one already running, or combine ROS 2 processes and native nodes in the same graph.
- **Tools your coding agent can use.** Cerulion MCP gives an agent tools for analyzing MCAP recordings, and Hindsight investigates recordings with evidence tied to topics and timestamps.
- **One workflow across machines.** Native single- and multi-process execution on Linux and macOS, with LAN topic delivery and remote data mirrored locally for your tools.

## Install

```bash
curl -fsSL https://raw.githubusercontent.com/cerulion-inc/cerulion/main/tools/scripts/install.sh | sh && . "$HOME/.cerulion/env"
```

In fish, end the line with `&& source ~/.cerulion/env.fish` instead. The installer links `cerulion` into `~/.local/bin` and adds one line to each of your shell startup files, and the end of the line puts it on the PATH of the terminal you pasted it in, so sign in straight away with `cerulion login`: a robot or a headless box prints a short code to approve from any browser. It also sets up Rust for building nodes: a Rust 1.95 or newer that is already there is left as it is, and otherwise it installs current stable Rust through rustup. To remove it, run the same script with `sh -s -- --uninstall`.

**Homebrew**

```bash
brew tap cerulion-inc/cerulion
brew install cerulion-inc/cerulion/cerulion
```

Homebrew treats a third-party tap as untrusted until you install a fully qualified name, which is why the second line spells the formula out. On Homebrew 6.0.x the tap itself can refuse first; run `brew trust --formula cerulion-inc/cerulion/cerulion` before the two lines and try again.

**Debian / Ubuntu**

```bash
sudo apt-get update && sudo apt-get install -y ca-certificates curl gnupg
keyring=$(mktemp) &&
  curl -fsSL https://apt.cerulion.com/cerulion-archive-keyring.gpg -o "$keyring" &&
  [ "$(gpg --show-keys --with-colons "$keyring" | awk -F: '$1 == "fpr" { print $10 }' | sort -u)" = "$(printf '%s\n' 953C69FD279D9256FA4B71C5DA5FDE5D0DC6D91B | sort -u)" ] &&
  sudo install -m 0644 "$keyring" /usr/share/keyrings/cerulion-archive-keyring.gpg &&
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/cerulion-archive-keyring.gpg] https://apt.cerulion.com stable main" | sudo tee /etc/apt/sources.list.d/cerulion.list >/dev/null &&
  rm -f "$keyring" &&
  sudo apt-get update && sudo apt-get install cerulion
```

The first line installs the three programs the second needs. The second is one chain, where every step runs only when the one before it succeeded, so a failed download, or a keyring holding any key other than the key the archive signs with, stops it before it writes a keyring file or a source list. The [APT distribution page](docs/packaging/apt.md) pins the same fingerprints in the same form, lists both during a key rotation, and carries a longer form of this recipe that names which step failed.

Homebrew and the Debian package do not provision Rust. To build your own nodes, run `cerulion-install-rust` once, which installs current stable Rust unless a Rust 1.95 or newer is already there, and put `${CARGO_HOME:-$HOME/.cargo}/bin` on your PATH. On a terminal, `cerulion workspace create` and `cerulion node build` offer to install it when Rust 1.95 or newer is missing.

<details>
<summary>Build the CLI from crates.io</summary>

```bash
cargo install --locked cerulion_cli cerulion_netd
```

The release installer places `cerulion-connectd` (the desk side of a remote connection) and `cerulion-remoted` (the robot side) beside the CLI. See the [installation guide](https://docs.cerulion.com/cerulion/installation) or [CONTRIBUTING](.github/CONTRIBUTING.md) for the complete source-build route.

</details>

Release binaries support Linux x86_64 and ARM64 (glibc 2.31+, Ubuntu 20.04 and later) and macOS Intel and Apple Silicon. The Linux packages also carry the ROS 2 Jazzy rmw and heap hook that `cerulion ros2 run` and `cerulion ros2 launch` use; the macOS packages do not, because ROS 2 Jazzy has no macOS binaries. The rmw libraries run where their ROS 2 distro runs: Jazzy on Ubuntu 24.04. For ROS 2 Lyrical, each release also publishes `cerulion-rmw-lyrical-<version>-<target>.tar.gz` for Linux x86_64 and ARM64: unpack it and set `CERULION_LIB_DIR` to the unpacked directory so `cerulion ros2 run` and `cerulion ros2 launch` load the Lyrical rmw. That library is built and tested in CI but has not been run on a robot, and unlike the Jazzy package it is not installed in a clean container before release.

Building nodes needs a C toolchain (`build-essential` on Debian and Ubuntu, `xcode-select --install` on macOS) and Rust 1.95 or newer. Native node libraries use a stable ABI and can load across Rust compiler versions and CLI updates while boundary major 1 remains supported. Rebuild existing node libraries once with `cerulion node build <node>`. The [installation details](docs/install.md) cover the toolchain setup.

## How it fits together

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/media/architecture-flow-dark.svg">
  <source media="(prefers-color-scheme: light)" srcset="docs/media/architecture-flow-light.svg">
  <img src="docs/media/architecture-flow.svg" alt="Robot-side graph scheduling and shared memory connect through separate LAN and WAN network planes to Cerulion Studio (early access), the CLI and coding agents.">
</picture>

- **On the robot:** the scheduler derives dependency levels from your graph, waits on data and timers and executes ready nodes across process groups. Nodes exchange loaned shared-memory messages; capture records the data and execution context.
- **Across machines:** mDNS discovers nearby robots and LAN topics stream to your computer. Account ownership and grants authorize remote access over Iroh, using direct peer-to-peer connections when possible and a relay when needed. The network daemon mirrors subscribed topics locally for your tools.
- **At your desk:** the CLI inspects, records and verifies the robot's data, and Cerulion MCP gives your coding agent tools for analyzing its recordings. A captured failure is a recording to inspect. Before re-running it, check `anchor.resimmable_reason` in the bag's `__cerulion/flashback.json` attachment.

Guides: [Execution](https://docs.cerulion.com/cerulion/execution-order) · [Multi-process graphs](https://docs.cerulion.com/cerulion/guides/run-multi-process) · [Networking and remote robots](https://docs.cerulion.com/cerulion/guides/network-and-remote-robots) · [Remote access](docs/remote_plane.md)

## Quickstart: Run, record and verify

This example connects a synthetic laser scanner to a safety controller. The controller publishes a forward velocity of `0.3` when the path is clear and `0.0` when an obstacle is within half a meter.

Choose the acquisition route for your installed CLI. The 2.0.0 CLI includes `--starter`, so it takes the second route; the first is for a 1.0.0 install.

### Released CLI 1.0.0

The public 1.0.0 binaries do not include the starter option. Obtain the matching
source tag and enter its complete example workspace:

```bash
git clone --depth 1 --branch v1.0.0 https://github.com/cerulion-inc/cerulion cerulion-starter-source
cd cerulion-starter-source/examples/obstacle_avoidance
```

Continue with the sign-in, build and run commands below. The
[example runbook](examples/obstacle_avoidance/README.md) describes the source.

### CLI builds with --starter

Use this route only when `cerulion workspace create --help` lists `--starter`:

```bash
cerulion workspace create safety_demo --starter obstacle_avoidance
cd safety_demo
```

This obtains complete source bundled with your CLI, including tests and all
variable-length scan fields, without GitHub credentials or a source download.
Exact release dependencies are selected and `starter.toml` records compiler
requirements. Existing paths are refused; failed installation leaves no partial
workspace. The generated README also shows explicit graph creation, staging and
wiring. To write nodes from scratch, start with
`cerulion workspace create my_robot` and the
[getting-started tutorial](docs/tutorials/01-getting-started.md).

Sign in once per machine. Cerulion commands run under an account, and the result is read locally from then on, so later commands keep working offline.

```bash
cerulion login
```

A robot or a headless box prints a short code to approve from a browser on any machine.

Build both nodes, check the graph, then run it with recording on. The first node build also compiles the Cerulion runtime, so it takes a few minutes on a laptop; later builds take seconds.

```bash
cerulion node build laser_scanner --release
cerulion node build safety_controller --release
cerulion graph validate obstacle_avoidance
cerulion graph run obstacle_avoidance --release --record
```

The graph declares no `process_groups:`, so `graph run` derives a layout of one process per node and runs it without asking, leaving the graph file unchanged. Add `--yes` to save that layout in the graph YAML (with a `.bak` backup), or run `cerulion graph partition obstacle_avoidance` to review it and save it interactively.

By default a run is open to every machine on your LAN, so the CLI on another computer can find and read it: LAN peers are not authenticated and the default `tcp/` carrier is not encrypted. The log says so when the run starts. `--network off` turns off only this run's own gateway; while `cerulion-netd` serves the LAN, a LAN peer that names a live topic of any local run is still sent it. To refuse every LAN peer on this machine, set `CERULION_LAN_ACCESS=paired` in the environment of every Cerulion command and daemon, and restart a `cerulion-netd` that is already running.

In a second terminal, watch the controller react:

```bash
cerulion topic echo /obstacle_avoidance/safety_controller/linear_velocity
```

The `x` field switches between `0.3` and `0.0` as the synthetic scan crosses the distance threshold. Let it run for at least five seconds, then stop it with **Ctrl+C**; Cerulion prints the completed recording's path, `recordings/obstacle_avoidance_<timestamp>.mcap`, where the timestamp reads like `20260918T181449Z`. To find the file again:

```bash
ls recordings/
```

Besides `recordings/`, the node builds leave `target/` in the example directory; a run leaves a `~/.cerulion/runs` directory and the `cerulion-netd` network daemon, which exits on its own after 30 seconds idle.

Set `BAG` to that printed path, then verify the unchanged code first:

```bash
BAG="recordings/obstacle_avoidance_<timestamp>.mcap"
cerulion bag play "$BAG" --resim all --verify
```

A successful verification prints `replay PASS` and exits successfully: the outputs covered by the report match the recording.

Now open `nodes/safety_controller/src/lib.rs` and change the clear-path velocity from `0.3` to `0.25`. Rebuild that node and verify against the **same recording**:

```bash
cerulion node build safety_controller --release
cerulion bag play "$BAG" --resim all --verify
```

This time verification reports a divergence and returns a nonzero exit code. On the same sensor inputs, the changed controller would command a different speed. In CI, that verdict flags the changed behavior for review before the code reaches a robot. The animation at the top of this page shows this whole run.

<details>
<summary>What replay verification checks</summary>

Cerulion re-executes the graph with recorded inputs and execution context, then compares the covered outputs against the recording. It checks behavior under those inputs; a passing result is not a claim about every environment the robot could encounter.

- Exit **0**: compared frames match.
- Exit **1**: output data diverged; the report names the topic or field.
- Exit **6**: the execution schedule diverged.
- Exits **2 through 5**: verification could not complete; the error identifies why.

Comparisons are byte-exact by default. Add `--tolerance` for acceptable per-field differences and `--report` for a machine-readable report.

</details>

Guides: [Quickstart](https://docs.cerulion.com/cerulion/quickstart) · [Record and replay](https://docs.cerulion.com/cerulion/guides/record-and-replay)

## Check a model change against a recording

The [perception example](examples/perception/) records a synthetic camera and pixel-based detector, verifies the baseline, then tests a detector whose boxes move by eight pixels. Its bounding-box overlap tolerance catches the change and names the affected field.

From the repository root:

```bash
cd examples/perception
```

```bash
RECORD_SECS=8 ./run_replay_demo.sh
```


Choose tolerances that match the behavior you care about: bounding-box IoU, absolute or relative error, RMSE and set comparisons. Comparisons are byte-exact unless you set a field, topic or document-wide tolerance. The same checks can run on pull requests against recordings your team keeps as regression cases.

## Performance

### Native graph round trips

Local shared-memory delivery stays nearly flat as payloads grow. These measurements include the native capture plane on the measured Linux x86_64 host.

Each cell below is **p50 / p99**, in microseconds: median latency and the threshold at or below which 99% of measured round trips fall. Both legs use the advertised payload size.

The native benchmark loans that payload without filling it. Timing starts after the publisher's loan and metadata setup and ends on echo receipt; the echo loans the same payload size and forwards the timestamp. Sensor production and payload processing add their own work.

| Payload | Cerulion multi-process (default), p50 / p99 in µs | Cerulion single-process, p50 / p99 in µs | ROS 2 Jazzy / `rmw_fastrtps_cpp` defaults, p50 / p99 in µs |
|---|---:|---:|---:|
| 64 B | 4.07 / 6.61 | 2.69 / 9.41 | 311 / 568 |
| 1 MiB | 4.08 / 7.38 | 2.76 / 10.74 | 10,952 / 43,402 |
| 16 MiB | 4.43 / 10.45 | 2.80 / 14.23 | 103,854 / 156,924 |

![Native round-trip latency, with median lines and p50-to-p99 bands, from 64 B to 16 MiB.](docs/media/native-rtt.svg)

This workload comparison uses a nominal 100 Hz offered load. The stock ROS 2 lane pipelines requests: at 16 MiB a round trip outlasts the 10 ms pacing interval, so requests overlap in flight. Native Cerulion includes recording; the ROS 2 and zenoh lanes do not. The [benchmark suite](benches/latency/README.md) documents the process layouts and timing boundaries.

### Free run and lockstep

A multi-process graph free-runs by default: each process steps on its own clock and none waits on a barrier. `CERULION_EXECUTION_MODE=lockstep` opts out into barrier lockstep. Measured on one build and one Linux x86_64 machine, two processes, fixed 100 Hz, six interleaved reps per mode, recording enabled:

| Payload | Free run (default), p50 / p99 in µs | Lockstep, p50 / p99 in µs |
|---|---:|---:|
| 64 B | 4.23 / 7.60 | 5.08 / 10.47 |
| 1 MiB | 4.56 / 9.01 | 5.44 / 10.63 |
| 16 MiB | 5.22 / 12.34 | 7.22 / 13.64 |

Free run has the lower median and the lower p99 at every measured size; the lockstep median is about 20% higher up to 1 MiB. This machine is not the one behind the table above, so compare within each table. [All ten sizes](docs/PERFORMANCE.md#free-run-against-lockstep) · [Results and provenance](docs/benchmarks/results/3e0b99a7a88f0f38-2026-10-20-fixed100-free-run-vs-lockstep/).

### ROS 2 over Cerulion

The `rmw_cerulion` line in the chart above is ROS 2 Jazzy running over Cerulion's transport, measured in the same harness, posture and pacing as every other line: three ROS 2 nodes in three processes, with loaned publishing and loaned subscription takes. At 1 MiB that is **11.12 µs p50 / 37.21 µs p99**, beside **11.0 ms / 43.4 ms** for stock `rmw_fastrtps_cpp`, the chart's own stock ROS 2 line. The median stays between 10.43 and 10.65 µs from 64 B to 256 KiB and reaches 17.47 µs at 16 MiB. [Methodology](benches/latency/METHODOLOGY.md) · [Results and provenance](docs/benchmarks/results/8a84baf25d5d1710-2026-09-18-fixed100-rmw-cerulion/).

### Across platforms

![Native round-trip latency on Linux x86_64, NVIDIA Jetson and Apple Silicon, with single- and multi-process results.](docs/media/platform-rtt.svg)

These native runs include recording. On the measured Apple M4, single-process medians are 3.38 to 3.75 µs; multi-process medians are 87 to 93 µs. That M4 row predates the macOS data wake: on macOS 14.4 and later, a two-process consumer now blocks on a kernel wake word that the producer's publish rings, instead of waking on the park's recheck timer. Measured before and after that change on an Apple M3 Max, five reps per cell with the two builds alternating. This pair uses quiescent pacing rather than the charts' fixed 100 Hz and a different machine, so compare within the table only, and its recorder state is not recorded:

| Payload | Two-process before, p50 / p99 in µs | Two-process after, p50 / p99 in µs |
|---|---:|---:|
| 64 B | 70.17 / 141.54 | 10.42 / 28.21 |
| 4 KiB | 77.75 / 143.79 | 12.79 / 29.83 |
| 1 MiB | 80.12 / 152.42 | 16.14 / 41.67 |

Single-process medians on that machine rose by one to three ticks of its 24 MHz clock, inside overlapping per-rep ranges (64 B: 4.46 before, 4.50 µs after). The M4 chart row has not been re-measured since the change. [Results and provenance](docs/benchmarks/results/a83b79d4f8a3eb1b-2026-10-02-quiescent-macos-wake/).

Source measurements, build provenance and configuration receipts are in [the benchmark results](docs/benchmarks/results/). Lines show p50 and bands extend to p99; both figures show round trips.

## Bring your ROS 2 stack

**When you control startup**, keep your launch file and start it through Cerulion:

```bash
cerulion ros2 launch my_robot_bringup camera.launch.py
cerulion ros2 run my_robot_drivers imu_node
```

Both set up `rmw_cerulion` and run the ROS 2 command, one launch file or one executable at a time. Your nodes keep their ROS 2 APIs and executors. Neither rewrites your source: publishers that copy keep copying until you migrate them. The ros2 CLI graph tools (`ros2 topic list`, `ros2 topic echo`, rqt) do not see Cerulion topics yet, because the rmw's discovery is process-local; inspect them with `cerulion topic list` and `cerulion topic echo`, which read the shared-memory plane.

<details>
<summary>Optional: migrate C++ publishers to zero-copy loans</summary>

```bash
cerulion ros2 migrate --workspace ~/ros2_ws
cerulion ros2 migrate --workspace ~/ros2_ws --write
```

The first command previews which C++ publish sites can move to the loaned-message API; `--write` applies the rewrite. Publishing and subscribing with loans are separate choices; the [ROS 2 guide](https://docs.cerulion.com/cerulion/guides/bridge-ros2) covers both and what to expect from each.

</details>

**When the stack is already running**, attach to its DDS topics:

```bash
cerulion ros2 attach --iface <local-interface-IP>
```

Use the IP address of this computer's interface on the robot's LAN. Attach discovers the existing DDS stack and bridges its topics; it does not restart those nodes.

Use `ros2:` entries to launch ROS 2 processes beside native nodes in one graph. The [MoveIt 2 example](examples/moveit_hero/) shows an unmodified planning stack running over `rmw_cerulion`.

Guides: [Bridge ROS 2](https://docs.cerulion.com/cerulion/guides/bridge-ros2) · [Compatibility and configuration](docs/ros2_compatibility.md)

## Write the node, declare the behavior

A typed input, a typed output and a `tick` method define the quickstart's safety controller. The macro supplies the port plumbing; the graph declares the wiring and the runtime handles scheduling, transport, recording and observability. You do not configure an executor, register callbacks or write a spin loop.

```rust
use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;
use native_ros2_messages::sensor_msgs::LaserScan;

#[cerulion_node]
#[derive(Default)]
struct SafetyControllerNode {
    #[input(trigger, depth = 1, expect_within_ms = 100)]
    scan: LaserScan,
    #[output(promise_within_ms = 100)]
    linear_velocity: Vector3,
}

#[cerulion_node_impl]
impl SafetyControllerNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let ranges = self.scan.ranges();
        let stop = ranges.is_empty()
            || ranges.iter().any(|&r| r.is_nan() || r < 0.5);
        self.linear_velocity.x = if stop { 0.0 } else { 0.3 };
        Ok(())
    }
}
```

Here, `#[input(trigger)]` runs the controller when a scan arrives. The port deadlines make timing expectations observable through miss counters, alongside delivery-failure counts.

Other nodes can run on synchronized input sets or external events. For a periodic node, use `#[cerulion_node(period_ms = 10)]` instead of a data trigger. Set queue depth and backpressure per input; for example, `backpressure = sample(3)` limits reads to one per 3 ms.

Fixed fields write directly into the shared-memory slot. Variable-length fields have loaned writers too: `self.image.data.fill_from(producer)?` lets a camera or generator fill the final buffer in place. An untouched output publishes nothing; if this node's `tick` returns an error, its outputs are discarded.

Guides: [Define a node](https://docs.cerulion.com/cerulion/guides/define-a-node) · [Wire a graph](https://docs.cerulion.com/cerulion/guides/wire-and-run-a-graph) · [Backpressure and deadlines](https://docs.cerulion.com/cerulion/guides/backpressure-and-deadlines)

## Cerulion on a real arm

Cerulion works on real hardware, here on a six-axis industrial arm running ROS 2 and MoveIt. The recording shows the arm's discovered topics, its live 3D model driven by its joint states and a controller state plot; a phone video of the real arm, inset in editing, moves in step with the model. All six moves play at real speed, and a fast-forward mark shows where each pause between moves was cut.

![A six-axis arm's live 3D model, its topics and a controller state plot, with a phone video of the real arm inset and moving in step.](docs/media/studio-arm.gif)

Inspect the quickstart graph's topics with the CLI:

```bash
cerulion topic list
cerulion topic hz /obstacle_avoidance/laser_scanner/scan
cerulion topic echo /obstacle_avoidance/safety_controller/linear_velocity
```

Guides: [Inspect topics](https://docs.cerulion.com/cerulion/guides/inspect-topics) · [Remote robots](https://docs.cerulion.com/cerulion/guides/network-and-remote-robots)

## Capture the failure with Flashback

A robot rarely fails on cue. Flashback keeps a memory-bounded window of recent activity in every running graph, so the moments before a problem are already there when you need them. It is event-based logging: nothing is written to disk until an event fires, and then the window around that event becomes a recording.

**It captures on its own** when the runtime sees a fault: a worker process dies, a node is disabled after repeated panics, the run it was bound to vanishes, the e-stop is engaged, a topic stalls while its process stays alive, or a topic's rate collapses inside the window. Each capture holds the retained history plus a short period after the trigger, written to MCAP. A capture on a route that never produced at all is the one trigger that ships off.

**You can also ask for one by hand** from another shell on the robot:

```bash
cerulion flashback --note "controller stopped late"
```

Re-execution uses the capture's execution trace and any state anchors its starting point needs; `bag info` reports whether a capture can be re-executed or only played back. To record an already-running graph instead, use `cerulion bag record --run=obstacle_avoidance`.

Guides: [Flashback](docs/flashback.md) · [Record and replay](https://docs.cerulion.com/cerulion/guides/record-and-replay) · [Bags and replay](docs/bag.md)

## Tools for your coding agent

<details>
<summary>Cerulion MCP and Hindsight: tools for your coding agent</summary>

**Cerulion MCP** gives Claude Code, Codex and other MCP clients tools for analyzing MCAP recordings served over `https://`. The hosted endpoint has no local workspace tools: build, run and inspect nodes and topics with the `cerulion` CLI.

**Hindsight** investigates MCAP recordings with answers grounded in specific topics and timestamps.

The hosted endpoints are in early access. Register them with your client:

```bash
claude mcp add --transport http cerulion https://mcp.cerulion.com/mcp/cerulion
claude mcp add --transport http hindsight https://mcp.cerulion.com/mcp/hindsight
```

Guide: [Connect an MCP client](https://docs.cerulion.com/cerulion/guides/connect-an-mcp-client)

</details>

## Roadmap

The direction is **review for robot changes**: bring recordings, simulations and behavioral evidence into the review process before a change reaches a robot.

| Area | What's next |
|---|---|
| **Faster execution** | Chain-fused nodes that pass data directly within a process; nanosecond-scale handoff as a measured target; lower macOS cross-process wake-up latency. |
| **Python and C++** | First-class node authoring and client libraries, with clear boundaries for zero-copy access, scheduling and reproducible execution. |
| **Robot building blocks** | Native transforms and URDF support, parameters and lifecycle, with a clear path from the corresponding ROS 2 concepts. |
| **Flashback from node code** | Declare an incident from a node's own code, so a bumper or a grasp check saves the moment it sees. |
| **Services and actions** | A native request/response API for node authors on the shipped deterministic service layer, then actions: long-running goals with feedback, cancellation and streaming replies. |
| **Pull-request verification** | A GitHub reviewer that runs recorded regressions and cloud simulations, reports behavioral differences and preserves the evidence with the change. |
| **Scenario libraries** | User-defined environments and expected behavior, prior failure bags promoted to regression cases, and generated stress scenarios with reviewable test expectations. |
| **Model and embodiment adapters** | Versioned robot capabilities, model interfaces, execution feedback and recorded outcomes so models can move between embodiments. |
| **Agents and inspection** | A tighter loop for inspecting data, changing code and replaying results, with live runs, recordings and verification reports compared side by side. |
| **Platforms and interoperability** | Windows support, broader hardware validation and easier exchange with existing robotics data tools. |
| **Remote access** | Hardening the account-based pairing and remote access layer: robot-side provisioning, hosted account services and grant management across owned and shared robots. |

The roadmap is open for use cases and contributions: [discuss a feature](https://github.com/cerulion-inc/cerulion/issues). Release changes are recorded in the [changelog](CHANGELOG.md).

## Examples

| Example | What it demonstrates |
|---|---|
| [Obstacle avoidance](examples/obstacle_avoidance/) | Typed nodes, a live controller and replay verification. |
| [Camera pipeline](examples/camera_pipeline/) | In-place image generation and a data-triggered detector. |
| [Perception regression](examples/perception/) | Image-derived detections and a repeatable tolerance failure. |
| [V4L2 camera](examples/v4l2_camera/) | A Linux camera driver node watching a device fd, and a data-triggered brightness meter. Needs a camera. |
| [RealSense](examples/realsense/) | Color and depth from librealsense2 through a blocking SDK wait, and a nearest-obstacle range. Needs a RealSense. |
| [MoveIt 2](examples/moveit_hero/) | An existing ROS 2 planning stack using `rmw_cerulion`. |
| [Go2 quadruped](examples/go2/) | A DDS robot bridged, its camera transcoded, and a teleop stack with a safety mux and a sport driver. |

## Documentation and contributing

[Documentation](https://docs.cerulion.com) · [API reference](docs/user-api.md) · [Android build support](docs/android.md) · [Contributing](.github/CONTRIBUTING.md) · [Issues](https://github.com/cerulion-inc/cerulion/issues) · [Code of conduct](.github/CODE_OF_CONDUCT.md)

Report vulnerabilities through the [security policy](.github/SECURITY.md). Cerulion is built by [Cerulion Inc.](https://cerulion.com), a YC company. Commercial questions: licensing@cerulion.com.

## License

[AGPL-3.0-only](LICENSE), with a [commercial license](docs/legal/COMMERCIAL.md) available. The client-side `cerulion_wire`, `cerulion_link`, `cerulion_ops_proto` and `cerulion_pairing` crates are MIT OR Apache-2.0; each manifest states its license.

## Citing Cerulion

Use GitHub's **Cite this repository** button or [CITATION.cff](CITATION.cff):

```bibtex
@software{cerulion,
  author  = {Sharma, Lakshay and Kariyawasam, Raveen and {Cerulion Inc. and contributors}},
  title   = {Cerulion},
  year    = {2026},
  version = {2.0.0},
  url     = {https://github.com/cerulion-inc/cerulion}
}
```
