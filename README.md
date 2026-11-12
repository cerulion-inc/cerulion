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

**[Install](#install) · [Run the example](#quickstart-run-record-and-verify) · [Performance](#performance) · [ROS 2](#bring-your-ros-2-stack) · [Architecture](#how-it-fits-together) · [Docs](https://docs.cerulion.com) · [Guides](https://docs.cerulion.com/cerulion/guides)**

![Record a run, verify it passes, change the controller, and watch the same recording fail verification.](docs/media/quickstart-record-verify.gif)

## Install

```bash
curl -fsSL https://raw.githubusercontent.com/cerulion-inc/cerulion/main/tools/scripts/install.sh | sh && . "$HOME/.cerulion/env"
```

The installer links `cerulion` into `~/.local/bin`, adds one line to your shell startup files and installs Rust when Rust 1.95 or newer is missing. Then sign in once with `cerulion login`, because every command runs under a free Cerulion account, which is used for robot pairing and works offline afterwards. To uninstall, run the same script with `sh -s -- --uninstall`.

<details>
<summary>Homebrew</summary>

```bash
brew tap cerulion-inc/cerulion
brew install cerulion-inc/cerulion/cerulion
```

If Homebrew 6.0.x refuses the tap, run `brew trust --formula cerulion-inc/cerulion/cerulion` first and try again.

</details>

<details>
<summary>Debian / Ubuntu</summary>

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

Each step runs only if the one before it succeeded, and the [APT distribution page](docs/packaging/apt.md) pins the same key fingerprints and has a longer form that names a failed step.

</details>

<details>
<summary>Build from crates.io or source</summary>

```bash
cargo install --locked cerulion_cli cerulion_netd
```

The release installer places `cerulion-connectd` (the desk side of a remote connection) and `cerulion-remoted` (the robot side) beside the CLI. See the [installation guide](https://docs.cerulion.com/cerulion/installation) or [CONTRIBUTING](.github/CONTRIBUTING.md) for the complete source-build route.

</details>

<details>
<summary>Platforms, ROS 2 libraries and toolchain</summary>

In fish, end the line with `&& source ~/.cerulion/env.fish` instead.

Homebrew and the Debian package do not provision Rust. To build your own nodes, run `cerulion-install-rust` once, which installs current stable Rust unless a Rust 1.95 or newer is already there, and put `${CARGO_HOME:-$HOME/.cargo}/bin` on your PATH. On a terminal, `cerulion workspace create` and `cerulion node build` offer to install it when Rust 1.95 or newer is missing.

Release binaries support Linux x86_64 and ARM64 (glibc 2.31+, Ubuntu 20.04 and later) and macOS Intel and Apple Silicon. The Linux packages also carry the ROS 2 Jazzy rmw and heap hook that `cerulion ros2 run` and `cerulion ros2 launch` use. The macOS packages do not, because ROS 2 Jazzy has no macOS binaries. The rmw libraries run where their ROS 2 distro runs, which for Jazzy is Ubuntu 24.04. For ROS 2 Lyrical, each release also publishes `cerulion-rmw-lyrical-<version>-<target>.tar.gz` for Linux x86_64 and ARM64. Unpack it and set `CERULION_LIB_DIR` to the unpacked directory so `cerulion ros2 run` and `cerulion ros2 launch` load the Lyrical rmw. That library is built and tested in CI.

Building nodes needs a C toolchain (`build-essential` on Debian and Ubuntu, `xcode-select --install` on macOS) and Rust 1.95 or newer. Nodes load through a compiler-independent interface, so any Rust 1.95 or newer can build them. The compiler does not have to match the one that built the `cerulion` CLI, and a new workspace pins no compiler. To bring a workspace from Cerulion 1.0, follow the migration steps in the [2.0.0 release notes](./CHANGELOG.md#200---2026-11-12). They move its `=1.0.0` pins to `=2.0.0` and finish with one `cerulion node build <node>` per node. The [installation details](docs/install.md) cover the toolchain setup.

</details>

## Quickstart: Run, record and verify

**Using a coding agent?** Paste this prompt into Claude Code, Codex or another agent that runs commands in its own terminal:

```text
Install Cerulion: curl -fsSL https://raw.githubusercontent.com/cerulion-inc/cerulion/main/tools/scripts/install.sh | sh && . "$HOME/.cerulion/env"
Run cerulion login in the background, show me the URL and code it prints, and wait for it to finish while I approve the sign-in in my browser.
Then run the quickstart at https://github.com/cerulion-inc/cerulion#quickstart-run-record-and-verify in your own terminal:
cerulion workspace create safety_demo --starter obstacle_avoidance, then cd safety_demo.
Build both nodes (cerulion node build laser_scanner --release, then cerulion node build safety_controller --release; the first build takes a few minutes).
Run cerulion graph validate obstacle_avoidance, then cerulion graph run obstacle_avoidance --release --record for at least five seconds, and stop it with Ctrl+C.
Verify the recording it printed: cerulion bag play "<recording>" --resim all --verify
In nodes/safety_controller/src/lib.rs, change the forward speed from 0.3 to 0.25, run cerulion node build safety_controller --release, and verify the same recording again.
Report what changed between the two verifications.
```

This example connects a synthetic laser scanner to a safety controller. The controller publishes a forward velocity of `0.3` when the path is clear and `0.0` when an obstacle is within half a meter.

```bash
cerulion workspace create safety_demo --starter obstacle_avoidance
cd safety_demo
```

To write nodes from scratch instead, follow the [getting-started tutorial](docs/tutorials/01-getting-started.md).

Sign in once per machine. Cerulion commands run under an account, and the result is read locally from then on, so later commands keep working offline.

```bash
cerulion login
```

A robot or a headless box prints a short code to approve from a browser on any machine.

Build both nodes, check the graph, then run it with recording on. The first node build also compiles the Cerulion runtime, so it takes a few minutes on a laptop. Later builds take seconds.

```bash
cerulion node build laser_scanner --release
cerulion node build safety_controller --release
cerulion graph validate obstacle_avoidance
cerulion graph run obstacle_avoidance --release --record
```

The run splits into one process per node without changing the graph file, as the [multi-process guide](https://docs.cerulion.com/cerulion/guides/run-multi-process) explains.

By default the run is readable from any computer signed in to your account, with no addresses or flags. Add `--network off` to keep a run on this machine.

In a second terminal, watch the controller react:

```bash
cerulion topic echo /obstacle_avoidance/safety_controller/linear_velocity
```

The `x` field switches between `0.3` and `0.0` as the synthetic scan crosses the distance threshold. Let it run for at least five seconds, then stop it with **Ctrl+C**. Cerulion prints the completed recording's path:

```text
recordings/obstacle_avoidance_20260918T181449Z.mcap
```

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

If Cerulion is useful to you, a star helps other robotics engineers find it.

<details>
<summary>What replay verification checks</summary>

Cerulion re-executes the graph with recorded inputs and execution context, then compares the covered outputs against the recording. A pass means the changed code produces the same covered outputs on those recorded inputs, and each recording you keep adds another case to check against.

- Exit **0**: compared frames match.
- Exit **1**: output data diverged, and the report names the topic or field.
- Exit **6**: the execution schedule diverged.
- Exits **2 through 5**: verification could not complete, and the error says why.

Comparisons are byte-exact by default. Add `--tolerance` for acceptable per-field differences and `--report` for a machine-readable report.

</details>

Guide: [Record and replay](https://docs.cerulion.com/cerulion/guides/record-and-replay)

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

On one x86-64 Linux desktop (24-core Intel Core Ultra 9 285K, stock settings, recording on, nominal 100 Hz load, 2026-09-16), a 1 MiB round trip has a median of 4.08 µs with the default multi-process layout and 10,952 µs on ROS 2 Jazzy with `rmw_fastrtps_cpp` defaults: about 2,700× faster than ROS 2 Jazzy defaults at 1 MiB. One machine, not a guarantee. The raw results are in [`docs/benchmarks/results/8a84baf25d5d1710-2026-09-16-fixed100-heroes`](docs/benchmarks/results/8a84baf25d5d1710-2026-09-16-fixed100-heroes/HEADLINE.md).

### Native graph round trips

Each cell is **p50 / p99** in microseconds, measured on the Linux x86_64 host above at a nominal 100 Hz offered load with the advertised payload size on both legs. Timing starts after the publisher's loan and ends on echo receipt, so sensor production and payload processing add their own work. The Cerulion figures include recording and the ROS 2 figures do not.

| Payload | Cerulion multi-process (default), p50 / p99 in µs | Cerulion single-process, p50 / p99 in µs | ROS 2 Jazzy / `rmw_fastrtps_cpp` defaults, p50 / p99 in µs |
|---|---:|---:|---:|
| 64 B | 4.07 / 6.61 | 2.69 / 9.41 | 311 / 568 |
| 1 MiB | 4.08 / 7.38 | 2.76 / 10.74 | 10,952 / 43,402 |
| 16 MiB | 4.43 / 10.45 | 2.80 / 14.23 | 103,854 / 156,924 |

![Native round-trip latency, with median lines and p50-to-p99 bands, from 64 B to 16 MiB.](docs/media/native-rtt.svg)

The stock ROS 2 lane pipelines requests, so at 16 MiB a round trip outlasts the 10 ms pacing interval and requests overlap in flight. The [benchmark suite](benches/latency/README.md) documents the process layouts and timing boundaries.

Free run, the multi-process default, against barrier lockstep: [Free run against lockstep](docs/PERFORMANCE.md#free-run-against-lockstep).

### ROS 2 over Cerulion

The `rmw_cerulion` line in the chart above is ROS 2 Jazzy running over Cerulion's transport, measured in the same harness, posture and pacing as every other line: three ROS 2 nodes in three processes, with loaned publishing and loaned subscription takes. At 1 MiB that is **11.12 µs p50 / 37.21 µs p99**, beside **11.0 ms / 43.4 ms** for stock `rmw_fastrtps_cpp`, the chart's own stock ROS 2 line. The median stays between 10.43 and 10.65 µs from 64 B to 256 KiB and reaches 17.47 µs at 16 MiB. [Methodology](benches/latency/METHODOLOGY.md) · [Results and provenance](docs/benchmarks/results/8a84baf25d5d1710-2026-09-18-fixed100-rmw-cerulion/).

### Across platforms

![Native round-trip latency on Linux x86_64, NVIDIA Jetson and Apple Silicon, with single- and multi-process results.](docs/media/platform-rtt.svg)

The Apple M4 row predates the macOS data wake change, which [macOS two-process data wake](docs/PERFORMANCE.md#macos-two-process-data-wake) measures.

Source measurements, build provenance and configuration receipts are in [the benchmark results](docs/benchmarks/results/). Lines show p50 and bands extend to p99. Both figures show round trips.

## Write the node, declare the behavior

A typed input, a typed output and a `tick` method define the quickstart's safety controller. The macro supplies the port plumbing. The graph declares the wiring, and the runtime handles scheduling, transport, recording and observability. You do not configure an executor, register callbacks or write a spin loop.

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

Nodes can also be written in Python: `cerulion node create <type> --lang python --policy period_ms=<N>` writes one that `graph run`, recording and replay treat like a Rust node. It needs the `cerulion` Python package and a `cerulion` CLI, both built from a Cerulion source checkout, as [Python](docs/python.md) describes.

Here, `#[input(trigger)]` runs the controller when a scan arrives. The port deadlines make timing expectations observable through miss counters, alongside delivery-failure counts.

Other nodes can run on synchronized input sets or external events. For a periodic node, use `#[cerulion_node(period_ms = 10)]` instead of a data trigger. Set queue depth and backpressure per input. For example, `backpressure = sample(3)` limits reads to one per 3 ms.

Fixed fields write directly into the shared-memory slot. Variable-length fields have loaned writers too, and `self.image.data.fill_from(producer)?` lets a camera or generator fill the final buffer in place. An untouched output publishes nothing. If this node's `tick` returns an error, its outputs are discarded.

Guides: [Define a node](https://docs.cerulion.com/cerulion/guides/define-a-node) · [Wire a graph](https://docs.cerulion.com/cerulion/guides/wire-and-run-a-graph) · [Backpressure and deadlines](https://docs.cerulion.com/cerulion/guides/backpressure-and-deadlines)

## Bring your ROS 2 stack

**When you control startup**, keep your launch file and start it through Cerulion:

```bash
cerulion ros2 launch my_robot_bringup camera.launch.py
cerulion ros2 run my_robot_drivers imu_node
```

Both set up `rmw_cerulion` and run the ROS 2 command, one launch file or one executable at a time. Your nodes keep their ROS 2 APIs and executors. Neither rewrites your source, so publishers that copy keep copying until you migrate them. Inspect their topics with `cerulion topic list` and `cerulion topic echo`, which read the shared-memory plane. The rmw's discovery is process-local, so the ros2 CLI graph tools (`ros2 topic list`, `ros2 topic echo`, rqt) do not list them.

<details>
<summary>Optional: migrate C++ publishers to zero-copy loans</summary>

```bash
cerulion ros2 migrate --workspace ~/ros2_ws
cerulion ros2 migrate --workspace ~/ros2_ws --write
```

The first command previews which C++ publish sites can move to the loaned-message API, and `--write` applies the rewrite. Publishing and subscribing with loans are separate choices, and the [ROS 2 guide](https://docs.cerulion.com/cerulion/guides/bridge-ros2) covers both and what to expect from each.

</details>

**When the stack is already running**, attach to its DDS topics:

```bash
cerulion ros2 attach --iface <local-interface-IP>
```

Use the IP address of this computer's interface on the robot's LAN. Attach discovers the existing DDS stack and bridges its topics. It does not restart those nodes.

Use `ros2:` entries to launch ROS 2 processes beside native nodes in one graph. The [MoveIt 2 example](examples/moveit_hero/) shows an unmodified planning stack running over `rmw_cerulion`.

`rmw_cerulion` is tested in CI on Foxy, Galactic, Humble, Jazzy and Lyrical, the Linux releases ship it prebuilt for Jazzy and Lyrical, and `cerulion ros2 attach` is validated on Jazzy and on Foxy and Humble with local type resolution, as [ROS 2 compatibility](docs/ros2_compatibility.md) details. Exact replay of one eligible ROS 2 process at a time (`cerulion ros2 run --record-deliveries` and `--replay-deliveries`) is runtime-tested on Humble. A replay publishes live on its machine and still drives any device the process writes to directly, such as a serial port or CAN bus, so run it where neither reaches a live robot. [ROS 2 replay](docs/ros2_replay.md) lists which processes qualify and where a replay is safe to run.

Guides: [Bridge ROS 2](https://docs.cerulion.com/cerulion/guides/bridge-ros2) · [Compatibility and configuration](docs/ros2_compatibility.md)

## How it fits together

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/media/architecture-flow-dark.svg">
  <source media="(prefers-color-scheme: light)" srcset="docs/media/architecture-flow-light.svg">
  <img src="docs/media/architecture-flow.svg" alt="Robot-side graph scheduling and shared memory connect through separate LAN and WAN network planes to Cerulion Studio (early access), the CLI and coding agents.">
</picture>

- **On the robot:** the scheduler derives dependency levels from your graph, waits on data and timers and executes ready nodes across process groups. Nodes exchange loaned shared-memory messages, and capture records the data and execution context.
- **Across machines:** robots on your account, and robots this computer is already paired with, are discovered automatically. Iroh connects directly when it can, including on one LAN, and through a relay when it cannot. The network daemon mirrors subscribed topics locally for your tools. An unpaired machine reads a robot's topics over the LAN only when the robot runs with `CERULION_LAN_ACCESS=open`.
- **At your desk:** the CLI inspects, records and verifies the robot's data, and Cerulion MCP gives your coding agent tools for its recordings, your Cerulion Cloud instances and the robots on your account. A captured failure is a recording to inspect. Before re-running it, check `anchor.resimmable_reason` in the bag's `__cerulion/flashback.json` attachment.

### Why Cerulion

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/media/development-loop-dark.svg">
  <source media="(prefers-color-scheme: light)" srcset="docs/media/development-loop-light.svg">
  <img src="docs/media/development-loop.svg" alt="Cerulion connects a running robot, live inspection, failure capture and replay verification in one development loop.">
</picture>

- **Zero-copy, beyond fixed-size messages.** Write camera frames, scans and schema-defined variable-length fields directly into shared memory. Other local processes read the same bytes.
- **A scheduler that knows your graph.** Dependency levels are computed ahead of time, and periodic, data-triggered, synchronized and external nodes run when their inputs are ready. Recorded scheduling, explicit backpressure and deadlines and a virtual clock make that execution testable and deterministic on replay.
- **Recordings you can check code against.** Standard MCAP, byte-exact output comparisons, per-field tolerances and machine-readable verdicts for CI. Test your changed code against the inputs that actually reached the robot.
- **Flashback when you forgot to record.** Save the moments before and after a failure from a rolling in-memory history, then inspect the incident or replay its recorded topics.
- **Your robots, wherever they are.** Sign in to the same account on the robot and on your computer, and your computer finds the robot and reads its topics with no addresses or flags to set. The same single- and multi-process workflow runs on Linux and macOS, on your LAN or across the internet.
- **A path from ROS 2.** Run an existing stack through `rmw_cerulion`, attach to one already running, or combine ROS 2 processes and native nodes in the same graph.
- **Tools your coding agent can use.** The hosted Cerulion MCP endpoint gives an agent tools for your Cerulion Cloud instances, your MCAP recordings and the robots on your account, and Hindsight investigates recordings with evidence tied to topics and timestamps.

Guides: [Execution](https://docs.cerulion.com/cerulion/execution-order) · [Multi-process graphs](https://docs.cerulion.com/cerulion/guides/run-multi-process) · [Networking and remote robots](https://docs.cerulion.com/cerulion/guides/network-and-remote-robots) · [Remote access](docs/remote_plane.md)

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

Flashback keeps a memory-bounded window of recent activity in every running graph, so the moments before a problem are already there when you need them. It is event-based logging: nothing is written to disk until an event fires, and then the window around that event becomes a recording.

**It captures on its own** when the runtime sees a fault, such as a dying worker process or a stalled topic, and [Flashback](docs/flashback.md) lists every trigger.

**You can also ask for one by hand** from another shell on the robot:

```bash
cerulion flashback --note "controller stopped late"
```

Re-execution uses the capture's execution trace and any state anchors its starting point needs. `bag info` reports whether a capture can be re-executed or only played back. To record an already-running graph instead, use `cerulion bag record --run=obstacle_avoidance`.

Guides: [Flashback](docs/flashback.md) · [Record and replay](https://docs.cerulion.com/cerulion/guides/record-and-replay) · [Bags and replay](docs/bag.md)

## Tools for your coding agent

**Cerulion MCP** at `https://mcp.cerulion.com/mcp/cerulion` gives Claude Code, Codex and other MCP clients tools for your Cerulion Cloud instances, analysis of MCAP recordings served over `https://`, and a list of the robots on your account.

**Hindsight** at `https://mcp.cerulion.com/mcp/hindsight` serves the recording-analysis tools on their own, with answers grounded in specific topics and timestamps.

Both sign in with OAuth using your Cerulion account. The hosted endpoints have no local workspace tools, so build, run and inspect nodes and topics with the `cerulion` CLI.

The hosted endpoints are in early access. Add each one to your client as its own connector:

```bash
claude mcp add --transport http cerulion https://mcp.cerulion.com/mcp/cerulion
claude mcp add --transport http hindsight https://mcp.cerulion.com/mcp/hindsight
```

Hindsight and the hosted MCP server are closed source. [Request access](https://docs.cerulion.com/cerulion/hindsight).

Guide: [Connect an MCP client](https://docs.cerulion.com/cerulion/guides/connect-an-mcp-client)

## Roadmap

The direction is **review for robot changes**: bring recordings, simulations and behavioral evidence into the review process before a change reaches a robot.

| Area | What's next |
|---|---|
| **Faster execution** | Chain-fused nodes that pass data directly within a process, nanosecond-scale handoff as a measured target, and lower macOS cross-process wake-up latency. |
| **C++** | First-class node authoring and a client library, with clear boundaries for zero-copy access, scheduling and reproducible execution. |
| **Robot building blocks** | Native transforms and URDF support, parameters and lifecycle, with a clear path from the corresponding ROS 2 concepts. |
| **Flashback from node code** | Declare an incident from a node's own code, so a bumper or a grasp check saves the moment it sees. |
| **Services and actions** | A native request/response API for node authors on the shipped deterministic service layer, then actions: long-running goals with feedback, cancellation and streaming replies. |
| **Pull-request verification** | A GitHub reviewer that runs recorded regressions and cloud simulations, reports behavioral differences and preserves the evidence with the change. |
| **Scenario libraries** | User-defined environments and expected behavior, prior failure bags promoted to regression cases, and generated stress scenarios with reviewable test expectations. |
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

[Documentation](https://docs.cerulion.com) · [API reference](docs/user-api.md) · [Android build support](docs/android.md) · [Contributing](.github/CONTRIBUTING.md) · [Issues](https://github.com/cerulion-inc/cerulion/issues) · [Code of conduct](.github/CODE_OF_CONDUCT.md) · [Talk to the founders](https://cerulion.com/demo)

Report vulnerabilities through the [security policy](.github/SECURITY.md). Cerulion is built by [Cerulion Inc.](https://cerulion.com), a YC company. Commercial questions: licensing@cerulion.com.

## License

[AGPL-3.0-only](LICENSE), with a [commercial license](docs/legal/COMMERCIAL.md) available. The client-side `cerulion_wire`, `cerulion_link`, `cerulion_ops_proto` and `cerulion_pairing` crates are MIT OR Apache-2.0, and each manifest states its license.

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
