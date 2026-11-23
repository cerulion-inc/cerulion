<p align="center">
  <a href="https://cerulion.com">
    <picture>
      <source media="(prefers-color-scheme: dark)" srcset="docs/media/cerulion-logo-dark.svg">
      <source media="(prefers-color-scheme: light)" srcset="docs/media/cerulion-logo-light.svg">
      <img src="docs/media/cerulion-logo.svg" alt="Cerulion" width="420">
    </picture>
  </a>
</p>

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-AGPL--3.0--only-blue.svg" alt="License: AGPL-3.0-only"></a>
  <a href="https://crates.io/crates/cerulion_cli"><img src="https://img.shields.io/crates/v/cerulion_cli.svg?label=crates.io" alt="crates.io: cerulion_cli"></a>
  <a href="https://github.com/cerulion-inc/cerulion/actions/workflows/ci.yml"><img src="https://github.com/cerulion-inc/cerulion/actions/workflows/ci.yml/badge.svg?branch=main" alt="CI"></a>
  <a href="https://www.rust-lang.org/"><img src="https://img.shields.io/badge/rust-1.95%2B-orange.svg" alt="Rust 1.95+"></a>
  <a href="https://docs.cerulion.com"><img src="https://img.shields.io/badge/docs-docs.cerulion.com-blue.svg" alt="Docs"></a>
  <a href="https://discord.gg/dqKFecngPy"><img src="https://img.shields.io/badge/chat-Discord-5865F2.svg" alt="Discord"></a>
</p>

<!-- FIRST_SCREEN_START -->

**Cerulion is an open-source, zero-copy robot runtime that runs your ROS 2 nodes, replays recorded runs against new code and reaches your robots from anywhere.** Its experimental harness for physical AI gives your AI model MCP tools to see and move a simulated arm.

- **For ROS 2 teams.** [Run a stock talker and listener on `rmw_cerulion`](#try-it-with-ros-2) in minutes, with Jazzy on Linux and no account ([also on a real arm](#runs-on-real-hardware)).
- **For robot developers.** [Record a run, change one constant and replay it](#try-the-runtime) to see which output changed, with a free account and no robot.
- **For agent builders.** [Give your model a simulated arm](#try-the-harness) after an install of about 4 GB. Gemini 3.8 Flash solved 73 of 100 start states with Cerulion and 25 without ([limits](#evidence)).
- **To check the claims first.** [Read the evidence and its limits](#evidence), with nothing to install.

<p align="center">
  <img src="docs/media/replay-verdict.gif" alt="A recorded bag passes replay, then fails it after one changed constant: differs at x, recorded 0.3, replayed 0.25.">
</p>

A real run on Cerulion 2.1.0: the recorded bag replays byte for byte, then one changed constant in the safety controller makes the same bag fail at the exact frame and field ([how this GIF was made](docs/media/README.md#assets-the-root-readme-uses)).

<p align="center">
  <img src="docs/media/harness-episode.gif" alt="Claude Opus 5.5 moves the simulated arm through the Cerulion harness and sets the bowl on the plate, and the last frame reads Benchmark verdict: solved." width="480">
</p>

A real episode, scored solved by the benchmark's own check. Claude Opus 5.5 solved this task on its third try, and the task is one the harness was built on ([how this GIF was made](docs/media/README.md#assets-the-root-readme-uses)).

<!-- FIRST_SCREEN_END -->

## What Cerulion adds to the tools you use

| You use | What Cerulion adds |
|---|---|
| A plain MCP robot server, or the three tools that a published study of zero-shot models on LIBERO-10 gave them (`move_to`, `done`, `give_up`) | The [harness](#try-the-harness) gives your model tools to look through the robot's cameras, move, touch, grasp and set things down. It also gives the model facts about the robot's body, such as its calibration record. You see the benchmark's own verdict for each episode. Against the three published tools, Gemini 3.8 Flash solved 73 of 100 start states with Cerulion and 25 without, on the benchmark family the harness was built on, and 4 of 16 models solved fewer ([limits](#evidence)). |
| rosbag2, which records topics and plays them back | A [replay check](#try-the-runtime) re-executes your native Cerulion nodes on a recording and compares their outputs. An output difference exits 1 and a schedule divergence exits 6, so a CI runner that is signed in can stop the change. |
| A ROS 2 middleware, such as `rmw_fastrtps_cpp` or `rmw_zenoh_cpp` | `rmw_cerulion` is a ROS 2 middleware too, so stock nodes run on it unchanged. Native nodes add typed inputs and outputs in loaned shared memory, always-on recording and replay. |
| iceoryx2 or zenoh directly | Cerulion is built on iceoryx2 for shared memory, and adds nodes, scheduling and recording. Between machines, account and paired access uses iroh, directly or through a relay, and zenoh carries only explicit open LAN access ([networking](docs/networking.md)). At 64 B on one desktop, raw iceoryx2 reads 2.42 µs / 3.02 µs (p50 / p99), and native Cerulion reads 4.07 µs / 6.61 µs ([package](docs/benchmarks/results/8a84baf25d5d1710-2026-09-16-fixed100-heroes/HEADLINE.md)). |

<a id="quickstart"></a>

## Install

Cerulion runs on Linux (x86_64 or ARM64, glibc 2.31 or later, such as Ubuntu 20.04) and macOS (Intel or Apple silicon). The install command needs `curl`.

The install script checks the release's SHA-256, puts the programs in `~/.cerulion/bin`, links them into `~/.local/bin`, adds one line to your shell startup files and adds Rust through rustup when yours is missing, or is older than 1.95 and rustup is installed. Release builds send coarse usage events, with your account id after sign-in (a CI run sends under a random id). To opt out, run `export DO_NOT_TRACK=1` before your first command ([what the events carry](docs/telemetry.md)).

```bash
curl -fsSL https://raw.githubusercontent.com/cerulion-inc/cerulion/main/tools/scripts/install.sh | sh && . "$HOME/.cerulion/env"
cerulion --version
```

The script installs the latest stable release, 2.1.0 today, so `cerulion --version` prints `cerulion 2.1.0` or a later version. On Linux it also installs `rmw_cerulion`, the ROS 2 Jazzy middleware that the ROS 2 trial uses. If a step fails, `cerulion doctor` names each problem and prints a `fix:` line where it knows one.

| Status | What |
|---|---|
| Works today (2.1.0) | The runtime and the `cerulion` CLI on Linux and macOS, and `rmw_cerulion` for ROS 2 Jazzy on Linux. A [separate Linux archive](docs/install.md#what-the-install-script-does) carries it for Lyrical, tested in CI but not on a robot. |
| Works today, with limits | Remote access to the robots on your account, directly or through a relay. The default public relays are for development only ([details](#reach-your-robots-from-another-network)). |
| Experimental | The harness, on a simulated arm only. |
| Early access | The hosted Cerulion MCP (closed source) and Cerulion Studio. |
| Not supported | Windows. SROS2. Cerulion topics in `ros2 topic list` and rqt. A packaged `rmw_cerulion` for macOS. |

**License.** The runtime is open source under [AGPL-3.0-only](LICENSE). Running it on your own robots needs no source release ([License](#license)).

<details>
<summary>Read the script first, see what it does to Rust, your PATH and telemetry, or remove it</summary>

To read the script before it runs, download the 2.1.0 copy, read it, then run it for that release:

```bash
curl -fsSLO https://raw.githubusercontent.com/cerulion-inc/cerulion/v2.1.0/tools/scripts/install.sh
less install.sh
sh install.sh --version v2.1.0 && . "$HOME/.cerulion/env"
```

Node builds need Rust 1.95 or newer. With neither Rust nor rustup, the script installs rustup and current stable. With rustup and an older default, it adds stable, keeps your default and prints how to select stable. With an older Rust and no rustup, it stops and installs nothing ([every case](docs/install.md#how-the-install-script-provisions-rust)).

`--no-modify-path` leaves your startup files and `~/.local/bin` alone and prints the PATH line to add yourself. [Installation details](docs/install.md#what-the-install-script-does) list every step.

To uninstall, run the line that matches how you installed:

```bash
# after the piped install
curl -fsSL https://raw.githubusercontent.com/cerulion-inc/cerulion/main/tools/scripts/install.sh | sh -s -- --uninstall
# after sh install.sh, in the folder that holds install.sh
sh install.sh --uninstall
```

An uninstall removes the programs and keeps your sign-in, settings, each harness copy (about 4 GB) and Rust ([Uninstall](docs/install.md#uninstall)).

A usage event carries coarse fields, such as the command's name, its exit code and a duration bucket. It never carries arguments, paths or message contents. Its id is a random one until this machine signs in, and your account id after that. A run under CI always sends under its own random id. `cerulion telemetry off` saves the choice in your consent file, `DO_NOT_TRACK=1` or `CERULION_TELEMETRY=0` turns events off for one shell, and the [first setting in this order](docs/telemetry.md#turning-it-off) wins. A `--network off` run sends none ([every event and field](docs/telemetry.md)).

</details>

<details>
<summary>Homebrew, Debian and Ubuntu (APT), or crates.io</summary>

```bash
brew tap cerulion-inc/cerulion
brew install cerulion-inc/cerulion/cerulion
```

On Linux the formula carries `rmw_cerulion`. No macOS package does, because ROS 2 Jazzy has no macOS binaries. If Homebrew refuses to load the formula, [Homebrew](docs/install.md#homebrew) has the fix.

For Debian and Ubuntu, [APT installation](docs/packaging/apt.md#apt-installation) has the recipe. It checks the archive key's fingerprint before it trusts the key, and the package carries `rmw_cerulion`.

```bash
cargo install --locked cerulion_cli cerulion_netd
```

This build has no `rmw_cerulion`, so it cannot run the ROS 2 trial. It also has no harness, `cerulion-connectd`, `cerulion-remoted` or telemetry key. The [harness guide](docs/harness.md#try-it) says how to point it at a harness. [CONTRIBUTING](.github/CONTRIBUTING.md) covers the full source build. [Installation details](docs/install.md) cover Rust on Homebrew and Debian, the ROS 2 libraries in each package, fish, and moving a workspace from Cerulion 2.0.

</details>

**Next, pick a trial.** [Try it with ROS 2](#try-it-with-ros-2) needs no account. [Try the runtime](#try-the-runtime) and [Try the harness](#try-the-harness) need a free Cerulion account, and each starts with `cerulion login`.

<a id="bring-your-ros-2-stack"></a>

## Try it with ROS 2

Your ROS 2 nodes keep their APIs, executors and source. `cerulion ros2 run` runs `ros2 run` with `rmw_cerulion` as the middleware.

**Before you start.** You need Linux with ROS 2 Jazzy and no sign-in. [Install Cerulion](#install) with the install script, the Debian package or Homebrew. A `cargo install` build does not carry `rmw_cerulion`. With only Docker, use the [container steps](#no-linux-ros-2-machine) below.

**1. Install the stock demo nodes** if you do not have them:

```bash
sudo apt-get install ros-jazzy-demo-nodes-cpp
```

**2. Start a talker in one terminal and a listener in a second.**

```bash
source /opt/ros/jazzy/setup.bash && cerulion ros2 run demo_nodes_cpp talker
```

```bash
source /opt/ros/jazzy/setup.bash && cerulion ros2 run demo_nodes_cpp listener
```

The listener prints a line like this for each message it gets from the talker over `rmw_cerulion`. That is your first success.

```text
[INFO] [<time>] [listener]: I heard: [Hello World: <n>]
```

**3. Optional: check that `rmw_cerulion` carries the messages.** In a third terminal, list the middleware libraries that the running listener has loaded. The check uses `pgrep`, from the `procps` package.

```bash
grep -o "/.*librmw_.*" /proc/$(pgrep -n -x listener)/maps | sort -u
```

One line ends in `librmw_cerulion.so`. In the container run below, the output was:

```text
/opt/ros/jazzy/lib/librmw_implementation.so
/root/.cerulion/bin/librmw_cerulion.so
```

In a default Jazzy environment, a plain `ros2 run demo_nodes_cpp listener` shows `librmw_fastrtps_cpp.so` in the same check. Stop both nodes with Ctrl+C.

<a id="no-linux-ros-2-machine"></a>

<details>
<summary>No Linux ROS 2 machine? Run it in a container</summary>

You need Docker and no sign-in. In a first terminal, start a Jazzy container, then install the demo nodes and Cerulion in it and start the talker. The installer also installs Rust in the container.

```bash
docker run -it --rm --name readme-ros2-trial -e DO_NOT_TRACK=1 -e CERULION_TELEMETRY=0 ros:jazzy
```

```bash
apt-get update && apt-get install -y curl procps ros-jazzy-demo-nodes-cpp
curl -fsSL https://raw.githubusercontent.com/cerulion-inc/cerulion/main/tools/scripts/install.sh | sh && . "$HOME/.cerulion/env"
source /opt/ros/jazzy/setup.bash && cerulion ros2 run demo_nodes_cpp talker
```

In a second terminal, open a shell in the same container and start the listener:

```bash
docker exec -it readme-ros2-trial bash
```

```bash
source /opt/ros/jazzy/setup.bash && cerulion ros2 run demo_nodes_cpp listener
```

From a third terminal, run the check from step 3 in the container:

```bash
docker exec readme-ros2-trial sh -c 'grep -o "/.*librmw_.*" /proc/$(pgrep -n -x listener)/maps | sort -u'
```

Expected output, from a run on arm64. Near its end, the installer prints these lines (`...` marks lines left out):

```text
Installed Cerulion v2.1.0 for aarch64-unknown-linux-gnu into /root/.cerulion/bin
...
The ROS 2 Jazzy rmw (librmw_cerulion.so) is installed beside the CLI for cerulion ros2 run and launch
```

The talker and the listener print:

```text
[INFO] [1795411120.490079009] [talker]: Publishing: 'Hello World: 1'
[INFO] [1795411129.487198388] [listener]: I heard: [Hello World: 10]
```

Both nodes also print `WARN rmw_cerulion::api::pubsub: requested QoS depth differs ...` lines at start and one `INFO auto-injecting the Cerulion heap hook ...` line. These are normal. The installer's last line asks you to sign in with `cerulion login`. This trial does not need it.

Stop both nodes with Ctrl+C, then `exit` both shells. `--rm` removes the container.

</details>

This trial shows delivery over `rmw_cerulion`, not latency. [Latency](#latency) has the separate benchmark in which Jazzy nodes publish and take loaned messages on `rmw_cerulion`.

<details>
<summary>Before you move your own stack: topic tools, loans, SROS2 and process replay</summary>

- `cerulion topic list` shows the topics after you sign in. `ros2 topic list` and rqt do not.
- Zero-copy needs the publisher to loan each message and the subscriber to take loaned ones. Jazzy lets a subscriber take loans only with `ROS_DISABLE_LOANED_MESSAGES=0`. Without loans, `rmw_cerulion` copies, and a loaned variable-size message can still copy some members, such as strings ([ROS 2 guide](https://docs.cerulion.com/cerulion/guides/bridge-ros2)).
- SROS2 is not supported. `ROS_SECURITY_ENABLE=true` does not secure a Cerulion transport, so keep a stack that needs SROS2 on its current middleware.
- `cerulion ros2 run` can also replay one recorded ROS 2 process against the deliveries it recorded. A replayed process really sends on its recorded topics and services, and it still drives the devices it writes to, so read the limits and safety rules in [ROS 2 replay](docs/ros2_replay.md) first.

</details>

Next: [the `cerulion ros2` commands](docs/ros2_compatibility.md#start-inspect-bridge-and-migrate-a-ros-2-stack) launch, inspect, bridge and migrate your own stack. `rmw_cerulion` is validated on Jazzy and tested in CI on Foxy, Galactic, Humble and Lyrical ([compatibility](docs/ros2_compatibility.md)). [`ros2:` entries](docs/user-api.md#ros2-entries-stock-ros-2-nodes-in-the-same-graph) run ROS 2 processes beside native nodes. The [MoveIt 2 example](examples/moveit_hero/) runs an unmodified planning stack in Docker.

## Try the runtime

Record a small robot program, replay it, then change one constant and see which output changed. You need no robot, simulator or ROS 2.

**Before you start.** [Install Cerulion](#install). You also need a free Cerulion account and a C toolchain: `sudo apt-get install -y curl git build-essential` on Debian or Ubuntu, or `xcode-select --install` on macOS.

![A terminal creates the starter, records a run, gets replay PASS, changes the controller and gets a divergence on the same recording.](docs/media/quickstart-record-verify.gif)

This is a recording of the commands below on Cerulion 2.1.0, made after an off-screen sign-in and warm-up build, with the waits sped up ([how it was recorded](docs/media/README.md#assets-the-root-readme-uses)).

A synthetic laser scanner reports an obstacle 0.3 m away for two seconds, then a clear path for two seconds, and repeats. A safety controller publishes a forward velocity `x` of `0.3` on a clear path and `0.0` when an obstacle is closer than 0.5 m.

**1. Sign in once on this machine.**

```bash
cerulion login
```

It prints a URL and a short code. Open the URL in a browser on any device, sign in to your Cerulion account and enter the code. The terminal then prints `Signed in as <account id>.` The CLI asks for an account because the account pairs your robots and computers, so they find each other with no addresses to set.

<a id="what-sign-in-stores-and-sends"></a>

<details>
<summary>What sign-in stores and sends, and the commands that skip it</summary>

- **What stays on this machine.** `~/.cerulion/auth.json` holds your account id and session. `~/.cerulion/identity/device.key` is this machine's key, which lets the robots on your account admit it with no addresses to set.
- **The account check is local after that.** From then on, the account check reads that file and needs no network. Telemetry, when it is on, and commands that reach other machines still use the network, and the first node build and the first simulator start download what they need.
- **What the first sign-in links.** With telemetry on, a machine's first sign-in links its earlier usage events, sent under a random id, to your account. With telemetry off, it links nothing ([Usage telemetry](docs/telemetry.md)).
- **Commands that skip the sign-in.** `doctor`, `telemetry`, `completions`, `ros2 run` and `ros2 launch`, among others ([full list](docs/user-api.md#cerulion-connect--rcp--login--logout--account)).

</details>

**2. Create and build the workspace.** The first node build downloads and compiles the runtime's dependencies, so it takes a few minutes. Later builds take seconds.

```bash
cerulion workspace create safety_demo --starter obstacle_avoidance
cd safety_demo
cerulion node build laser_scanner --release
cerulion node build safety_controller --release
```

Each build ends with `Built 'laser_scanner'` or `Built 'safety_controller'`.

**3. Record a run.** `--network off` keeps it on this machine. Without it, other computers on your account can read a run's topics.

```bash
cerulion graph run obstacle_avoidance --release --record --network off
# wait about five seconds, then press Ctrl+C in this terminal
```

The run prints the first two lines, and Ctrl+C prints the last once the recording is written:

```text
<time>  WARN local execution selected: this run is LOCAL-ONLY; ...
graph 'obstacle_avoidance' running: 2 node(s) in 2 process(es); press Ctrl+C to stop
^Crecording written to recordings/obstacle_avoidance_<time>.mcap
```

`<time>`, `<n>` and `<f>` change from run to run, and `...` marks a cut line. A WARN line about a dropped queued scan or a missed `promise_within_ms` deadline does not change the verdicts.

**4. Replay the recording against the same code.**

```bash
BAG=$(ls -t recordings/*.mcap | head -n 1)
cerulion bag play "$BAG" --resim all --verify
```

```text
replay PASS: recordings/obstacle_avoidance_<time>.mcap (<n> tick(s) replayed, 2/2 topic(s) matched byte-for-byte and were credited)
```

It exits 0: on the recorded inputs, your nodes produce the recorded outputs. That is your first success.

**5. Change one constant and replay the same recording.** The first line sets the clear-path speed to `0.25`.

```bash
perl -pi -e 's/else \{ 0\.3 \}/else { 0.25 }/' nodes/safety_controller/src/lib.rs
cerulion node build safety_controller --release
cerulion bag play "$BAG" --resim all --verify
```

```text
FRAME-CONTENT DIVERGENCE: recordings/obstacle_avoidance_<time>.mcap
  <n> tick(s) replayed; 1/2 topic(s) matched and were credited; 1 violation(s):
  - /obstacle_avoidance/safety_controller/linear_velocity [byte-mismatch]: frame <f> on '/obstacle_avoidance/safety_controller/linear_velocity' differs at x: recorded 0.3, replayed 0.25 (recorded 64 B, replayed 64 B)
  Topics passed: 1/2
```

It exits 1 and names the topic, the field and both values: on the same sensor inputs, the changed controller commands a different speed. In CI, that exit code stops the change before it reaches a robot. The CI runner must be a signed-in machine, because a gated command refuses on a machine that never signed in, and `cerulion login` is a device-code sign-in ([`cerulion login`](docs/user-api.md#cerulion-connect--rcp--login--logout--account)). [Bags and replay](docs/bag.md) explains every exit code and verdict line, and `--tolerance` for inexact data.

**Before you replay your own nodes.** The starter's nodes touch no hardware. A replay re-executes your node, so a node that drives hardware or another network directly still does so during the replay. Replay such a node only where its devices are safe to drive or disconnected ([Bags and replay](docs/bag.md#playback-vs-re-simulation-two-modes-of-one-verb)).

If this worked for you, a [star](https://github.com/cerulion-inc/cerulion) helps other robotics engineers find Cerulion.

<details>
<summary>Watch the controller while it runs</summary>

During step 3, run `CERULION_NETWORK=off cerulion topic echo /obstacle_avoidance/safety_controller/linear_velocity` in a second terminal in `safety_demo`. Stop it before you stop the run.

</details>

<details>
<summary>Let your coding agent run this trial</summary>

```text
Run Cerulion's record-and-verify trial with me, in your own terminal. Ask before you install anything, and give me any system install command a step needs.
1. If cerulion --version fails or prints a version older than 2.1.0, ask me, then run: curl -fsSL https://raw.githubusercontent.com/cerulion-inc/cerulion/main/tools/scripts/install.sh | sh && . "$HOME/.cerulion/env" (if a later command cannot find cerulion, run . "$HOME/.cerulion/env" in it first)
2. Run cerulion doctor --check account.login. If it fails, apply its fix: line. When the fix is cerulion login, run it in the background and show me its URL and code.
3. Run cerulion workspace create safety_demo --starter obstacle_avoidance. In safety_demo, run cerulion node build laser_scanner --release, then cerulion node build safety_controller --release.
4. Start the run in the background and save its PID: cerulion graph run obstacle_avoidance --release --record --network off > run.log 2>&1 & echo $! > run.pid
5. Five seconds after run.log shows "running", run kill -TERM "$(cat run.pid)" and wait until run.log shows "recording written to <path>".
6. Run cerulion bag play <path> --resim all --verify and confirm that it prints replay PASS.
7. Change else { 0.3 } to else { 0.25 } in nodes/safety_controller/src/lib.rs, rebuild that node, verify the same recording again, and explain the "differs at x" line.
```

</details>

Next: the [getting-started tutorial](docs/tutorials/01-getting-started.md) writes nodes from scratch, and the [perception example](examples/perception/) turns a recording of generated images into a tolerance check.

## Try the harness

**Experimental: in this release the harness drives a simulated arm, not a physical robot.** Your model gets tools to look through the robot's cameras, move, touch, grasp and set things down. Each call answers with what the robot measured.

**Before you start.** [Install Cerulion](#install) 2.1.0 or later. You also need:

- A free Cerulion account.
- macOS on Apple silicon, or Linux on x86_64 or ARM64, with `git` and `curl`. On Ubuntu, also `sudo apt-get install libosmesa6 libegl1 libgl1 libglib2.0-0`. No GPU.
- 5 GiB of free disk: the first run installs about 4 GB and refuses to start with less.
- An MCP client whose model calls tools and reads images, such as Claude Code. Your provider bills the model calls, and the local simulator spends no sim credits.

If you have early access, the [hosted Cerulion MCP](#give-your-model-cerulion) needs no install.

**1. Sign in once on this machine**, unless you did for the runtime trial.

```bash
cerulion login
```

It prints a URL and a short code to enter in a browser, then `Signed in as <account id>.` ([what sign-in stores and sends](#what-sign-in-stores-and-sends)).

**2. Start the simulator and leave it running.** The first run takes minutes. Apart from usage telemetry, the simulator reaches no Cerulion service.

```bash
cerulion rcp --sim --http 127.0.0.1:8765
```

It is ready when it logs `serving MCP at http://127.0.0.1:8765/mcp`.

**Keep `--http` on a loopback address.** Anyone who can reach it can start simulators on your machine and watch every episode.

**3. Connect your client and ask for a task.** In Claude Code, run both lines in one folder. The first adds the server for that folder only.

```bash
claude mcp add --transport http cerulion-sim http://127.0.0.1:8765/mcp
claude "With cerulion-sim, put the bowl on the plate, and give me the link to watch the episode."
```

**4. Watch the episode** at the link your model gives you, `http://127.0.0.1:8765/view/<episode id>`. With no link, `http://127.0.0.1:8765/episodes` lists every episode's `id`. The viewer shows the camera pictures, each tool call and result, and the model's claim, then the benchmark's own check: `Benchmark verdict: solved` or `Benchmark verdict: not solved`.

In the episode above, the reply from Claude Opus 5.5 began like this. Your model chooses its own words.

```text
The bowl is on the plate. It worked.
...
- **Watch it:** http://127.0.0.1:8765/view/<episode id>
```

**Your setup works even when the model fails** if your client lists the `cerulion-sim` tools and the viewer shows calls and a verdict. Trust the verdict, not the model's claim. Each MCP session is one episode, so to try again, run the `claude` line again.

**5. Stop the simulator** with Ctrl+C in its terminal.

<details>
<summary>Other MCP clients and the other two tasks</summary>

In another MCP client, add `http://127.0.0.1:8765/mcp` as a streamable HTTP server and reconnect. A client that keeps no MCP session names its episode in the URL, such as `http://127.0.0.1:8765/mcp?episode=a`.

`--task open-drawer` and `--task soup-in-basket` set up the other two tasks. All three are start states the harness was developed on.

</details>

<details>
<summary>Let your coding agent start the simulator</summary>

```text
Start Cerulion's local simulated arm for me, in your own terminal. Ask before you install anything, and give me any system install command a step needs.
1. If cerulion --version fails or prints a version older than 2.1.0, ask me, then run: curl -fsSL https://raw.githubusercontent.com/cerulion-inc/cerulion/main/tools/scripts/install.sh | sh && . "$HOME/.cerulion/env" (if a later command cannot find cerulion, run . "$HOME/.cerulion/env" in it first)
2. Run cerulion doctor --check account.login. If it fails, apply its fix: line. When the fix is cerulion login, run it in the background and show me its URL and code.
3. Run cerulion rcp --sim --http 127.0.0.1:8765 > rcp-sim.log 2>&1 & echo $! > rcp-sim.pid
4. Wait until curl -fsS http://127.0.0.1:8765/health answers. The first run installs about 4 GB and needs 5 GiB free. If the server exits first, tell me why from rcp-sim.log.
5. Tell me to add http://127.0.0.1:8765/mcp to my MCP client as a streamable HTTP server named cerulion-sim, reconnect it and ask it to put the bowl on the plate. Tell me that kill -TERM "$(cat rcp-sim.pid)" stops the simulator.
```

</details>

<a id="give-your-model-cerulion"></a>

<details>
<summary>The hosted Cerulion MCP (early access, closed source)</summary>

The hosted Cerulion MCP is closed source and in early access ([request access](https://docs.cerulion.com/cerulion/hindsight)). Its simulated arm runs in Cerulion's cloud with no install, at one sim credit for each second it is held open, and your model stops it with `sim_end`.

```bash
claude mcp add --transport http cerulion https://mcp.cerulion.com/mcp/cerulion
```

[The hosted Cerulion MCP](docs/harness.md#the-hosted-cerulion-mcp) says how to sign in and lists its tools, which also analyze MCAP recordings and show your robots.

</details>

Next: [rerun a start state](docs/evidence.md#rerun-a-start-state-against-your-own-model) to score your own model on a published start state. The [harness guide](docs/harness.md#try-it) covers stdio and `cargo install`.

## Evidence

Every figure below is copied from a results package in this repository. The [evidence page](docs/evidence.md) is the full report.

### Harness results

Each of 16 models ran the same 100 LIBERO-PRO start states twice: with Cerulion, and without it on a published minimal interface of three tools (arXiv:2610.30051). A start state is one task, such as "put the bowl on the stove", from one starting scene. Both runs had the same simulator, budget and success check.

- **The main test, declared before the first episode.** Gemini 3.8 Flash solved 73 of 100 start states with Cerulion and 25 without (2.9 times as many; exact two-sided sign test p = 1.10e-11).
- **All 16 models.** With Cerulion, 12 solved more and 4 solved fewer. After Holm's correction, the gain is established for 9 of 16.

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/media/harness-success-rate-dark.svg">
  <source media="(prefers-color-scheme: light)" srcset="docs/media/harness-success-rate-light.svg">
  <img src="docs/media/harness-success-rate.svg" alt="Success rate per model, without and with Cerulion, for 16 models on the same 100 start states.">
</picture>

Four limits change what these results mean:

- **One benchmark family.** The result is on the benchmark family the harness was built and checked on. The 100 start states are new, but their 25 task rows are ones the harness was developed on.
- **More than tools.** The run with Cerulion also reads the simulated robot's calibration record, so the comparison is the whole package against the published minimal interface.
- **Not every model gains.** The 4 models that solved fewer with Cerulion are the 4 with the highest scores without it.
- **Simulation only.** Nothing here measures a model on a physical robot.

The [harness results package](docs/benchmarks/results/7cb9f322c10f6fa7-2026-11-18-harness-libero-pro/) holds a row for every episode and a script that computes every figure again.

### Latency

On one x86-64 Linux desktop, a native Cerulion multi-process round trip had a median of 4.07 µs at 64 B and 4.08 µs at 1 MiB, with always-on recording. At the same sizes, ROS 2 Jazzy read 311.11 µs and 11.0 ms on `rmw_fastrtps_cpp` defaults. Composed in one process with intra-process, it read 24.64 µs and 29.79 µs.

<details>
<summary>The latency table: p50 / p99 at 64 B, 1 MiB and 16 MiB</summary>

Round trips, **p50 / p99**, on one x86-64 Linux desktop (24-core Intel Core Ultra 9 285K, performance governor, no real-time tuning), 2026-09-16, at a nominal 100 Hz offered load. The native Cerulion columns include always-on Flashback, the default recorder.

| Payload | Native Cerulion, multi-process (default) | Native Cerulion, single process | ROS 2 Jazzy, `rmw_fastrtps_cpp` defaults | ROS 2 Jazzy, composed with intra-process |
|---|---:|---:|---:|---:|
| 64 B | 4.07 µs / 6.61 µs | 2.69 µs / 9.41 µs | 311.11 µs / 568.17 µs | 24.64 µs / 31.47 µs |
| 1 MiB | 4.08 µs / 7.38 µs | 2.76 µs / 10.74 µs | 11.0 ms / 43.4 ms | 29.79 µs / 36.99 µs |
| 16 MiB | 4.43 µs / 10.45 µs | 2.80 µs / 14.23 µs | 103.9 ms / 156.9 ms | 33.41 µs / 37.34 µs |

</details>

- **Transport only.** The timer leaves out the application filling the payload, so a node's own work adds to these figures. A copy the transport makes on receive is timed, as in the ROS 2 defaults lane ([methodology](benches/latency/METHODOLOGY.md)).
- **One desktop, against defaults.** These figures are not a guarantee. The largest gap is against ROS 2 defaults, not a tuned ROS 2 setup.
- **ROS 2 on `rmw_cerulion`.** With loaned publishing and loaned takes, Jazzy nodes read 10.45 µs / 36.18 µs at 64 B on the same desktop ([package](docs/benchmarks/results/8a84baf25d5d1710-2026-09-18-fixed100-rmw-cerulion/)). The [native package](docs/benchmarks/results/8a84baf25d5d1710-2026-09-16-fixed100-heroes/HEADLINE.md) has every lane.

The [evidence page](docs/evidence.md) has the rest: the cost of a solved start state, the other limits (a filtered baseline, more false claims per failure, one draw each), latency charts across Linux, NVIDIA Jetson and Apple silicon, and a rerun against your own model.

### Runs on real hardware

<p align="center">
  <img src="docs/media/studio-arm.gif" alt="A real six-axis arm and its live 3D model move together, beside the arm's ROS 2 topics and a controller plot." width="640">
</p>

Cerulion Studio (early access) streams a real six-axis arm's ROS 2 and MoveIt data through Cerulion into a live 3D view. No AI model drives the arm, and the pauses between moves are cut ([how it was made](docs/media/README.md#assets-the-root-readme-uses)).

Where Cerulion has run: this arm with MoveIt 2 on `rmw_cerulion`, and an [NVIDIA Jetson Orin NX](docs/benchmarks/results/9b0c5fbf0f55dea4-2026-09-17-fixed100-jetson-orin-nx/) and an [Apple M4](docs/benchmarks/results/446db2afb7d26dbc-2026-11-13-fixed100-apple-m4/), each with a latency package. The [examples](#examples) are written for a Unitree Go2, an Intel RealSense camera and a V4L2 camera.

## Build on the runtime

### Write your own node

This is the controller you changed, without its comments, lint line and test module. You write a typed input, a typed output and `tick`. The graph file does the wiring, and the runtime does scheduling, transport and recording, with no executor, callbacks or spin loop.

```rust
use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;
use native_ros2_messages::sensor_msgs::LaserScan;

#[cerulion_node]
#[derive(Default)]
pub struct SafetyControllerNode {
    #[input(trigger, depth = 1, expect_within_ms = 100)]
    scan: LaserScan,
    #[output(promise_within_ms = 100)]
    linear_velocity: Vector3,
}

#[cerulion_node_impl]
impl SafetyControllerNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let ranges = self.scan.ranges();
        let obstacle_close = ranges.is_empty() || ranges.iter().any(|&r| r.is_nan() || r < 0.5);
        self.linear_velocity.x = if obstacle_close { 0.0 } else { 0.3 };
        Ok(())
    }
}
```

`#[input(trigger)]` runs `tick` when a scan arrives, and the two deadlines count misses, so a stalled sensor shows in counters. [Define a node](https://docs.cerulion.com/cerulion/guides/define-a-node), the [API reference](docs/user-api.md) and [Python](docs/python.md) cover periodic nodes, backpressure, parameters and Python nodes.

### Capture failures with Flashback

A running graph keeps about the last 30 seconds in memory by default. A fault, such as a worker that dies or a topic that stalls, writes that window to disk as a recording, and so does this command from another shell:

```bash
cerulion flashback --note "controller stopped late"
```

[Flashback](docs/flashback.md) lists the triggers, which captures re-execute and how to turn it off.

### Reach your robots from another network

Sign in to the same Cerulion account on the robot and on your computer. A graph that runs on the robot without `--network off` serves its topics, and your computer finds the robot through the account, directly or through a relay, with no address to set. Then `cerulion topic list` shows the robot's topics and `cerulion topic echo <topic>` reads one. `cerulion connect <robot> --topic <topic>`, with the robot's endpoint id or a name pinned in `~/.cerulion/robots.toml`, copies a topic into this machine's shared memory and carries data from the robot to you only. The default public relays are for development only, so run your own for a robot in service ([what a desk can do today](docs/remote_plane.md#what-a-desk-can-do-today), [networking](docs/networking.md)).

### How Cerulion fits together

A node runs on a timer, as the starter's laser scanner does every 20 ms, or when its trigger input gets a message, as the safety controller does. [Execution](https://docs.cerulion.com/cerulion/execution-order) covers the other policies. Nodes exchange loaned shared-memory messages.

<details>
<summary>Diagrams: the architecture and the development loop</summary>

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/media/architecture-flow-dark.svg">
  <source media="(prefers-color-scheme: light)" srcset="docs/media/architecture-flow-light.svg">
  <img src="docs/media/architecture-flow.svg" alt="Robot-side graph scheduling and shared memory connect through separate LAN and WAN network planes to Cerulion Studio (early access), the CLI and coding agents.">
</picture>

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/media/development-loop-dark.svg">
  <source media="(prefers-color-scheme: light)" srcset="docs/media/development-loop-light.svg">
  <img src="docs/media/development-loop.svg" alt="Cerulion connects a running robot, live inspection, failure capture and replay verification in one development loop.">
</picture>

</details>

Guides: [Multi-process graphs](https://docs.cerulion.com/cerulion/guides/run-multi-process) · [Remote robots](https://docs.cerulion.com/cerulion/guides/network-and-remote-robots) · [Remote access](docs/remote_plane.md) · [Inspect topics](https://docs.cerulion.com/cerulion/guides/inspect-topics) · [Record and replay](https://docs.cerulion.com/cerulion/guides/record-and-replay)

## Examples

<details>
<summary>Seven examples: replay, cameras, perception, RealSense, MoveIt 2 and a Go2 quadruped</summary>

| Example | What it shows |
|---|---|
| [Obstacle avoidance](examples/obstacle_avoidance/) | Typed nodes, a live controller and replay verification. |
| [Camera pipeline](examples/camera_pipeline/) | In-place image generation and a data-triggered detector. |
| [Perception regression](examples/perception/) | Image-derived detections and a repeatable tolerance failure. |
| [V4L2 camera](examples/v4l2_camera/) | A Linux camera driver node and a brightness meter. Needs a camera. |
| [RealSense](examples/realsense/) | Color, depth and a nearest-obstacle range. Needs a RealSense. |
| [MoveIt 2](examples/moveit_hero/) | An existing ROS 2 planning stack on `rmw_cerulion`. |
| [Go2 quadruped](examples/go2/) | A DDS robot bridged, its camera transcoded, and teleop with a safety mux. |

</details>

## Roadmap

The direction is **review for robot changes**: recordings, simulations and behavioral evidence in review before a change reaches a robot. The [changelog](CHANGELOG.md) records each release. Propose a use case or a feature in [GitHub Discussions](https://github.com/cerulion-inc/cerulion/discussions).

<details>
<summary>What is next, by area</summary>

| Area | What's next |
|---|---|
| **Faster execution** | Chain-fused nodes that pass data directly within a process, nanosecond-scale handoff as a measured target, and lower macOS cross-process wake-up latency. |
| **C++** | First-class node authoring and a client library, with clear boundaries for zero-copy access, scheduling and reproducible execution. |
| **Robot building blocks** | Native transforms and URDF, parameters that change while a graph runs (typed parameters set at graph load already ship), and lifecycle, each with a path from ROS 2. |
| **Flashback from node code** | A node declares an incident itself, so a bumper or a grasp check saves the moment it sees. |
| **Services and actions** | A native request and response API on the shipped deterministic service layer, then actions with feedback, cancellation and streaming replies. |
| **Pull-request verification** | A GitHub reviewer that runs recorded regressions and cloud simulations, reports behavioral differences and keeps the evidence with the change. |
| **Scenario libraries** | User-defined environments and expected behavior, failure bags promoted to regression cases, and generated stress scenarios. |
| **Agents and inspection** | A tighter loop of inspecting data, changing code and replaying, with runs, recordings and reports side by side. |
| **Platforms and interoperability** | Windows support, broader hardware validation and easier exchange with robotics data tools. |
| **Remote access** | Hardened account-based pairing: robot-side provisioning, hosted account services and grant management across owned and shared robots. |

</details>

## Get help and contribute

- Questions and designs: [Discussions](https://github.com/cerulion-inc/cerulion/discussions). Real-time help: [Discord](https://discord.gg/dqKFecngPy).
- Bugs you can reproduce: [Issues](https://github.com/cerulion-inc/cerulion/issues). Vulnerabilities: the [security policy](.github/SECURITY.md), not a public issue.
- Read [Contributing](.github/CONTRIBUTING.md) before you send code or docs. Contributions come in under the [contributor license agreement](.github/CLA/individual.md): you keep your copyright and grant Cerulion a license to offer the code under AGPL-3.0 and commercial terms.

To build and test a checkout, as [Contributing](.github/CONTRIBUTING.md) describes:

```bash
git clone https://github.com/<your-username>/cerulion.git   # your fork
cd cerulion
cargo build --workspace
cargo test -p cerulion_core --tests --lib -- --test-threads=1
```

Do not run `cargo test --workspace`, because its shared-memory tests collide. [Contributing](.github/CONTRIBUTING.md) lists the per-crate commands that CI runs.

[Documentation](https://docs.cerulion.com) · [Guides](https://docs.cerulion.com/cerulion/guides) · [API reference](docs/user-api.md) · [Android build support](docs/android.md) · [Code of conduct](.github/CODE_OF_CONDUCT.md) · [Talk to the founders](https://cerulion.com/demo)

Cerulion is built by [Cerulion Inc.](https://cerulion.com), a YC S24 company. Commercial questions: licensing@cerulion.com.

## License

Cerulion is [AGPL-3.0-only](LICENSE), with a [commercial license](docs/legal/COMMERCIAL.md) available.

- Internal use on your own robots, research and evaluation are free, with no obligation to release your source.
- If you convey Cerulion, you must offer its corresponding source (AGPL sections 4 to 6).
- If you modify it and users interact with your version over a network, you must offer them its source (AGPL section 13).
- The commercial license covers a closed-source product that includes or links the AGPL code. It also covers a network service for third parties that does not offer them its source.
- The client crates in `crates/cerulion_wire`, `crates/cerulion_link`, `crates/cerulion_ops_proto` and `crates/cerulion_pairing` are MIT OR Apache-2.0, so a program that uses only them needs no commercial license.

## Citing Cerulion

Use GitHub's **Cite this repository** button or [CITATION.cff](CITATION.cff):

```bibtex
@software{cerulion,
  author  = {Sharma, Lakshay and Kariyawasam, Raveen and {Cerulion Inc. and contributors}},
  title   = {Cerulion},
  year    = {2026},
  version = {2.1.0},
  url     = {https://github.com/cerulion-inc/cerulion}
}
```
