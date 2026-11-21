<p align="center">
  <a href="https://cerulion.com">
    <picture>
      <source media="(prefers-color-scheme: dark)" srcset="docs/media/cerulion-logo-dark.svg">
      <source media="(prefers-color-scheme: light)" srcset="docs/media/cerulion-logo-light.svg">
      <img src="docs/media/cerulion-logo.svg" alt="Cerulion" width="420">
    </picture>
  </a>
</p>

**Cerulion (YC S24) is the harness for physical AI: the tools and body knowledge a general model needs to work a robot it was never trained on. It is built on an open-source, zero-copy robot runtime that records runs, replays failures against new code, and reaches your robots from anywhere, and it is ROS 2 compatible.**

[![License: AGPL-3.0-only](https://img.shields.io/badge/license-AGPL--3.0--only-blue.svg)](LICENSE)
[![crates.io: cerulion_cli 2.1.0](https://img.shields.io/badge/crates.io-cerulion__cli%202.1.0-brightgreen.svg)](https://crates.io/crates/cerulion_cli)
[![Rust 1.95+](https://img.shields.io/badge/rust-1.95%2B-orange.svg)](https://www.rust-lang.org/)
[![Docs](https://img.shields.io/badge/docs-docs.cerulion.com-blue.svg)](https://docs.cerulion.com)

**[Quickstart](#quickstart) · [Harness results](#harness-results) · [Performance](#performance) · [ROS 2](#bring-your-ros-2-stack) · [Architecture](#how-it-fits-together) · [Docs](https://docs.cerulion.com) · [Guides](https://docs.cerulion.com/cerulion/guides)**

**Same model, with and without Cerulion.** On LIBERO-PRO, simulated robot manipulation, each of 15 models ran the same 100 start states twice: once through a published three-tool interface and once through the harness's tools. The start states are new start states of 25 task rows the harness was developed on at other start states; the tasks themselves are not new to it, and task rows it never met exist in the benchmark and are on no list here. The result is on one benchmark family, the one the harness was built and checked on. The arm with Cerulion also reads the simulated robot's calibration record, which the other arm does not have, so the comparison is the whole package against the published interface; it cannot be split into tools, record and documents. One episode per start state in each arm, at temperature 1.0; provider model versions are known only by the model's name. Other groups publish higher rates on this benchmark under their own protocols, and nothing here ranks against them.

On the main test, as declared before the first episode, Gemini 3.8 Flash solved 73 against 25 of 100 start states, with Cerulion against without (2.9 times as many; exact two-sided sign test p = 1.10e-11). Over the 15 models, the arm with Cerulion solved more start states in 12 of 15, fewer in 3 and the same number in 0. Fewer, no difference shown for each: GPT-6 Astra (70 against 71, p = 1); Gemini 4 Carbon (76 against 82, p = 0.377); GPT-6.2 Astra (77 against 81, p = 0.541). [Every model, cost and episode](docs/benchmarks/results/7cb9f322c10f6fa7-2026-11-18-harness-libero-pro/).

![Success rate per model, without and with Cerulion, ordered by the success rate without it, with the factor, the success rate with Cerulion over the rate without, at the right of each row. Its footer names the 3 models that solved fewer start states with Cerulion, no difference shown for each after Holm's correction, and the one arm whose scored episodes ended on replies the provider's content filter stopped, with its counts; three other arms had one filtered reply each, ending no scored episode.](docs/media/harness-success-rate.svg)

## Quickstart

One install and one Cerulion account give you two things: the robot runtime, which runs, records and verifies your robot software, and the harness, which gives a model the tools to work a simulated robot. Through the hosted Cerulion MCP your model reaches the harness and your recordings, and lists your robots; your terminal runs both on your machine.

### Install

```bash
curl -fsSL https://raw.githubusercontent.com/cerulion-inc/cerulion/main/tools/scripts/install.sh | sh && . "$HOME/.cerulion/env"
cerulion login
```

The installer links `cerulion` into `~/.local/bin`, adds one line to your shell startup files and installs Rust when Rust 1.95 or newer is missing. `cerulion login` signs this machine in once: every command runs under a free Cerulion account, which is used for robot pairing and works offline afterwards, and a robot or a headless box prints a short code to approve from a browser on any machine. To uninstall, run the same script with `sh -s -- --uninstall`.

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

The release installer places `cerulion-connectd` (the desk side of a remote connection), `cerulion-remoted` (the robot side) and the agent harness beside the CLI; a `cerulion` built this way has no harness beside it, so [Run the harness on your own machine](#run-the-harness-on-your-own-machine) says how to point it at one. See the [installation guide](https://docs.cerulion.com/cerulion/installation) or [CONTRIBUTING](.github/CONTRIBUTING.md) for the complete source-build route.

</details>

<details>
<summary>Platforms, ROS 2 libraries and toolchain</summary>

In fish, end the line with `&& source ~/.cerulion/env.fish` instead.

Homebrew and the Debian package do not provision Rust. To build your own nodes, run `cerulion-install-rust` once, which installs current stable Rust unless a Rust 1.95 or newer is already there, and put `${CARGO_HOME:-$HOME/.cargo}/bin` on your PATH. On a terminal, `cerulion workspace create` and `cerulion node build` offer to install it when Rust 1.95 or newer is missing.

Release binaries support Linux x86_64 and ARM64 (glibc 2.31+, Ubuntu 20.04 and later) and macOS Intel and Apple Silicon. The Linux packages also carry the ROS 2 Jazzy rmw and heap hook that `cerulion ros2 run` and `cerulion ros2 launch` use. The macOS packages do not, because ROS 2 Jazzy has no macOS binaries. The rmw libraries run where their ROS 2 distro runs, which for Jazzy is Ubuntu 24.04. For ROS 2 Lyrical, each release also publishes `cerulion-rmw-lyrical-<version>-<target>.tar.gz` for Linux x86_64 and ARM64. Unpack it and set `CERULION_LIB_DIR` to the unpacked directory so `cerulion ros2 run` and `cerulion ros2 launch` load the Lyrical rmw. That library is built and tested in CI.

Building nodes needs a C toolchain (`build-essential` on Debian and Ubuntu, `xcode-select --install` on macOS) and Rust 1.95 or newer. Nodes load through a compiler-independent interface, so any Rust 1.95 or newer can build them. The compiler does not have to match the one that built the `cerulion` CLI, and a new workspace pins no compiler. To bring a workspace from Cerulion 2.0, follow the migration steps in the [2.1.0 release notes](./CHANGELOG.md#210---2026-11-18); a workspace from Cerulion 1.0 takes the steps in the [2.0.0 release notes](./CHANGELOG.md#200---2026-11-12) first. They move its pins to `=2.1.0` and finish with one `cerulion node build <node>` per node. The [installation details](docs/install.md) cover the toolchain setup.

</details>

### Give your model Cerulion

Add the hosted Cerulion MCP to Claude Code once. This step needs no install, only your account: the first time, run `/mcp` in Claude Code, pick `cerulion` and sign in with your Cerulion account in the browser. In another MCP client, add the same URL as a connector.

```bash
claude mcp add --transport http cerulion https://mcp.cerulion.com/mcp/cerulion
```

The hosted endpoints are in early access ([request access](https://docs.cerulion.com/cerulion/hindsight)).

**The runtime side.** With an MCAP recording at a public or presigned `https://` URL, ask:

```text
Open the recording at https://<your host>/run.mcap. Summarize it and show me the worst anomaly, with its topic and time.
```

Your model opens and indexes the recording (`open_bag`; the first open of a large one can take a few minutes), reads its summary (`get_debrief`), lists the anomalies worst first (`find_anomalies`), looks at the window around the worst one (`scrub`) and cites each finding as `[topic @ t=<nanoseconds>]`, with a Foxglove link to that moment when the URL is public (`get_foxglove_link`). The hosted tools cannot read a file on your machine. The same connector lists the robots on your account and whether each is online (`robot_list`), and starts and connects to your Cerulion Cloud instance (`cloud_instance_start`, `cloud_instance_connect`).

**The harness side.** Ask for a task on a simulated arm:

```text
Start the simulated arm and put the bowl on the plate.
```

Your model starts a simulated robot arm for your account in Cerulion's cloud (`sim_start`), reads its task (`sim_read` on `robot://task`), looks through the robot's cameras (`look`) and has the harness's other tools for the arm; `sim_end` stops the simulator and reports the credits it charged. `bowl-on-plate` is the default scene; ask for `open-drawer` or `soup-in-basket` for the other two. These are start states the harness was developed on, not the start states of the published results. The simulator runs on your account's sim credits, one credit for each second it is held open.

The hosted tools cannot run a graph, record or verify. Those run in your terminal:

### Run, record and verify

![Record a run, verify it passes, change the controller, and watch the same recording fail verification.](docs/media/quickstart-record-verify.gif)

For your robot software: a synthetic laser scanner feeds a safety controller that publishes a forward velocity of `0.3` when the path is clear and `0.0` when an obstacle is within half a meter. Create the workspace, build both nodes, check the graph and run it with recording on. The first node build also compiles the Cerulion runtime, so it takes a few minutes on a laptop; later builds take seconds.

```bash
cerulion workspace create safety_demo --starter obstacle_avoidance
cd safety_demo
cerulion node build laser_scanner --release
cerulion node build safety_controller --release
cerulion graph validate obstacle_avoidance
cerulion graph run obstacle_avoidance --release --record
```

The run splits into one process per node without changing the graph file, as the [multi-process guide](https://docs.cerulion.com/cerulion/guides/run-multi-process) explains, and any computer signed in to your account can read it with no addresses or flags; add `--network off` to keep a run on this machine. To write nodes from scratch instead, follow the [getting-started tutorial](docs/tutorials/01-getting-started.md).

In a second terminal, watch the controller react:

```bash
cerulion topic echo /obstacle_avoidance/safety_controller/linear_velocity
```

The `x` field switches between `0.3` and `0.0` as the synthetic scan crosses the distance threshold. After at least five seconds, stop the run with **Ctrl+C**. Cerulion prints the completed recording's path, such as `recordings/obstacle_avoidance_20260918T181449Z.mcap`. Set `BAG` to it and verify the unchanged code first:

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

This time verification reports a divergence that names the topic, the field that changed and both values (`differs at x: recorded 0.3, replayed 0.25`), and returns a nonzero exit code. On the same sensor inputs, the changed controller would command a different speed. In CI, that verdict flags the changed behavior for review before the code reaches a robot. The animation above shows this whole run.

<details>
<summary>What replay verification checks</summary>

Cerulion re-executes the graph with recorded inputs and execution context, then compares the covered outputs against the recording. A pass means the changed code produces the same covered outputs on those recorded inputs, and each recording you keep adds another case to check against.

- Exit **0**: compared frames match.
- Exit **1**: output data diverged, and the verdict names the topic and what differs (for a changed value, the first field that differs and both values).
- Exit **6**: the execution schedule diverged.
- Exits **2 through 5**: verification could not complete, and the error says why.

Comparisons are byte-exact by default. Add `--tolerance` for acceptable per-field differences and `--report` for a machine-readable report.

</details>

Guide: [Record and replay](https://docs.cerulion.com/cerulion/guides/record-and-replay)

### Run the harness on your own machine

`cerulion rcp --sim` serves the harness's tools and its three scenes on your machine. It reaches no Cerulion service, so it spends no sim credits and the network kill switch (`CERULION_NETWORK=off`) does not refuse it; its first run downloads the simulator from the benchmark's and the Python packages' own hosts. Experimental: in this release the harness drives a simulated robot. It needs Cerulion 2.1.0 or later (`cerulion --version`). With a release package there is nothing to clone or set: every release package installs the harness folder beside `cerulion`. A `cerulion` installed with `cargo install` has no harness beside it: clone this repository at the tag of its version (`git clone --depth 1 --branch v<version> https://github.com/cerulion-inc/cerulion`, with the version `cerulion --version` prints) and set `CERULION_HARNESS_DIR` to the clone's `harness/` folder, or, for one installed from a checkout (`cargo install --path`), to that checkout's `harness/`. Then:

```bash
cerulion rcp --sim --http 127.0.0.1:8765         # leave it running
claude mcp add --transport http cerulion-sim http://127.0.0.1:8765/mcp
```

Then ask your model: "With cerulion-sim, put the bowl on the plate." Each MCP connection gets its own episode, set up for one task: `bowl-on-plate` by default, or start the server with `--task open-drawer` or `--task soup-in-basket`. They are start states the harness was developed on, not the start states of the published results. Watch each episode, its camera pictures and the benchmark's verdict at `http://127.0.0.1:8765/view/<episode id>` (the server gives your model that link); `http://127.0.0.1:8765/episodes` lists every episode. Keep `--http` on a loopback address: anyone who can reach the server can start simulators on your machine and watch every episode.

The first `cerulion rcp --sim` installs the simulator and the benchmark scenes once, which takes minutes and about 4 GB of disk (it refuses to start with less than 5 GB free). The harness runs on macOS on Apple Silicon and on Linux on x86_64 or ARM64, with `git` and `curl`; on Ubuntu, also run `sudo apt-get install libosmesa6 libegl1 libgl1 libglib2.0-0` for the CPU renderer. The published results were measured on Linux x86_64. On macOS the pictures are rendered by Apple's OpenGL and differ slightly from the measured ones, so an episode there is a try of the harness, not a reproduction of a published row.

To rerun a start state of the published benchmark with and without the harness against your own model, set `OPENAI_BASE_URL` to any OpenAI-compatible chat completions endpoint whose model takes tool calls and images, and `OPENAI_API_KEY` to its key. Pick a start state from the results package's [cells.csv](docs/benchmarks/results/7cb9f322c10f6fa7-2026-11-18-harness-libero-pro/cells.csv) and run:

```bash
cerulion rcp bench --model YOUR_MODEL --cell CELL_ID
```

It runs two episodes on your endpoint, one after the other (each up to 3 attempts when the endpoint fails), and prints the benchmark's verdict for each; you pay for the model calls. Without `--cell` it runs a start state the harness was developed on, not a published one. One episode at temperature 1.0 can go either way; the published rates come from 100 start states per model. `cerulion rcp bench --verify` checks every file of the harness's measured code against the provenance ledger that ties it to the code the benchmark measured. The [harness guide](docs/harness.md) covers what the harness is, what the benchmark measured, the flags and what `--verify` checks.

If Cerulion is useful to you, a star helps other robotics engineers find it.

## Harness results

The benchmark ran each LIBERO-PRO start state twice with the same model: once without Cerulion, through a published three-tool interface (arXiv:2610.30051), and once with Cerulion, through the harness's tools. Same simulator, same budget of 80 model replies and 900 control steps, and the benchmark's own success check. Fifteen models on the same 100 start states, one episode per start state in each arm, at temperature 1.0; provider model versions are known only by the model's name. The result is on one benchmark family, the one the harness was built and checked on. The start states are new start states of 25 task rows the harness was developed on at other start states; the tasks themselves are not new to it, and task rows it never met exist in the benchmark and are on no list here. The arm with Cerulion also reads the simulated robot's calibration record, which the other arm does not have, so the comparison is the whole package against the published interface; it cannot be split into tools, record and documents. Other groups publish higher rates on this benchmark under their own protocols, and nothing here ranks against them.

On the main test, as declared before the first episode, Gemini 3.8 Flash solved 73 against 25 of 100 start states, with Cerulion against without (2.9 times as many; Wilson 95 percent intervals 63.6 to 80.7 and 17.5 to 34.3 percent; exact two-sided sign test p = 1.10e-11). Over the 15 models, the arm with Cerulion solved more start states in 12 of 15, fewer in 3 and the same number in 0. Fewer, no difference shown for each: GPT-6 Astra (70 against 71, p = 1); Gemini 4 Carbon (76 against 82, p = 0.377); GPT-6.2 Astra (77 against 81, p = 0.541). After Holm's correction over the 15 models, the gain on solved start states is established for 10 of 15; one of them, GPT-6.1 Terra, was added to the grid after its first rows had been seen. Claude Fable 5.2 without Cerulion ended 86 of its 100 episodes on 3 replies in a row with no tool call, which end an episode, and all 86 ended on replies the provider's content filter stopped, with no claim from the model (the filter stopped 337 of its 1,101 replies in scored episodes). 86 of the 90 start states it did not solve without Cerulion ended that way, so its gain on solved start states is mostly against an arm that the content filter stopped.

Fewer episodes ended with a false success claim: pooled over the 15 models, a false claim without Cerulion only on 204 pairs of model and start state, and with Cerulion only on 136, p = 2.68e-04. That does not mean Cerulion stops false claims. Per unsolved episode the rate is higher with Cerulion: 180 false claims in 497 unsolved episodes (0.362) against 248 in 898 (0.276) without it, over all 15 models; as high or higher with Cerulion for 6 of the 15 (with against without): Gemini 3.8 Flash 0.593 against 0.400; Claude Sonnet 5.5 0.431 against 0.190; Claude Haiku 5.5 0.373 against 0.042; Claude Fable 5.2 0.500 against 0.044 with the 86 episodes that ended on filtered replies counted, 0.500 against 1.000 with them left out (descriptive; no rule declared this reading), two readings that point opposite ways, so the comparison measures the filter as much as the model and is read neither way; GPT-6 Luna 0.197 against 0.120; Grok 4.7 0.319 against 0.062. The rule narrows the statement to "fewer because it solves more" if, per unsolved episode, the arm with Cerulion claims falsely as often or more for most models: it does for 6 of the 15, which is not most, so the statement stands as written. With Claude Fable 5.2 left out (both arms, so each pool holds the same 14 models), the pooled rate is 171 in 479 (0.357) with Cerulion against 244 in 808 (0.302) without (descriptive; no rule declared this reading). With each arm's unsolved episodes that ended on filtered replies left out of its count, the arm with Cerulion claims falsely as often or more for 5 of the 15 models (descriptive; no rule declared this reading), against 6 with them counted; not most, either way. In all, with Cerulion, 5 of the 15 models made more false claims: Claude Haiku 5.5 (22 against 4), Grok 4.7 (15 against 6), Claude Sonnet 5.5 (22 against 16), Claude Fable 5.2 (9 against 4, a comparison the content filter reading above says is read neither way) and GPT-6 Luna (14 against 12), each a descriptive count.

Model cost per solved start state for Gemini 3.8 Flash, at list prices, model calls only (simulator compute is not included; nothing here is a billed amount): 6.67 USD without Cerulion, 0.48 with. Part of the gap is caching: the arm without Cerulion had the lower cached share of input tokens for 15 of the 15 models.

![Model cost per successful episode, without and with Cerulion, in the same order, on a linear scale cut at 30 USD, with the factor, the cost per success without Cerulion over the cost with it, at the right of each row. Its footer says that for Claude Fable 5.2 without Cerulion the provider's content filter stopped 337 of its 1,101 replies and 86 of its 100 scored episodes ended on them, with no claim from the model, and that three other arms had one filtered reply each, ending no scored episode.](docs/media/harness-cost-per-success.svg)

The [results package](docs/benchmarks/results/7cb9f322c10f6fa7-2026-11-18-harness-libero-pro/) holds every count, cost and episode, the reading rules, each frozen before the rows of its own lists, and a script that computes every figure again from the rows.

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

## Capture the failure with Flashback

Flashback keeps a memory-bounded window of recent activity in every running graph, so the moments before a problem are already there when you need them. It is event-based logging: nothing is written to disk until an event fires, and then the window around that event becomes a recording.

**It captures on its own** when the runtime sees a fault, such as a dying worker process or a stalled topic, and [Flashback](docs/flashback.md) lists every trigger.

**You can also ask for one by hand** from another shell on the robot:

```bash
cerulion flashback --note "controller stopped late"
```

Re-execution uses the capture's execution trace and any state anchors its starting point needs. `bag info` reports whether a capture can be re-executed or only played back. To record an already-running graph instead, use `cerulion bag record --run=obstacle_avoidance`.

Guides: [Flashback](docs/flashback.md) · [Record and replay](https://docs.cerulion.com/cerulion/guides/record-and-replay) · [Bags and replay](docs/bag.md)

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

The `rmw_cerulion` line in the chart above is ROS 2 Jazzy running over Cerulion's transport, measured with the same benchmark suite, posture and pacing as every other line: three ROS 2 nodes in three processes, with loaned publishing and loaned subscription takes. At 1 MiB that is **11.12 µs p50 / 37.21 µs p99**, beside **11.0 ms / 43.4 ms** for stock `rmw_fastrtps_cpp`, the chart's own stock ROS 2 line. The median stays between 10.43 and 10.65 µs from 64 B to 256 KiB and reaches 17.47 µs at 16 MiB. [Methodology](benches/latency/METHODOLOGY.md) · [Results and provenance](docs/benchmarks/results/8a84baf25d5d1710-2026-09-18-fixed100-rmw-cerulion/).

### Across platforms

![Native round-trip latency on Linux x86_64, NVIDIA Jetson and Apple Silicon, with single- and multi-process results.](docs/media/platform-rtt.svg)

The Apple M4 row is measured on Cerulion 2.0.0 under macOS 27.2.1, a later build than the x86-64 and Jetson rows, in which a multi-process consumer wakes on the publish. [macOS two-process data wake](docs/PERFORMANCE.md#macos-two-process-data-wake) measures that change on its own, before and after, on an Apple M3 Max.

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

## Tools for your coding agent

**Cerulion MCP** at `https://mcp.cerulion.com/mcp/cerulion` gives Claude Code, Codex and other MCP clients tools for your Cerulion Cloud instances, analysis of MCAP recordings served over `https://`, a list of the robots on your account and a simulated robot arm to drive with the harness's tools; [Give your model Cerulion](#give-your-model-cerulion) shows a prompt for each side.

**Hindsight** at `https://mcp.cerulion.com/mcp/hindsight` serves the recording-analysis tools on their own, with answers grounded in specific topics and timestamps.

Both sign in with OAuth using your Cerulion account. The hosted endpoints have no local workspace tools, so build, run, record, verify and inspect nodes and topics with the `cerulion` CLI.

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
  version = {2.1.0},
  url     = {https://github.com/cerulion-inc/cerulion}
}
```
