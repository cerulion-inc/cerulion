# The Cerulion harness

Experimental: in this release the harness drives a simulated robot.

Cerulion is the harness for physical AI: the tools and body knowledge a general model needs to work a robot it
was never trained on. The model is your own, any model that calls tools and reads images. The harness gives it
tools to look through the robot's cameras, move, touch, take a thing and set it down, and answers every call with
what the robot measured: where the hand stopped, what it felt, what a look shows, and which bound held a motion.
The model works from what this body and this scene show.

You reach the harness through `cerulion rcp`, Cerulion's MCP server command:

| Command | What it does |
|---|---|
| `cerulion rcp --sim` | Serves the harness's tools with a simulated robot on this machine, to any MCP client. |
| `cerulion rcp bench` | Reruns one start state with and without the harness, against your own model: one of the published benchmark with `--cell`, otherwise one the harness was developed on. |
| `cerulion rcp bench --verify` | Checks every file of the harness's measured code against the provenance ledger that ties it to the code the benchmark measured. |

Both run under the free Cerulion account, as almost every `cerulion` command does: run `cerulion login` once.

## Try it

The [README](../README.md#try-the-harness) starts the local `cerulion rcp --sim --http` server and runs one task on it.

Every release package installs the harness folder beside `cerulion`: the install script, the release archives,
the Debian package (`/usr/bin/harness`) and the Homebrew formula. A release's `cerulion` finds it there with no
clone and no setting, and a `cerulion` built in a checkout of this repository finds that checkout's own
([Where the harness lives](#where-the-harness-lives)). A `cerulion` installed with `cargo install` has no harness beside it: clone this repository at the tag of its version (`git clone --depth 1 --branch v<version> https://github.com/cerulion-inc/cerulion`, with the version `cerulion --version` prints) and set `CERULION_HARNESS_DIR` to the clone's `harness/` folder, or, for one installed from a checkout (`cargo install --path`), to that checkout's `harness/`.

```bash
cerulion login                                   # once, on this machine
cerulion rcp --sim --http 127.0.0.1:8765         # leave it running
```

In a second terminal, add it to your MCP client and ask for a task:

```bash
claude mcp add --transport http cerulion-sim http://127.0.0.1:8765/mcp
claude "With cerulion-sim, put the bowl on the plate."
```

Any MCP client works the same way: point it at `http://127.0.0.1:8765/mcp`. Each connection gets its own episode
of the simulated robot, set up for one task. The default task is `bowl-on-plate` (pick up a bowl and place it on
the plate); `--task open-drawer` (open the middle drawer of the cabinet) and `--task soup-in-basket` (pick up a
can and place it in the basket) are the other two. They are start states the harness was developed on. An
episode ends when the model makes its claim, when the episode's clock runs out, when the client goes away or
when no call arrives for a while; connect again for a new one.

Without `--http`, `cerulion rcp --sim` serves MCP over stdio, so a client can start it itself:

```bash
claude mcp add cerulion-sim -- cerulion rcp --sim
```

For a `cerulion` installed with `cargo install`, pass the variable to the server the client starts. Give it an
absolute path: the value is captured when the server is added, and the client starts it from its own directory.

```bash
claude mcp add cerulion-sim -e CERULION_HARNESS_DIR="$CERULION_HARNESS_DIR" -- cerulion rcp --sim
```

Run `cerulion rcp --sim` once in a terminal before you add it this way: the first run installs the simulator
and the benchmark scenes, which takes minutes, and a client that starts a stdio server waits only seconds for
its first answer.

Then run a start state yourself against your own model. Pick a published one from the results package's
`cells.csv`; without `--cell` the run uses a start state the harness was developed on, not a published one.

```bash
export OPENAI_BASE_URL=https://your-endpoint.example/v1
export OPENAI_API_KEY=your-key
cerulion rcp bench --model YOUR_MODEL --cell CELL_ID
```

The hosted Cerulion MCP also serves the harness's tools on a simulated arm, with nothing to install
([The hosted Cerulion MCP](#the-hosted-cerulion-mcp)).

## The hosted Cerulion MCP

The hosted Cerulion MCP gives your model a simulated arm in Cerulion's cloud (paid in sim credits), analysis of
your MCAP recordings and a view of your robots. It needs no install, only your Cerulion account. It is closed
source and in early access: request access on the
[Hindsight page](https://docs.cerulion.com/cerulion/hindsight). After you add it, run `/mcp` in Claude Code, pick `cerulion` and sign in with your Cerulion account. Another MCP client takes the same
URL.

```bash
claude mcp add --transport http cerulion https://mcp.cerulion.com/mcp/cerulion
```

What the hosted tools do:

- **A simulated arm in Cerulion's cloud.** Ask: "Start the simulated arm and put the bowl on the plate." Your
  model starts it (`sim_start`), reads its task (`sim_read` on `robot://task`), looks through its cameras
  (`look`) and uses the harness's other tools. `sim_end` stops it and reports the sim credits charged: the
  simulator runs on your account's sim credits, one credit for each second it is held open. The scenes are
  `bowl-on-plate`, `open-drawer` and `soup-in-basket`.
- **Recording analysis.** Put an MCAP recording at a public or presigned `https://` URL and ask: "Open the
  recording at https://<your host>/run.mcap. Summarize it and show me the worst anomaly, with its topic and
  time." Your model indexes it (`open_bag`), reads its summary (`get_debrief`), ranks anomalies
  (`find_anomalies`), looks around one (`scrub`) and cites each finding as `[topic @ t=<nanoseconds>]`, with a
  Foxglove link when the URL is public (`get_foxglove_link`).
- **Your robots and cloud instances.** `robot_list` shows your robots and which are online.
  `cloud_instance_start` and `cloud_instance_connect` reach your Cerulion Cloud instance.

The hosted tools cannot read a file on your machine, run a graph, record or verify; the `cerulion` CLI does
those. Hindsight serves only the recording-analysis tools, as its own connector:

```bash
claude mcp add --transport http hindsight https://mcp.cerulion.com/mcp/hindsight
```

Guide: [Connect an MCP client](https://docs.cerulion.com/cerulion/guides/connect-an-mcp-client).

## What the first run installs

The first `cerulion rcp --sim` or `cerulion rcp bench` installs, once: the simulator's Python environment, the
LIBERO and LIBERO-PRO benchmark trees at their pinned revisions with every pinned data file checked against its
SHA-256, and the episode server's Python environment. Its progress goes to stderr, so over stdio the MCP stream
on stdout stays clean. A later run checks the install and starts in seconds. Ctrl+C, or a SIGTERM or SIGHUP
sent to `cerulion`, stops the install and the programs it runs, and `cerulion` exits 130. An install that
stops (a lost connection, Ctrl+C) resumes where it stopped the next time you run the command.

The install also makes sure the simulated robot has a calibration record for this machine. Linux on x86-64
finds the shipped one. On another platform the robot measures itself once, about a minute, and stores the
record; a task with another robot body does the same the first time you use it, and says so.

| Requirement | |
|---|---|
| Hosts | macOS on Apple silicon; Linux on x86-64 or arm64. Windows is not supported. |
| Tools | `git` and `curl`. `uv` is used from `PATH`, or a pinned copy is fetched into the folder the harness runs from. |
| Linux | the CPU renderer: `sudo apt-get install libosmesa6 libegl1 libgl1 libglib2.0-0` on Ubuntu |
| GPU | none: the simulator renders on the CPU on Linux and through Apple's OpenGL on macOS |
| Disk | about 4 GB in the folder the harness runs from (see [Where the harness lives](#where-the-harness-lives)) |

The published results were measured on Linux x86-64. On macOS the pictures are rendered by Apple's OpenGL and
differ slightly from the measured ones, so an episode there is a try of the harness, not a reproduction of a
published row.

The install downloads from the benchmark's and the Python packages' own hosts. It reaches no Cerulion service,
so the network kill switch (`CERULION_NETWORK=off`) does not refuse it.

## What it measured

The benchmark is LIBERO-PRO: simulated robot manipulation tasks, each start state run twice with the same model.
Once without Cerulion, through the three-tool interface a published zero-shot study used (arXiv:2610.30051:
`move_to`, `done`, `give_up`). Once with Cerulion, through the harness's tools. Same simulator, same start
state, same budget of 80 model replies and 900 control steps, and the benchmark's own success check. Sixteen
models, 100 start states each, one episode a start state an arm. The start states are new start states of task
rows the harness was developed on at other start states, so the tasks themselves are not new to it.

Every count, cost and episode is in the [results package](https://github.com/cerulion-inc/cerulion/tree/main/docs/benchmarks/results/7cb9f322c10f6fa7-2026-11-18-harness-libero-pro),
with the reading rules, each frozen before the rows of its own lists, and a script that computes every figure
again from the rows.

Write-up: [Can an LLM control a robot arm? 73 of 100 in LIBERO-PRO sim](https://cerulion.com/blog/llm-robot-control-harness-libero-pro).

## How `cerulion rcp bench` reproduces it

`cerulion rcp bench --model YOUR_MODEL` runs one cell (one task at one start state) twice, once in each setup the
benchmark compared, with the same budget, temperature and scorer, and prints the benchmark's own verdict for
each. The code that runs is the harness's measured code, and `--verify` checks every file of it against the
provenance ledger that ties it to the code the benchmark measured.

| Flag | Meaning |
|---|---|
| `--model MODEL` | the model name your endpoint serves (required) |
| `--base-url URL` | any OpenAI-compatible chat completions endpoint that takes tool calls and images. Default: `OPENAI_BASE_URL` |
| `--key-env NAME` | the environment variable that holds your key, which is sent only to that endpoint. Default: `OPENAI_API_KEY` |
| `--cell ID` | the cell to run, named as in the results package's `cells.csv`. Default: the harness's development cell |
| `--out DIR` | where the two episodes' results go. Default: a new `cerulion-bench-<UTC time>` folder in the current directory |

Before the simulator starts it sends one small request to your endpoint, so a wrong address, key or model name
is reported at once. It exits 0 when both episodes ran and were scored, 1 when one was not scored, and 2 when it
refused to start (no endpoint, no key, an endpoint that refuses the key or the model). Ctrl+C, or a SIGTERM or
SIGHUP sent to `cerulion`, passes the stop to the run, its episode driver and the model client, so no further
request reaches your endpoint, and `cerulion` exits 130. The episode's simulator and episode server run in
sessions of their own, and the stop does not reach them.

One episode is one draw at temperature 1.0, so a single pair can go either way. The published rates come from
100 start states a model.

### Pictures

Both setups keep the newest 4 pictures in the model's conversation. The picture mode decides what happens to the
older ones:

| Mode | What the model is sent |
|---|---|
| `append` | every earlier picture stays where it was, so until the conversation passes the context limit each request starts with the whole previous request, and the endpoint's prompt cache can read it instead of writing it again. Past the limit, the pictures beyond the newest 4 go, all of them in one request |
| `swap` | on every request, each picture beyond the newest 4 is replaced by a short text inside its older turn |

A Claude model (a model name that holds `claude`, in any case) runs `append` with the harness, and its opening
(the tools, the system text and the first message) carries a prompt cache mark. Every other model, and every
model without the harness, runs `swap`. The start line names each setup's mode, and `summary.json` records it
with the operator flags it adds.

The mark covers only the opening. Whether the older pictures are read from the cache depends on the endpoint: one
that caches the growing conversation by itself bills them at its cached rate (on the endpoint the results package
ran through, 97.4 to 97.9 percent of each Claude model's input with the harness was read from the cache), and one
that caches only marked blocks bills them at its full input price on every request.

`cerulion rcp bench` runs these defaults. To name one mode for both setups, run the program behind it from the
harness folder with `--image-mode swap` or `--image-mode append`, once a first `cerulion rcp bench` has installed
the harness:

```bash
cd harness
venv/bin/python -P -m cerulion_harness run-cell --model YOUR_MODEL --image-mode swap --out ~/cerulion-bench-swap
```

It takes the other flags of `cerulion rcp bench` too (`--base-url`, `--key-env`, `--cell`). With `append` named,
the prompt cache mark still goes only to a Claude model with the harness; `swap` carries no mark.

## What `--verify` checks

`cerulion rcp bench --verify` hashes every file of the measured code in the harness folder and checks it against
the provenance ledger. For each shipped file the ledger holds the hash of the measured file it came from and
whether it is that file byte for byte or differs in comments and docstrings, prose, strings or code, with each
changed string and code edit listed. The measured files are those of the manifests of the two measured code
trees, one for each setup. The SHA-256 of a manifest is that tree's snapshot id:
`sha256:4f8bbe79fdf38df1770c068ec470fd3d5a629fd371f7288887cb3d275f9f747e` with the harness and
`sha256:6f040ca78bae0efd437005afc04ead78c4295e1e471d854b3e7ff0f9ea01f2a4` without. They are the trees
`sha256:7cb9f322c10f6fa7e64a3f8da7c22c67d67280eca436a03d2d87683496b4d5b8` and
`sha256:b73c548a1f0436f52ce41cffc7d4a4a2784e5d6170e2dd2c757c9cabdf146637` with the `append` mode added to two
operator files, `bench/operators/chat_support.py` and `bench/operators/chat_operator.py`; in `swap` mode they send
what those trees sent. It exits 0 only when both manifests have their snapshot ids and every file matches its
ledger row. It installs nothing; it needs `python3` on your PATH or a finished install, and, as every
`cerulion rcp` command does, a signed-in Cerulion account.

## Where the harness lives

The harness is the `harness/` folder of the Cerulion repository (see [harness/README.md](../harness/README.md)).
Every release package installs it beside the `cerulion` binary, without its tests and development files, and a
`cerulion` built in a checkout finds that checkout's own. `cerulion rcp --sim` and `cerulion rcp bench` find it
in this order:

1. `CERULION_HARNESS_DIR`, when set and not empty, to run another harness folder, such as a checkout's. A value
   that names a missing folder, or a folder that holds no harness, is an error; the search never falls through to
   the places below.
2. `harness/` beside the canonicalized `cerulion` binary, where the release packages install it. A link to
   `cerulion` in a `bin` directory still reaches the folder beside the real binary.
3. `harness/` at the root of the Cerulion source checkout the `cerulion` binary sits in: the nearest folder above
   the binary that holds `crates/cerulion_cli/Cargo.toml`. So a `cerulion` that `cargo run` or `cargo build` made
   in a checkout finds that checkout's harness; one built into a target folder outside the checkout
   (`CARGO_TARGET_DIR`) does not, and needs `CERULION_HARNESS_DIR`.

The current directory is never searched, so a folder you run `cerulion` in, such as a repository you downloaded,
cannot supply a harness. `CERULION_HARNESS_DIR` is taken as given. Places 2 and 3 never take a folder that every
user of the machine can write to, such as `/tmp`: the search for a checkout stops at the first one, so a
`harness/` someone else put there never runs.
When none of the three holds a harness, the error names where it looked.

The install writes inside the folder the harness runs from. Nothing writes into the folder beside the `cerulion`
binary, even when your user can write there: the first run copies it once, byte for byte, to `harness/<version>`
under the Cerulion config directory (`~/.cerulion`, or `CERULION_HOME`), and the install and every later run of
that version use the copy, also when `CERULION_HARNESS_DIR` names that folder. A new version makes its
own copy and leaves the old one in place; delete an old version's folder to free its space. Any other folder, one
that `CERULION_HARNESS_DIR` names or a source checkout's, runs in place and the install writes inside it; when your
user cannot write there, the first run copies it the same way.
