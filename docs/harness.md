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
| `cerulion rcp bench` | Reruns one task of the published benchmark with and without the harness, against your own model. |
| `cerulion rcp bench --verify` | Checks that the harness code on disk is the code the benchmark measured. |

Both run under the free Cerulion account, as the other `cerulion` commands do: run `cerulion login` once.

## Try it

```bash
cerulion login                                   # once, on this machine
cerulion rcp --sim --http 127.0.0.1:8765         # leave it running
```

In a second terminal, add it to your MCP client and ask for a task:

```bash
claude mcp add --transport http cerulion-sim http://127.0.0.1:8765/mcp
claude "Put the bowl on the plate."
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

Run `cerulion rcp --sim` once in a terminal before you add it this way: the first run installs the simulator
and the benchmark scenes, which takes minutes, and a client that starts a stdio server waits only seconds for
its first answer.

Then rerun a task of the benchmark against your own model:

```bash
export OPENAI_BASE_URL=https://your-endpoint.example/v1
export OPENAI_API_KEY=your-key
cerulion rcp bench --model YOUR_MODEL
```

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
Once without Cerulion, through the three-tool interface the benchmark's authors published for zero-shot models
(`move_to`, `done`, `give_up`). Once with Cerulion, through the harness's tools. Same simulator, same start
state, same budget of 80 model replies and 900 control steps, and the benchmark's own success check. Fifteen
models, 100 start states each, one episode a start state an arm. The start states are new start states of task
rows the harness was developed on at other start states, so the tasks themselves are not new to it.

Every count, cost and episode is in the [results package](https://github.com/cerulion-inc/cerulion/tree/main/docs/benchmarks/results/7cb9f322c10f6fa7-2026-11-18-harness-libero-pro),
with the reading rules frozen before the first row and a script that computes every figure again from the rows.

## How `cerulion rcp bench` reproduces it

`cerulion rcp bench --model YOUR_MODEL` runs one cell (one task at one start state) twice, once in each setup the
benchmark compared, with the same budget, temperature and scorer, and prints the benchmark's own verdict for
each. The code that runs is the code the benchmark measured: the harness folder carries it unchanged, and
`--verify` proves it.

| Flag | Meaning |
|---|---|
| `--model MODEL` | the model name your endpoint serves (required) |
| `--base-url URL` | any OpenAI-compatible chat completions endpoint that takes tool calls and images. Default: `OPENAI_BASE_URL` |
| `--key-env NAME` | the environment variable that holds your key, which is sent only to that endpoint. Default: `OPENAI_API_KEY` |
| `--cell ID` | the cell to run, named as in the results package's `cells.csv`. Default: the harness's development cell |
| `--out DIR` | where the two episodes' results go. Default: a new `cerulion-bench-<UTC time>` folder in the current directory |

Before the simulator starts it sends one small request to your endpoint, so a wrong address, key or model name
is reported at once. It exits 0 when both episodes ran and were scored, 1 when one was not scored, and 2 when it
refused to start (no endpoint, no key, an endpoint that refuses the key or the model).

One episode is one draw at temperature 1.0, so a single pair can go either way. The published rates come from
100 start states a model.

## What `--verify` checks

`cerulion rcp bench --verify` hashes every file of the measured code in the harness folder and compares it with
the manifests of the two code trees the benchmark ran, one for each setup. The SHA-256 of a manifest is that
tree's snapshot id, and the results package names the same ids. It exits 0 only when every file matches. It
installs nothing and needs only `python3`.

## Where the harness lives

The harness is the `harness/` folder of the Cerulion repository (see [harness/README.md](../harness/README.md)).
`cerulion rcp --sim` and `cerulion rcp bench` find it in this order:

1. `CERULION_HARNESS_DIR`, when set and not empty. A value that names a missing folder, or a folder that holds no
   harness, is an error; the search never falls through to the places below.
2. `harness/` beside the `cerulion` binary, where the release packages install it. A link to `cerulion` in a
   `bin` directory still reaches the folder beside the real binary.
3. `harness/` at the root of the Cerulion source checkout the `cerulion` binary sits in: the nearest folder above
   the binary that holds `crates/cerulion_cli/Cargo.toml`. So a `cerulion` that `cargo run` or `cargo build` made
   in a checkout finds that checkout's harness; one built into a target folder outside the checkout
   (`CARGO_TARGET_DIR`) does not, and needs `CERULION_HARNESS_DIR`.

The current directory is never searched, so a folder you run `cerulion` in, such as a repository you downloaded,
cannot supply a harness. When none of the three holds a harness, the error names where it looked.

The install writes inside the folder the harness runs from. Nothing writes into the folder the release packages
install, even when your user can write there: the first run copies it once, byte for byte, to `harness/<version>`
under the Cerulion config directory (`~/.cerulion`, or `CERULION_HOME`), and the install and every later run of
that version use the copy, also when `CERULION_HARNESS_DIR` names the installed folder. A new version makes its
own copy and leaves the old one in place; delete an old version's folder to free its space. Any other folder, one
that `CERULION_HARNESS_DIR` names or a source checkout's, runs in place and the install writes inside it; when your
user cannot write there, the first run copies it the same way.
