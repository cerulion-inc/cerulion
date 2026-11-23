---
name: verify-harness
description: Prove that the Cerulion harness (the harness/ folder behind `cerulion rcp --sim` and `cerulion rcp bench`) works by driving it the way a user and an MCP client do. Covers the offline gates, the one-time simulator install, the sim server over HTTP and stdio, one scored robot episode with its verdict, and cleanup. Use before and after any change under harness/, when a harness check fails, or when an agent must act as the model on the simulated robot.
---

# Verify the harness

The harness serves MCP tools for a simulated robot arm (LIBERO-PRO on MuJoCo), one episode per client, and
scores each episode with the benchmark's own success check. Read `harness/AGENTS.md` first: it holds the CLI
contract and the invariants that this skill checks.

All commands run from the repository root of the checkout you change. The helper is
`.claude/skills/verify-harness/scripts/hv.py` (standard library Python). It works on the `harness/` folder of
the checkout it sits in, so each git worktree drives its own harness. `--harness DIR` and `HV_HARNESS` name
another folder: leave both unset unless the operator asks for one.

```bash
HV=.claude/skills/verify-harness/scripts/hv.py
```

## Fences

- Never edit, format or lint `harness/measured/` or `harness/provenance/`. They are the code the published
  benchmark ran, byte for byte, and `rcp.sh verify` fails on any change. The tool behavior a model sees
  (`look`, `put`, `robot://body`) lives there. A change to it is a new measured snapshot and a new benchmark
  run: stop and give the maintainer the proposal, with your episode evidence.
- Never run `cerulion rcp bench` or `run-cell` against a model endpoint without the operator's yes in this
  session. Each setup gets up to 80 model replies with images, billed to the operator's key, and `run-cell`
retries an episode that was not scored up to 3 times, each with a new budget.
- Never drive a server that this checkout did not start (`hv.py doctor` shows the one it started).
- Never commit `harness/bx/`, `harness/venv/`, `harness/out/` or a body record measured on this machine.

## What you may change, and the gate for each

| Path | What it is | Gates before you call it done |
|---|---|---|
| `harness/playground/` | the sim server: transports, episodes, labels, idle limit, pages, settings | Tier 1, `$HV stdio`, `$HV smoke`, the [sim-http](features/sim-http.md) recipes |
| `harness/src/cerulion_harness/` | `run-cell` (bench), body records, prices, `verify` | Tier 1, the [bench](features/bench.md) refusals, [verify](features/verify-provenance.md) |
| `harness/rcp.sh`, `harness/install.sh` | the CLI contract and the install | Tier 1, `$HV stdio`, [install](features/install.md); a flag change also edits `harness_cmd.rs` and `harness/tests/test_rcp_launcher.py` |
| `crates/cerulion_cli_engine/src/harness_cmd.rs` | how `cerulion rcp` finds and runs the harness | the two cargo tests in Tier 1, then `$HV up --cli` with the built binary |
| `harness/requirements/`, `harness/benchmarks/` | pins of every package and data file | a fresh install in a new worktree, then `$HV smoke` |
| `harness/measured/`, `harness/provenance/` | the tools, their texts and the body knowledge a model sees | frozen (see Fences) |

Measure a change by what it touches. A server, runner or install change must keep every recipe it touches
green: run each before and after, and report both. `$HV smoke` must still end `solved=True` (the step count
varies by a few steps between runs, so compare the verdict, not the steps). A change in what a model sees is
measured only by the benchmark over many start states and models (`docs/harness.md`, "What it measured");
episodes you drive yourself are examples for the proposal, not a rate.

## Doctor

Run it first, and again after any surprise. It is read-only.

```bash
$HV doctor            # add --cli PATH to check a cerulion binary for `rcp --sim`
```

Each line is `PASS` or `FAIL` with the reason: harness folder, install, `measured/` untouched, disk free, a
clean Python for the unit tests, the optional `cerulion rcp`, and the server this checkout started (pid
alive and `/health` 200). Exit 0 means every required line passed.

## Tier 1: offline gates (no install, no network)

```bash
$HV test
```

It runs, and logs each step to `~/.cache/cerulion-verify-harness/test-<time>/`:

1. `python -I -B -m unittest discover -s harness/tests` on a Python 3.10+ with no MCP SDK (under a minute).
   The CI job `Harness (Linux)` runs the same.
2. With an install, the transport tests (`test_playground_*.py`) again with `harness/venv/bin/python`, which
   has the pinned MCP SDK, so the tests that skip in step 1 run.
3. `shellcheck -x` over every tracked `harness/**/*.sh`.
4. `bash harness/rcp.sh verify`: the measured code against its provenance ledger.

A Python that has an MCP SDK of another version than the pin makes step 1 fail, and slowly. `hv.py` picks a
clean interpreter for that reason. Do not "fix" those failures in the harness.

A change to `crates/cerulion_cli_engine/src/harness_cmd.rs` or `rcp.sh` flags also needs
`cargo test -p cerulion_cli_engine --lib harness_cmd` and
`cargo test -p cerulion_cli --test login_gate_e2e_test -- --test-threads=1`.

## Install (once per harness folder)

`hv.py up` runs `rcp.sh`, which installs on first use. To install ahead, or to see the progress:

```bash
bash harness/install.sh            # minutes; progress on stdout (rcp.sh sends it to stderr); resumes
bash harness/install.sh --check    # exit 0 when installed; writes nothing
```

It needs `git`, `curl`, network access to GitHub, Hugging Face and PyPI (on Linux also
download.pytorch.org), and 5 GB free to start. It writes
`harness/bx/`, `harness/venv/` and a body record for this machine, all ignored by git. With uv's cache warm,
another worktree installs in minutes and uses less disk than the first. On a small or shared disk, ask the
operator before the first install. The first episode of a robot body with no record on this machine
measures one first (one to two minutes).

## Tier 2: launch the sim server

```bash
$HV up                                  # rcp.sh sim --http on a free loopback port
$HV up --task open-drawer               # bowl-on-plate (default), open-drawer, soup-in-basket
$HV up --cli "$(command -v cerulion)"   # the user path: cerulion rcp --sim, needs cerulion 2.1.0+ and login
```

Ready means `/health` answers 200. `up` prints the MCP URL and the run folder. Leave `--port` unset: a
fixed port can collide with another server. One server per checkout: `up` refuses while one runs. With
`rcp.sh`, the server's own record goes to the run folder's `home/` (`CERULION_HOME`). With `--cli`, the CLI
needs the real `~/.cerulion` for its login, so episode records go to `~/.cerulion/sim/episodes/`.

## Tier 3: drive one episode as the model

An episode is named by a label (`--episode a`, the default). Calls with the same label share one robot, so
several commands in a row drive one episode. Use a new label for each new episode.

```bash
$HV tools --episode a                    # starts the episode (seconds); saves instructions and listing
$HV read robot://task --episode a        # the task sentence
$HV call look '{"camera": "agentview"}' --episode a     # scene look C1; prints the image paths
$HV call put '{"pick": {"look": "C1", "px": [U, V]}, "to": {"look": "C1", "px": [U, V]}, "clear": true}' --episode a
$HV call claim '{"statement": "...", "success": true}' --episode a
$HV episode                              # status, end reason and the benchmark verdict of every episode
```

`$HV smoke` runs the same two steps by a fixed script on a new label (`look` C1, then `put` from the bowl
to the plate) and exits 0 only when the verdict is `solved=True`. Its default pixels fit the
`bowl-on-plate` picture; `--pick` and `--to` take others as JSON.

Read every image path that `call` prints with your image viewer before you pick pixels. Read the saved
instructions file once: it is the system text a model gets, with the units, frames and budget (80 calls,
45 simulated seconds, 900 control steps). The feature files below hold tested recipes.

The proof of an episode is the verdict from `hv.py episode`, not the tool's own words. `placed: true` with
`solved=False` is a real outcome (wrong object or spot), not a harness fault. The simulator ends an episode by
itself when the success check turns true (`ended (over)`, no claim needed).

## Evidence

Everything goes under `~/.cache/cerulion-verify-harness/<run>/` (`HV_RUNS` moves it):

- `run.json` (pid, URL, command), `server.log`, `server.stdout`.
- `evidence/NNN-<tool>.json`: each request with its full answer; images decoded to `NNN-<tool>-K.jpg`.
- `evidence/NNN-episode-<label>-instructions.md` and `-listing.json`: what the model was told and offered.
- `evidence/NNN-episode-record-<id>.json`: the viewer record with the verdict.
- `home/sim/episodes/<id>/`: the server's own transcript and pictures (rcp.sh mode).

A proof cites the run folder and the verdict line, and drives the user's path (`/mcp`, `rcp.sh`, the CLI),
never a test stand-in. Besides the run folder, a run writes: `harness/out/verify-current` (the pointer to the
run), each episode's work folder under `/tmp/pg<uid>/` (removed when the episode ends), a body record in
`harness/body_records/` on a first episode of a body, and, in `--cli` mode, `~/.cerulion/sim/`.

## Cleanup

```bash
$HV down
```

It checks that the recorded pid still runs the server it started (its command line names the listen
address; a reused pid is left alone). It sends SIGTERM to the server's process group, waits while the
server scores and stops live episodes, then
stops any process whose command line still names one of this server's episode work folders (the simulators
run in sessions of their own). It removes the `harness/out/verify-current` pointer and keeps the run folder.
Check `ls /tmp/pg$(id -u)/` is empty of this run's episode ids afterwards. Delete old run folders only when
the operator asks. To remove an install, delete `harness/bx` and `harness/venv`.

## Helper reference

`hv.py` verbs: `doctor`, `test`, `up [--task T] [--port P] [--cli PATH]`, `tools`, `call TOOL [JSON]`,
`read URI`, `episode [ID] [--solved]`, `smoke [--pick JSON] [--to JSON]`, `stdio`, `down`. Exit codes: 0 ok,
1 failed (for `episode --solved` and `smoke`: an episode not solved), 2 usage, 3 the tool answered with
`isError`. It speaks MCP in the per-request form (protocol `2026-07-28`, no session), as
`harness/tests/test_playground_transports.py` does.

## Feature map

[`features/README.md`](features/README.md) lists every user-facing feature with a recipe and the end state
that proves it. A proof that drives one entry point is incomplete when the map lists others.
