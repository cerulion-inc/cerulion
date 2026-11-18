# The sim server

Experimental. `cerulion rcp --sim` starts this server: the harness's MCP tools on a simulated robot arm, one
episode per connection, on your own machine. Your model client does the thinking; the simulator runs here.

It also runs on its own, with the Python of the harness install (it carries the MCP SDK and the server's pins):

```bash
python harness/playground                          # MCP over stdio (the default)
python harness/playground --http                   # MCP at http://127.0.0.1:8765/mcp, with viewer pages
python harness/playground --http 127.0.0.1:9000 --state open-drawer
python harness/playground --check                  # is everything installed? JSON, exit 0 or 1
```

Exit codes: 0 served (or ready, for `--check`), 1 something it needs is not installed (the message names it),
2 a usage or settings error.

## An episode

Each episode is one simulator: the measured snapshot's episode server and its simulator host. It starts when a
client connects and is ready in seconds. The first episode of a robot body that has no calibration record on
this machine measures one first, once (one to two minutes), and says so. An episode ends at the claim, when the
simulator halts it (its success check turned true, or its 900 control steps are used), at the 5,400 s episode
clock, when no call has arrived for the idle limit, when the client goes away, or when the server stops. The
answer to the call that ended it, and to every call after, says how it ended and, once it was scored, the
benchmark's own verdict. A stop signal (SIGINT, SIGTERM) scores and stops every live episode before the server
exits.

Over stdio the server has one client. Once its episode has ended, listing the tools again starts the next one.
The transcript and the pictures of every episode are kept in `~/.cerulion/sim/episodes/ID/` (under
`CERULION_HOME` when that is set).

Over HTTP each MCP session gets its own episode, which no other client reaches; `?state=LABEL` on the URL
picks the start state, and `&episode=N` a second episode for a client that keeps no session. An idle client
connection is kept open for the idle limit (the episode clock when no limit is set), so a client can keep one
connection between turns. The pages beside `/mcp`:

| path | what |
|---|---|
| `/` | the address to add to a client, and the start states |
| `/view/ID` | one episode as it runs: pictures, each call and its result, the claim, the benchmark's verdict. Not private: anyone who can reach the server can watch it, and `/episodes` lists every episode |
| `/api/episode/ID`, `/frames/ID/NAME` | what the viewer reads |
| `/episodes`, `/health` | the live and ended episodes; the server's state and its idle limit |
| `/source` | the license and where the source is |

The server has no sign-in of its own: on `--http` anyone who can reach the address can start simulators and
watch every episode. It listens on loopback unless given another address, answers only the Host names of the
address it listens on, and warns when that address is not loopback.

## Start states

Only development start states are served: the ones the tools were built on, never a start state of the published
results. Each must be on `development_states.txt` (the measured snapshot's development list), and the measured
code's frozen cell lists must not seal it. `settings.json` offers three; `bowl-on-plate` is the default.

## Settings

`settings.json` is the default; `--settings FILE` names another with the same keys. A relative path is read
against the harness folder; a path may name `{uid}` (this user's id) and `{cerulion_home}` (`CERULION_HOME`, or
`~/.cerulion`). An unknown key is refused.

| key | what |
|---|---|
| `snapshot_root`, `bench_root`, `body_records` | the measured code, the simulator and its scenes, the robot body records |
| `data_dir`, `keep_root` | where the episode history (`episodes.sqlite`) and the kept episodes go |
| `work_root` | each running episode's simulator files and sockets; short, because a socket path must fit 103 bytes |
| `states`, `default_state` | the start states offered (`cell`, `title`, `note`) and the default |
| `idle_reap`, `idle_s` | the idle limit: factor times a stated percentile of the gaps between calls, with the figure's source, or of this server's own history once it holds more gaps and puts the percentile higher (it never shortens the limit); `idle_s` is a fixed limit in seconds |
| `lazy_records`, `body_record_module` | measure a missing robot body record before an episode, and the module that does it |
| `gl`, `macos_envhost`, `env` | the renderer on Linux, the macOS simulator host script, the simulator's environment |
| `budget_requests`, `budget_steps`, `clock_s`, `pool_size` | the episode's replies, control steps and clock; how many simulators at once |
| `keep_full_pictures_s`, `keep_last_pictures`, `keep_viewer_s` | every picture is kept 1 day, then the last 8 until the viewer link ends at 30 days |
| `public_url`, `allowed_hosts`, `allowed_origins` | over HTTP, when the port is published under another address |

## Tests

```bash
python3 -B -m unittest discover -s harness/tests -p 'test_playground_*.py'
```

They need no simulator: a stand-in launcher and bridge in `harness/tests/fixtures/fake_snapshot/` take the
measured code's place. The transport tests run the server as a program and need the MCP SDK; without it they are
skipped.
