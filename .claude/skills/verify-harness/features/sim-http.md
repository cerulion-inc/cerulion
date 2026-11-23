# Sim episode over HTTP

`cerulion rcp --sim --http ADDR` serves the harness's MCP tools on a simulated robot arm. Each MCP session,
or each `?episode=LABEL` of a client without a session, gets its own episode of one task. The episode ends
at the claim, when the success check turns true, at its budget, when idle, or when the server stops, and the
answer that ends it carries the benchmark's verdict. Pages beside `/mcp` show each episode as it runs.

## Sub-features

- `http-start` starts the server and answers `/health`.
- `http-episode` starts an episode on the first request of a label, with its instructions, tools and
  resources.
- `http-act` runs a tool call on the robot and returns its JSON answer and pictures.
- `http-claim` ends the episode with a claim and returns the verdict.
- `http-over` ends the episode by itself when the success check turns true.
- `http-ended` answers a call on an ended episode with "ended", never with a new robot.
- `http-label` refuses a request with no session and no `episode` label.
- `http-pages` serves `/`, `/view/ID`, `/api/episode/ID`, `/episodes`, `/health` and `/source`.
- `http-stop` scores and stops every live episode on SIGTERM.

## How to get to it (user POV)

- Run `cerulion rcp --sim --http 127.0.0.1:8765` (optionally `--task open-drawer`), then add
  `http://127.0.0.1:8765/mcp` to an MCP client.
- A checkout runs the same program with `bash harness/rcp.sh sim --http ADDR`.

## Driving it with hv.py

Preconditions:

- `$HV doctor` passes, including the install line.
- No server of this checkout runs.

- **Start.** Run `$HV up`. It prints `up: http://127.0.0.1:PORT/mcp` and a `health:` line with `"ok":true`
  and the three start states. With `--cli "$(command -v cerulion)"` the same runs through the CLI.
- **Episode.** Run `$HV tools --episode a`. It prints the episode id, the instructions file, 42 tools plus
  `claim`, the resources (`robot://task`, `robot://body`, `robot://cameras` and others) and three templates.
- **Task.** Run `$HV read robot://task --episode a`. On `bowl-on-plate` the instruction is "pick the akita
  black bowl between the plate and the ramekin and place it on the plate".
- **Look.** Run `$HV call look '{"camera": "agentview"}' --episode a`. The answer has `"look_id": "C1"` and
  two image paths. Read the raw one and find the plate, the ramekin and the black bowl between them.
- **Act.** Run `$HV call put '{"pick": {"look": "C1", "px": [U, V]}, "to": {"look": "C1", "px": [U, V]}, "clear": true}' --episode a`
  with the bowl's and the plate's pixels. On macOS arm64 the bowl was near `[195, 152]` and the plate near
  `[140, 185]` in C1; read your own picture first.
- **Over.** When the bowl lands on the plate, the answer is `"episode": "ended"`, `"reason": "over"`, and a
  message that the success check turned true. No claim is needed.
- **Claim.** In another episode (`--episode b`), run
  `$HV call claim '{"statement": "verification run", "success": false}' --episode b`. The answer says the
  claim was recorded and gives the verdict.
- **Ended.** Run `$HV call state '{}' --episode b`. The answer is `"episode": "ended"`, `"reason": "claim"`.
- **Label.** Run `curl -s -X POST "http://127.0.0.1:PORT/mcp" -H 'content-type: application/json' -H 'accept: application/json, text/event-stream' -H 'mcp-protocol-version: 2026-07-28' -H 'mcp-method: server/discover' -d '{"jsonrpc":"2.0","id":1,"method":"server/discover","params":{"_meta":{"io.modelcontextprotocol/protocolVersion":"2026-07-28","io.modelcontextprotocol/clientCapabilities":{}}}}'`.
  The error message asks for `episode=LABEL` and `/health` still shows the same `live_simulators`.
- **Pages.** Run `curl -s http://127.0.0.1:PORT/episodes` and `curl -s http://127.0.0.1:PORT/view/ID`. The
  first lists live and ended episodes; the second is the viewer page of one.
- **Proof.** Run `$HV episode`. Each episode prints one line, for example
  `episode ID: ended (over), 3 calls, 11 events; verdict: solved=True claimed=None false_claim=False steps=153/900`.
- **Stop.** Run `$HV down` while an episode is live. It prints `0 leftover processes stopped`, and that
  episode's record ends with reason `server_stopped`.

## Gotchas

- `placed: true` in the answer of `put` does not mean solved. Only the verdict does: one run set a bowl on
  the plate from a wrist look, claimed success, and got `solved=False false_claim=True`.
- The wrist camera's top view is easy to misread. The fixed `agentview` camera shows the whole table.
- The first episode of a robot body with no record on this machine measures one first (one to two minutes).
- In `--cli` mode the episode records go to `~/.cerulion/sim/episodes/`, not to the run folder.
- After an end, run `$HV episode` before the next call, and start the next episode under a new label.
- Anyone who can reach the port can watch every episode. Keep the server on loopback.
