# rcpx over MCP (stdio)

`rcpx/mcp_stdio.py` puts one episode's RCP surface behind the Model Context Protocol, so any MCP client (Claude
Code, Claude Desktop, others) can work the robot, with frames delivered inline as images. It is a thin bridge:
the episode server (`rcpx/server.py`) still owns the surface, the budget, the refusals and the journal; the bridge
asks its socket `EPDIR/rcp.sock` and translates. Nothing about the tools is copied: names, descriptions and
schemas come from the server's `list` answer at run time.

Launch (cwd ROOT; the episode server should be up, but the bridge also starts first and asks again later):

    ROOT/venv/bin/python -m rcpx.mcp_stdio --ep EPDIR [--timeout S] [--verbose] [--plain-schema]

Add it to Claude Code (a person types this; no script here runs it). PYTHONPATH lets `-m` find the package from
whatever directory the client starts it in:

    claude mcp add rcpx-episode -e PYTHONPATH=/path/to/project -- /path/to/project/venv/bin/python -m rcpx.mcp_stdio --ep /absolute/path/to/EPDIR

Claude Desktop takes the same command, args and env under `mcpServers` in its config file. A long move can
outlast a client's tool timeout (Claude Code: `MCP_TOOL_TIMEOUT`, in ms); the bridge itself waits `--timeout`
seconds (default 900) per request. ROOT/README.md section 2 shows the whole route from a started episode, and
tests/test_docs_fix2.py runs that launch line against a live episode.

What a client sees:
- initialize: instructions. One line names the resources the episode lists, in the order robot://task,
  robot://practices, robot://body, robot://devices (v0 serves no practices, so it is not named there); it orders no
  step. Then the result layout, refusals and claim; then the surface's conventions (the list answer's `about`: the
  opening generated from the session's own tools, one line a tool with what it costs here in control periods, then
  units, frames, which calls are not charged against the server's call count, how a brief answer is made whole),
  the budget when the harness or the person stated one ("none is stated" otherwise) and the practices text.
- tools/list: every tool of the episode's surface (description plus its worked example; its argument schema as
  inputSchema), then `claim`, which ends the episode with the operator's own statement. `--plain-schema` (or the
  environment variable RCPX_MCP_PLAIN_SCHEMA=1) serves each schema in a plain form for a model gateway that refuses
  keywords such as minimum or minItems (they move into the argument's description); the episode server still
  checks every call against the full schema.
- tools/call: one text block with the server's JSON answer (result, budget, frames), then one JPEG image per
  frame, in the order of `frames`, each under 500000 base64 characters. A frame pillow had to shrink is
  preceded by a note giving its scale (to map pixels back for `mark`); a missing frame, or one outside EPDIR,
  gets a note in place of its image. A long answer is brief and says what it left out: send `"detail": "full"`
  in the arguments, or read `robot://result/{seq}` with the answer's seq, for the whole of it.
- A refusal (`{"refused": {check, why, hint}}`) is a normal result; its why and hint say what stopped the call
  (an `internal` refusal can follow motion). Only an ended episode (claim, e-stop, server stopped) or an
  unreachable one is `isError`, with a plain hint. Checks the derived bounds add: `clear_needs_look` (the word
  clear with no look newer than the last contact or act that moved a thing: the hint holds the look call),
  `force_unknown` (a push with no force stated and nothing known of the part: the hint offers the ramp with a
  max_n to state), `selfknow_missing` (the robot has not measured itself and may not here: the hint names the
  owner's setting).
- resources/list and resources/templates/list: whatever the server lists, under robot:// (on vnext: body,
  practices, world, devices, devices/{id}, journal, state, asks, epoch, experience, look/{look_id},
  result/{seq}, task; on v0: task, body, devices, devices/{id}, result/{seq}); resources/read gives JSON or text; an unknown URI
  is error -32002 carrying the refusal's why and hint.
- Requests reach the socket one at a time, in arrival order, through the bridge's own client (the rcpx.cli
  exchange, except that a failed write shutdown after the server answered an episode-ending request and closed
  still reads that answer, so a claim or e-stop result is never lost). stdout carries protocol only;
  diagnostics go to stderr.

## What the owner of a deployment may state (SPEC-R4 A6)

rcpx types no speed, force, torque, travel or time limit. Every bound is derived for the session from the robot's
own ratings, what it measured on itself, what contact has shown and what it has reason to expect in the way, and
every result that moved or pushed names the one bound that held it and what would lift it. Numbers a tool text
shows are this session's derived values with their source, put in when the list is served; a tool schema carries
no range for a speed, a force, a travel or a time. The owner of a deployment may state the following through the
executor's cfg (the launcher of the episode server passes it). None of them has a default; a session with none
stated says so in `robot://body` under `session` and in its first result that moved:

- `site`: a site limit: a tip speed, a contact force and, if wanted, zones (a box in the base frame with its own
  speed and force). The lower of the site's value and the derived bound stands, and the result names its source
  as site.
- `people`: true when people or machines may enter the reach. Space the robot cannot see again is then unseen at
  every call, and the operating model's word `clear` needs a look taken after the last call that moved.
- `support`: the plane the robot stands on, with its error. A height stated with no error is checked by the first
  look or touch.
- `selfknow_dir` and `selfknow`: where the robot's own measured numbers are kept (one file per robot and
  controller, nothing about a place) and whether a session may measure them: measure lets a backend that reports
  hardware run the self-test, require refuses to start without the stored file (a benchmark measures it once per
  robot, outside every scored episode).

The budget (calls, simulated seconds) is the harness's or the person's to state; none stated means none. The
operating model states its own intent in the call: a lower speed, `clear` (true or a look id: it looked and judges
the way clear), a force, `max_n`, a travel, `expect` contact. The socket's idle bound (the environment variable
RCPX_SERVE_IDLE_S, in seconds) is a transport bound that cuts no motion: when it passes, the session parks the arm
and then closes as abandoned.

Tests: `ROOT/venv/bin/python tests/test_mcp_stdio.py` (the `mcp` SDK is installed in the venv) and, against a
real episode, `ROOT/venv/bin/python tests/test_docs_fix2.py mcp`.
