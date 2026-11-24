# Sim server over stdio

Without `--http`, `cerulion rcp --sim` serves MCP over stdin and stdout, so a client such as Claude Code can
start it itself. It has one client. Stdout carries only MCP messages, the first-run install included, and
listing the tools again after an episode ends starts the next one.

## Sub-features

- `stdio-clean` writes only JSON-RPC messages on stdout.
- `stdio-episode` starts the episode at `tools/list` and runs tool calls on it.
- `stdio-claim` ends the episode at the claim and answers later calls with "ended".
- `stdio-again` starts a new episode when the tools are listed again.
- `stdio-exit` exits 0 when stdin closes.

## How to get to it (user POV)

- `claude mcp add cerulion-sim -- cerulion rcp --sim`, then ask the client for a task.
- A checkout runs the same program with `bash harness/rcp.sh sim`.

## Driving it with hv.py and the shell

Preconditions:

- `$HV doctor` passes, including the install line.

- **Exchange.** Run `$HV stdio`. It starts `rcp.sh sim` with its own `CERULION_HOME`, sends `initialize`,
  `tools/list`, `state`, `claim`, `state`, `tools/list` and `state`, then closes stdin.
- **Proof.** Every line is `PASS`: `initialize: cerulion-rcp-sim`, `tools/list starts the episode: 43 tools`,
  `a call reaches the robot`, `the claim ends the episode`, `a call after the end is answered ended`,
  `listing the tools again starts the next episode`, `the server exits 0 when stdin closes: exit 0`. The
  run folder holds `transcript.json` and `server.log`.
- **Stray output.** A line on stdout that is not JSON prints `FAIL  stdout carries only MCP messages` with
  the line. That is a broken contract of the CLI, not a test flake.

## Gotchas

- Over stdio the ended message says "list the tools again or restart the server"; over HTTP it says
  "connect again". Both are correct for their transport.
- A client that starts a stdio server waits only seconds for its first answer. Install first
  (`bash harness/install.sh`) or the client gives up during the install.
