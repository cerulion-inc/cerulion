# SPDX-License-Identifier: AGPL-3.0-only
"""The Cerulion sim server: the harness's MCP tools on a simulated robot, one episode per client session.

It serves MCP over stdio (the default) or over streamable HTTP (`--http`), on this machine. Each episode is one
simulator: the measured snapshot's episode server and its simulator host, started when a client connects and
stopped at the claim, at the episode clock, when the client goes away, when no call has arrived for the idle
limit, or when the server stops. README.md beside this file describes the settings and the HTTP pages.

Modules: settings (the settings file and the install check), common (small shared helpers), history (the
record of ended episodes), store (what is kept of an episode for its viewer page), simulator (start, score and
stop through the measured snapshot), episode, pool, mcp_server (the MCP handlers), web (the HTTP pages), app
(the two transports) and cli (the command line).
"""
