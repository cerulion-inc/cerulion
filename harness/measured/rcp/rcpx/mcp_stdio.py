# SPDX-License-Identifier: AGPL-3.0-only
"""MCP stdio endpoint for one rcpx episode: any MCP client reaches the episode's RCP surface (SPEC section 3).

    ROOT/venv/bin/python -m rcpx.mcp_stdio --ep EPDIR [--timeout S] [--verbose] [--plain-schema]

The episode server (rcpx/server.py) answers on EPDIR/rcp.sock. This process speaks MCP (official "mcp" SDK,
low-level Server API) on stdin and stdout and answers from that socket. Requests reach the socket one at a time,
in arrival order; the list answer is asked for once (at startup, else at the first listing) and kept:

- tools/list: every tool in the socket's list answer, in its order: name, description plus its worked example,
  argument schema as inputSchema; then claim (the claim op: it ends the episode with the operator's own
  statement). Nothing is copied from rcpx/surfaces.py. With --plain-schema (or the environment variable
  RCPX_MCP_PLAIN_SCHEMA=1) each inputSchema is the plain form of bench.operators.portable.plain_schema: the keywords
  a strict model endpoint may refuse (minimum, maximum, minItems, pattern, default, ...) are said in the argument's
  description instead, and anyOf or oneOf is flattened where a branch is plain. Use it when the client's model
  gateway answers the tool list with an error such as "number types do not support minimum": the bridge never sees
  that answer (the client does), so it cannot switch by itself. The episode server still checks every call against
  its full schema.
- tools/call: the call op (claim: the claim op). Content: one text block holding the episode server's JSON answer
  (result, budget, frames), then one JPEG image block per path in its frames list, in that order, each under
  MAX_IMAGE_B64 base64 characters (pillow downscales a frame that does not fit, and a text block before it gives
  the scale, so pixels read off it can be mapped back for mark). A frame that cannot be read, or that resolves
  outside EPDIR, gets a text block in its place saying why. A refusal is a normal result; only an ended or
  unreachable episode is isError, with a plain hint.
- resources/list, resources/templates/list, resources/read: the socket's resource list and read op. A refused
  read is an MCP error (-32002 for an unknown URI) carrying the refusal's why and hint.
- initialize: instructions. One line names the resources, only those the startup
  list shows, in the order task, practices, body, devices (v0 serves no practices, so it is not named there); the
  bridge adds no advice of its own about what decides a state (the practices and the surface's conventions say
  that, in one place). Then the result layout, refusals and claim; then, when the startup list was answered, the
  surface's conventions, the budget and the practices text the episode server sends with its first full list
  answer. When the startup list got no answer, those practices ride with a later answer instead: the bridge
  keeps the ones its lazy full list brings for the next call's JSON (the server never sends them with a read).

The socket exchange is rcpx.cli.send's (one JSON line out, one back) with one difference: a failed write shutdown
is not a failure. The server answers an episode-ending request (claim, an e-stop result) and closes at once, and
macOS can then refuse the shutdown with ENOTCONN while the answer already waits in the receive buffer.

Nothing but protocol goes to stdout (the SDK points fd 1 at stderr while it serves); diagnostics go to stderr.
"""
import argparse
import base64
import io
import json
import logging
import math
import os
import socket
import sys
import time

import anyio
import anyio.to_thread
import mcp.types as types
from mcp.server.lowlevel import Server
from mcp.server.stdio import stdio_server
from mcp.shared.exceptions import MCPError

from bench.operators.portable import plain_schema

from .server import END_NAME, SOCK_NAME

log = logging.getLogger("rcpx.mcp_stdio")
PLAIN_ENV = "RCPX_MCP_PLAIN_SCHEMA"   # 1, true or yes: serve plain schemas (as --plain-schema)

SERVER_NAME = "rcpx"
SERVER_VERSION = "0.1"
CLAIM = "claim"                    # the claim op, offered as one more tool
PRACTICES_URI = "robot://practices"
TASK_URI = "robot://task"
MAX_IMAGE_B64 = 500_000            # base64 characters per image block (about 366 KB of JPEG)
JPEG_QUALITY = 85
MIN_SIDE_PX = 16
# Client transport bound: cuts no motion; must be at least the longest primitive's own time bound.
DEFAULT_TIMEOUT_S = 900.0
PROCESS_POLL_S = 0.5               # while the startup list is awaited: how often the server's process is looked at
RESOURCE_NOT_FOUND = -32002        # the MCP error code for an unknown resource URI

INTRO = "This server is one robot episode on the Robot Context Protocol."
READ_UNKNOWN = (f"The resources: {TASK_URI} and the others resources/list shows ({PRACTICES_URI}, robot://body "
                f"and robot://devices where it shows them); reading one is free.")
READ_ORDER = (TASK_URI, PRACTICES_URI, "robot://body", "robot://devices")
LAYOUT = (
    "Each tool returns one text block holding the episode server's JSON answer (result, budget, frames), then one "
    "image per entry of its frames list, in that order (a short text stands in for a frame that is not attached, "
    "and precedes one that was scaled down). A result holding refused is an answer, not an error: its why and hint "
    "say what stopped the call and what to do next; read them, then change the plan. claim ends the episode with "
    "your own statement of what you achieved; nothing runs after it."
)


def _unix_socket():
    return socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)


def ask_socket(sock_path, request, timeout_s, open_socket=_unix_socket):
    """(response dict, None) when the episode server answers, else (None, why). As rcpx.cli.send, except that a
    failed write shutdown is ignored and the answer is still read (see the module docstring)."""
    s = open_socket()
    buf = b""
    try:
        s.settimeout(timeout_s)
        s.connect(sock_path)
        s.sendall((json.dumps(request) + "\n").encode("utf-8"))
        try:
            s.shutdown(socket.SHUT_WR)
        except OSError:  # the server answered and closed first; its answer is in the receive buffer
            pass
        while b"\n" not in buf:
            chunk = s.recv(65536)
            if not chunk:
                break
            buf += chunk
    except OSError as exc:
        return None, f"{type(exc).__name__}: {exc}"
    finally:
        s.close()
    line = buf.split(b"\n", 1)[0]
    if not line.strip():
        return None, "the server closed the connection without an answer"
    try:
        response = json.loads(line.decode("utf-8", "replace"))
    except ValueError as exc:
        return None, f"the server's answer is not JSON ({exc})"
    if not isinstance(response, dict):
        return None, "the server's answer is not a JSON object"
    return response, None


def encode_frame(path, limit=MAX_IMAGE_B64, quality=JPEG_QUALITY):
    """(base64 JPEG text, sent (w, h), source (w, h)) for an image file, downscaled until the text fits limit.
    Raises on a file pillow cannot read and on an image that cannot fit even at MIN_SIDE_PX."""
    from PIL import Image
    with Image.open(path) as im:
        im.load()
        source = im.size
        img = im.convert("RGB")
    while True:
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=quality)
        data = base64.b64encode(buf.getvalue()).decode("ascii")
        if len(data) <= limit:
            return data, img.size, source
        w, h = img.size
        if min(w, h) <= MIN_SIDE_PX:
            raise ValueError(f"does not fit in {limit} base64 characters even at {w}x{h}")
        k = min(0.9, math.sqrt(limit / len(data)) * 0.95)  # each side 10 percent smaller or more, to MIN_SIDE_PX
        img = img.resize((max(MIN_SIDE_PX, int(w * k)), max(MIN_SIDE_PX, int(h * k))), Image.Resampling.LANCZOS)


def inside(path, root):
    """True when path (symlinks resolved) lies inside the directory root."""
    real, base = os.path.realpath(path), os.path.realpath(root)
    return real == base or real.startswith(base.rstrip(os.sep) + os.sep)


def describe_tool(entry):
    """The tool's one-line description followed by its worked example."""
    text = str(entry.get("description", "")).strip()
    example = entry.get("example") or {}
    if "args" in example:
        means = str(example.get("means", "")).strip()
        text += f" Example arguments {json.dumps(example['args'])}" + (f": {means}." if means else ".")
    return text


def tools_from_listing(listing, plain=False):
    """MCP tools for the list op's result: the surface's tools in order, then claim. plain: each input schema in its
    plain form (bounds said in the descriptions; module docstring)."""
    shape = plain_schema if plain else (lambda schema: schema)
    tools = []
    for entry in listing.get("tools", []):
        if entry.get("name") == CLAIM:
            log.warning("the surface lists a tool named claim; the claim op takes that name")
            continue
        tools.append(types.Tool(name=entry["name"], description=describe_tool(entry),
                                input_schema=shape(entry.get("args") or {"type": "object"})))
    claim = listing.get("claim") or {}
    tools.append(types.Tool(
        name=CLAIM,
        description=describe_tool(claim) if claim.get("description") else
        "End the episode with your own statement of what you achieved; nothing else runs after it.",
        input_schema=shape(claim.get("args") or {"type": "object"})))
    return tools


def plain_from_env(environ=None):
    """True when RCPX_MCP_PLAIN_SCHEMA asks for plain schemas (1, true or yes, any case)."""
    return str((environ if environ is not None else os.environ).get(PLAIN_ENV, "")).strip().lower() in (
        "1", "true", "yes")


def read_first(listing):
    """The line naming the resources (no ordered procedure, no step first): those of READ_ORDER
    the list answer shows, in that order. With no list answer yet, a line conditional on the listing; with none of
    them listed, nothing."""
    if listing is None:
        return READ_UNKNOWN
    uris = [r.get("uri") for r in listing.get("resources", []) if isinstance(r, dict)]
    names = [uri for uri in READ_ORDER if uri in uris]
    if not names:
        return ""
    listed = names[0] if len(names) == 1 else ", ".join(names[:-1]) + " and " + names[-1]
    return f"The resources: {listed}; reading one is free."


def instructions_for(listing, practices):
    """The initialize instructions: what to read first, the result layout, then what the episode server sent
    (conventions, budget, practices) when it answered at startup."""
    text = " ".join(part for part in (INTRO, read_first(listing), LAYOUT) if part)
    if listing is not None:
        if listing.get("about"):
            text += "\n\nConventions: " + str(listing["about"])
        total = listing.get("budget_total") or {}
        stated = [f"{total[key]} {unit}" for key, unit in (("calls", "calls"), ("sim_seconds", "simulated seconds"))
                  if total.get(key) is not None]
        text += "\nBudget: " + (" and ".join(stated) + "." if stated else "none is stated.")
    if isinstance(practices, str) and practices.strip():
        text += f"\n\nPractices ({PRACTICES_URI}):\n" + practices.strip()
    return text


class Bridge:
    """One episode's socket, seen from MCP. Requests go to the socket one at a time."""

    def __init__(self, ep_dir, timeout_s=DEFAULT_TIMEOUT_S, plain_schema=False):
        self.ep_dir = os.path.abspath(ep_dir)
        self.sock_path = os.path.join(self.ep_dir, SOCK_NAME)
        self.timeout_s = float(timeout_s)
        self.plain = bool(plain_schema)  # tools/list serves plain schemas
        self.listing = None             # the list op's result once the episode server has answered it
        self.pending_practices = None   # practices sent with a list answer after startup, for the next call or read
        self._lock = None
        self.open_socket = _unix_socket  # a test hands in a socket whose shutdown fails

    # -- the socket ------------------------------------------------------------------------------------------

    def ask_now(self, request, timeout_s=None):
        """(response, None) or (None, why), blocking."""
        return ask_socket(self.sock_path, request, self.timeout_s if timeout_s is None else timeout_s,
                          self.open_socket)

    async def ask(self, request):
        if self._lock is None:
            self._lock = anyio.Lock()
        async with self._lock:
            return await anyio.to_thread.run_sync(self.ask_now, request)

    def _server_process(self):
        """True while the episode server's process (the pid in EPDIR/episode.json) runs, False once it has
        exited, None when no pid is on record."""
        try:
            with open(os.path.join(self.ep_dir, "episode.json"), encoding="utf-8") as f:
                pid = int(json.load(f)["pid"])
        except (OSError, ValueError, KeyError, TypeError):
            return None
        try:
            if os.waitpid(pid, os.WNOHANG)[0] == pid:
                return False         # it was started by this process, and has exited
        except OSError:              # not started by this process: look at it from outside
            pass
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return False
        except OSError:              # it runs, under another user
            return True
        return True

    def start(self):
        """Ask for the list before serving; returns the practices sent with that first answer, if any. The wait
        lasts until the episode server answers its tool list or its process exits: a server that is still
        starting is asked again, however long its start takes. With no pid on record it is asked once."""
        while True:
            response, why = self.ask_now({"op": "list", "full": True})
            if response is not None or not self._server_process():
                break
            time.sleep(PROCESS_POLL_S)
        try:
            return self._take_listing(response, why)
        except MCPError as exc:
            log.warning("no tool list at startup: %s; tools/list will ask again", exc.message)
            return None

    def _take_listing(self, response, why):
        if response is None or response.get("episode") == "ended":
            gone = self.gone(why, response)
            raise MCPError(types.INTERNAL_ERROR, gone["hint"], data=gone)
        result = response.get("result")
        if not isinstance(result, dict) or not isinstance(result.get("tools"), list):
            raise MCPError(types.INTERNAL_ERROR, "the episode server's list answer has no tools",
                           data={"response": response})
        self.listing = result
        return response.get("practices")

    def _end_reason(self):
        """The reason in EPDIR/episode_end.json, or None when the episode has not ended there."""
        path = os.path.join(self.ep_dir, END_NAME)
        if not os.path.exists(path):
            return None
        try:
            with open(path, encoding="utf-8") as f:
                return str(json.load(f).get("reason", "unknown"))
        except (OSError, ValueError, AttributeError):
            return "unknown"

    def gone(self, why, response=None):
        """What to tell the client when the episode will not run anything: it has ended, or nobody answers."""
        reason = self._end_reason()
        if reason is not None or (response is not None and response.get("episode") == "ended"):
            reason = reason or "unknown"
            return {"episode": "ended", "reason": reason,
                    "hint": f"this episode has ended ({reason}); nothing more runs in it. Start a new episode "
                            "to act again."}
        if os.path.exists(self.sock_path):
            hint = (f"the episode server on {self.sock_path} did not answer ({why}). If it is busy, call again; "
                    "otherwise restart the episode.")
        else:
            hint = (f"no episode server answers for {self.ep_dir} ({why}): the episode is not running. Start it, "
                    "then call again.")
        return {"episode": "not running", "why": str(why), "hint": hint}

    # -- MCP requests ----------------------------------------------------------------------------------------

    async def get_listing(self):
        if self.listing is None:
            response, why = await self.ask({"op": "list", "full": True})
            practices = self._take_listing(response, why)
            if practices is not None:
                self.pending_practices = practices
        return self.listing

    async def list_tools(self):
        return types.ListToolsResult(tools=tools_from_listing(await self.get_listing(), plain=self.plain))

    async def call_tool(self, name, arguments):
        args = {} if arguments is None else arguments
        request = {"op": "claim", "args": args} if name == CLAIM else {"op": "call", "name": name, "args": args}
        response, why = await self.ask(request)
        if response is None or response.get("episode") == "ended":
            gone = self.gone(why, response)
            return types.CallToolResult(content=[types.TextContent(text=json.dumps(gone, indent=2))], is_error=True)
        if self.pending_practices is not None and "practices" not in response:
            response = {**response, "practices": self.pending_practices}
        self.pending_practices = None
        frames = response.get("frames")
        frames = [p for p in frames if isinstance(p, str)] if isinstance(frames, list) else []
        blocks = await anyio.to_thread.run_sync(self.frame_blocks, frames)
        return types.CallToolResult(content=[types.TextContent(text=json.dumps(response, indent=2)), *blocks],
                                    is_error=False)

    def frame_blocks(self, frames):
        """Per frame path: an image block, a scale note then the image when it was downscaled, or a text block
        saying why it is not attached."""
        blocks = []
        for i, path in enumerate(frames):
            full = path if os.path.isabs(path) else os.path.join(self.ep_dir, path)
            if not inside(full, self.ep_dir):
                blocks.append(types.TextContent(text=f"frames[{i}] {path}: not attached, it is outside the "
                                                     "episode dir"))
                continue
            try:
                data, (w, h), (w0, h0) = encode_frame(full)
            except Exception as exc:  # a missing or unreadable frame is reported in place, never raised
                blocks.append(types.TextContent(text=f"frames[{i}] {path}: not attached ({type(exc).__name__}: "
                                                     f"{exc})"))
                continue
            if (w, h) != (w0, h0):
                blocks.append(types.TextContent(
                    text=f"frames[{i}] {path}: sent at {w}x{h}, scaled from {w0}x{h0}; multiply a pixel read off "
                         f"this image by {w0 / w:.4f} (u) and {h0 / h:.4f} (v) before mark"))
            blocks.append(types.ImageContent(data=data, mime_type="image/jpeg"))
        return blocks

    async def list_resources(self):
        listing = await self.get_listing()
        return types.ListResourcesResult(resources=[
            types.Resource(uri=r["uri"], name=_resource_name(r["uri"]), description=r.get("description") or None)
            for r in listing.get("resources", []) if "{" not in r.get("uri", "{")])

    async def list_resource_templates(self):
        listing = await self.get_listing()
        return types.ListResourceTemplatesResult(resource_templates=[
            types.ResourceTemplate(uri_template=r["uri"], name=_resource_name(r["uri"]),
                                   description=r.get("description") or None)
            for r in listing.get("resources", []) if "{" in r.get("uri", "")])

    async def read_resource(self, uri):
        response, why = await self.ask({"op": "read", "name": uri})
        if response is None or response.get("episode") == "ended":
            gone = self.gone(why, response)
            raise MCPError(types.INTERNAL_ERROR, gone["hint"], data=gone)
        result = response.get("result")
        if not isinstance(result, dict):
            raise MCPError(types.INTERNAL_ERROR, "the episode server's read answer has no result",
                           data={"response": response})
        refused = result.get("refused")
        if isinstance(refused, dict):
            check = refused.get("check")
            code = (RESOURCE_NOT_FOUND if check == "unknown_resource" else
                    types.INVALID_PARAMS if check in ("args", "request") else types.INTERNAL_ERROR)
            raise MCPError(code, f"{refused.get('why')}; {refused.get('hint')}", data=refused)
        contents = [_contents(str(result.get("uri", uri)), result.get("content"))]
        practices = response.get("practices", self.pending_practices)
        self.pending_practices = None
        if isinstance(practices, str) and uri != PRACTICES_URI:
            contents.append(_contents(PRACTICES_URI, practices))
        return types.ReadResourceResult(contents=contents)


def _resource_name(uri):
    return uri.split("://", 1)[-1] or uri


def _contents(uri, content):
    if isinstance(content, str):
        return types.TextResourceContents(uri=uri, mime_type="text/plain", text=content)
    return types.TextResourceContents(uri=uri, mime_type="application/json", text=json.dumps(content, indent=2))


def build_server(bridge, instructions):
    """The low-level MCP server over one Bridge."""
    async def on_list_tools(ctx, params):
        return await bridge.list_tools()

    async def on_call_tool(ctx, params):
        return await bridge.call_tool(params.name, params.arguments)

    async def on_list_resources(ctx, params):
        return await bridge.list_resources()

    async def on_list_resource_templates(ctx, params):
        return await bridge.list_resource_templates()

    async def on_read_resource(ctx, params):
        return await bridge.read_resource(params.uri)

    return Server(SERVER_NAME, version=SERVER_VERSION, title="Robot Context Protocol episode",
                  instructions=instructions, on_list_tools=on_list_tools, on_call_tool=on_call_tool,
                  on_list_resources=on_list_resources, on_list_resource_templates=on_list_resource_templates,
                  on_read_resource=on_read_resource)


async def serve_stdio(server):
    async with stdio_server() as (read_stream, write_stream):
        await server.run(read_stream, write_stream, server.create_initialization_options())


def build_parser():
    p = argparse.ArgumentParser(prog="python -m rcpx.mcp_stdio",
                                description="Serve one episode's RCP surface as an MCP server on stdio.")
    p.add_argument("--ep", required=True, help="the episode directory (holds the episode server's socket)")
    p.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                   help="seconds to wait for the episode server's answer to one request")
    p.add_argument("--verbose", action="store_true", help="log at info level (to stderr)")
    p.add_argument("--plain-schema", action="store_true",
                   help=f"serve input schemas without the keywords a strict model endpoint may refuse (bounds said "
                        f"in the descriptions); the environment variable {PLAIN_ENV}=1 does the same")
    return p


def main(argv=None):
    parser = build_parser()
    ns = parser.parse_args(argv)
    if not ns.timeout > 0:
        parser.error("--timeout must be a positive number of seconds")
    logging.basicConfig(stream=sys.stderr, level=logging.INFO if ns.verbose else logging.WARNING,
                        format="rcpx.mcp_stdio: %(levelname)s %(message)s")
    bridge = Bridge(ns.ep, ns.timeout, plain_schema=ns.plain_schema or plain_from_env())
    practices = bridge.start()
    log.info("episode %s: %s", bridge.ep_dir, "tool list in hand" if bridge.listing else "no answer yet")
    anyio.run(serve_stdio, build_server(bridge, instructions_for(bridge.listing, practices)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
