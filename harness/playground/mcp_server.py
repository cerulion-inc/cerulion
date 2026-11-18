# SPDX-License-Identifier: AGPL-3.0-only
"""The MCP handlers: one connection, one simulated robot episode; its tools are the episode server's own."""
import asyncio
import contextlib
import json
import os
import time

from .common import ENDED, NEW_CONNECTION, source_link, source_offer
from .pool import Refused

SERVER_NAME = "cerulion-rcp-sim"
SERVER_VERSION = "0.1"


def build_mcp(settings, pool, viewer_url=None):
    """The MCP server. viewer_url: where the viewer pages are served (HTTP), or None (stdio: no pages; the
    instructions name the folder the episode is kept in).

    A client that keeps an MCP session (HTTP) gets one episode per session: a second session that names the
    same start state and `episode` label while that episode runs joins it, and once it has ended the session's
    calls are answered "ended". A client with no session (stdio, or the per-request form of HTTP) gets one
    episode per start state and label; once it has ended, its next connection (initialize, server/discover or
    tools/list) starts a new one, never the ended one."""
    import mcp.types as types
    from mcp.server.lowlevel import Server
    from mcp.shared.exceptions import MCPError
    from rcpx import mcp_stdio

    intro = (f"{settings.name} simulated robot: this connection is one simulated robot episode. The tools, the "
             "resources and the practices below are that episode's own.")
    offer = source_offer(settings)

    def query(ctx):
        """The URL's query of this request (HTTP), or {} (stdio)."""
        req = getattr(ctx, "request", None)
        return dict(req.query_params) if req is not None else {}

    def open_for(ctx):
        conn = ctx.session._connection
        ep = conn.state.get("episode")
        if ep is not None:
            return ep
        q = query(ctx)
        state = q.get("state") or settings.default_state
        key = f"{state}:{q.get('episode', '1')}"
        ep = pool.by_key.get(key)
        if ep is None or (ep.status in ENDED and ctx.method in NEW_CONNECTION):
            try:
                ep = pool.open(state, key)
            except Refused as exc:
                raise MCPError(types.INVALID_REQUEST, str(exc)) from None
        if conn.session_id:
            conn.state["episode"] = ep
            ep.sessions.add(conn.session_id)
            conn.exit_stack.push_async_callback(ep.session_closed, conn.session_id)
        return ep

    async def ready(ctx):
        ep = open_for(ctx)
        await ep.ready.wait()
        if ep.status == "failed":
            raise MCPError(types.INTERNAL_ERROR, ep.ended_message())
        return ep

    def instructions(ep):
        if viewer_url:
            watch = (f" Watch it: {viewer_url}/view/{ep.eid} (anyone you give this link to can watch; it is "
                     "listed nowhere).")
        else:
            watch = f" Its transcript and pictures are kept in {pool.store.dir(ep.eid)}."
        base = mcp_stdio.instructions_for(ep.bridge.listing if ep.bridge else None, ep.practices)
        return f"{intro}{watch}{ep.record_note()}\n\n{base}\n\n{offer}"

    async def middleware(ctx, call_next):
        if ctx.method in ("initialize", "server/discover") and ctx.request_id is not None:
            ep = open_for(ctx)            # refuses here: an unknown start state, a full pool, a server stopping
            result = await call_next(ctx)
            await ep.ready.wait()
            if ep.status == "failed":
                raise MCPError(types.INTERNAL_ERROR, ep.ended_message())
            if isinstance(result, dict):
                result = {**result, "instructions": instructions(ep)}
            return result
        return await call_next(ctx)

    def ended_result(ep):
        return types.CallToolResult(content=[types.TextContent(text=json.dumps(
            {"episode": "ended", "reason": ep.reason, "message": ep.ended_message()}, indent=2))], is_error=True)

    async def on_list_tools(ctx, params):
        ep = await ready(ctx)
        return await ep.bridge.list_tools()

    async def on_call_tool(ctx, params):
        ep = await ready(ctx)
        if ep.status in ENDED:
            return ended_result(ep)
        name, args = params.name, params.arguments or {}
        dropped = name == mcp_stdio.CLAIM and "cannot" in args and args["cannot"] is None
        if dropped:
            # a model that fills every field of the schema sends "cannot": null for "nothing to state": null is
            # taken as absent, so the claim reaches the episode as the same claim without the field
            args = {k: v for k, v in args.items() if k != "cannot"}
        ep.calls += 1
        ep.call_begins()
        ep.event("call", name=name, args=args, **({"null_taken_as_absent": ["cannot"]} if dropped else {}))
        task, t0 = asyncio.ensure_future(ep.bridge.call_tool(name, args)), time.monotonic()
        try:
            while True:
                done, _ = await asyncio.wait({task}, timeout=settings.progress_every_s)
                if done:
                    break
                waited = time.monotonic() - t0
                with contextlib.suppress(Exception):
                    await ctx.session.report_progress(round(waited, 1), None, f"{name} is running: {waited:.0f} s")
            try:
                result = task.result()
            except Exception:
                if ep.status in ENDED:        # the simulator was stopped under the call
                    return ended_result(ep)
                raise
        finally:
            ep.call_ends()
        if ep.status in ENDED:            # the episode ended while the call ran
            return ended_result(ep)
        text = next((b.text for b in result.content if isinstance(b, types.TextContent)), "")
        try:
            doc = json.loads(text)
        except ValueError:
            doc = {}
        frames = [os.path.basename(p) for p in doc.get("frames", []) if isinstance(p, str)] \
            if isinstance(doc, dict) and isinstance(doc.get("frames"), list) else []
        ep.keep_frames(frames)
        ep.event("claim" if name == mcp_stdio.CLAIM else "result", name=name, text=text, frames=frames,
                 seconds=round(ep.last_m - t0, 2), images=sum(isinstance(b, types.ImageContent)
                                                              for b in result.content),
                 error=bool(result.is_error))
        reason = ep.end_reason()
        if reason is not None:
            # the claim (or a stop the robot itself made) ended the episode: score and stop the simulator before
            # this answer goes out, so the next request already sees the ended state
            await ep.finish(reason)
            if result.is_error:
                return ended_result(ep)
        return result

    async def on_list_resources(ctx, params):
        return await (await ready(ctx)).bridge.list_resources()

    async def on_list_resource_templates(ctx, params):
        return await (await ready(ctx)).bridge.list_resource_templates()

    async def on_read_resource(ctx, params):
        ep = await ready(ctx)
        if ep.status in ENDED:
            raise MCPError(types.INTERNAL_ERROR, ep.ended_message())
        ep.event("read", name=str(params.uri))
        ep.call_begins()
        try:
            return await ep.bridge.read_resource(str(params.uri))
        finally:
            ep.call_ends()

    server = Server(SERVER_NAME, version=SERVER_VERSION, title=f"{settings.name} simulated robot",
                    description=offer, website_url=source_link(settings), instructions=f"{intro} {offer}",
                    on_list_tools=on_list_tools, on_call_tool=on_call_tool, on_list_resources=on_list_resources,
                    on_list_resource_templates=on_list_resource_templates, on_read_resource=on_read_resource)
    server.middleware.append(middleware)
    return server
