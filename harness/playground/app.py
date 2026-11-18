# SPDX-License-Identifier: AGPL-3.0-only
"""The two transports: MCP over stdio (one client, the process's stdin and stdout) and over streamable HTTP."""
import asyncio
import contextlib
import ipaddress
import logging
import os
import signal
import sys
import threading
import time

from .mcp_server import build_mcp
from .pool import AGAIN_HTTP, AGAIN_STDIO, make_pool
from .web import build_routes

log = logging.getLogger("cerulion.sim")
DEFAULT_HTTP = "127.0.0.1:8765"
# A terminal's Ctrl+C reaches cerulion and this server at once, and cerulion then forwards the signal it got, so
# a stop signal arriving within this many seconds of the first is that same stop, not a second one.
SAME_STOP_S = 1.0
LOOPBACK_NAMES = ("localhost",)


class AddressError(ValueError):
    """An --http address that is not HOST:PORT."""


def parse_addr(text):
    """(host, port) of HOST:PORT, IPV4:PORT or [IPV6]:PORT. Raises AddressError."""
    text = (text or "").strip()
    if text.startswith("["):
        host, sep, rest = text[1:].partition("]")
        if not sep or not rest.startswith(":"):
            raise AddressError(f"{text!r} is not [IPV6]:PORT")
        port = rest[1:]
    else:
        host, sep, port = text.rpartition(":")
        if not sep or ":" in host:
            raise AddressError(f"{text!r} is not HOST:PORT (an IPv6 address goes in brackets: [::1]:8765)")
    if not host or not port.isdigit() or not 0 < int(port) < 65536:
        raise AddressError(f"{text!r} is not HOST:PORT with a port from 1 to 65535")
    return host, int(port)


def is_loopback(host):
    if host in LOOPBACK_NAMES:
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def http_names(host, port):
    """(the public URL, the Host values /mcp answers to) for a server listening on host:port."""
    shown = f"[{host}]" if ":" in host else host
    if is_loopback(host):
        hosts = [f"127.0.0.1:{port}", f"localhost:{port}", f"[::1]:{port}"]
        if f"{shown}:{port}" not in hosts:
            hosts.insert(0, f"{shown}:{port}")
        return f"http://{shown}:{port}", hosts
    return f"http://{shown}:{port}", [f"{shown}:{port}"]


def build_http_app(settings, host, port, simulator=None):
    """The ASGI app of the HTTP transport: /mcp and the pages. simulator: a test hands in its own."""
    from mcp.server.transport_security import TransportSecuritySettings

    public_url, hosts = http_names(host, port)
    settings.public_url = settings.public_url or public_url
    allowed = list(settings.allowed_hosts or hosts)
    origins = [f"{scheme}://{h}" for h in allowed for scheme in ("http", "https")]
    pool = make_pool(settings, simulator, AGAIN_HTTP)
    server = build_mcp(settings, pool, viewer_url=settings.public_url)
    security = TransportSecuritySettings(enable_dns_rebinding_protection=True, allowed_hosts=allowed,
                                         allowed_origins=origins + list(settings.allowed_origins or []))
    # The idle limit ends EPISODES (the episode's own watch, with the reason idle). A SESSION the client never
    # closed is dropped once it has been silent for as long as any episode can last.
    routes = build_routes(settings, pool, f"{settings.public_url}/mcp")
    inner = server.streamable_http_app(host=host, transport_security=security, session_idle_timeout=settings.clock_s,
                                       custom_starlette_routes=routes)
    background = []

    async def app(scope, receive, send):
        if scope["type"] != "lifespan":
            await inner(scope, receive, send)
            return

        async def wrapped_send(message):
            if message["type"] == "lifespan.startup.complete":
                app.loop = asyncio.get_running_loop()
                cleared = await pool.clear_orphans()
                if cleared:
                    log.info("removed %d work folder(s) a stopped server left in %s", cleared, settings.work_root)
                background.append(pool.spawn(pool.sweep_loop()))
            elif message["type"] == "lifespan.shutdown.complete":
                for task in background:
                    task.cancel()
                await pool.close()
            await send(message)

        async def wrapped_receive():
            message = await receive()
            if message["type"] == "lifespan.shutdown":
                await pool.close()       # before the sessions are closed, so the episodes carry the stop's reason
            return message

        await inner(scope, wrapped_receive, wrapped_send)

    def on_signal(sig):
        """A stop signal: end the live episodes now, before the open connections are waited for."""
        loop = getattr(app, "loop", None)
        if loop is not None:
            loop.call_soon_threadsafe(pool.close)

    app.pool, app.on_signal, app.loop = pool, on_signal, None
    return app


def serve_http(settings, addr):
    """Serve MCP at http://ADDR/mcp, with the pages beside it, until a stop signal."""
    import uvicorn

    host, port = parse_addr(addr)
    if not is_loopback(host):
        log.warning("listening on %s, not a loopback address: anyone who can reach it can start simulators "
                    "on this machine and watch every episode", addr)
    app = build_http_app(settings, host, port)

    class Server(uvicorn.Server):
        @contextlib.contextmanager
        def capture_signals(self):
            """uvicorn's own, less its raising the signal again once stopped: a stopped server exits 0, as it
            does over stdio."""
            if threading.current_thread() is not threading.main_thread():
                yield
                return
            original = {sig: signal.signal(sig, self.handle_exit) for sig in (signal.SIGINT, signal.SIGTERM)}
            try:
                yield
            finally:
                for sig, handler in original.items():
                    signal.signal(sig, handler)

        first_stop = None

        def handle_exit(self, sig, frame):
            now = time.monotonic()
            if self.first_stop is not None and now - self.first_stop <= SAME_STOP_S:
                return
            if self.first_stop is None:
                self.first_stop = now
            app.on_signal(sig)
            super().handle_exit(sig, frame)

    log.info("serving MCP at %s/mcp; the start page is %s/", settings.public_url, settings.public_url)
    Server(uvicorn.Config(app, host=host, port=port, log_level="info",
                          timeout_graceful_shutdown=settings.stop_grace_s)).run()


async def serve_stdio_async(settings, simulator=None):
    """Serve one client over this process's stdin and stdout until stdin closes or a stop signal arrives.

    The client closing stdin ends its live episode with the reason session_closed. A stop signal (SIGINT,
    SIGTERM) ends every live episode with the reason server_stopped (scored, its simulator stopped, recorded) and
    then ends the process: the SDK reads stdin in a worker thread that no cancel interrupts, so the process exits
    without waiting for that read. A second stop signal, more than SAME_STOP_S after the first, exits at once,
    with code 1."""
    from mcp.server.stdio import stdio_server

    pool = make_pool(settings, simulator, AGAIN_STDIO)
    server = build_mcp(settings, pool, viewer_url=None)
    loop, signals = asyncio.get_running_loop(), []

    async def stop():
        pool.stop_reason = "server_stopped"
        await pool.close()
        log.info("stopped by a signal; every live episode has ended")
        logging.shutdown()
        sys.stderr.flush()
        os._exit(0)

    def on_signal(sig):
        now = time.monotonic()
        if signals and now - signals[0] <= SAME_STOP_S:
            return
        signals.append(now)
        if len(signals) == 1:
            pool.spawn(stop())
        else:
            os._exit(1)

    for sig in (signal.SIGINT, signal.SIGTERM):
        with contextlib.suppress(NotImplementedError, RuntimeError):
            loop.add_signal_handler(sig, on_signal, sig)
    cleared = await pool.clear_orphans()
    if cleared:
        log.info("removed %d work folder(s) a stopped server left in %s", cleared, settings.work_root)
    sweep = pool.spawn(pool.sweep_loop())
    try:
        async with stdio_server() as (read, write):
            await server.run(read, write, server.create_initialization_options())
        if not signals:
            pool.stop_reason = "session_closed"
    finally:
        sweep.cancel()
        await pool.close()
    return pool


def serve_stdio(settings):
    log.info("serving MCP over stdio; start state %s; episodes are kept in %s", settings.default_state,
             settings.keep_root)
    asyncio.run(serve_stdio_async(settings))
    sys.stderr.flush()
