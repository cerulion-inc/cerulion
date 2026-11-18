# SPDX-License-Identifier: AGPL-3.0-only
"""The sim server's command line.

    python harness/playground                     MCP over stdio (the default)
    python harness/playground --http [ADDR]       MCP over streamable HTTP at http://ADDR/mcp (127.0.0.1:8765)
    python harness/playground --check             is everything the server needs installed? (JSON; exit 0 or 1)

--state LABEL is the start state of a connection that names none (over HTTP a URL can name another with
?state=LABEL); --settings FILE replaces settings.json. Exit codes: 0 served or ready, 1 not installed (the
message names what is missing), 2 a usage or settings error.
"""
import argparse
import json
import logging
import sys

from .app import DEFAULT_HTTP, AddressError, parse_addr, serve_http, serve_stdio
from .settings import DEFAULT_SETTINGS, Settings, SettingsError, check_states, missing

NOT_INSTALLED = 1
USAGE = 2


def parser():
    p = argparse.ArgumentParser(prog="python harness/playground",
                                description="The Cerulion sim server: the harness's MCP tools on a simulated "
                                            "robot, one episode per connection.")
    how = p.add_mutually_exclusive_group()
    how.add_argument("--stdio", action="store_true", help="serve MCP over stdin and stdout (the default)")
    how.add_argument("--http", nargs="?", const=DEFAULT_HTTP, default=None, metavar="ADDR",
                     help=f"serve MCP over streamable HTTP at http://ADDR/mcp (default ADDR {DEFAULT_HTTP})")
    how.add_argument("--check", action="store_true",
                     help="print whether everything the server needs is installed, as JSON; exit 0 or 1")
    p.add_argument("--state", default=None, metavar="LABEL",
                   help="the start state of a connection that names none (default: the settings' default_state)")
    p.add_argument("--settings", default=DEFAULT_SETTINGS, metavar="FILE", help="the settings file "
                   "(default: settings.json beside the server)")
    return p


def main(argv=None):
    ns = parser().parse_args(argv)
    logging.basicConfig(stream=sys.stderr, level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    try:
        if ns.http is not None:
            parse_addr(ns.http)
        settings = Settings(ns.settings, state=ns.state)
    except (SettingsError, AddressError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return USAGE
    lacking, problem = missing(settings), None
    if not lacking:
        try:
            check_states(settings)
        except SettingsError as exc:
            problem = str(exc)
    if ns.check:
        print(json.dumps({"ready": not lacking and problem is None,
                          "missing": [{"what": w, "path": p} for w, p in lacking], "problem": problem,
                          "settings": settings.path, "states": list(settings.states),
                          "default_state": settings.default_state}, indent=2))
        return NOT_INSTALLED if lacking else USAGE if problem else 0
    if lacking:
        print("error: the sim server cannot start, this machine lacks "
              + "; ".join(f"{what} ({path})" for what, path in lacking)
              + ". `cerulion rcp --sim` installs the simulator and its scenes on its first run.", file=sys.stderr)
        return NOT_INSTALLED
    if problem:
        print(f"error: {problem}", file=sys.stderr)
        return USAGE
    if settings.snapshot_root not in sys.path:
        sys.path.insert(0, settings.snapshot_root)      # the measured bridge (rcpx.mcp_stdio)
    if ns.http is not None:
        serve_http(settings, ns.http)
    else:
        serve_stdio(settings)
    return 0
