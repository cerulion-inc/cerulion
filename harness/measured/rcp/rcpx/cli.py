# SPDX-License-Identifier: AGPL-3.0-only
"""Command line client for one episode's RCP server (SPEC section 3).

    python -m rcpx.cli --ep EPDIR list                    the compact index: every tool in one line, the resources
    python -m rcpx.cli --ep EPDIR list touch press        those tools' full description, schema and worked example
    python -m rcpx.cli --ep EPDIR list --full             every tool's full entry at once (large)
    python -m rcpx.cli --ep EPDIR read robot://body
    python -m rcpx.cli --ep EPDIR call move_tip '{"dz": -20, "frame": "base"}'
    python -m rcpx.cli --ep EPDIR claim '{"statement": "...", "success": true}'

Prints the response as JSON in a compact layout (rcpx/surface_brief.py dumps: a value stays on one line when it
fits, else one item per line), each frame path once: the response's own "frames" list is left out of the print
when the result names every one of those paths already, and a path the result does not name is printed on its own
line after the JSON. With --json (before the verb) the whole response is printed as one JSON document and nothing
after it (the frame paths are in its "frames" list). A call's answer is brief by default (the server names what it
left out and where the whole result is); call NAME ARGS --full sends "detail": "full" for the whole answer.
call takes --op-id ID: the same call sent again with the same id returns the first answer and runs nothing.
Exit codes: 0 a served response (a refusal is a served response), 2 the episode has ended, 3 no server answers,
64 a malformed command line.

The socket exchange is one JSON line out and one back. A failed write shutdown is not a failure: the server
answers an episode-ending request (claim, an e-stop result) and closes at once, and the shutdown can then fail
with ENOTCONN while the answer already waits in the receive buffer.
"""
import argparse
import json
import os
import socket
import sys

from .server_socket import END_NAME, ENDED, SOCK_NAME
from .surface_brief import dumps

EXIT_SERVED, EXIT_ENDED, EXIT_NO_SERVER, EXIT_USAGE = 0, 2, 3, 64


class _Parser(argparse.ArgumentParser):
    def error(self, message):
        self.print_usage(sys.stderr)
        sys.stderr.write(f"{self.prog}: error: {message}\n")
        raise SystemExit(EXIT_USAGE)


def build_parser():
    p = _Parser(prog="python -m rcpx.cli", description="Talk to one episode's RCP server.")
    p.add_argument("--ep", required=True, help="the episode directory (holds the server socket and journal)")
    # Client transport bound: cuts no motion; must be at least the longest primitive's own time bound.
    p.add_argument("--timeout", type=float, default=900.0,
                   help="seconds to wait for an answer (a long move can take a while)")
    p.add_argument("--json", action="store_true",
                   help="print one JSON document and nothing after it (frame paths are in its frames list)")
    sub = p.add_subparsers(dest="cmd", required=True)
    ls = sub.add_parser("list", help="the tools in one line each, the resources and the budget; with names, "
                                     "those tools' full description, argument schema and worked example")
    ls.add_argument("names", nargs="*", help="tool names (claim is one) to show in full")
    ls.add_argument("--full", action="store_true", help="every tool's full entry at once (large)")
    r = sub.add_parser("read", help="read a resource such as robot://body")
    r.add_argument("uri")
    c = sub.add_parser("call", help="call a tool with JSON arguments")
    c.add_argument("name")
    c.add_argument("args", nargs="?", default="{}", help="a JSON object, in single quotes")
    c.add_argument("--op-id", dest="op_id", default=None,
                   help="an id for this call: sent again with the same id, the first answer comes back and "
                        "nothing runs")
    c.add_argument("--full", action="store_true",
                   help='the whole answer (sends "detail": "full"); by default a long answer is brief')
    k = sub.add_parser("claim", help="end the episode with a statement of what you achieved")
    k.add_argument("args", help='a JSON object such as {"statement": "...", "success": true}')
    return p


def build_request(ns):
    if ns.cmd == "list":
        request = {"op": "list"}
        if ns.names:
            request["name"] = ns.names[0] if len(ns.names) == 1 else list(ns.names)
        elif ns.full:
            request["full"] = True
        return request
    if ns.cmd == "read":
        return {"op": "read", "name": ns.uri}
    request = {"op": ns.cmd}
    if ns.cmd == "call":
        request["name"] = ns.name
        if ns.op_id is not None:
            request["op_id"] = ns.op_id
    try:
        request["args"] = json.loads(ns.args)
    except ValueError:
        request["args_text"] = ns.args  # the server refuses it with a hint, and the journal keeps the attempt
    if ns.cmd == "call" and getattr(ns, "full", False) and isinstance(request.get("args"), dict):
        request["args"] = {**request["args"], "detail": "full"}
    return request


def _unix_socket():
    return socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)


def send(sock_path, request, timeout_s, open_socket=_unix_socket):
    """(response dict, None) when the server answers, else (None, why). A failed write shutdown is ignored and
    the answer is still read (see the module docstring); open_socket lets a test hand in a socket whose
    shutdown fails."""
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


def _named_paths(obj, found):
    if isinstance(obj, dict):
        for v in obj.values():
            _named_paths(v, found)
    elif isinstance(obj, list):
        for v in obj:
            _named_paths(v, found)
    elif isinstance(obj, str):
        found.add(obj)
    return found


def print_response(response, out=sys.stdout, json_only=False):
    """The response in the compact layout, each frame path once (module docstring)."""
    if json_only:
        out.write(dumps(response) + "\n")
        out.flush()
        return
    frames = [p for p in response.get("frames", []) if isinstance(p, str)]
    named = _named_paths({k: v for k, v in response.items() if k != "frames"}, set())
    shown = {k: v for k, v in response.items() if k != "frames"} if frames else response
    out.write(dumps(shown) + "\n")
    for path in frames:
        if path not in named:
            out.write(f"{path}\n")
    out.flush()


def main(argv=None):
    ns = build_parser().parse_args(argv)
    ep = os.path.abspath(ns.ep)
    response, why = send(os.path.join(ep, SOCK_NAME), build_request(ns), ns.timeout)
    if response is None:
        if os.path.exists(os.path.join(ep, END_NAME)):
            print_response(dict(ENDED), json_only=ns.json)
            return EXIT_ENDED
        sys.stderr.write(f"no RCP server answers for episode {ep} ({why})\n")
        return EXIT_NO_SERVER
    print_response(response, json_only=ns.json)
    return EXIT_ENDED if response.get("episode") == "ended" else EXIT_SERVED


if __name__ == "__main__":
    raise SystemExit(main())
