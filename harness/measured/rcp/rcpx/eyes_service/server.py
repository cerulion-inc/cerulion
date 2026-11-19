# SPDX-License-Identifier: AGPL-3.0-only
"""The robot's vision service: a small HTTP server on the robot's own computer (127.0.0.1 by default).

    python -m rcpx.eyes_service.server --equipment FILE.json [--host 127.0.0.1] [--port 8791] [--pid-file F]
                                       [--frames-root DIR]

    GET  /health    {"ok": true, "level": "words" | "segments" | "absent", "up_s": seconds, "phrases": true,
                     "image_path": true when a frames root was given, ...}
    GET  /describe  the declared equipment: per role (propose, find, outline) the interface kind, the weights and
                    the licence as the equipment file states them, half (16 bit floats) or not, memory_mb (what
                    the process holds at this call); for find the threshold and the floor; the level; the ranking
                    rule's numbers; a sha256 of the declared entries; limits; load seconds
    POST /propose   {"image_b64" | "image_path", "max": 8, "exclude": [[[u, v], ...], ...]}
    POST /find      {"image_b64" | "image_path", "phrase": "WORDS FOR ONE PART", "max": 8, "exclude": [...]}
                    or "phrases": ["WORDS", ...] (1 to 6 texts in one request; "max" then counts per phrase)

Answers are JSON (rcpx/eyes_service/protocol.py has the candidate shape and the answer words). "exclude" is the
robot's own body in the picture as polygons, from its body model: a candidate whose point lies on the robot itself
is dropped and counted. The service reads the picture, the words and those polygons, nothing else: no scene, no
device list, no truth. It opens no file but its weights and a request's picture: image_path is honoured only
under --frames-root (the folder the robot writes its camera frames to); a path that resolves outside it is
answered 400 and never opened, and with no frames root every image_path is 400 (send image_b64). With no
equipment file the level is "absent": propose answers from colour regions, find answers not_equipped.

A request the service cannot read is answered 400 {"error": text}; a body that does not arrive is answered 400
after BODY_WAIT_S, never left hanging. One request is served at a time (the model is not shared between threads):
two requests at once are answered one after the other, and an answer that takes long is still sent ("seconds" is
the model's time, "queued_s" the wait for an earlier request). The start and stop scripts beside this file keep a
pid file and stop by that pid only. Standard library for the server; a backend imports what it needs.
"""
import argparse
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

if __package__ in (None, ""):   # run as a file: make "rcpx" importable
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    __package__ = "rcpx.eyes_service"

from . import protocol as P  # noqa: E402
from .backends import Eyes  # noqa: E402

MAX_BODY = 12 * 1024 * 1024
BODY_WAIT_S = 10.0   # how long a request's bytes may take to arrive before it is refused
SLOW_S = 1.5         # an answer slower than this is counted in /health (it is still sent)
PATHS = ["/health", "/describe", "/propose", "/find"]


def make_handler(eyes, started, frames_root=None):
    lock = threading.Lock()
    counts = {"propose": 0, "find": 0, "bad": 0}
    slow = {"n": 0}

    class Handler(BaseHTTPRequestHandler):
        server_version = "eyes/2"
        timeout = BODY_WAIT_S   # the socket's wait for bytes: a stalled sender is refused, not waited for

        def log_message(self, fmt, *args):   # one line per request on stderr, no bodies
            sys.stderr.write("%s %s\n" % (time.strftime("%H:%M:%S"), fmt % args))

        def _send(self, code, obj):
            body = json.dumps(obj).encode("utf-8")
            try:
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except OSError:   # the caller has gone (its own timeout): nothing to answer, keep serving
                self.close_connection = True

        def _path(self):
            return self.path.split("?", 1)[0].rstrip("/") or "/"

        def do_GET(self):
            path = self._path()
            if path == "/health":
                up = round(time.time() - started, 1)
                self._send(200, {"ok": True, "level": eyes.level, "phrases": True, "up_s": up, "uptime_s": up,
                                 "image_path": bool(frames_root), "served": dict(counts),
                                 "slow_answers": slow["n"], "pid": os.getpid()})
            elif path == "/describe":
                self._send(200, eyes.describe())
            else:
                self._send(404, {"error": "unknown path", "paths": PATHS})

        def _body(self):
            """The request's JSON object, or BadRequest saying what is wrong with it."""
            try:
                n = int(self.headers.get("Content-Length") or 0)
            except ValueError as exc:
                raise P.BadRequest("Content-Length is not a number") from exc
            if n <= 0 or n > MAX_BODY:
                raise P.BadRequest(f"the request body must be from 1 to {MAX_BODY} bytes (Content-Length says {n})")
            try:
                raw = self.rfile.read(n)
            except OSError as exc:   # the bytes did not arrive in time
                self.close_connection = True
                raise P.BadRequest(f"the request body did not arrive within {BODY_WAIT_S:g} s") from exc
            if len(raw) < n:
                self.close_connection = True
                raise P.BadRequest(f"the request body ended after {len(raw)} of {n} bytes")
            try:
                req = json.loads(raw)
            except (ValueError, RecursionError) as exc:
                raise P.BadRequest("the request body is not JSON") from exc
            if not isinstance(req, dict):
                raise P.BadRequest("the request body is a JSON object")
            return req

        def do_POST(self):
            path = self._path()
            if path not in ("/propose", "/find"):
                self._send(404, {"error": "unknown path", "paths": PATHS})
                return
            try:
                req = self._body()
                img = P.read_image(req, frames_root)
                k = P.count_arg(req)
                exclude = P.polygons_arg(req)
                texts, many = P.phrases_arg(req) if path == "/find" else (None, False)
                t = time.time()
                with lock:
                    queued = time.time() - t
                    if path == "/find":
                        out = eyes.find(img, texts if many else texts[0], k, exclude)
                    else:
                        out = eyes.propose(img, k, exclude)
                out["queued_s"] = round(queued, 3)
                counts[path[1:]] += 1
                if time.time() - t > SLOW_S:
                    slow["n"] += 1
                self._send(200, out)
            except P.BadRequest as exc:
                counts["bad"] += 1
                self._send(400, {"error": str(exc)})
            except Exception as exc:   # a backend that failed on this picture: say so, keep serving
                counts["bad"] += 1
                self._send(500, {"error": f"{type(exc).__name__}: {exc}"[:300]})

    return Handler


def serve(equipment, host="127.0.0.1", port=8791, pid_file=None, ready=None, frames_root=None):
    """Load the equipment and serve until the process is stopped. ready(server) is called once it listens.
    frames_root: the one folder a request's image_path may name a file under (None: no image_path is served)."""
    if frames_root is not None and not os.path.isdir(frames_root):
        raise ValueError(f"the frames root is not a folder: {frames_root}")
    eyes = Eyes(equipment).load()
    warm = eyes.warm()   # the model's first pass is its slowest: it is paid here, not by the first request
    srv = ThreadingHTTPServer((host, int(port)), make_handler(eyes, time.time(), frames_root))
    if pid_file:
        with open(pid_file, "w") as f:
            f.write(str(os.getpid()))
    sys.stderr.write(f"eyes: level {eyes.level}, listening on {host}:{srv.server_address[1]}, pid {os.getpid()}, "
                     f"warm-up {warm} s\n")
    if ready is not None:
        ready(srv)
    try:
        srv.serve_forever()
    finally:
        srv.server_close()
        if pid_file and os.path.exists(pid_file):
            os.unlink(pid_file)


def main(argv=None):
    ap = argparse.ArgumentParser(description="the robot's vision service")
    ap.add_argument("--equipment", default=None, help="the installation's equipment file (JSON); none: level absent")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8791)
    ap.add_argument("--pid-file", default=None)
    ap.add_argument("--frames-root", default=None,
                    help="the folder a request's image_path may name a file under; none: image_path is refused")
    a = ap.parse_args(argv)
    equipment = {}
    if a.equipment:
        with open(a.equipment, encoding="utf-8") as f:
            equipment = json.load(f)
    serve(equipment, a.host, a.port, a.pid_file, frames_root=a.frames_root)
    return 0


if __name__ == "__main__":
    sys.exit(main())
