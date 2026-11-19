# SPDX-License-Identifier: AGPL-3.0-only
"""The env host protocol: newline-delimited JSON over a unix socket, shared by the env host (the benchmark's Python,
3.10) and its clients (the rcpx side, 3.12). Standard library only.

Framing. One request per line, one response per line, in order, on a connection that stays open for as many
requests as the client sends:
    request   {"id": ID, "op": NAME, "args": {...}}
    response  {"id": ID, "ok": true, "result": {...}}
              {"id": ID, "ok": false, "error": {"kind": KIND, "message": TEXT}}
ID is any JSON scalar the client picks; it comes back unchanged. Lines are UTF-8, at most MAX_LINE_BYTES. Floats
travel as JSON numbers written by repr, so a float64 crosses the socket bit for bit; a non-finite number is sent as
null. A request that is not one JSON object gets kind "request".

Two sockets per host. The PUBLIC socket (PUBLIC_OPS) is the robot: it moves, reads joints and wrist, renders the
two robot cameras. Its answers never carry object poses, object state, object names, contact names or the success
flag. The SCORING socket (SCORING_OPS, its path under the scoring folder) is ground truth: object poses, the
predicate, the first success step. Neither socket belongs in an episode folder an operator can read.

The step budget rule (StepLedger) is here so both sides count steps the same way: every control step any request
runs is one step against the cell's budget; the predicate is polled after every step; with end_on_success the
first step whose predicate is true is the last one run (the reference evaluations stop there); a request asking
for more steps than are left runs only the steps left, and its answer says how many ran.

serve() is the host's socket loop (both sockets, one request at a time); Client is the caller's side.
"""

import json
import math
import os
import selectors
import signal
import socket
import time

PROTOCOL = "envhost/1"
MAX_LINE_BYTES = 4 * 1024 * 1024
MAX_STEPS_PER_REQUEST = 1000

PUBLIC_OPS = {
    "hello": "protocol, pid, runtime and renderer of this host",
    "reset": "build the cell's env (reference build, settle, step counter reset); answers the robot state",
    "act": "native 7-D actions (OSC_POSE delta, gripper -1 open +1 close): one action n times, or a list",
    "servo_pose": "absolute end-effector target (world m, orientation) through OSC_POSE for n control steps",
    "servo_joints": "joint goal held closed loop through JOINT_POSITION for n control steps",
    "gripper": "open, close or a native gripper value for n control steps, the arm holding its last goal",
    "read": "joints, torques, fingers, end-effector pose and velocity, tared wrist force and torque, step count",
    "render": "agentview or wrist camera to a PNG file in true orientation",
    "cam": "camera intrinsics and pose (OpenCV axes)",
    "robot_xml": "the robot and gripper bodies only, as an MJCF file with its meshes beside it",
    "contact_rates": "the stiffest contact rate each robot body can meet the scene with (the simulator's mixing "
                     "rule), by the robot's own body names; no scene item is named",
    "steps_left": "control steps left in the cell's budget",
    "close": "end the episode: the truth log is closed and the host exits",
}
SCORING_OPS = {
    "hello": "protocol and pid",
    "status": "the cell, its language sentence, steps, first success step, halted, state hash, truth log path",
    "truth": "object and fixture poses and world boxes, region sites, non-robot joints and joint axes, the goal "
             "predicates and their values",
    "contacts": "every contact touching the robot or gripper, with geom and body names and normal force",
    "render": "any model camera to a PNG (audit only)",
    "things": "the simulator's own object model: objects, fixtures, regions, the body tree, geoms, joints, the goal",
    "segment": "what a public camera's colour picture shows at this step, per pixel (object type, id), to a file",
    "ray": "for pixels of a public camera: the first surface the picture draws, by a ray cast (geom, body, depth)",
    "shapes": "the geoms of named bodies as they stand now (type, size, world pose)",
    "kin": "a kinematic copy of the scene with the arm at given joints: what is inside what, what lies between "
           "the pads, how near points are to named bodies (the census; no step, the episode untouched)",
    "peaks": "per control step since the reset: the largest net contact force per pair of bodies, the whole push "
             "per body, where the free things and the hand stood",
}
ERROR_KINDS = ("request", "unknown_op", "bad_args", "no_episode", "runtime", "withheld", "internal")


class EnvHostError(Exception):
    """An error answer from the host (kind is one of ERROR_KINDS) or a transport failure (kind "transport")."""

    def __init__(self, kind, message):
        super().__init__(f"{kind}: {message}")
        self.kind = kind
        self.message = message


def plain(obj):
    """A JSON-safe copy: tuples to lists, anything with tolist() (numpy arrays) or item() (numpy scalars) to
    Python values, non-finite floats to None. Keys become strings."""
    if obj is None or isinstance(obj, (bool, str)):
        return obj
    if isinstance(obj, int):
        return int(obj)
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {str(k): plain(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [plain(v) for v in obj]
    if hasattr(obj, "tolist"):
        return plain(obj.tolist())
    if hasattr(obj, "item"):
        return plain(obj.item())
    return str(obj)


def encode(obj):
    """One protocol line (bytes, newline included)."""
    return (json.dumps(plain(obj), separators=(",", ":"), allow_nan=False) + "\n").encode("utf-8")


def decode(line):
    """The object on one line; ValueError when it is not one JSON object."""
    if isinstance(line, (bytes, bytearray)):
        line = line.decode("utf-8")
    obj = json.loads(line)
    if not isinstance(obj, dict):
        raise ValueError("a protocol line must hold one JSON object")
    return obj


def ok(req_id, result):
    return {"id": req_id, "ok": True, "result": result}


def err(req_id, kind, message):
    if kind not in ERROR_KINDS:
        kind = "internal"
    return {"id": req_id, "ok": False, "error": {"kind": kind, "message": str(message)}}


class Client:
    """A persistent connection to one host socket. call(op, **args) returns the result dict or raises
    EnvHostError (kind from the host, or "transport" when the socket fails)."""

    def __init__(self, path, timeout_s=600.0):
        self.path = path
        self._next = 0
        self._buf = b""
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._sock.settimeout(timeout_s)
        try:
            self._sock.connect(path)
        except OSError as exc:
            self._sock.close()
            raise EnvHostError("transport", f"cannot connect to {path}: {exc}") from exc

    def call(self, op, **args):
        self._next += 1
        req_id = self._next
        try:
            self._sock.sendall(encode({"id": req_id, "op": op, "args": args}))
            while b"\n" not in self._buf:
                chunk = self._sock.recv(65536)
                if not chunk:
                    raise EnvHostError("transport", "the host closed the connection")
                self._buf += chunk
                if len(self._buf) > MAX_LINE_BYTES:
                    raise EnvHostError("transport", "the answer is longer than the line limit")
        except OSError as exc:
            raise EnvHostError("transport", f"{type(exc).__name__}: {exc}") from exc
        line, self._buf = self._buf.split(b"\n", 1)
        resp = decode(line)
        if resp.get("id") != req_id:
            raise EnvHostError("transport", f"answer id {resp.get('id')!r} for request {req_id}")
        if resp.get("ok"):
            return resp.get("result")
        e = resp.get("error") or {}
        raise EnvHostError(e.get("kind", "internal"), e.get("message", ""))

    def close(self):
        try:
            self._sock.close()
        except OSError:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class StepLedger:
    """The step budget rule (module docstring). A pure counter: no clock, no simulator."""

    def __init__(self, budget, end_on_success=True):
        if int(budget) < 0:
            raise ValueError("budget must be 0 or more")
        self.budget = int(budget)
        self.end_on_success = bool(end_on_success)
        self.step = 0
        self.first_success = None
        self.halted = False

    @property
    def steps_left(self):
        return 0 if self.halted else self.budget - self.step

    def allowed(self, n):
        """How many of n requested steps may run now."""
        n = int(n)
        if n < 0:
            raise ValueError("n must be 0 or more")
        return min(n, self.steps_left)

    def record(self, success):
        """Count one step just run whose predicate read success; False when no further step may run."""
        if self.steps_left <= 0:
            raise RuntimeError("a step ran past the budget")
        self.step += 1
        if success and self.first_success is None:
            self.first_success = self.step
            if self.end_on_success:
                self.halted = True
        return self.steps_left > 0


# Key fragments a public answer must never contain (a test walks every public answer for them; the host also
# refuses to send any answer that names a scene item, see envhost.Host._guard).
FORBIDDEN_KEY_PARTS = ("object", "obj_", "success", "goal", "predicate", "truth", "contact", "fixture", "region",
                       "bddl", "task", "done", "reward", "site", "body", "geom", "language")


def walk_keys(obj, prefix=""):
    """Every key path in a nested answer, as 'a.b.c' strings."""
    out = []
    if isinstance(obj, dict):
        for k, v in obj.items():
            path = f"{prefix}.{k}" if prefix else str(k)
            out.append(path)
            out.extend(walk_keys(v, path))
    elif isinstance(obj, list):
        for v in obj:
            out.extend(walk_keys(v, prefix))
    return out


def forbidden_keys(obj):
    """Key paths of obj whose last part holds a FORBIDDEN_KEY_PARTS fragment."""
    bad = []
    for path in walk_keys(obj):
        last = path.rsplit(".", 1)[-1].lower()
        if any(part in last for part in FORBIDDEN_KEY_PARTS):
            bad.append(path)
    return bad


def names_in(obj, names):
    """The names (case-insensitive) that appear in any key or string value of obj."""
    text = json.dumps(plain(obj)).lower()
    return sorted({n for n in names if n and n.lower() in text})


def _listen(path):
    if len(os.fsencode(path)) > 107:
        raise ValueError(f"socket path {path} is longer than 107 bytes")
    if os.path.exists(path):
        os.unlink(path)
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.bind(path)
    os.chmod(path, 0o600)
    s.listen(8)
    s.setblocking(False)
    return s


def serve(handle, public_path, scoring_path, ready_path, ready_info, idle_s, should_stop):
    """The host's socket loop: both sockets, any number of connections, one request at a time in arrival order.
    handle(line, scoring) -> response dict; should_stop() is read after every request. ready_path gets
    {"pid", "sock", "truth_sock", "protocol", **ready_info} once both sockets listen. Returns why it stopped:
    "close" (should_stop), "sigterm" or "idle" (no request for idle_s seconds)."""
    sel = selectors.DefaultSelector()
    listeners = {_listen(public_path): False, _listen(scoring_path): True}
    for s, scoring in listeners.items():
        sel.register(s, selectors.EVENT_READ, ("listen", scoring))
    stop = {"sig": None}
    signal.signal(signal.SIGTERM, lambda signum, frame: stop.__setitem__("sig", signum))
    with open(ready_path + ".tmp", "w") as f:
        json.dump(dict(ready_info, pid=os.getpid(), sock=public_path, truth_sock=scoring_path, protocol=PROTOCOL), f)
    os.replace(ready_path + ".tmp", ready_path)
    last = time.monotonic()
    try:
        while not should_stop() and stop["sig"] is None and time.monotonic() - last < idle_s:
            for key, _ in sel.select(timeout=0.5):
                if key.data[0] == "listen":
                    conn, _ = key.fileobj.accept()
                    conn.settimeout(60.0)
                    sel.register(conn, selectors.EVENT_READ, ["conn", key.data[1], b""])
                    continue
                conn = key.fileobj
                try:
                    chunk = conn.recv(65536)
                except OSError:
                    chunk = b""
                if not chunk:
                    sel.unregister(conn)
                    conn.close()
                    continue
                key.data[2] += chunk
                while b"\n" in key.data[2]:
                    line, key.data[2] = key.data[2].split(b"\n", 1)
                    if line.strip():
                        last = time.monotonic()
                        try:
                            conn.sendall(encode(handle(line, key.data[1])))
                        except OSError:
                            break
                if len(key.data[2]) > MAX_LINE_BYTES:
                    key.data[2] = b""
                    try:
                        conn.sendall(encode(err(None, "request", "the request is longer than the line limit")))
                    except OSError:
                        pass
                if should_stop():
                    break
    finally:
        for s in listeners:
            s.close()
        for p in (public_path, scoring_path):
            if os.path.exists(p):
                os.unlink(p)
    return "close" if should_stop() else "sigterm" if stop["sig"] is not None else "idle"
