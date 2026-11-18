# SPDX-License-Identifier: AGPL-3.0-only
"""One RCP session per episode (SPEC section 3): dispatch, argument checks, budget, journal. The unix socket loop
is rcpx/server_socket.py (serve and the episode file names are re-exported here).

Requests are JSON objects {"op": "list"|"read"|"call"|"claim", "name": ..., "args": {...}}. Every response while the
episode runs is a JSON object with "seq", "op", "result" and "budget" (calls and simulated seconds remaining). A call
the session refuses (bad argument, unknown tool, spent budget) gets result {"refused": {"check", "why", "hint"}} and
the executor is never called. Every
request and response goes to the journal in order. After claim (or an e-stop class result) the session
answers only {"episode": "ended", "message": "episode ended"}.

list has three answers (rcpx/surface_list.py). {"op": "list"} is the compact index, sized to be shown whole: per
tool its name, one line and its argument names with types, then the claim form, the resources and the budget.
{"op": "list", "name": NAME} (or a list of names; "claim" is one) returns those tools' full description, argument
schema and worked example. {"op": "list", "full": true} returns every tool's full entry at once: the form a client
that builds tool definitions from the listing asks for.

The practices ride once, on the first response that is not a compact or named list answer; reading
robot://practices counts as having had them. A compact list names robot://practices as the first read instead.

The executor is a plain object with one method per tool, called with keyword arguments named as in the surface
table (rcpx/surfaces.py); a tool entry may name a different "method" or "rename" arguments. Simulated time comes
from executor.sim_time() or executor.backend.read()["t"] unless a clock is passed.
"""
import difflib
import json
import math
import os
import re
import sys
import traceback

from . import surface_brief, surface_session, surfaces
from .server_edge import RESULT_URI, EdgeMixin, result_seq
from .server_socket import (END_NAME, ENDED, FRAMES_DIR, JOURNAL_NAME, MAX_REQUEST_BYTES,  # noqa: F401
                            MAX_SOCK_PATH, SOCK_NAME, _clear_stale, _send, _serve_conn, serve)

BUDGET_KEYS = ("calls", "sim_seconds")


def __getattr__(name):
    """The runtime has no default budget (the harness or the person states one, or there is none). Outside readers
    that still import the old name get a budget with nothing stated."""
    if name == "DEFAULT_BUDGET":
        return {"calls": None, "sim_seconds": None}
    raise AttributeError(name)


OPS = ("list", "read", "call", "claim")
PRACTICES_URI = "robot://practices"
practices_brief = surface_brief.practices_brief   # the practices as they ride on a call answer


def refusal(check, why, hint, **extra):
    body = {"check": check, "why": why, "hint": hint}
    body.update(extra)
    return {"refused": body}


class JsonlJournal:
    """Fallback journal with the rcpx.context.Journal call shape: append(kind, **fields) writes one JSON line
    {"seq", "kind", ...}, flushed per line. Pass the episode's context.Journal instead to share one ordered file."""

    def __init__(self, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.path = path
        self._seq = 0
        self._f = open(path, "a", encoding="utf-8")

    def append(self, kind, **fields):
        if self._f is None:
            return None
        record = {"seq": self._seq, "kind": str(kind), **fields}
        self._f.write(json.dumps(record, sort_keys=True, allow_nan=False) + "\n")
        self._f.flush()
        self._seq += 1
        return record

    def close(self):
        if self._f is not None:
            self._f.close()
            self._f = None


def _find_clock(executor):
    fn = getattr(executor, "sim_time", None)
    if callable(fn):
        return fn
    backend = getattr(executor, "backend", None)
    if backend is not None and callable(getattr(backend, "read", None)):
        return lambda: backend.read()["t"]
    return lambda: 0.0


def jsonable(obj, save_image=None, key="value"):
    """A JSON-safe copy: numpy to Python, non-finite floats to None, an H x W x 3 uint8 image saved as a file."""
    if obj is None or isinstance(obj, (bool, str, int)):
        return obj
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {str(k): jsonable(v, save_image, str(k)) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [jsonable(v, save_image, key) for v in obj]
    if isinstance(obj, os.PathLike):
        return os.fspath(obj)
    np = sys.modules.get("numpy")  # an ndarray can only arrive if numpy is already loaded
    if np is not None:
        if isinstance(obj, np.ndarray):
            if _image_like(obj, np):
                if save_image is not None:
                    return save_image(obj, key)
                return f"image {obj.shape[1]}x{obj.shape[0]} not saved (no episode dir)"
            return jsonable(obj.tolist(), save_image, key)
        if isinstance(obj, np.generic):
            return jsonable(obj.item(), save_image, key)
    return str(obj)


def _image_like(arr, np):
    if arr.dtype != np.uint8:
        return False
    return (arr.ndim == 3 and arr.shape[2] in (3, 4)) or (arr.ndim == 2 and min(arr.shape) >= 16)


def _template_regex(uri):
    parts = re.split(r"\{(\w+)\}", uri)
    pattern = "".join(re.escape(p) if i % 2 == 0 else f"(?P<{p}>[^/]+)" for i, p in enumerate(parts))
    return re.compile(pattern)


class Session(EdgeMixin):
    """One episode's operator session. dispatch(request) -> response never raises (whole answers); handle_text(line)
    is an operator's request and gets the brief answer (rcpx/server_edge.py)."""

    def __init__(self, executor, surface, resources, budget=None, *, journal=None, ep_dir=None, clock=None,
                 handlers=None):
        self.executor = executor
        self.surface = surfaces.get(surface) if isinstance(surface, str) else surface
        self.surface = surfaces.for_robot(self.surface, _joint_names(executor))  # examples name a real joint
        self._edge_init(executor)   # no gripper listed on a fixed finger, the dark rule (rcpx/server_edge.py)
        self.tools = {t["name"]: t for t in self._callable}
        self.resources = dict(resources or {})
        self.resources.setdefault(RESULT_URI, self._whole_result)
        self._templates = [(uri, _template_regex(uri), fn) for uri, fn in self.resources.items() if "{" in uri]
        budget = dict(budget or {})
        unknown = set(budget) - set(BUDGET_KEYS)
        if unknown:
            raise ValueError(f"unknown budget keys {sorted(unknown)}; known: {sorted(BUDGET_KEYS)}")
        self.budget = {key: value for key, value in budget.items() if value is not None}
        self.ep_dir = os.path.abspath(ep_dir) if ep_dir is not None else None
        self._own_journal = journal is None and self.ep_dir is not None
        if self._own_journal:
            journal = JsonlJournal(os.path.join(self.ep_dir, JOURNAL_NAME))
        self.journal = journal
        self.clock = clock if clock is not None else _find_clock(executor)
        self.handlers = dict(handlers or {})
        self.seq = 0
        self.calls_used = 0
        self.refusals = 0
        self.ended = False
        self.end_reason = None
        self._pending_end = None
        self._practices_sent = False
        self._last_t = 0.0
        self.t0 = self._now()
        self._log("session", event="start", surface=self.surface["name"], budget=self.budget,
                  tools=sorted(self.tools), resources=sorted(self.resources))

    # -- bookkeeping -------------------------------------------------------------------------------------------

    def _now(self):
        try:
            t = float(self.clock())
            if math.isfinite(t):
                self._last_t = t
        except Exception:  # a broken clock must not break the session; keep the last good reading
            pass
        return self._last_t

    def _log(self, kind, **fields):
        """One journal record via journal.append(kind, **fields), journal.write(dict) or journal(dict)."""
        if self.journal is None:
            return
        fields, _ = surfaces.scrub_truth(jsonable(fields))
        try:
            if callable(getattr(self.journal, "append", None)):
                self.journal.append(kind, **fields)
            elif callable(getattr(self.journal, "write", None)):
                self.journal.write({"kind": kind, **fields})
            elif callable(self.journal):
                self.journal({"kind": kind, **fields})
        except Exception as exc:  # the journal failing is reported on stderr, never to the operator
            print(f"rcpx.server: journal write failed: {type(exc).__name__}: {exc}", file=sys.stderr)

    def sim_seconds_used(self):
        """Simulated seconds of work: the session clock since the start, minus the seconds spent waiting on the
        world (world_wait_s)."""
        return max(0.0, self._now() - self.t0 - self.world_wait_s)

    def remaining(self):
        """The budget block every response carries (rcpx/server_edge.py: calls_used counts every call, calls_free
        the free ones, calls_left what the budget has left of the charged ones)."""
        return self._budget_block()

    def _save_image(self, seq):
        def save(arr, key):
            stem = f"srv_{seq:04d}_" + (re.sub(r"[^A-Za-z0-9_]", "_", key)[:40] or "image")
            if callable(getattr(self.journal, "save_frame", None)):  # one frames dir for the episode
                try:
                    return self.journal.save_frame(arr, stem)
                except Exception as exc:
                    return f"image not saved: {type(exc).__name__}: {exc}"
            if self.ep_dir is None:
                return f"image {arr.shape[1]}x{arr.shape[0]} not saved (no episode dir)"
            try:
                from PIL import Image
                frames = os.path.join(self.ep_dir, FRAMES_DIR)
                os.makedirs(frames, exist_ok=True)
                path = os.path.join(frames, f"{stem}.png")
                Image.fromarray(arr).save(path)
                return path
            except Exception as exc:
                return f"image not saved: {type(exc).__name__}: {exc}"
        return save

    def _end(self, reason, **info):
        if self.ended:
            return
        self.ended = True
        self.end_reason = reason
        record = {"reason": reason, "calls_used": self.calls_used, "calls_free": self.calls_free,
                  "motion_calls": self.motion_calls, "refusals": self.refusals,
                  "sim_seconds_used": round(self.sim_seconds_used(), 3), "world_wait_s": round(self.world_wait_s, 3),
                  **info}
        self._log("session", event="end", **record)
        if self.ep_dir is not None:
            try:
                with open(os.path.join(self.ep_dir, END_NAME), "w", encoding="utf-8") as f:
                    json.dump(record, f, indent=2, sort_keys=True)
            except OSError as exc:
                print(f"rcpx.server: could not write {END_NAME}: {exc}", file=sys.stderr)

    def close(self, reason="closed"):
        """End the episode (if it has not ended) and close a journal this session opened."""
        self._end(reason)
        if self._own_journal and self.journal is not None:
            self.journal.close()

    # -- requests ----------------------------------------------------------------------------------------------

    def handle_text(self, text):
        """Parse one operator request line and dispatch it; a line that is not JSON gets a refusal. The answer is
        the operator's form (brief, rcpx/server_edge.py operator_answer); the journal holds the whole one."""
        try:
            request = json.loads(text)
        except ValueError as exc:
            why = f"the request is not JSON ({exc})"
            return self._respond({"unparsed": text[:2000]}, lambda seq: self._bad_request(seq, why))
        return self.operator_answer(request, self.dispatch(request))

    def dispatch(self, request):
        """Answer one request object. Never raises."""
        return self._respond(request, lambda seq: self._route(seq, request))

    def refuse_request(self, why):
        """Journal and refuse a request the transport could not hand over whole."""
        return self._respond({"unparsed": why}, lambda seq: self._bad_request(seq, why))

    def _respond(self, request, produce):
        self.seq += 1
        seq = self.seq
        self._log("request", req=seq, t_s=self._now(), request=request)
        try:
            response = produce(seq)
        except Exception as exc:  # never raise to the client (SPEC section 0)
            self._log("error", req=seq, where="server", trace=traceback.format_exc())
            op = request.get("op") if isinstance(request, dict) else None
            response = self._envelope(seq, op, None, refusal(
                "internal", f"the server failed on this request: {type(exc).__name__}: {exc}",
                "nothing ran; send the request again or a different one"))
        self._log_response(seq, request, response)
        if self._pending_end is not None:  # the end record follows the response that caused it
            reason, info = self._pending_end
            self._pending_end = None
            self._end(reason, **info)
        return response

    def _log_response(self, seq, request, response):
        """A call is journaled as kind "call" (name, args as sent, result: what context.replay_calls reads);
        every other answer as kind "response" with the whole response."""
        result = response.get("result")
        refused = result.get("refused") if isinstance(result, dict) else None
        extra = {"refused": refused.get("check")} if isinstance(refused, dict) else {}
        if isinstance(request, dict) and request.get("op") == "call" and "result" in response:
            args = request.get("args") if "args" in request else request.get("args_text", {})
            self._log("call", req=seq, name=str(request.get("name")), args=args, result=result,
                      call=self.calls_used, t_s=self._now(), budget=response.get("budget"),
                      frames=response.get("frames", []), **extra)
        else:
            self._log("response", req=seq, t_s=self._now(), response=response, **extra)

    def _envelope(self, seq, op, name, result):
        response = {"seq": seq, "op": op}
        if name is not None:
            response["name"] = name
        response["result"] = result
        if isinstance(result, dict) and "refused" in result:
            self.refusals += 1
        frames = surfaces.frame_paths(result, self.ep_dir or getattr(self.journal, "dir", None))
        if frames:
            response["frames"] = frames
        response["budget"] = self.remaining()
        practices = self._ride_practices(op, name, result)
        if practices is not None:
            response["practices"] = practices
        return response

    def _ride_practices(self, op, name, result):
        """The practices reach the operator once, whatever it asks first. They ride on the first call answer
        or full list answer (a client that builds its tools from the list); never on a read (the read the operator
        asked for is shown as it is, and a read of robot://practices is that once) and never on a compact or named
        list answer (those are sized to be shown whole, and the compact one says to read robot://practices
        first)."""
        if self._practices_sent or PRACTICES_URI not in self.resources:
            return None
        refused = isinstance(result, dict) and "refused" in result
        if op == "read":
            if name == PRACTICES_URI:
                self._practices_sent = not refused
            return None
        full_list = op == "list" and isinstance(result, dict) and bool(result.get("full"))
        if op == "list" and not full_list:
            return None
        practices = self._read_resource(PRACTICES_URI)
        if "refused" in practices:
            return None
        self._practices_sent = True
        if full_list or not isinstance(practices.get("content"), str):
            return practices["content"]
        # on a call answer the practices ride brief (every default answer short enough to read whole):
        # each rule's opening words, and the free read that gives them whole
        return practices_brief(practices["content"])

    def _bad_request(self, seq, why):
        if self.ended:
            return dict(ENDED)
        return self._envelope(seq, None, None, refusal(
            "request", why, 'send one JSON object per line, for example {"op": "list"}'))

    def _route(self, seq, request):
        if self.ended:
            return dict(ENDED)
        if not isinstance(request, dict):
            return self._bad_request(seq, "the request must be a JSON object")
        op = request.get("op")
        if op not in OPS:
            near = difflib.get_close_matches(str(op), OPS, n=1, cutoff=0.5)
            hint = (f"did you mean {json.dumps(near[0])}? " if near else "") + f"ops: {', '.join(OPS)}"
            return self._envelope(seq, None, None, refusal("request", f"unknown op {_short(op)}", hint))
        name = request.get("name")
        if op == "list":
            return self._envelope(seq, op, name if isinstance(name, str) else None, self._list_answer(request))
        if op == "read":
            return self._envelope(seq, op, name, self._read_resource(name))
        if op == "call":
            result = self._repeat_aware(name, request, self._call(seq, name, request))
            self._keep_whole(seq, name, result)
            response = self._envelope(seq, op, name, result)
            if isinstance(result, dict) and result.get("estop"):  # the one result class that ends a session
                self._pending_end = ("estop", {"tool": name})
                response["episode"] = "stopped"
            return response
        return self._claim(seq, request)

    def _list_answer(self, request):
        """The list op's result: compact by default, the named tools' full entries, or everything ("full")."""
        listing = self._list()
        names = request.get("name")
        if names is not None and names != [] and names != "":
            return surfaces.named(listing, names if isinstance(names, list) else [names])
        if request.get("full") is True:
            return {**listing, "full": True}
        compact = surfaces.compact(listing, {t["name"]: t["brief"] for t in self.surface["tools"] if t.get("brief")})
        return surface_session.render(compact, surface_session.facts(self.executor))

    def _list(self):
        """The full listing: every tool with its description, argument schema and worked example, the claim,
        the resources and the budget. A subclass that changes the claim overrides this; every list answer is
        made from it."""
        tools = []
        for entry in self.surface["tools"]:
            args = entry["example"]["args"]
            tools.append({"name": entry["name"], "description": entry["description"],
                          "args": surfaces.public_schema(entry["args"]),
                          "example": {"args": args, "means": entry["example"]["means"],
                                      "cli": f"call {entry['name']} '{json.dumps(args)}'"}})
        claim = surfaces.CLAIM
        described = self.surface.get("resources", {})
        listing = {"surface": self.surface["name"], "about": self.surface.get("about", ""), "tools": tools,
                   "claim": {"description": claim["description"], "args": surfaces.public_schema(claim["args"]),
                             "example": {"args": claim["example"]["args"], "means": claim["example"]["means"],
                                         "cli": f"claim '{json.dumps(claim['example']['args'])}'"}},
                   "resources": [{"uri": uri, "description": described.get(uri, "")} for uri in self.resources],
                   "budget_total": dict(self.budget)}   # only what the harness or the person stated; {} = none
        # the tables are in words; a list answer shows this session's derived values behind them, as they are now
        return surface_session.render(listing, surface_session.facts(self.executor))

    def _resolve(self, uri):
        if uri in self.resources:
            return self.resources[uri], {}
        for _, rx, fn in self._templates:
            m = rx.fullmatch(uri)
            if m:
                return fn, m.groupdict()
        return None, None

    def _read_resource(self, uri):
        if not isinstance(uri, str) or not uri:
            return refusal("args", "read needs a resource URI in name", "resources: " + ", ".join(self.resources),
                           field="name")
        full_uri = uri
        uri, params = surface_brief.split_query(uri)
        seq = result_seq(uri)
        if seq is not None and uri != RESULT_URI:
            return {"uri": full_uri, "content": self._whole_result(seq)}
        fn, kwargs = self._resolve(uri)
        if fn is None:
            near = difflib.get_close_matches(uri, list(self.resources), n=3, cutoff=0.5)
            hint = (f"did you mean: {', '.join(near)}? " if near else "") + "resources: " + ", ".join(self.resources)
            return refusal("unknown_resource", f"no resource {_short(uri)}", hint, near=near)
        try:
            content = fn(**kwargs)
        except Exception as exc:
            self._log("error", where=f"resource {uri}", trace=traceback.format_exc())
            return refusal("internal", f"reading {uri} failed: {type(exc).__name__}: {exc}",
                           "read another resource or try again after a call")
        content, removed = surfaces.scrub_truth(jsonable(content))
        if removed:
            self._log("scrubbed", where=uri, keys=removed)
        content = self._body_session(uri, content)
        return {"uri": full_uri, "content": self._page(uri, params, content)}

    def _args(self, request, entry):
        """The call's arguments, or (None, refusal) when they are missing their JSON shape."""
        if "args" in request and request["args"] is not None:
            return request["args"], None
        if isinstance(request.get("args_text"), str):
            text = request["args_text"]
            try:
                return json.loads(text), None
            except ValueError as exc:
                example = json.dumps(entry["example"]["args"]) if entry else "{}"
                return None, refusal("args", f"the arguments are not valid JSON ({exc})",
                                     f"write them as one JSON object in single quotes, for example '{example}'",
                                     field="args")
        return {}, None

    def _budget_spent(self):
        if "calls" in self.budget and self.calls_charged() >= int(self.budget["calls"]):
            return (f"the call budget is spent ({self.calls_charged()} of {int(self.budget['calls'])} charged calls "
                    f"used)")
        used = self.sim_seconds_used()
        if "sim_seconds" in self.budget and used >= float(self.budget["sim_seconds"]):
            return (f"the simulated time budget is spent ({round(used, 1)} of "
                    f"{surfaces._fmt(float(self.budget['sim_seconds']))} s used)")
        return None

    def _call(self, seq, name, request):
        spent = self._budget_spent()
        if spent:
            return refusal("budget", spent, "list, read and claim still work; claim what you achieved")
        self.calls_used += 1
        if not isinstance(name, str) or name not in self.tools:
            if name == "claim":  # the claim is its own verb (T13): say how, do not suggest look-alike tools
                return refusal("unknown_tool", "claim is not a tool", surfaces.CLAIM_HOW +
                               "; it ends the episode with your statement of what you achieved", near=[])
            near = difflib.get_close_matches(str(name), list(self.tools), n=5, cutoff=0.5)
            listed = near or sorted(self.tools)
            hint = ("did you mean: " if near else "tools: ") + ", ".join(listed)
            return refusal("unknown_tool", f"no tool named {_short(name)}", hint, near=listed)
        entry = self.tools[name]
        args, bad = self._args(request, entry)
        if bad is None:
            args, bad = self._pop_detail(seq, entry, args)
        if bad is not None:
            self._account(name, entry, None, self._now())
            return bad
        args, shape_notes = surfaces.normalize_args(entry, args)
        problem = surfaces.check_args(entry, args, list(self.tools.values()))
        if problem is not None:
            self._account(name, entry, None, self._now())
            return {"refused": problem}
        args = surfaces.coerce(entry, args)
        rename = entry.get("rename", {})
        kwargs = {rename.get(k, k): v for k, v in args.items()}
        method = entry.get("method", name)
        fn = self.handlers.get(name) or getattr(self.executor, method, None)
        if not callable(fn):
            return refusal("unavailable", f"{name} is not available in this runtime",
                           "send list for the tools this session serves")
        kwargs = self._detail_kwargs(seq, entry, kwargs, fn)
        kwargs, fit_notes, bad = self._fit_call(seq, name, entry, fn, kwargs)
        if bad is not None:
            self._account(name, entry, None, self._now())
            return bad
        t_before = self._now()
        try:
            out = fn(**kwargs)
        except Exception as exc:
            self._log("error", req=seq, where=f"tool {name}", trace=traceback.format_exc())
            self._note_act(name, t_before)
            self._account(name, entry, None, t_before)
            return refusal("internal", f"{name} failed inside the runtime: {type(exc).__name__}: {exc}",
                           "the arm may have moved before the failure; look or read robot://world before the "
                           "next move")
        self._note_act(name, t_before)
        out = jsonable(out, self._save_image(seq), "image")
        if not isinstance(out, dict):
            out = {"value": out}
        self._account(name, entry, out, t_before)
        out = self._light_facts(entry, self._clear_hint(self._dark_hint(out)))
        out = self._motion_session(out, t_before)
        out, removed = surfaces.scrub_truth(out)
        if removed:
            self._log("scrubbed", req=seq, where=f"tool {name}", keys=removed)
        return self._arguments_notes(out, args, shape_notes + fit_notes)

    def _note_act(self, name, t_before):
        """Tell the world that this call moved the arm, for an executor that does not say so itself
        (reports_motion). The sign is simulated time: a call that only reads or looks takes none. verify judges
        its evidence against the last such call (rcpx/context.py World.note_act)."""
        if getattr(self.executor, "reports_motion", False) or not self._now() > t_before:
            return
        note = getattr(getattr(self.executor, "world", None), "note_act", None)
        if callable(note):
            try:
                note(name)
            except Exception:
                self._log("error", where=f"note_act {name}", trace=traceback.format_exc())

    def _repeat_aware(self, name, request, result):
        """A refused call sent again unchanged will be refused again. From the third identical refusal in a row
        the hint says so first, so an operator stuck in a loop is told to change the call."""
        if not (isinstance(result, dict) and "refused" in result):
            self._last_refused = None
            return result
        try:
            key = (name, json.dumps(request.get("args", request.get("args_text")), sort_keys=True, default=str))
        except Exception:
            key = (name, repr(request.get("args")))
        last = getattr(self, "_last_refused", None)
        count = last[1] + 1 if last and last[0] == key else 1
        self._last_refused = (key, count)
        if count >= 3:
            body = dict(result["refused"])
            body["repeated"] = count
            body["hint"] = (f"This exact call has now been refused {count} times in a row and will be refused "
                            f"again: change the call. " + str(body.get("hint", "")))
            result = dict(result, refused=body)
        return result

    def _claim(self, seq, request):
        entry = {"name": "claim", "args": surfaces.CLAIM["args"], "example": surfaces.CLAIM["example"]}
        args, bad = self._args(request, entry)
        if bad is None:
            problem = surfaces.check_args(entry, args)
            bad = None if problem is None else {"refused": problem}
        if bad is not None:
            return self._envelope(seq, "claim", None, bad)
        self._pending_end = ("claim", {"claim": args})
        response = {"seq": seq, "op": "claim", "episode": "claimed",
                    "result": {"claim": args, "message": "claim recorded; the episode has ended"},
                    "budget": self.remaining()}
        return response


def _joint_names(executor):
    """The robot's joint names in order when the executor's model gives them, else None."""
    names = getattr(getattr(executor, "model", None), "joint_names", None)
    try:
        return [str(n) for n in names] if names else None
    except TypeError:
        return None


def _short(value):
    text = json.dumps(value) if isinstance(value, (str, int, float, type(None))) else repr(value)
    return text if len(text) <= 60 else text[:57] + "..."
