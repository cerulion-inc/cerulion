# SPDX-License-Identifier: AGPL-3.0-only
"""What the session adds at its edges (rcpx/server.py Session inherits EdgeMixin): the budget's accounting, the
brief answer an operator gets, the whole results behind it, the journal's paging, the dark rule in refusal hints
and the light facts of a raw picture.

BUDGET. calls_used counts every call (each call record keeps its own index). A call of a tool in
surfaces.FREE_TOOLS (crop, mark, triangulate, verify, reach_check, picture, device_state: they compute on stored
looks or the model and move nothing) is also counted in calls_free and does not spend the call budget: calls_left
is the budget minus (calls_used - calls_free). motion_calls counts the calls during which simulated time passed
(the arm moved, or the world ran: a hold, a wait). Seconds a result declares as waiting on the world
(world_wait_s, or a tool's own key in surfaces.WORLD_WAIT_KEYS: wait_for_light's waited_s) are world time, not
work time: sim_seconds_used leaves them out and world_wait_s reports them. A spent budget still refuses
every call, free ones included.

BRIEF. Session.dispatch answers whole (in-process callers: the oracle, tests, replays). A request that
arrives as a text line (Session.handle_text: the socket, so the command line, the MCP bridge and the chat
operator) is answered in brief form (rcpx/surface_brief.py) unless the call sent "detail": "full"; the journal
always holds the whole answer, and robot://result/{seq} (a free read) returns it. Under the opt-in lean profile
(rcpx/r2/lean.py wraps dispatch) lean renders the answers and the brief step stands aside.

DARK HINTS. A refusal with check dark or can_see carries the surface's dark rule as its hint
(surfaces.dark_rule over this session's tools), so a hint names only tools this session has.
"""
import inspect
import json
import math
import os
import re

from . import surface_brief, surface_session, surfaces

RESULT_URI = "robot://result/{seq}"
JOURNAL_URI = "robot://journal"
DARK_CHECKS = ("dark", "can_see")
WHOLE_KEPT = 400            # whole results kept for robot://result (a call's answer is a few kB)
THINK_KEYS = ("world_seconds", "think_s_per_call")   # SessionR2's think-time keys: left out of a brief budget at 0
MM = 1000.0                 # unit: metres to millimetres
CLOCK_NOISE_S = 1e-9        # numeric: two reads of one clock value can differ by float rounding, never by this much


def _num(v):
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else None


class EdgeMixin:
    # -- set up ------------------------------------------------------------------------------------------------
    def _edge_init(self, executor):
        self._session_said = False
        self._guarded_said = False
        self.calls_free = 0
        self.motion_calls = 0
        self.world_wait_s = 0.0
        self._wholes = {}
        self._last_whole = None
        self._detail = {}
        self._callable = list(self.surface["tools"])   # every tool runs; the listing may hide one (below)
        self.surface = _without_absent_gripper(self.surface, executor)
        world = getattr(executor, "world", None)
        if world is not None and hasattr(world, "spot_rule") and world.spot_rule is None:
            world.spot_rule = spot_rule(executor)
        clock = getattr(executor, "sim_time", None)
        if world is not None and hasattr(world, "clock") and world.clock is None and callable(clock):
            world.clock = clock   # observations are stamped when they are taken (FINDINGS S9)
        claim = getattr(self, "claim_spec", None) or surfaces.CLAIM
        self.dark_hint = surfaces.dark_rule([t["name"] for t in self.surface["tools"]],
                                            cannot="cannot" in (claim.get("args") or {}).get("properties", {}))

    # -- budget ------------------------------------------------------------------------------------------------
    def calls_charged(self):
        return self.calls_used - self.calls_free

    def _account(self, name, entry, out, t_before):
        """After a call that reached a known tool: free, motion and world-wait bookkeeping."""
        if surfaces.is_free(name, entry):
            self.calls_free += 1
        t_after = self._now()
        elapsed = max(0.0, t_after - t_before)
        if elapsed > CLOCK_NOISE_S:
            self.motion_calls += 1
        if isinstance(out, dict) and "refused" not in out and elapsed > 0:
            wait = _num(out.get("world_wait_s"))
            key = surfaces.WORLD_WAIT_KEYS.get(name)
            if wait is None and key is not None:
                wait = _num(out.get(key))
            if wait is not None and wait > 0:
                self.world_wait_s += min(wait, elapsed)

    def _budget_block(self):
        used_s = self.sim_seconds_used()
        out = {}
        if "calls" in self.budget:
            out["calls_left"] = max(0, int(self.budget["calls"]) - self.calls_charged())
        if "sim_seconds" in self.budget:
            out["sim_seconds_left"] = round(max(0.0, float(self.budget["sim_seconds"]) - used_s), 3)
        out.update(calls_used=self.calls_used, calls_free=self.calls_free, motion_calls=self.motion_calls,
                   sim_seconds_used=round(used_s, 3))
        if self.world_wait_s > 0:
            out["world_wait_s"] = round(self.world_wait_s, 3)
        return out

    # -- answers to an operator ----------------------------------------------------------------------------------
    def _lean_attached(self):
        return bool(getattr(self.__dict__.get("dispatch"), "lean", False))

    def operator_answer(self, request, response):
        """The answer an operator's text request gets: the brief form of a call result (unless the call asked
        for detail full, or the lean profile renders answers itself) and a budget block without zero think keys."""
        if not isinstance(response, dict) or self._lean_attached():
            return response
        out = response
        result = response.get("result")
        if response.get("op") == "call" and isinstance(result, dict) and "refused" not in result \
                and self._detail.get(response.get("seq")) != "full":
            small = surface_brief.brief(result, response.get("seq"), response.get("name"))
            if small is not result:
                out = dict(response, result=small)
                frames = surfaces.frame_paths(small, self.ep_dir or getattr(self.journal, "dir", None))
                if frames:
                    out["frames"] = frames
                else:
                    out.pop("frames", None)
        budget = out.get("budget")
        if isinstance(budget, dict) and not budget.get("think_s_per_call"):
            trimmed = {k: v for k, v in budget.items() if not (k in THINK_KEYS and not v)}
            if trimmed != budget:
                out = dict(out, budget=trimmed)
        return out

    def _keep_whole(self, seq, name, result):
        if not isinstance(result, dict):
            return
        rec = {"seq": seq, "name": str(name), "result": result}
        self._wholes[seq] = rec
        self._last_whole = rec
        while len(self._wholes) > WHOLE_KEPT:
            self._wholes.pop(next(iter(self._wholes)))

    def _whole_result(self, seq=None):
        """robot://result/{seq}: the whole result of call SEQ as it was journaled ("last": the newest call)."""
        if seq in (None, "", "last"):
            rec = self._last_whole
        else:
            try:
                rec = self._wholes.get(int(str(seq)))
            except ValueError:
                rec = None
        if rec is None:
            kept = list(self._wholes)
            return {"refused": {"check": "seq", "why": f"no call result {seq!r} in this session",
                                "hint": "seq is the number in a call's answer (its \"seq\"); results kept: "
                                        + (f"{kept[0]} to {kept[-1]}" if kept else "none yet") + "; robot://result/last "
                                        "is the newest"}}
        return rec

    # -- what a call returns --------------------------------------------------------------------------------------
    def _pop_detail(self, seq, entry, args):
        """(the arguments without detail, None) or (None, a refusal for a detail that is not brief or full). The
        call's detail is kept by seq for operator_answer. A tool whose own schema takes detail (a package that
        extends look with it, for example) keeps it in its arguments, as sent; for any other tool it is taken
        out, and a tool entry marked pass_detail gets detail=True back when it is full (_detail_kwargs)."""
        if not (isinstance(args, dict) and "detail" in args):
            return args, None
        own = "detail" in ((entry.get("args") or {}).get("properties") or {})
        args = dict(args)
        value = args["detail"] if own else args.pop("detail")
        if value not in surfaces.DETAIL_VALUES:
            return None, {"refused": {"check": "args", "field": "detail",
                                      "why": f"detail must be one of {', '.join(surfaces.DETAIL_VALUES)}, got "
                                             f"{surfaces._show(value)}",
                                      "hint": "detail \"full\" returns the whole result; leave it out for the brief "
                                              "one"}}
        self._detail[seq] = value
        return args, None

    def _detail_kwargs(self, seq, entry, kwargs, fn):
        """detail=True for a tool entry marked pass_detail when the call asked for detail full and the callable
        takes it."""
        if self._detail.get(seq) == "full" and entry.get("pass_detail") and _takes(fn, "detail"):
            return {**kwargs, "detail": True}
        return kwargs

    def _fit_call(self, seq, name, entry, fn, kwargs):
        """Fit a call to the primitive that will run it, while the core's round four change lands: an argument
        marked x-retired (guarded) or x-intent (clear, expect, max_n) is passed only when the primitive takes it,
        else dropped with a note; an argument marked x-offered that the primitive still requires and the call
        left out is a refusal that names it. Returns (kwargs, notes, a refusal or None); nothing is invented."""
        # A harness's wrapper forwards every call with **kwargs: fit the real executor's own method instead.
        if not self.handlers.get(name):
            core = surface_session.core_executor(self.executor)
            target = getattr(core, entry.get("method", name), None) if core is not self.executor else None
            if callable(target):
                fn = target
        kwargs, notes = dict(kwargs), []
        reverse = {v: k for k, v in entry.get("rename", {}).items()}
        props = entry["args"].get("properties", {})
        for key in list(kwargs):
            arg = reverse.get(key, key)
            prop = props.get(arg, {})
            if (prop.get("x-retired") or prop.get("x-intent")) and not _takes(fn, key):
                value = kwargs.pop(key)
                if arg == "guarded" and value is False and not self._guarded_said:
                    notes.append(surface_session.GUARDED_RETIRED)
                if prop.get("x-intent"):
                    notes.append(f"{arg}: this runtime's {name} does not take it yet; the call ran without it")
        try:
            params = inspect.signature(fn).parameters
        except (TypeError, ValueError):
            return kwargs, notes, None
        # Only for arguments the schema leaves out because RCP offers a value ("x-offered"): a primitive that still
        # requires one gets a refusal that names it. Any other missing argument is the call's own business (a
        # package's hook may fill it before the primitive runs).
        missing = [p.name for p in params.values() if p.default is inspect.Parameter.empty
                   and p.kind not in (inspect.Parameter.VAR_POSITIONAL, inspect.Parameter.VAR_KEYWORD)
                   and p.name not in kwargs and props.get(reverse.get(p.name, p.name), {}).get("x-offered")]
        if missing:
            names = ", ".join(missing)
            example = entry["example"].get("cli") or f"call {name} '{json.dumps(entry['example']['args'])}'"
            return kwargs, notes, {"refused": {"check": "args", "field": missing[0],
                                              "why": f"{name} needs {names} in this runtime",
                                              "hint": f"state {names}; " + example}}
        return kwargs, notes, None

    def _arguments_notes(self, out, args, notes):
        notes = list(out.get("arguments_note") or []) + notes
        if args.get("guarded") is False and not self._guarded_said:
            if "retired" not in json.dumps(out) and surface_session.GUARDED_RETIRED not in notes:
                notes.append(surface_session.GUARDED_RETIRED)
            self._guarded_said = True
        if notes:
            out["arguments_note"] = notes
        return out

    def _body_session(self, uri, content):
        if uri == "robot://body" and isinstance(content, dict):
            statement = surface_session.first_statement(self.executor)
            if statement is not None:
                return dict(content, session=statement)
        return content

    def _motion_session(self, out, t_before):
        if not self._session_said and "refused" not in out and self._now() > t_before:
            self._session_said = True
            if "session" not in out:
                statement = surface_session.first_statement(self.executor)
                if statement is not None:
                    out["session"] = statement
        return out

    def _clear_hint(self, out):
        body = out.get("refused") if isinstance(out, dict) else None
        if isinstance(body, dict) and body.get("check") == "clear_needs_look":
            hint = str(body.get("hint", ""))
            if "look" not in hint and "snapshot" not in hint:
                return dict(out, refused=dict(body, hint=surface_session.look_hint(self.tools)))
        return out

    def park_arm(self, why):
        """Park the arm for a session that is about to close without its operator (SPEC-R4 A4.1: an abandoned
        session parks): through the gates an operator's park would meet (rcpx/r2/core.py gates, asked on the
        real executor) and then the executor's own park, which runs through the harness's wrapper like any call.
        It charges no call, is journaled as a session record with event park, and never raises."""
        try:
            if self.ended or not callable(getattr(self.executor, "park", None)):
                return None
            gates = getattr(surface_session.core_executor(self.executor), "gates", None)
            result = gates("park", {}) if callable(gates) else None
            if result is None:
                result = self.executor.park()
        except Exception as exc:
            result = {"refused": {"check": "internal", "why": f"{type(exc).__name__}: {exc}",
                                  "hint": "the arm was not parked"}}
        try:
            body = result.get("refused") if isinstance(result, dict) else None
            self._log("session", event="park", why=why,
                      stopped=result.get("stopped") if isinstance(result, dict) else None,
                      refused=body.get("check") if isinstance(body, dict) else None)
        except Exception:
            pass
        return result

    def _dark_hint(self, out):
        if isinstance(out, dict) and isinstance(out.get("refused"), dict) \
                and out["refused"].get("check") in DARK_CHECKS:
            body = dict(out["refused"], hint=self.dark_hint)
            return dict(out, refused=body)
        return out

    def _light_facts(self, entry, out):
        """A raw picture that says nothing of its light (v0 snapshot) gets brightness, can_see and, when it is not
        ok, its bright spots, read from the saved frame (rcpx/perception.py: v0 gets the facts)."""
        if not (entry.get("light_facts") and isinstance(out, dict) and "brightness" not in out
                and isinstance(out.get("image"), str)):
            return out
        path = out["image"]
        if not os.path.isabs(path):
            base = self.ep_dir or getattr(self.journal, "dir", None)
            path = os.path.join(str(base), path) if base is not None else path
        try:
            import numpy as np
            from PIL import Image

            from . import perception
            img = np.asarray(Image.open(path).convert("RGB"))
            facts = perception.light(img)
        except Exception as exc:   # an unreadable frame: say so rather than guess
            return {**out, "brightness": None, "can_see": None, "light_note": f"the frame could not be read: "
                    f"{type(exc).__name__}"}
        return {**out, **facts}

    # -- reads -------------------------------------------------------------------------------------------------
    def _page(self, base, params, content):
        if base == JOURNAL_URI or (params and isinstance(content, dict) and isinstance(content.get("records"), list)):
            return surface_brief.page_journal(content, params)
        return content


def _takes(fn, name):
    try:
        params = inspect.signature(fn).parameters
    except (TypeError, ValueError):
        return False
    return name in params or any(p.kind is inspect.Parameter.VAR_KEYWORD for p in params.values())


def _without_absent_gripper(surface, executor):
    """The surface as listed, without its gripper tool on a robot whose model says it has none (a fixed finger;
    FINDINGS T6). A call of it still runs and gets the primitive's own refusal."""
    spec = getattr(getattr(executor, "model", None), "spec", None)
    if not (isinstance(spec, dict) and "gripper" in spec and not spec["gripper"]):
        return surface
    tools = [t for t in surface.get("tools", []) if t.get("name") != "gripper"]
    if len(tools) == len(surface.get("tools", [])):
        return surface
    return dict(surface, tools=tools)


def result_seq(uri):
    m = re.fullmatch(r"robot://result(?:/(\w+))?", str(uri or ""))
    return None if m is None else (m.group(1) or "last")


def _tip_half_width(ex):
    try:
        q = ex.model.q_home
        tip = ex.model.fk(q)["tip"][0]
        nearest = None
        for capsule in ex.model.collision_capsules(q):
            _, a, b, radius = capsule
            delta = [float(y) - float(x) for x, y in zip(a, b)]
            offset = [float(t) - float(x) for x, t in zip(a, tip)]
            length_sq = sum(v * v for v in delta)
            along = max(0, min(1, sum(x * y for x, y in zip(offset, delta)) / length_sq)) if length_sq else 0
            distance = math.sqrt(sum((x - along * y) ** 2 for x, y in zip(offset, delta)))
            if nearest is None or distance < nearest[0]:
                nearest = (distance, float(radius))
        width = nearest[1] * MM if nearest else 0
        return width if math.isfinite(width) and width >= 0 else 0
    except Exception:
        return 0


def spot_rule(ex):
    """The session's "same spot" for two stored contacts (SPEC-R4 A4.3): their combined position error plus the
    tool tip's own half width, in mm; None when the session has no bounds. An entry's error is what it states
    (data.error_mm: what its speed and the reading's step left open), else what the bounds hold for that thing,
    else the travel the robot can tell from none at that contact's force (rigid): never a typed length."""
    B = surface_session.bounds_of(ex)
    if B is None:
        return None
    width = _tip_half_width(surface_session.core_executor(ex))

    def sigma_mm(entry):
        data = entry.get("data") or {}
        if data.get("error_mm") is not None:
            return float(data["error_mm"])
        try:
            known = B.thing(entry["id"]).sigma_p
        except Exception:
            known = None
        if known is None:
            known = B.rigid(None, None, data.get("force_n"))
        return float(known) * MM

    def rule(a, b):
        try:
            sigmas = [sigma_mm(a), sigma_mm(b)]
            if not all(math.isfinite(v) and v >= 0 for v in sigmas):
                return None
            value = math.hypot(*sigmas) + width
            return value if math.isfinite(value) else None
        except Exception:
            return None
    return rule
