# SPDX-License-Identifier: AGPL-3.0-only
"""The plug-in core of round two (SPEC-R2.md sections 1.1 and 1.2).

A package is a module rcpx/r2/<name>.py that may export ID, Mixin, setup(ex), TOOLS, RESOURCES, RESOURCE_PATCH,
HANDLERS and PRACTICES. This file loads the packages, builds one Executor class from the round one Executor plus
every package's Mixin, gives the executor its hook points and assembles the operator surface. Nothing here names a
robot, a device or a package's behaviour.

Hooks (the only way a package changes an existing primitive):
    ex.add_pre(fn, names=None)    fn(name, kwargs) -> None | {"refused": ...} | ("skip", result)
    ex.add_post(fn, names=None)   fn(name, kwargs, result) -> None | dict (a dict replaces the result)
    ex.add_tick(fn)               fn(sense) -> None | str, once per control period inside motion
    ex.add_idle(fn)               fn(seconds), after the world idled
    ex.on_event(kind, fn)         fn(**fields); ex.note_event(kind, **fields) raises one
    ex.state                      shared dict, keyed by package name
    ex.epoch                      the world epoch (0 unless a package raises it)
Pre hooks get the live kwargs and may pop their own extra arguments; post hooks get the kwargs as the operator
sent them. A hook that raises is journaled and skipped: a package bug never ends a session.

TOOLS entries: a whole tool entry (surfaces._tool) adds a tool, or replaces a round one tool of that name; an
entry {"extends": NAME, "args": {name: schema}, "optional": [names], "note": text} changes one in place: args are
added (or replace an argument's schema), every name in optional leaves the tool's required list, and note is
appended to its description. So a package that gives a tool another way to say an argument extends it and makes
the old argument optional, instead of replacing the whole entry.

The executor tells the world each control period that the arm is moving (World.note_act, reports_motion): verify
judges its evidence against the last call that moved the arm.

OPT-IN packages (OPT_IN) are in PACKAGES for their place in the order, but "all" leaves them out: one loads only
when it is named ("all,lean", "lean", or a list that names it). IMPLIES names what a package needs loaded with it:
naming carry also loads lean (SPEC-R3 2.2 C0; carry's profile is served through lean's LEAN_PROFILE).

HELD packages (HELD, a part of OPT_IN) are built and tested but stay out of "all" while a known fault of the
highest class is open against them (a tool that can act on a part the device sheet forbids, a result that can say
done, changed or confirmed without the evidence, a tool that fails on the input it was built for): no episode
gets such a tool unless it asks for it by name ("all,act"). The setting RCPX_R2_HELD (a comma list of HELD names)
puts those names back into "all" for one process and its children: a held package's own tests set it, so that
they exercise the package the way an episode that names it would. A name that is not in HELD is ignored there.

GATES (SPEC-R3 3.8): a primitive called from inside another runs bare, so a move the runtime makes by itself would
skip the dark gate, the person gates and the path gates. ex.gates(name, kwargs) runs every pre hook an operator's
call of name would meet (a hook registered without names covers every primitive), in registration order, on one
copy of kwargs, and returns the first refusal (a ("skip", result) answer counts as none); None when every gate
passes. A runtime move calls it before each nested motion and stops with that refusal.

FILTER: a package may export FILTER(surface, resources, handlers) -> (surface, resources, handlers). assemble
applies it after every package's tools, resources, handlers and practices are merged, in package order, so a
package may narrow or re-word what a session declares. A FILTER that names the parameters ex or box gets them as
keywords (the executor, and the Box whose session is set later). A FILTER that raises, or returns anything but
the three tables, is reported (r2_problem, ex.r2_problems) and skipped: the tables stay as they were.
box.on_session(fn) asks for fn(session) once the runner sets box.session.

UNCHARGED WORLD TIME: ex.uncharge(seconds, why) marks world seconds the operator's work budget is not charged for
(the waiting package's wait for light). assemble wraps the session's clock to subtract ex.uncharged_s, so
sim_seconds_used and journal t_s leave them out, and the budget block gains uncharged_s once it is over 0. A
session that leaves world waits out itself (it has world_wait_s: it reads a result's world_wait_s key) is left as
it is, so the same seconds are never taken off twice; the waiting package's result carries world_wait_s for it.
"""

import copy
import functools
import importlib
import inspect
import os
import traceback

from rcpx.context import jsonable, refusal
from rcpx.control import Executor

PACKAGES = ["selfcheck", "device", "obstacles", "approach", "person", "epochs", "grasp", "probe", "waiting",
            "frames", "survey", "selfsee", "witness", "sight", "act", "finish", "carry", "lean"]
HELD = ["witness", "sight", "act", "finish"]   # built, loadable by name, out of "all": a high finding is open
OPT_IN = HELD + ["carry", "lean"]   # in PACKAGES (lean last: its FILTER sees the others' tables) but not part of "all"
HELD_ENV = "RCPX_R2_HELD"   # a comma list of HELD names that "all" takes in again (a held package's own tests)
IMPLIES = {"carry": ["lean"]}   # naming the key also loads every package in its list
PRACTICES_HEADING = "## Rules of the tools this robot adds"   # the packages' PRACTICES lines follow it


def _enabled(enabled):
    """The names to load, in PACKAGES order. "all" (the default, also the empty text) is every package that is
    not opt-in; "none" is nothing; any other name is that package, so "all,lean" is every default one plus the
    opt-in lean, and a name in IMPLIES brings its list too ("all,carry" is every default one, carry and lean). A
    list is read the same way, item by item. HELD names listed in the setting HELD_ENV count as default ones."""
    if enabled is None:
        enabled = os.environ.get("RCPX_R2", "all")
    if isinstance(enabled, str):
        enabled = [p.strip() for p in enabled.strip().lower().split(",") if p.strip()] or ["all"]
    back = {p.strip().lower() for p in os.environ.get(HELD_ENV, "").split(",")} & set(HELD)
    names = set()
    for item in enabled:
        if item == "all":
            names.update(p for p in PACKAGES if p not in OPT_IN or p in back)
        elif item != "none":
            names.add(item)
    for item in list(names):
        names.update(IMPLIES.get(item, ()))
    return [p for p in PACKAGES if p in names]


class PackageImportError(ImportError):
    """A listed package that does not import: the session does not start."""


def load(enabled=None):
    """Import every enabled package. the design rulings: a listed package that does not import (it fails to import,
    or it is not there) is an error that stops the session start, never a dropped line: PackageImportError names
    each with the last line of its trace (load.problems and load.absent keep the whole record)."""
    mods, load.problems, load.absent = [], {}, []
    for name in _enabled(enabled):
        full = f"rcpx.r2.{name}"
        try:
            mods.append(importlib.import_module(full))
        except ModuleNotFoundError as exc:
            if exc.name == full:
                load.absent.append(name)
            else:
                load.problems[name] = traceback.format_exc()
        except Exception:
            load.problems[name] = traceback.format_exc()
    if load.problems or load.absent:
        raise PackageImportError("a listed package does not import, so the session does not start: "
                                 + "; ".join([f"{k}: {str(v).strip().splitlines()[-1]}" for k, v in
                                              sorted(load.problems.items())] + [f"{k}: not there" for k in load.absent]))
    return mods


load.problems, load.absent = {}, []


def _pkg(mod):
    return mod.__name__.rsplit(".", 1)[-1]


def _is_primitive(attr):
    return callable(attr) and hasattr(attr, "__wrapped__")


class HookMixin:
    """Hook points around the round one Executor. Must come first in the bases."""

    reports_motion = True            # _period tells the world when the arm moves; a session need not infer it

    def __init__(self, model, backend, journal=None, world=None, cfg=None, mods=()):
        self._act_name = None        # the operator's own call under way, named to the world when the arm moves
        self._pre, self._post, self._ticks, self._idles, self._listeners = [], [], [], [], {}
        self.state = {}
        self.epoch = 0
        self._tick_stop = None       # a tick hook's stop request; the motion loops read it through Guard.check
        self._hook_depth = 0
        self.context = {}            # what the runner knows and the operator may know too (see set_context)
        self.r2_problems = {}
        self.uncharged_s = 0.0       # world seconds the session's active clock does not count (see uncharge)
        self.mods = list(mods)
        super().__init__(model, backend, journal, world, cfg)
        self._wrap_primitives()
        self.add_post(self._core_events)
        for mod in self.mods:
            fn = getattr(mod, "setup", None)
            if fn is None:
                continue
            try:
                fn(self)
            except Exception:
                self.r2_problems[_pkg(mod)] = traceback.format_exc()
                self._journal("r2_problem", package=_pkg(mod), where="setup", trace=traceback.format_exc())

    # ---------------------------------------------------------------- registration
    def add_pre(self, fn, names=None):
        self._pre.append((fn, None if names is None else set(names)))

    def add_post(self, fn, names=None):
        self._post.append((fn, None if names is None else set(names)))

    def add_tick(self, fn):
        self._ticks.append(fn)

    def add_idle(self, fn):
        self._idles.append(fn)

    def on_event(self, kind, fn):
        self._listeners.setdefault(kind, []).append(fn)

    def note_event(self, kind, **fields):
        self._journal("event", event=kind, epoch=self.epoch, **jsonable(fields))
        for fn in self._listeners.get(kind, []):
            self._guarded(fn, f"event {kind}", **fields)

    def set_context(self, **fields):
        """The runner hands over operator-visible context after construction: devices (the robot://devices
        document), task (the robot://task document, with constraints), person (callable(kind, question, evidence)
        -> answer text, the scripted person), experience_path (a file that outlives the episode), think_s.
        Packages read ex.context lazily at call time; a missing key means "not available here"."""
        self.context.update(fields)
        self.note_event("context", keys=sorted(fields))

    def primitive_names(self):
        return sorted(self._primitive_names)

    def uncharge(self, seconds, why=""):
        """World seconds the operator's work budget is not charged for (waiting for light is world time, not
        active work). The session's active clock (sim_seconds_used, journal t_s) subtracts ex.uncharged_s: assemble
        wraps the session's clock once the runner sets box.session. Journaled as "uncharged" (seconds, total_s,
        why) so a replay or a scorer can put the world clock back together."""
        s = max(0.0, float(seconds))
        if s > 0.0:
            self.uncharged_s += s
            self._journal("uncharged", seconds=round(s, 6), total_s=round(self.uncharged_s, 6), why=str(why))
        return round(self.uncharged_s, 3)

    def gates(self, name, kwargs=None):
        """The pre hooks an operator's call of `name` would meet, for a move the runtime makes by itself (see the
        module text, GATES): the first refusal (journaled as "gate", by runtime) or None. The hooks see one copy of
        kwargs, so nothing the caller holds is changed; a hook that raises is journaled and skipped."""
        args = copy.deepcopy(dict(kwargs or {}))
        self._hook_depth += 1
        try:
            for fn, names in self._pre:
                if names is not None and name not in names:
                    continue
                r = self._guarded(fn, f"gate {name}", name, args)
                if isinstance(r, dict) and "refused" in r:
                    out = jsonable(r)
                    self._journal("gate", name=name, by="runtime", refused=out.get("refused"))
                    return out
        finally:
            self._hook_depth -= 1
        return None

    # ---------------------------------------------------------------- plumbing
    def _guarded(self, fn, where, *args, **kwargs):
        """A hook's answer. the design rulings: a PRE hook or a gate that raises REFUSES the call (fails closed:
        check hook_failed, the hook's error in the answer); a post hook, a tick, an idle hook or an event listener
        that raises is journaled and leaves the result as it was."""
        try:
            return fn(*args, **kwargs)
        except Exception as exc:
            trace = traceback.format_exc()
            self._journal("r2_problem", where=where, hook=getattr(fn, "__qualname__", repr(fn)), trace=trace)
            if where.startswith(("pre ", "gate ")):
                return refusal("hook_failed", f"the check {getattr(fn, '__qualname__', repr(fn))} before this call "
                               f"raised {type(exc).__name__}: {str(exc)[:200]}; the call was not run (a check that "
                               "cannot answer stops the call)", "the same call again once the check answers")
            return None

    def _wrap_primitives(self):
        self._primitive_names = set()
        for name in dir(type(self)):
            if name.startswith("_"):
                continue
            if _is_primitive(getattr(type(self), name, None)):
                self._primitive_names.add(name)
                setattr(self, name, self._hooked(name, getattr(self, name)))

    def _hooked(self, name, bound):
        @functools.wraps(bound)
        def call(*args, **kwargs):
            if self._depth > 0 or self._hook_depth > 0:
                return bound(*args, **kwargs)        # nested: hooks run on the operator's own call only
            opened = getattr(self, "_open_call", None)
            if opened is not None:                   # the design rulings: an earlier call's passage frees nothing here
                opened()                             # (the hooks' checks read the same opening as the call)
            self._call_opened = True
            self._jaws_enter()                       # the hooks' path checks see the jaws where they are too
            try:
                return _call(*args, **kwargs)
            finally:
                self._call_opened = False
                self._jaws_exit()

        def _call(*args, **kwargs):
            sent = copy.deepcopy(kwargs)
            out = None
            self._hook_depth += 1
            try:
                for fn, names in self._pre:
                    if names is not None and name not in names:
                        continue
                    r = self._guarded(fn, f"pre {name}", name, kwargs)
                    if isinstance(r, tuple) and len(r) == 2 and r[0] == "skip":
                        out = r[1]
                        break
                    if isinstance(r, dict) and "refused" in r:
                        out = r
                        break
            finally:
                self._hook_depth -= 1
            if out is not None:
                out = jsonable(out if isinstance(out, dict) else {"value": out})
                self._journal("primitive", name=name, args=jsonable(sent), result=out, by="pre_hook")
            else:
                self._tick_stop = None
                self._act_name = name
                out = bound(*args, **kwargs)
            self._hook_depth += 1
            try:
                for fn, names in self._post:
                    if names is not None and name not in names:
                        continue
                    r = self._guarded(fn, f"post {name}", name, sent, out)
                    if isinstance(r, dict):
                        out = jsonable(r)
            finally:
                self._hook_depth -= 1
            return out

        return call

    def _core_events(self, name, kwargs, result):
        """Events every package may listen for, read off results so no motion code is touched."""
        if not isinstance(result, dict) or "refused" in result:
            return None
        stopped = result.get("stopped")
        if stopped in ("contact", "cap", "part_hit") or result.get("contact") is True:
            self.note_event("contact", primitive=name, stopped=stopped, result=result)
            if name in ("touch", "press"):
                self.note_event("device_act", primitive=name, args=kwargs, result=result)
        moved = result.get("base_moved")
        if isinstance(moved, dict):
            moved = moved.get("moved")
        if moved or (name in ("relocalize", "standard_look") and
                     (result.get("moved") or (result.get("relocalize") or {}).get("moved"))):
            self.note_event("base_moved", primitive=name, result=result)
        if name in ("look", "standard_look") and result.get("look_id") is not None:
            self.note_event("look", look_id=result.get("look_id"), can_see=result.get("can_see"), result=result)
        return None

    # ---------------------------------------------------------------- ticks and idle time
    def _period(self, q_des):
        s = super()._period(q_des)
        note = getattr(self.world, "note_act", None)
        if note is not None:         # the arm was commanded through a control period: this call is an act
            note(self._act_name)
        for fn in self._ticks:
            r = self._guarded(fn, "tick", s)
            if isinstance(r, str) and self._tick_stop is None:
                self._tick_stop = r
        return s

    def idle(self, seconds):
        """Let the world run for `seconds` with the arm holding its present command (think time, hold, wait).
        Devices, light schedules and timed interventions run; nothing is commanded anew."""
        seconds = max(0.0, float(seconds))
        n = int(round(seconds / self.dt))
        if n > 0:
            if self._q_des is not None:
                self._command(self._q_des)
            self.backend.step(n)
            self._sense()
        for fn in self._idles:
            self._guarded(fn, "idle", n * self.dt)
        if n > 0:
            self.note_event("idle", seconds=round(n * self.dt, 3))
        return {"idled_s": round(n * self.dt, 3)}


@functools.lru_cache(maxsize=None)
def _class_for(mod_names):
    mods = [importlib.import_module(n) for n in mod_names]
    bases = [HookMixin]
    for mod in mods:
        mixin = getattr(mod, "Mixin", None)
        if mixin is None:
            continue
        clash = [k for k, v in vars(mixin).items()
                 if not k.startswith("__") and hasattr(Executor, k) and k not in getattr(mod, "OVERRIDES", ())]
        if clash:
            raise TypeError(f"package {_pkg(mod)} redefines Executor names {sorted(clash)}; change behaviour through "
                            "hooks, or list the name in OVERRIDES with a reason in the module docstring")
        bases.append(mixin)
    bases.append(Executor)
    return type("ExecutorR2", tuple(bases), {})


def executor_class(mods=()):
    """The Executor class with hooks and every package Mixin (cached per package set)."""
    return _class_for(tuple(m.__name__ for m in mods))


def make_executor(model, backend, journal=None, world=None, cfg=None, mods=None, enabled=None):
    """An executor with hooks and the loaded packages. mods=None loads them (see load)."""
    if mods is None:
        mods = load(enabled)
    return executor_class(mods)(model, backend, journal, world, cfg, mods=mods)


class Box:
    """Late-bound holder: handlers and resources are built before the Session exists. on_session(fn) asks for
    fn(session) when the runner sets box.session (at once when it is set already); a fn that raises is kept in
    box.problems, reported through box.report(trace) when that is set, and skipped."""

    def __init__(self):
        self._session = None
        self._waiting = []
        self.problems = []
        self.report = None
        self.base_handlers = {}
        self.base_resources = {}

    @property
    def session(self):
        return self._session

    @session.setter
    def session(self, value):
        self._session = value
        if value is not None:
            for fn in list(self._waiting):
                self._tell(fn, value)

    def on_session(self, fn):
        self._waiting.append(fn)
        if self._session is not None:
            self._tell(fn, self._session)

    def _tell(self, fn, session):
        try:
            fn(session)
        except Exception:
            trace = traceback.format_exc()
            self.problems.append(trace)
            if self.report is not None:
                self.report(trace)


def round_one_surface(base):
    """The round one surface under its own name, so reports can tell vnext1 from vnext."""
    out = copy.deepcopy(base)
    out["name"] = "vnext1"
    return out


def _merge_tools(surface, mods, problems):
    by_name = {t["name"]: t for t in surface["tools"]}
    for mod in mods:
        for entry in getattr(mod, "TOOLS", ()) or ():
            if "extends" in entry:
                target = by_name.get(entry["extends"])
                if target is None:
                    problems[_pkg(mod)] = f"extends unknown tool {entry['extends']!r}"
                    continue
                props = target["args"].setdefault("properties", {})
                for key, schema in (entry.get("args") or {}).items():
                    props[key] = schema
                relax = list(entry.get("optional") or [])
                unknown = [k for k in relax if k not in props]
                if unknown:
                    problems[_pkg(mod)] = f"extends {entry['extends']!r}: optional names no argument {unknown}"
                if relax:
                    target["args"]["required"] = [k for k in target["args"].get("required", []) if k not in relax]
                if entry.get("note"):
                    target["description"] = target["description"].rstrip() + " " + entry["note"].strip()
            else:
                entry = dict(entry)
                entry.setdefault("x-package", _pkg(mod))
                if entry["name"] in by_name:
                    surface["tools"] = [entry if t["name"] == entry["name"] else t for t in surface["tools"]]
                else:
                    surface["tools"].append(entry)
                by_name[entry["name"]] = entry


def _filtered(mods, surface, resources, handlers, ex, box, problems):
    """Each package's FILTER over the merged tables, in package order (see the module text)."""
    for mod in mods:
        fn = getattr(mod, "FILTER", None)
        if fn is None:
            continue
        try:
            params = inspect.signature(fn).parameters
            extra = {k: v for k, v in (("ex", ex), ("box", box)) if k in params}
            out = fn(surface, resources, handlers, **extra)
            if not (isinstance(out, (tuple, list)) and len(out) == 3 and isinstance(out[0], dict)
                    and isinstance(out[0].get("tools"), list) and isinstance(out[1], dict)
                    and isinstance(out[2], dict)):
                raise TypeError("FILTER must return (surface with a tools list, resources dict, handlers dict)")
            new_surface, new_resources, new_handlers = out
        except Exception:
            problems[_pkg(mod)] = traceback.format_exc()
            _report(ex, package=_pkg(mod), where="FILTER", trace=traceback.format_exc())
            continue
        surface, resources, handlers = new_surface, new_resources, new_handlers
    return surface, resources, handlers


def _report(ex, **fields):
    journal = getattr(ex, "_journal", None)
    if callable(journal):
        journal("r2_problem", **fields)


def _uncharged_clock(ex):
    """fn(session): the session's clock less ex.uncharged_s (world seconds a package has not charged, ex.uncharge),
    and every budget block gains uncharged_s once it is over 0. The clock is the session's own (think time already left out by a
    round two session); the start time t0 was read before any uncharge, so the active clock stays monotone."""
    def wrap(session):
        inner = getattr(session, "clock", None)
        if not callable(inner) or getattr(inner, "_r2_uncharged", False):
            return
        if hasattr(session, "world_wait_s"):
            # the session leaves world waits out of sim_seconds itself (a result's world_wait_s): wrapping its
            # clock too would subtract the same seconds twice
            return

        def clock():
            return float(inner()) - float(getattr(ex, "uncharged_s", 0.0) or 0.0)

        clock._r2_uncharged = True
        session.clock = clock
        rem = getattr(session, "remaining", None)
        if callable(rem):
            def remaining():
                out = rem()
                left = float(getattr(ex, "uncharged_s", 0.0) or 0.0)
                if isinstance(out, dict) and left > 0.0:      # a session with nothing uncharged reads as before
                    out["uncharged_s"] = round(left, 3)
                return out

            session.remaining = remaining
    return wrap


def assemble(ex, mods, base_surface, base_resources=None, base_handlers=None):
    """(surface, resources, handlers, box) for a Session over `ex` with the packages' tools, resources, handlers
    and practices merged into the round one tables, then narrowed by every package FILTER. Set box.session once
    the Session exists."""
    box = Box()
    box.report = lambda trace: _report(ex, where="on_session", trace=trace)
    box.on_session(_uncharged_clock(ex))
    problems = getattr(ex, "r2_problems", {})
    surface = copy.deepcopy(base_surface)
    surface["tools"] = list(surface["tools"])
    surface["resources"] = dict(surface.get("resources", {}))
    resources = dict(base_resources or {})
    handlers = dict(base_handlers or {})
    box.base_handlers, box.base_resources = dict(handlers), dict(resources)
    _merge_tools(surface, mods, problems)
    for mod in mods:
        for uri, item in (getattr(mod, "RESOURCES", None) or {}).items():
            fn, text = item if isinstance(item, tuple) else (item, "")
            resources[uri] = (lambda fn=fn: (lambda **kw: fn(ex, box.session, **kw)))()
            surface["resources"][uri] = text or surface["resources"].get(uri, "")
        for uri, patch in (getattr(mod, "RESOURCE_PATCH", None) or {}).items():
            inner = resources.get(uri)
            if inner is None:
                continue                              # the resource's owner is switched off: nothing to patch

            def patched(inner=inner, patch=patch, uri=uri, **kw):
                content = inner(**kw)
                try:
                    new = patch(ex, box.session, content)
                except Exception:
                    ex._journal("r2_problem", where=f"resource patch {uri}", trace=traceback.format_exc())
                    return content
                return content if new is None else new

            resources[uri] = patched
        for name, fn in (getattr(mod, "HANDLERS", None) or {}).items():
            handlers[name] = (lambda fn=fn: (lambda **kw: fn(ex, box.session, **kw)))()
    lines = [getattr(m, "PRACTICES", "").strip() for m in mods if getattr(m, "PRACTICES", "").strip()]
    if lines and "robot://practices" in resources:
        inner = resources["robot://practices"]

        def practices(**kw):
            content = inner(**kw)
            extra = PRACTICES_HEADING + "\n" + "\n".join(lines)     # FINDINGS T10: under a heading, not a run
            if isinstance(content, str):
                return content.rstrip() + "\n" + extra + "\n"
            if isinstance(content, dict) and isinstance(content.get("text"), str):
                return {**content, "text": content["text"].rstrip() + "\n" + extra + "\n"}
            return content

        resources["robot://practices"] = practices
    surface, resources, handlers = _filtered(mods, surface, resources, handlers, ex, box, problems)
    return surface, resources, handlers, box


__all__ = ["HELD", "HELD_ENV", "IMPLIES", "OPT_IN", "PACKAGES", "HookMixin", "assemble", "executor_class", "load",
           "make_executor", "round_one_surface", "refusal"]
