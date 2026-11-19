# SPDX-License-Identifier: AGPL-3.0-only
"""Package bimanual (BM1): one rcpx vnext session for a robot with several arms on one base.

    session = build_session(multi, specs, ep_dir=..., task_doc=..., budget={"calls", "sim_seconds"},
                            packages="all,place", session_base=SessionR2 (plus a bench's mixins), claim=CLAIM_R2,
                            law_module=None, cfg={arm: executor cfg})

multi is a MultiArmBackend (rcpx/r2/bimanual_arms.py: the robot's own arm names, one world). Per arm: its own body
model (specs[arm]), an ArmView over multi, its own World (rcpx/r2/bimanual_world.py ArmWorld: every stored point is
mirrored into the other arms' Worlds) and an executor with the loaded round two packages, the scene camera package,
the robot's servo law module (law_module, added like a package) and the clearance module (rcpx/r2/bimanual_clear.py).
One Journal (every executor record carries "arm"; each arm's frames go to frames/<arm>/), one call budget, one
clock (the world's).

The tool table is the executors' vnext table (rcpx.r2.core.assemble per arm, merged by name) with a REQUIRED
argument arm on every tool (enum: the arms that serve it), plus move_arms, home_arms and handover. A call without
arm, with an arm that is not the robot's, with another arm's look id or another arm's wrist camera is refused with
check "arm" before anything runs; it counts as a call. Results carry "arm". Look ids: a wrist look of arm A is
"A.L<n>" (the executor's own L<n>, so it is unambiguous across arms); a fixed camera look is "C<n>", numbered
once for the session (the scene camera state is shared by the arms), and any arm may mark it (its ray is the
camera's, in that arm's frame). Resources: robot://body (rcpx/r2/bimanual_body.py), robot://world (frame R and every
arm's frame B), robot://cameras, one robot://journal; any other resource is the arms' common answer, or {"by_arm"}.

Not supported with several arms (dropped and reported in bimanual.json): the opt-in packages that wrap the session
itself (lean, carry). Nothing here names a robot, a task or an object: arm names come from multi.arms.
"""

import copy
import importlib
import inspect
import json
import os
import re
import types
from pathlib import Path

import numpy as np

from rcpx import surfaces
from rcpx.context import jsonable, refusal as _refusal
from rcpx.r2 import bimanual_bounds as D
from rcpx.r2 import bimanual_body as BB
from rcpx.r2 import bimanual_clear as BC
from rcpx.r2 import bimanual_handover as BH
from rcpx.r2 import bimanual_sync as BS
from rcpx.r2 import bimanual_world as BW
from rcpx.r2.bimanual_arms import ArmView, Coordinator, MultiArmBackend  # noqa: F401

def refusal(check, why, hint):
    hint = hint if '(' in hint else 'home_arms()'
    return {'said':'Refused: '+why+'.',**_refusal(check,why,hint)}


ID = "BM1"
UNSUPPORTED = ("lean", "carry")         # opt-in packages that wrap the session itself (module docstring)
NOT_PACKAGES = ("all", "none", "scene_camera", "bimanual")
JOURNAL_TAIL = 200
ARM_TOKEN = "ARM"                       # stands for an arm name in TOOLS until build_session knows the robot's
INSTALL_HOW = "how the robot is installed, as the person who set it up gave it"
DEVICES_DOC = {"level": "none", "devices": [],
               "note": "no device knowledge for this task: find what the task names by looking and name it yourself"}
PRACTICES = ("This robot has several arms: every tool that moves or looks through one takes arm. Each arm's tools "
             "use its own base frame B; a point any arm stores is shared, and robot://world shows it in frame R and "
             "in every arm's frame B.\n"
             "move_arms moves arms together (time aligned), home_arms takes every arm home with the grippers open, "
             "handover passes a held object from one arm to another.\n"
             "A move that would bring two arms nearer than the clearance margin is refused (check arm_clearance): "
             "move the other arm away first, or move both with move_arms.")
ABOUT = (" This robot has several arms ({arms}). Every tool that moves or looks through an arm takes arm; positions "
         "are in that arm's own base frame B (robot://body gives each base in frame R).")
_OWN = r"(?<![\w.]){arm}\.L(\d+)\b"
_RAW = re.compile(r"(?<![\w./])L(\d+)\b")
_ANY = re.compile(r"(?<![\w.])([A-Za-z0-9_-]+)\.L(\d+)\b")
IMAGE_EXT = (".png", ".jpg", ".jpeg")


def clear_schema():
    return {"oneOf": [{"type": "boolean"}, {"type": "object", "properties": {"look": {"type": "string"}}, "required": ["look"], "additionalProperties": False}], "description": "true after your own look when you judge the way clear, or {look: id}: recorded in the result; it changes no speed"}


def tools(arms, lockstep_names=()):
    """The package's own tool entries for these arm names (TOOLS shows them with the placeholder ARM)."""
    arms = list(arms)
    a0, a1 = arms[0], arms[1] if len(arms) > 1 else arms[0]
    move = surfaces._obj({"tool": surfaces._str("the motion tool this arm runs", list(lockstep_names) or None),
                          "args": {"type": "object", "description": "that tool's own arguments, without arm"}},
                         ["tool"])
    arm_enum = surfaces._str("an arm name: " + ", ".join(arms), arms)
    return [
        surfaces._tool(
            "move_arms", "Move several arms at once, time aligned: one motion tool per arm (any tool that moves an "
            "arm, with its own arguments), all starting in the same control period; an arm that finishes first holds "
            "still while the others go on. Every arm's call is checked first: if any arm's call is refused, no arm "
            "moves. The answer holds each arm's own result, periods (control periods the move took) and "
            "started_period.",
            surfaces._obj({"clear": clear_schema(), "moves": {"type": "object", "properties": {a: move for a in arms},
                                     "additionalProperties": False,
                                     "description": "per arm name: {tool, args}; leave out an arm that stays still"}},
                          ["moves"]),
            {"moves": {a0: {"tool": "home", "args": {}}, a1: {"tool": "home", "args": {}}}},
            "take both arms home together"),
        surfaces._tool(
            "home_arms", "Take every arm to its home pose at the same time, the grippers opened first (open_grippers "
            "false keeps them as they are, so a held object stays held). Each arm's path is checked against the "
            "floor, itself and the other arms.",
            surfaces._obj({"clear": clear_schema(), "open_grippers": surfaces._bool("open every gripper first (default true)")}),
            {"open_grippers": True}, "open both grippers, then take both arms home together"),
        surfaces._tool(
            "handover", "Pass the object one arm holds (a pick said held true) to another arm: both tips meet at "
            "at_mm (default: the midpoint of the two bases, raised to a height both reach, said in the answer), the "
            "taker comes in open and closes on what stands out of the giver's jaws, its hold is checked, then the "
            "giver opens and backs off. held_by says which arm holds it after; a failed hold reopens nothing.",
            surfaces._obj({"giver": arm_enum, "taker": arm_enum,
                           "at_mm": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3, "description": "where the tips meet, mm; derived when omitted"},
                           "clear": clear_schema(), "frame": surfaces._str("the frame at_mm is in: R (default) or an arm name",
                                                  ["R"] + arms)}, ["giver", "taker"]),
            {"giver": a0, "taker": a1}, f"pass what arm {a0} holds to arm {a1}"),
    ]


TOOLS = tools((ARM_TOKEN + "_1", ARM_TOKEN + "_2"))


# ---------------------------------------------------------------------------------------------- journal per arm
class ArmJournal:
    """One arm's view of the session Journal: every record gains "arm"; frames go to frames/<arm>/ (packages name
    frames after per-executor counters, so two arms must not share a folder)."""

    def __init__(self, journal, arm):
        self._j, self.arm = journal, arm
        safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in str(arm)) or "arm"
        self._sub = safe
        self.frames_dir = Path(journal.dir) / "frames" / safe

    def append(self, kind, **fields):
        fields.setdefault("arm", self.arm)
        return self._j.append(kind, **fields)

    def save_frame(self, img, stem):
        from PIL import Image

        self.frames_dir.mkdir(parents=True, exist_ok=True)
        safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in stem)
        rel = Path("frames") / self._sub / f"{safe}.png"
        Image.fromarray(np.ascontiguousarray(np.asarray(img)[:, :, :3])).save(Path(self._j.dir) / rel)
        return str(rel)

    def __getattr__(self, name):
        return getattr(self.__dict__["_j"], name)


# ---------------------------------------------------------------------------------------------- look ids
def ids_out(obj, arm):
    """A result of arm `arm` with its wrist look ids L<n> written arm.L<n> (file paths are left alone)."""
    if isinstance(obj, str):
        if "/" in obj or obj.lower().endswith(IMAGE_EXT):
            return obj
        return _RAW.sub(lambda m: f"{arm}.L{m.group(1)}", obj)
    if isinstance(obj, dict):
        return {ids_out(k, arm) if isinstance(k, str) else k: ids_out(v, arm) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [ids_out(v, arm) for v in obj]
    return obj


def ids_in(obj, arm):
    """Arguments for arm `arm` with its own arm.L<n> back to L<n>."""
    rx = re.compile(_OWN.format(arm=re.escape(str(arm))))
    if isinstance(obj, str):
        return rx.sub(lambda m: f"L{m.group(1)}", obj)
    if isinstance(obj, dict):
        return {k: ids_in(v, arm) for k, v in obj.items()}
    if isinstance(obj, list):
        return [ids_in(v, arm) for v in obj]
    return obj


def _foreign(obj, arm, arms):
    """(the other arm, the id) of the first look id of another arm in obj, or None."""
    if isinstance(obj, str):
        for m in _ANY.finditer(obj):
            if m.group(1) in arms and m.group(1) != arm:
                return m.group(1), m.group(0)
        return None
    items = obj.values() if isinstance(obj, dict) else obj if isinstance(obj, list) else ()
    for v in items:
        hit = _foreign(v, arm, arms)
        if hit:
            return hit
    return None


# ---------------------------------------------------------------------------------------------- the session
class Router:
    """What the Session sees as its executor: one clock, no world of its own (each arm reports its motion)."""

    reports_motion = True
    world = None

    def __init__(self, bm):
        self.bimanual = bm
        self.executors = bm.exs
        grip = next((bm.exs[a].model.spec.get("gripper") for a in bm.arms if bm.exs[a].model.spec.get("gripper")),
                    None)
        self.model = types.SimpleNamespace(joint_names=list(bm.exs[bm.arms[0]].model.joint_names),
                                           spec={"gripper": grip})

    def sim_time(self):
        return self.bimanual.sim_time()


class ArmSessionMixin:
    """The arm check of every arm tool before the session's own argument check (module docstring)."""

    def _call(self, seq, name, request):
        bm = getattr(self, "bimanual", None)
        entry = self.tools.get(name) if (bm is not None and isinstance(name, str)) else None
        if entry is None or not entry.get("x-arm") or self._budget_spent():
            return super()._call(seq, name, request)
        args, bad = self._args(request, entry)
        if bad is None and isinstance(args, dict):
            args, _ = surfaces.normalize_args(entry, args)
            problem = bm.arm_problem(name, args)
            if problem is not None:
                self.calls_used += 1
                self._account(name, entry, None, self._now())
                return problem
        return super()._call(seq, name, request)


class Bimanual:
    """Everything one session over several arms holds (module docstring)."""

    def __init__(self, multi, journal, frames):
        self.multi, self.journal, self.frames = multi, journal, frames
        self.arms = tuple(multi.arms)
        self.dt = float(multi.dt)
        self.coord = Coordinator(multi)
        self.store = BW.Store(frames)
        cams = dict(multi.cameras() or {})
        self.fixed_cameras = [n for n, c in sorted(cams.items()) if c.get("mount") == "fixed"]
        self.views = {a: ArmView(multi, a, self.coord, frames) for a in self.arms}
        self.exs, self.entries, self.handlers, self.boxes = {}, {}, {}, {}
        self.session = None

    def sim_time(self):
        return float(self.multi.read(self.arms[0])["t"])

    def note(self, event, **fields):
        self.journal.append("bimanual", event=event, **jsonable(fields))

    def idle(self, seconds):
        """The world runs `seconds` with every arm holding its present command (think time, a wait)."""
        n = int(round(max(0.0, float(seconds)) / self.dt))
        stops = {}
        if n > 0:
            for a in self.arms:
                ex = self.exs[a]
                if ex._q_des is not None:
                    D.command_hold(ex)
            for k in range(n):
                self.coord._advance(1)
                for a in self.arms:
                    ex = self.exs[a]
                    sense = D.sense(ex)
                    reason = BC.check_idle(ex,sense)
                    if reason:
                        stops[a] = reason
                        D.command_hold(ex,sense.q)
        for a in self.arms:
            ex = self.exs[a]
            for fn in ex._idles:
                D.idle_hook(ex, fn, n * self.dt)
            if n > 0:
                ex.note_event("idle", seconds=round(n * self.dt, 3))
        result = {"said":"The arms held while time passed.","idled_s":round(n*self.dt,3),"periods":n}
        if stops:
            result['stopped'] = next(iter(stops.values()))
            result['arm_clearance'] = {a:self.exs[a].state['bimanual_clear'].get('stop') for a in stops}
            items = [r['bounds'] for r in result['arm_clearance'].values() if r]
            D.of(self.exs[next(iter(stops))]).report(result,items)
        return result

    # ------------------------------------------------------------------ calls through one arm
    def arm_problem(self, name, args):
        serving = [a for a in self.arms if name in self.entries[a]]
        arm = args.get("arm")
        listed = ", ".join(serving)
        if "arm" not in args:
            return refusal("arm", f"{name} moves or looks through one arm: say which with arm ({listed}); nothing ran",
                           f"for example {{\"arm\": \"{serving[0]}\", ...}}")
        if not isinstance(arm, str) or arm not in serving:
            return refusal("arm", f"no arm {arm!r} serves {name}: this robot's arms are {', '.join(self.arms)}; "
                           "nothing ran", f"arm is one of {listed}")
        hit = _foreign({k: v for k, v in args.items() if k != "arm"}, arm, self.arms)
        if hit:
            return refusal("arm", f"look {hit[1]} was taken through arm {hit[0]}, not arm {arm}; nothing ran",
                           f"send it with arm {hit[0]}, or look again through arm {arm}")
        cam = args.get("camera")
        if isinstance(cam, str):
            for b in self.arms:
                if b != arm and cam == self.views[b].wrist_name:
                    return refusal("arm", f"camera {cam} rides on arm {b}, not arm {arm}; nothing ran",
                                   f"look through it with arm {b}")
        return None

    def prepare(self, arm, name, args):
        """(kwargs, None) for one arm's call of tool `name` as the session would check it, or (None, refusal)."""
        entry = self.entries[arm].get(name)
        if entry is None:
            return None, refusal("unknown_tool", f"arm {arm} has no tool {name!r}", "send list for the tools")
        args, _ = surfaces.normalize_args(entry, args)
        problem = surfaces.check_args(entry, args, list(self.entries[arm].values()))
        if problem is not None:
            return None, {"refused": problem}
        args = surfaces.coerce(entry, args)
        rename = entry.get("rename", {})
        return ids_in({rename.get(k, k): v for k, v in args.items()}, arm), None

    def call_raw(self, arm, name, kwargs):
        """One arm's call of tool `name` with checked kwargs: that arm's handler or executor method, as the
        operator's own call (its hooks run). The result carries arm and arm.L<n> look ids."""
        entry = self.entries[arm][name]
        fn = self.handlers[arm].get(name) or getattr(self.exs[arm], entry.get("method", name), None)
        if not callable(fn):
            return refusal("unavailable", f"{name} is not available on arm {arm}", "send list for the tools")
        out = fn(**kwargs)
        if not isinstance(out, dict):
            out = {"value": out}
        out = ids_out(out, arm)
        out["arm"] = arm
        return out

    def route(self, name, kwargs):
        """The session's handler of an arm tool (arguments checked by the session, arm still among them)."""
        kwargs = dict(kwargs)
        arm = kwargs.pop("arm")
        cam = kwargs.get("camera")
        if isinstance(cam, str) and cam == self.views[arm].wrist_name:
            kwargs["camera"] = "wrist"
        return self.call_raw(arm, name, ids_in(kwargs, arm))


# ---------------------------------------------------------------------------------------------- building it
def _packages(packages, problems):
    """The round two packages core.load reads from PACKAGES, then every other name as rcpx.r2.<name>; lean and
    carry are dropped (module docstring)."""
    from rcpx.r2 import core

    names = [n.strip().lower() for n in str(packages or "").split(",") if n.strip()]
    keep = [n for n in names if n not in UNSUPPORTED] or ["none"]
    for n in names:
        if n in UNSUPPORTED:
            problems[n] = "not supported with several arms: it wraps the session itself; dropped"
    mods = [m for m in core.load(",".join(keep)) if m.__name__.rsplit(".", 1)[-1] not in UNSUPPORTED]
    problems.update({k: str(v).strip().splitlines()[-1] for k, v in core.load.problems.items()})
    absent = list(core.load.absent)
    for n in keep:
        if n in core.PACKAGES or n in NOT_PACKAGES or not re.fullmatch(r"[a-z_][a-z0-9_]*", n):
            continue
        if any(m.__name__ == "rcpx.r2." + n for m in mods):
            continue
        try:
            mods.append(importlib.import_module("rcpx.r2." + n))
        except ModuleNotFoundError as exc:
            (absent.append(n) if exc.name == "rcpx.r2." + n else problems.__setitem__(n, f"{exc}"))
        except Exception as exc:
            problems[n] = f"{type(exc).__name__}: {exc}"
    return mods, absent


def _make_executor(model, view, journal, world, cfg, mods, fixed, problems):
    from rcpx.r2 import core

    while True:
        try:
            return core.make_executor(model, view, journal=journal, world=world, cfg=cfg, mods=mods), mods
        except TypeError:   # a package whose Mixin clashes: dropped and reported, never fatal
            bad = next((m for m in mods if m not in fixed and not _class_ok(m)), None)
            if bad is None:
                raise
            problems[bad.__name__.rsplit(".", 1)[-1]] = "its Mixin does not join the executor class"
            mods = [m for m in mods if m is not bad]


def _class_ok(mod):
    from rcpx.r2 import core

    try:
        core.executor_class([mod])
        return True
    except TypeError:
        return False


def _with_arm(entry, serving):
    e = copy.deepcopy(entry)
    args = e.setdefault("args", {"type": "object", "properties": {}, "required": []})
    props = args.get("properties", {})
    arm = surfaces._str("which arm: " + ", ".join(serving), serving)
    args["properties"] = {"arm": arm, **{k: v for k, v in props.items() if k != "arm"}}
    args["required"] = ["arm"] + [k for k in args.get("required", []) if k != "arm"]
    ex = e.setdefault("example", {"args": {}, "means": ""})
    ex["args"] = {"arm": serving[0], **{k: v for k, v in (ex.get("args") or {}).items() if k != "arm"}}
    e["x-arm"] = True
    return e


def _merge_tools(bm, surfaces_by_arm):
    order = []
    for a in bm.arms:
        for t in surfaces_by_arm[a]["tools"]:
            if t["name"] not in order:
                order.append(t["name"])
    merged = []
    for name in order:
        serving = [a for a in bm.arms if name in bm.entries[a]]
        merged.append(_with_arm(bm.entries[serving[0]][name], serving))
    return merged


def _same_or_by_arm(fns, arms):
    def read(**kw):
        docs = {}
        for a in arms:
            if a in fns:
                docs[a] = jsonable(ids_out(fns[a](**kw), a))
        first = next(iter(docs.values()))
        if all(d == first for d in docs.values()):
            return first
        return {"by_arm": docs, "note": "each arm's own answer; positions in its own frame B"}
    return read


def _templated(fns, arms):
    def read(**kw):
        for a in arms:
            pre = f"{a}."
            if a in fns and any(isinstance(v, str) and v.startswith(pre) for v in kw.values()):
                inner = {k: (v[len(pre):] if isinstance(v, str) and v.startswith(pre) else v) for k, v in kw.items()}
                out = jsonable(ids_out(fns[a](**inner), a))
                return {**out, "arm": a} if isinstance(out, dict) else out
        last = None
        for a in arms:
            if a in fns:
                last = jsonable(ids_out(fns[a](**kw), a))
                if not (isinstance(last, dict) and "refused" in last):
                    return {**last, "arm": a} if isinstance(last, dict) else last
        return last
    return read


def build_session(multi, specs, *, ep_dir, task_doc, budget, packages, session_base, claim, law_module=None,
                  cfg=None, **session_kw):
    """The session (module docstring). session_kw goes to session_base as it is (think_s, claim_check, ...)."""
    from rcpx.context import Journal, practices_text, read_journal
    from rcpx.model import RobotModel
    from rcpx.model_body import with_installation
    from rcpx.r2 import core

    arms = tuple(multi.arms)
    if len(set(arms)) != len(arms) or not arms or any(a in BS.RESERVED or a == "bimanual" for a in arms):
        raise ValueError(f"arm names must be distinct and none of {BS.RESERVED + ('bimanual',)}: {arms}")
    cfg = dict(cfg or {})
    own = dict(cfg.pop("bimanual", None) or {})
    ep_dir = os.path.abspath(ep_dir)
    journal = Journal(ep_dir)
    frames = BW.Frames({a: multi.T_R_B(a) for a in arms}, errors_mm={a: getattr(multi.arm_extra(a), "error_mm", None) for a in arms})
    bm = Bimanual(multi, journal, frames)
    problems = {}
    mods, absent = _packages(packages, problems)
    scene_mod = importlib.import_module("rcpx.r2.scene_camera")
    tail = [scene_mod] + ([law_module] if law_module is not None else []) + [BC]
    mods = [m for m in mods if m not in tail] + tail
    models = {a: RobotModel(specs[a]) for a in arms}
    bm.clear = BC.Clearance(multi, frames, {a: RobotModel(specs[a]) for a in arms})
    think_s = float(session_kw.get("think_s", 0.0) or 0.0)
    tables = {}
    for a in arms:
        aj = ArmJournal(journal, a)
        world = BW.ArmWorld(aj, a, bm.store)
        ex, mods = _make_executor(models[a], bm.views[a], aj, world, cfg.get(a), mods, tail, problems)
        ex._bm_clear, ex._bm_arm = bm.clear, a
        world.clock = bm.sim_time
        ex.set_context(devices=DEVICES_DOC, task=task_doc, experience_path=None, think_s=think_s)
        bm.exs[a] = ex
        bm.clear.executors[a] = ex

        def body(ex=ex, model=models[a]):
            doc = with_installation(model.describe(), kind="fixed", standing_surface=ex.standing_surface(),
                                    kind_how=INSTALL_HOW)
            doc["base"]["measured_give"] = ex.base_give
            return doc

        base = {"robot://practices": practices_text, "robot://task": lambda: dict(task_doc), "robot://body": body,
                "robot://world": world.to_json, "robot://devices": lambda: dict(DEVICES_DOC),
                "robot://devices/{id}": lambda id: refusal("unknown_device", f"no device {id!r}: this task has no "
                                                           "device knowledge", "find what the task names by looking"),
                "robot://journal": lambda: {}}
        tables[a] = core.assemble(ex, mods, surfaces.get("vnext"), base, {})
        surf, res, handlers, box = tables[a]
        bm.entries[a] = {t["name"]: t for t in surf["tools"]}
        bm.handlers[a] = handlers
        bm.boxes[a] = box
    shared = bm.exs[arms[0]].state.get("scene_camera")
    for a in arms[1:]:      # one scene camera state: C ids once per session (module docstring)
        own_st = bm.exs[a].state.get("scene_camera")
        if isinstance(shared, dict) and isinstance(own_st, dict):
            own_st["last_echo"] = shared.setdefault("last_echo", {})
            bm.exs[a].state["scene_camera"] = shared
    surf0, res0 = tables[arms[0]][0], tables[arms[0]][1]
    merged = _merge_tools(bm, {a: tables[a][0] for a in arms})
    names = BS.lockstep_tools(bm, arms[0])
    merged += tools(arms, names)
    surface = dict(surf0, tools=merged, about=str(surf0.get("about", "")) + ABOUT.format(arms=", ".join(arms)))
    surface["resources"] = dict(surf0.get("resources", {}))
    resources = {}
    uris = []
    for a in arms:
        uris += [u for u in tables[a][1] if u not in uris]
    for uri in uris:
        fns = {a: tables[a][1][uri] for a in arms if uri in tables[a][1]}
        resources[uri] = _templated(fns, arms) if "{" in uri else _same_or_by_arm(fns, arms)
    per = {u: {a: tables[a][1][u] for a in arms if u in tables[a][1]} for u in ("robot://body", "robot://world",
                                                                                 "robot://cameras")}
    resources["robot://body"] = lambda: BB.body_doc(bm, {a: f() for a, f in per["robot://body"].items()})
    resources["robot://world"] = lambda: BW.world_doc(bm.store, {a: ids_out(jsonable(f()), a)
                                                                 for a, f in per["robot://world"].items()})
    if per["robot://cameras"]:
        resources["robot://cameras"] = lambda: BB.cameras_doc(bm, {a: f() for a, f in per["robot://cameras"].items()})
    prac = res0.get("robot://practices")
    if prac is not None:
        def practices(prac=prac):
            text = prac()
            add = PRACTICES if core.PRACTICES_HEADING in str(text) else core.PRACTICES_HEADING + "\n" + PRACTICES
            return str(text).rstrip() + "\n" + add + "\n" if isinstance(text, str) else text
        resources["robot://practices"] = practices

    def journal_doc():
        recs = read_journal(journal.path)
        return {"count": len(recs), "shown": min(len(recs), JOURNAL_TAIL), "records": recs[-JOURNAL_TAIL:]}
    resources["robot://journal"] = journal_doc
    handlers = {e["name"]: (lambda name=e["name"]: (lambda **kw: bm.route(name, kw)))() for e in merged
                if e.get("x-arm")}
    for name, fn in (("move_arms", BS.move_arms), ("home_arms", BS.home_arms), ("handover", BH.handover)):
        handlers[name] = (lambda fn=fn: (lambda **kw: fn(bm, bm.session, **kw)))()
    cls = type("SessionBimanual", (ArmSessionMixin, session_base), {})
    kw = dict(session_kw)
    params = inspect.signature(session_base.__init__).parameters
    if claim is not None:
        kw["claim"] = claim
    if "idle" in params and "idle" not in kw:
        kw["idle"] = bm.idle
    session = cls(Router(bm), surface, resources, dict(budget), journal=journal, ep_dir=ep_dir, handlers=handlers,
                  **kw)
    session.bimanual = bm
    session.multi = multi
    if getattr(session, "ext_backend", None) is None:
        session.ext_backend = multi
    bm.session = session
    for a in arms:
        bm.boxes[a].session = session
    rec = {"arms": list(arms), "bases_R": {a: frames.base_doc(a) for a in arms}, "requested": packages,
           "loaded": [m.__name__.rsplit(".", 1)[-1] for m in mods], "absent": absent,
           "problems": {**problems, **{f"{a}:{k}": str(v) for a in arms for k, v in bm.exs[a].r2_problems.items()}},
           "clearance": "derived from both live speeds and stated base errors", "fixed_cameras": bm.fixed_cameras,
           "wrist_cameras": {a: bm.views[a].wrist_name for a in arms}}
    with open(os.path.join(ep_dir, "bimanual.json"), "w") as f:
        json.dump(rec, f, indent=2, sort_keys=True, default=str)
    return session


__all__ = ["ArmJournal", "ArmSessionMixin", "Bimanual", "ID", "PRACTICES", "Router", "TOOLS", "build_session",
           "ids_in", "ids_out", "tools"]
