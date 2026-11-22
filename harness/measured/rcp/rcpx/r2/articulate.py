# SPDX-License-Identifier: AGPL-3.0-only
"""Pull follows a held part. Twist ramps measured torque about the tool axis.

Every phase carries the operator intent and reports its derived binding bound.
"""

import importlib
import sys

import numpy as np

from rcpx import surfaces
from rcpx.r2 import articulate_bounds as A, articulate_grasp as C
from rcpx.r2.articulate_grasp import refusal
from rcpx.control import Executor
from rcpx.control_base import primitive
from rcpx.r2 import core
from rcpx.r2.articulate_follow import unit
from rcpx.r2.articulate_grasp import GraspMixin, StepsSpent, UP, direction
from rcpx.r2.articulate_pull import PullMixin
from rcpx.r2.articulate_tools import PRACTICES, TOOLS
from rcpx.r2.articulate_twist import TwistMixin
from rcpx.r2.place_hand import Jaws

ID = "A1"
OWN = ("pull", "twist")


def clashes(mods, base_tools=None):
    hits = []
    for tool in surfaces.VNEXT_TOOLS if base_tools is None else base_tools:
        if tool.get("name") in OWN and "extends" not in tool:
            hits.append(("rcpx.surfaces", tool["name"]))
    for mod in mods:
        if mod.__name__ in (__name__, "rcpx.r2.articulate_tools"):
            continue
        for tool in getattr(mod, "TOOLS", ()) or ():
            if "extends" not in tool and tool.get("name") in OWN:
                hits.append((mod.__name__, tool["name"]))
    hits.extend(("rcpx.control.Executor", name) for name in OWN if hasattr(Executor, name))
    return list(dict.fromkeys(hits))


def clash_text(hits):
    return "articulate not loaded: " + "; ".join(f"tool name {name} already defined by {where}" for where, name in hits)


def _scan():
    mods = []
    for name in list(core.PACKAGES) + ["place", "scene_camera", "mobile"]:
        try:
            mods.append(importlib.import_module("rcpx.r2." + name))
        except Exception:
            pass
    mods.extend(m for name, m in list(sys.modules.items()) if name.startswith("rcpx.r2.") and m is not None)
    hits = clashes(mods)
    if hits:
        raise ImportError(clash_text(hits))


_scan()


def number(value, name, *, positive=False):
    if value is None:
        return None, None
    try:
        n = float(value)
        if not isinstance(value, (bool, str)) and np.isfinite(n) and (not positive or n > 0):
            return n, None
    except (TypeError, ValueError):
        pass
    return None, refusal("args", f"{name} must be a finite number" + (" above zero" if positive else ""),
                         "give the argument again as a plain number")


class Mixin(GraspMixin, PullMixin, TwistMixin):
    def _ar_common(self, target, width_mm):
        off = self.state["articulate"].get("off")
        if off:
            return None, refusal("not_built", off, "rename the other package's tool")
        if self._ar_jaws is None:
            return None, refusal("no_gripper", "this arm has no gripper", "use touch or press")
        if self.state.get("place", {}).get("held"):
            return None, refusal("holding", "the jaws already hold a part", "let go before gripping another part")
        data, bad = self._ar_target(target)
        if bad:
            return None, bad
        if width_mm is not None:
            w, bad = number(width_mm, "width_mm", positive=True)
            if bad:
                return None, bad
            if w > self._ar_jaws.max_m * 1000:
                return None, refusal("too_wide", f"the model's jaws open only {self._ar_jaws.max_m * 1000:.0f} mm",
                                     "pull(target='handle', width_mm=None)")
            data["width_mm"] = w
        else:
            data["width_mm"] = None
        return data, None

    def _ar_run(self, fn, *args):
        return C.run(self, fn, *args)

    def _ar_intent(self, data, intent):
        B = A.of(self)
        B.begin("call")
        self._ar_phases = []
        if intent.get("clear") is not None:
            word = B.word(intent)
            if word.get("refused"):
                return word["refused"]
        ref = data["target"]["from"] if data["entry"] is not None else data["p"]
        data.update(ref=ref, part=B.thing(ref), intent=intent)
        return None

    def _ar_result(self, result, items):
        phases = getattr(self, "_ar_phases", [])
        if phases:
            phases[-1]["seconds"] = C.sense(self).t - self._ar_phase_start
        result.update(periods=sum(p["periods"] for p in phases), phases=phases)
        return A.of(self).report(result, items)

    def _ar_failed_grasp(self, plan, grasp, tool, start, end):
        if grasp.get("refused", {}).get("check") == "not_held":
            return grasp
        stop = "steps_spent" if self._ar_spent() else grasp.get("stop", grasp.get("stopped", "gate"))
        if end == start and "refused" in grasp:
            return {"said": grasp.get("said", "The grasp was refused."), **grasp}
        s_stop = C.sense(self)
        hold = self._ar_hold()
        C.stop(self)
        self._ar_ease()
        C.begin(self, "release")
        released, release = self._ar_release(plan)
        self._ar_stage(tool, "release", release)
        evidence = {k: v for k, v in grasp.items() if k not in ("s", "felt")}
        return {"said": f"The grasp stopped at {stop}.", "stopped": stop, "held": False,
                "released": released, "refusal": evidence, "approach": plan["approach"],
                "target": plan["target"], "tip_mm": (s_stop.tip * 1000).tolist(), "hold": {"grasp": hold, "stop": hold},
                "world_s": {"grasp": float(end - start), tool: 0.0,
                "release": float(C.sense(self).t - end)}}

    @primitive
    @C.end_word_call
    def pull(self, target, travel_mm=None, max_n=None, width_mm=None, way=None, along=None, clear=None):
        data, bad = self._ar_common(target, width_mm)
        if bad:
            return bad
        travel, bad = number(travel_mm, "travel_mm", positive=True)
        if bad:
            return bad
        force, bad = number(max_n, "max_n", positive=True)
        if bad:
            return bad
        bad = self._ar_intent(data, {"travel": None if travel is None else travel / 1000,
                                     "max_n": force, "clear": clear, "move": True})
        if bad:
            return bad
        if travel is None and data["part"].end is not None:
            end = np.asarray(data["part"].end, float)
            travel = float(np.linalg.norm(end - data["p"])) * 1000 if end.ndim else float(end) * 1000
            data["intent"]["travel"] = travel / 1000
            data["offered_travel"] = A.item("travel", travel / 1000, "m", "the end stored for this part", "contact")
        a0, source, bad = self._ar_approach(data["p"], way, True)
        if bad:
            return bad
        if along is None or isinstance(along, str) and along == "horizontal":
            side = unit(np.cross(UP, a0))
        elif isinstance(along, str) and along == "vertical":
            side = UP.copy()
        else:
            side = direction(along) if not isinstance(along, str) else None
        if side is None or not np.any(side) or abs(side @ a0) > 1 - 1e-6:
            return refusal("args", "along must be horizontal, vertical or a direction across the approach",
                           "give the handle's long side")
        J = self._ar_jaws
        opening = A.opening_m(A.of(self), data["part"], J, None if data["width_mm"] is None else data["width_mm"] / 1000)
        plan, bad = self._ar_plan(data, a0, source, side, J.frac_for_width(opening), True)
        if bad:
            return bad
        fs = A.of(self).force("follow", data["part"], C.sense(self).q, -a0, data["intent"])
        if getattr(fs, "refused", None):
            return fs.refused
        return self._ar_run(self._ar_pull_impl, plan, None if travel is None else travel / 1000,
                            fs, force is not None and fs.ceiling_n < force)

    @primitive
    @C.end_word_call
    def twist(self, target, angle_deg, max_nm=None, grip="close", width_mm=None, approach=None, across=None, clear=None):
        data, bad = self._ar_common(target, width_mm)
        if bad:
            return bad
        angle, bad = number(angle_deg, "angle_deg")
        if bad or angle is None or angle == 0 or abs(angle) > 360:
            return bad or refusal("args", "angle_deg must be nonzero", "give a signed angle up to 360 degrees")
        torque, bad = number(max_nm, "max_nm", positive=True)
        if bad:
            return bad
        if grip not in ("close", "pinch"):
            return refusal("args", "grip must be close or pinch", "close squeezes; pinch rests lightly")
        bad = self._ar_intent(data, {"travel": np.radians(angle), "max_nm": torque, "clear": clear})
        if bad:
            return bad
        t, source, bad = self._ar_approach(data["p"], approach, False)
        if bad:
            return bad
        if across is None or isinstance(across, str) and across == "horizontal":
            side = unit(np.cross(UP, t))
        elif isinstance(across, str) and across == "vertical":
            side = UP.copy()
        else:
            side = direction(across) if not isinstance(across, str) else None
        if side is None or np.linalg.norm(side - (side @ t) * t) < 1e-6:
            return refusal("args", "across must be horizontal, vertical or a direction across the tool axis",
                           "give the jaws' closing direction")
        J = self._ar_jaws
        opening = A.opening_m(A.of(self), data["part"], J, None if data["width_mm"] is None else data["width_mm"] / 1000)
        plan, bad = self._ar_plan(data, t, source, side, J.frac_for_width(opening), False, angle, across is None)
        if bad:
            return bad
        ceiling = plan["turn"]["ceiling_nm"]
        effective = ceiling if torque is None else min(torque, ceiling)
        plan["clamped_nm"] = torque is not None and torque > ceiling
        plan["torque_item"] = A.item("torque", effective, "N m", "joint torque rating less the gravity torque this pose needs")
        return self._ar_run(self._ar_twist_impl, plan, angle, effective, grip)


def setup(ex):
    ex.state["articulate"] = {"off": None, "last": None}
    ex._ar_jaws = getattr(ex, "_pl_jaws", None)
    if ex._ar_jaws is None and ex.model.spec.get("gripper"):
        ex._ar_jaws = Jaws(ex.model)
    hits = clashes(ex.mods)
    if hits:
        text = clash_text(hits)
        ex.r2_problems["articulate"] = text
        ex.state["articulate"]["off"] = text
        C.journal(ex, "r2_problem", package="articulate", where="setup", text=text)


def FILTER(surface, resources, handlers, ex=None):
    if ex is not None and ex.state.get("articulate", {}).get("off"):
        surface = {**surface, "tools": [t for t in surface["tools"] if t.get("x-package") != "articulate"]}
    return surface, resources, handlers
