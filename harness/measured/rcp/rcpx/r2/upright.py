# SPDX-License-Identifier: AGPL-3.0-only
"""Turn a held thing, tip a standing thing and read an axis."""
import math

import numpy as np

from rcpx.r2.shove_bounds import refuse as refusal
from rcpx.control_base import _mm, primitive
from rcpx.r2.shove_args import push_args, resolve_point, finite
from rcpx.r2 import shove_bounds as b
from rcpx.r2.shove_bounds import whole
from rcpx.r2.shove_law import MM, UP, PushLawMixin, install
from rcpx.r2.upright_tools import PRACTICES, TOOLS
from rcpx.r2.upright_turn import WRIST_BEARINGS_DEG, TurnMixin, tilt_deg

ID = "N7"


def _held(ex):
    return bool((ex.state.get("place") or {}).get("held"))


class Mixin(TurnMixin, PushLawMixin):
    @primitive
    @whole
    def set_upright(self, up_axis=None, axis_from=None, axis_to=None, place_point_mm=None, place_label=None,
                    release_mm=None, clear=None):
        if not self.model.spec.get("gripper"):
            return refusal("no_gripper", "this robot has no gripper: it holds nothing to turn", "push it instead")
        if not _held(self):
            return refusal("nothing_held", "nothing is held: there is nothing between the jaws to turn",
                           "pick it first (pick); a pick that says held false holds nothing")
        B = b.of(self)
        intent = {"clear": clear}
        if clear is not None:
            word = B.word(intent)
            if word.get("refused"):
                return word["refused"]
        a_T, f_T, t_T, bad = self._up_axis(up_axis, axis_from, axis_to)
        if bad:
            return bad
        if (place_point_mm is None) == (place_label is None):
            return refusal("place_point", "give where to set it down as exactly one of place_point_mm or "
                           "place_label", "place_point_mm [x, y, z] mm: the spot on the surface")
        place, _, bad = resolve_point(self, place_point_mm if place_label is None else place_label,
                                      "place_point_mm" if place_label is None else "place_label")
        if bad:
            return bad
        try:
            ok, heights = self._up_support_ok(place)
        except ValueError as exc:
            return refusal("support", str(exc), "touch the surface where it is to stand, or state the support plane")
        if not ok:
            known = ", ".join(f"{h * MM:.0f}" for h in sorted(set(heights)))
            return refusal("support", f"the place point's height (z {place[2] * MM:.0f} mm) is on no surface the hand "
                           f"touched or a look located near it (known heights: {known} mm)",
                           "give the height of the surface there: touch it, or locate a point on it")
        if release_mm is not None and not (finite(release_mm) and float(release_mm) >= 0):
            return refusal("release_mm", f"release_mm must be finite and at or above zero, got {release_mm!r}",
                           "leave it out to lower until the surface takes the thing")
        rel = None if release_mm is None else float(release_mm) / MM
        plan = self._up_plan(a_T, f_T, t_T, place, rel)
        if plan is None:
            return refusal("turn_not_reachable", "no arm pose inside the joint limits holds the thing that way up "
                           f"over the place point (tried {len(WRIST_BEARINGS_DEG)} wrist bearings, each clear of "
                           "the surface)",
                           "a place point nearer the base, or pick it across another side so the turn is smaller")
        f, bad = self._up_run(plan, a_T, f_T, t_T, place, rel, intent=intent)
        return upright_report(f, bad)

    @primitive
    @whole
    def tip_over(self, point_mm=None, label=None, toward_mm=None, toward_label=None, direction_deg=None,
                 height_mm=None, distance_mm=None, contact=None, standoff_mm=None, clear=None):
        a, bad = push_args(self, point_mm, label, toward_mm, toward_label, direction_deg,
                           distance_mm, contact, height_mm, None,
                           standoff_mm, clear=clear, allow_derived_distance=True)
        if bad:
            return bad
        f, bad = self._sv_push_run(a["point"], a["d"], a["distance"], a["height"], a["frac"], a["force"],
                                   a["standoff"], None, False, "tip", intent=a["intent"],
                                   distance_stated=a["distance_stated"])
        if bad is not None and not f:
            return bad
        pushed = round(float(f.get("pushed_m", 0.0)) * MM, 1)
        tipped = f.get("stopped") == "lost_contact" and f.get("pushed_m", 0) > f.get("moved_min_m", 0) and f.get("followed") and not f.get("slipped")
        look = f.get("look") or {}
        lk = f" Look {look['look_id']} shows how it lies." if look.get("look_id") else ""
        stop = f.get("stopped", "refused")
        said = ("Tipped it over, as far as the push can tell: after " if tipped else "Did not tip it over: ") + {
            "lost_contact": (f"{pushed:.0f} mm the thing stopped pushing back and fell away from the hand." if tipped
                             else f"contact was lost after {pushed:.0f} mm ({f.get('why', 'it slid away')})."),
            "done": f"it went the whole {a['distance'] * MM:.0f} mm without falling (it slid, or the push was too "
                    "low).",
            "force": f"the push force reached its limit after {pushed:.0f} mm (held, blocked or too heavy).",
            "no_contact": "the hand felt nothing along the push.",
            "on_top": "the hand came down on the thing itself, not behind it (give a larger standoff_mm).",
            "too_tall": str(f.get("why", "the thing is too tall for this hand")) + ".",
        }.get(stop, f"the push stopped ({stop}) after {pushed:.0f} mm.")
        out = {"said": said + lk, "tipped": bool(tipped), "pushed_mm": pushed,
               "contact_force_n_peak": f.get("peak_n", 0.0), "stopped": stop, "look": look, "next_stroke_mm": f.get("next_stroke_mm")}
        for k in ("met_at_mm", "push_height_mm", "height_note", "top_z_mm", "support_z_mm", "support",
                  "hand_tilt_deg", "direction_deg", "world_s", "why", "periods", "bounds", "phases", "next_stroke_mm"):
            if f.get(k) is not None:
                out[k] = f[k]
        if bad is not None:
            out["refused_after_moving"] = bad.get("refused")
            out["said"] += " A later move was refused: " + str((bad.get("refused") or {}).get("why", ""))
        if a["label"] and f.get("met_at_mm") is not None:
            ids = [e["id"] for e in self.world.query(label=a["label"]) if not e.get("stale")]
            self.world.mark_stale(ids, "tipped over: it no longer stands where it was located", source="upright")
        return out

    @primitive
    def axis_tilt(self, axis_to=None, axis_from=None):
        st = self.state["upright"]
        if axis_to is None:
            return refusal("axis_to", "give axis_to: a point located again after the thing was set down",
                           "mark its upper end in two looks, triangulate, and give that label")
        to, _, bad = resolve_point(self, axis_to, "axis_to")
        if bad:
            return bad
        for value in (axis_to, axis_from):
            if isinstance(value, str) and st.get("set_call") is not None:
                e = (self.world.query(label=value) or [{}])[-1]
                if int((e.get("observed_at") or {}).get("call", 0)) <= int(st["set_call"]):
                    return refusal("stale_mark", f"{value!r} was located before the thing was last set down",
                                   "mark it again in two looks and triangulate, then measure")
        if axis_from is None:
            if st.get("set_down") is None:
                return refusal("axis_from", "no set_upright has set a thing down: give axis_from",
                               "axis_from: a located point at the thing's lower end")
            frm = np.asarray(st["set_down"], dtype=float)
        else:
            frm, _, bad = resolve_point(self, axis_from, "axis_from")
            if bad:
                return bad
        v = to - frm
        if float(np.linalg.norm(v)) <= b.axis_min(b.of(self), frm, to):
            return refusal("axis", "the two points overlap within their position bands: no axis follows", "two points a few cm apart")
        t = tilt_deg(v)
        lean = math.degrees(math.atan2(float(v[1]), float(v[0])))
        return {"said": f"The axis leans {t:.1f} deg from up" + (f", toward {lean:.0f} deg." if t >= 1.0 else "."),
                "tilt_deg": round(t, 1), "lean_bearing_deg": round(lean, 1), "from_mm": _mm(frm), "to_mm": _mm(to)}


def upright_report(f, bad):
    if bad is not None and not f:
        return bad
    stop = f.get("stopped", "refused")
    landed = bool(f.get("landed"))
    look = f.get("look") or {}
    lk = f" Look {look['look_id']} looks down at the spot." if look.get("look_id") else ""
    if stop in ("touchdown", "release_height", "no_touchdown"):
        how = {"touchdown": "the surface took its weight", "release_height": "it was let go at the release height",
               "no_touchdown": "nothing took its weight down to the lowest the lower goes, and it was let go there "
                               "(it may have dropped a little)"}[stop]
        said = (f"Set it down upright: turned {f.get('turned_deg', 0):.0f} deg, {how}, the jaws opened and the hand "
                f"rose; as the jaws held it the axis stood {f.get('tilt_deg_held', 0):.1f} deg from up.")
    elif stop == "dropped":
        said = "Lost the thing while lowering it: the jaws closed on nothing; look where it fell."
    else:
        said = f"Stopped before setting it down ({stop}): " + str(f.get("why", "")) + "; the thing is still held."
    done = stop in ("touchdown", "release_height", "no_touchdown")
    out = {"said": said + lk, "placed": done, "landed": landed, "held": not done and stop != "dropped",
           "stopped": stop, "look": look,
           "tilt_deg_after": None,
           "tilt_how": "locate the upper end again (two looks, triangulate) and call axis_tilt to measure it"}
    for k in ("set_down_mm", "tilt_deg_held", "turned_deg", "wrist_bearing_deg", "world_s", "why", "periods", "bounds", "phases", "extent_note"):
        if f.get(k) is not None:
            out[k] = f[k]
    if bad is not None:
        out["refused_after_moving"] = bad.get("refused")
    return out


def record_grasp(ex):
    """Post hook on pick: the tool pose when the jaws closed (the pose after the test lift, less the lift)."""
    def fn(name, kwargs, result):
        if not isinstance(result, dict) or not result.get("held"):
            return None
        s = b.sense(ex)
        lift = float(((result.get("evidence") or {}).get("test_lift_mm")) or 0.0) / MM
        ex.state["upright"]["grasp"] = {"tip": (s.tip - UP * lift).tolist(), "R": s.R.tolist(),
                                        "call": int(ex.world.now.get("call", 0))}
        return None
    return fn


def lower_tick(ex):
    def fn(s):
        w = ex.state["upright"].get("watch")
        return None if w is None else w(s)
    return fn


def mark_set(ex):
    def fn(name, kwargs, result):
        if isinstance(result, dict) and result.get("placed"):
            ex.state["upright"]["set_call"] = int(ex.world.now.get("call", 0))
        return None
    return fn


def setup(ex):
    install(ex)
    ex.state["upright"] = {"grasp": None, "watch": None, "set_down": None, "set_call": None}
    ex.add_post(record_grasp(ex), names=["pick"])
    ex.add_post(mark_set(ex), names=["set_upright"])
    ex.add_tick(lower_tick(ex))


__all__ = ["ID", "Mixin", "PRACTICES", "TOOLS", "setup"]
