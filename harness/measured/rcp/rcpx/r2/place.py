# SPDX-License-Identifier: AGPL-3.0-only
"""Package place (N4), round six (the design notes, packages PICK and PLACE; contracts C8, C11, C12, C14): picking
up, carrying and setting down FREE things with a gripper, for any arm with a gripper and any thing, with no model
of the thing given.

A spot is {"look": id, "px": [u, v]} (a pixel anywhere on the thing or the place), a located label, or [x, y, z]
mm; each is resolved through rcpx.r2.place_point when that package is there (a Seen with the spot it rests on),
with no motion and no control period; else a label or a point is taken as it is.

Tools (Mixin primitives), each one sentence first (said), then the facts:
- pick(pick, grasp, ...): the take is rcpx/r2/place_pick.py's (once per item: this file never takes again).
  Refused before anything moves: no gripper, already holding, no spot, a width stated wider than the jaws open.
- carry(to): rcpx/r2/place_carry.py; ONE stream up to the layer, across and down to the carry height over the
  place. What is held is the core's one record (rcpx/control_hold.py): a push on the carried thing or a lost
  hold ends the stream with the core's own words.
- place(on | in | to, release): the carry, then the way down as a motion of its own, an expected contact
  (rcpx/r2/place_set.py): it ends where the thing lands, or at the planned height with the drop said; into an
  opening the hand stops over the rim; a release above only on the operator's word, its height stated; under a
  cover the space book holds through the open side (rcpx/r2/place_side.py). Before the jaws open the hand is
  held against the plan. Arrival is evidence (where the jaws opened, what the same view shows), never a colour.
- put(pick, to | on | in, items): the whole step, or several in order inside one budget. The spot of both ends
  is resolved before anything moves (a place that cannot be found costs no pick and no motion; a take whose
  pixel the look shows on an empty place is ANSWERED, stopped "answered", with what the look shows near it:
  rcpx/r2/place_near.py, the design rulings); before each item
  its control periods are counted as the plan will run (rcpx/r2/place_count.py) and compared with those left:
  an item that does not fit is not started.

No tool takes or passes a speed, a force or a length limit: every move is planned by rcpx.bounds for its kind
(carry, lower, lift), the contact stop armed, and each result names the bound that set the pace, the strike of
each motion and the doubt it acted on. The operator's word (clear) is information: it is recorded and changes
no speed and no guard. Jaw geometry: rcpx/r2/place_hand.py. Nothing here reads ground truth or names a robot
or a thing.
"""

import numpy as np

from rcpx import bounds, control_hold
from rcpx.context import refusal
from rcpx.control_base import primitive
from rcpx.r2 import place_near, place_spot
from rcpx.r2.place_carry import CarryMixin
from rcpx.r2.place_count import CountMixin, budget as _budget, call_of as _call, doubts as _doubts, worst as _worst
from rcpx.r2.place_hand import Jaws, finite
from rcpx.r2.place_motion import hold_tick, period_tick, phase, reset_log, split as _split
from rcpx.r2.place_pick import PickMixin, approach_vector
from rcpx.r2.place_set import SetMixin
from rcpx.r2.place_side import SideMixin
from rcpx.r2.place_target import resolve_point, resolve_target
from rcpx.r2.place_tools import PRACTICES, TOOLS

ID = "N4"
MM = 1000.0
GRASPS = ("auto", "top", "rim", "side")


def _no_gripper():
    return refusal("no_gripper", "this robot has no gripper: it cannot pick up a free thing",
                   "push it with the tip (press) instead")


def _nothing_held():
    return refusal("nothing_held", "nothing is held: there is nothing between the jaws to carry or set down",
                   "pick it first (pick or put); a pick that says held false holds nothing")


def _is_pixel(v):
    return isinstance(v, dict) and "look" in v and "px" in v


def _is_seen(v):
    return isinstance(v, dict) and "kind" in v and "top_z" in v


class Mixin(PickMixin, CarryMixin, SetMixin, SideMixin, CountMixin):
    # ---------------------------------------------------------------- intent, spots, budget
    def _pl_intent(self, clear=None, gentle=None, firm=None, expect=None, release=None, clearance_mm=None,
                   touch=None, standoff_mm=None, max_n=None):
        """(intent, refusal). The operator's wishes for this call, checked before anything moves. The word clear
        is information: it is recorded and changes no speed and no guard; it is refused only when it
        names a look this session does not hold (a malformed argument)."""
        intent = {}
        if gentle and firm:
            return None, refusal("intent", "gentle and firm were both given", "one of them, or neither")
        for key, value, allowed in (("expect", expect, ("contact",)), ("release", release, ("drop",))):
            if value is not None and value not in allowed:
                return None, refusal(key, f"{key} must be {' or '.join(allowed)}, got {value!r}",
                                     f"leave {key} out, or {key}: {allowed[0]!r}")
        for key, value in (("clearance_mm", clearance_mm), ("standoff_mm", standoff_mm)):
            if value is None:
                continue
            if not finite(value) or float(value) < 0:
                return None, refusal(key, f"{key} is more room than the robot derives, in mm: it must be a "
                                     f"number that is not negative, got {value!r}", f"leave {key} out")
            intent[key[:-3] + "_m"] = float(value) / MM
        for key, value in (("gentle", gentle), ("firm", firm), ("touch", touch)):
            if value:
                intent[key] = True
        if max_n is not None:      # a limit on the call; a malformed one is refused by the plan (C3)
            intent["max_n"] = max_n
        if expect is not None:
            intent["expect"] = expect
        if release is not None:
            intent["release"] = release
        word = None
        if clear is not None and clear is not False:
            intent["clear"] = clear
            word = bounds.of(self).word(intent)
            bad = word.get("refused") if isinstance(word, dict) else None
            if bad:
                return None, bad if "refused" in bad else {"refused": bad}
        self.state["place"]["word"] = word if isinstance(word, dict) else None
        self.state["place"]["intent"] = intent
        return intent, None

    def _pl_spot(self, value, role):
        """One spot: (Seen or None, point m or None, label or None, refusal or None, doubt or None). Through the
        pointing resolver when it is loaded, with no motion and no control period: a pixel, a label and a point
        all come back as a Seen with the spot it rests on. Without the resolver a label or a point is taken as
        given and a pixel is refused (no plan can be made of it)."""
        if _is_seen(value):
            xy = value.get("pointed_xy") or value.get("centre_xy")
            return value, np.array([float(xy[0]), float(xy[1]), float(value["top_z"])]), value.get("label"), None, None
        try:
            from rcpx.r2 import place_point
        except ImportError:
            place_point = None
        doubt = None
        if isinstance(value, str):        # a label: where it was last known, also when it is stale (said)
            _, _, bad, doubt = resolve_point(self, value, role)
            if bad:
                return None, None, None, bad, None
        if place_point is not None and not (isinstance(value, str) and doubt):
            seen = place_point.resolve(self, value, "pick" if role == "pick" else "place",
                                       self.state["place"].get("intent"))
            if isinstance(seen, dict) and "refused" in seen and not isinstance(value, str):
                return None, None, None, seen, None       # (a name the world holds is taken as the world holds it)
            typed = not _is_pixel(value)
            if _is_seen(seen) and not (typed and seen.get("kind") != "thing"):
                seen = dict(seen)
                if isinstance(value, str):
                    seen.setdefault("label", value)
                if _is_pixel(value) and not seen.get("looks"):
                    seen["looks"] = [value["look"]]
                xy = seen.get("pointed_xy") or seen.get("centre_xy")
                return (seen, np.array([float(xy[0]), float(xy[1]), float(seen["top_z"])]), seen.get("label"), None,
                        doubt)
            # a label or a point that no look shows a thing at is taken as the operator gave it, height and all
        if _is_pixel(value):
            return None, None, None, refusal("spot", "a pixel needs the pointing resolver, and this session has "
                                             "none loaded", "give a located label or [x, y, z] in base frame mm"), None
        p, label, bad, doubt = resolve_point(self, value, role)
        return None, p, label, bad, doubt

    def _pl_periods_left(self):
        """Control periods left, when the backend or the runner states a budget; else None."""
        fn = getattr(self.backend, "steps_left", None)
        try:
            v = fn() if callable(fn) else self.backend.read().get("steps_left")
        except Exception:
            v = None
        if v is None:
            v = self.context.get("periods_left")
            v = v() if callable(v) else v
        return int(v) if finite(v) else None

    # ---------------------------------------------------------------- pick
    def _pl_pick_args(self, pick, point_mm, label, approach, width_mm, jaw_yaw_deg, grasp):
        if self._pl_jaws is None:
            return None, _no_gripper()
        st = self.state["place"]
        if self._pl_held() is not None:
            return None, refusal("holding", "the gripper already holds a thing", "place it (place or put) first")
        given = [v for v in (pick, point_mm, label) if v is not None]
        if len(given) != 1:
            return None, refusal("point", "give the thing as exactly one spot: pick {look, px}, a located label "
                                 "or [x, y, z] mm (the older point_mm and label are still read)",
                                 "pick {\"look\": ID, \"px\": [u, v]}: a pixel anywhere on it in a look you saw")
        t = approach_vector(approach)
        if t is None:
            return None, refusal("approach", f"approach must be a direction name or [x, y, z], got {approach!r}",
                                 "down (the default), or a vector such as [0, 0, -1]")
        if width_mm is not None and (not finite(width_mm) or float(width_mm) <= 0):
            return None, refusal("width_mm", f"width_mm must be a width in mm, got {width_mm!r}",
                                 "leave width_mm out: the robot reads the width from the look and the jaws")
        w = None if width_mm is None else float(width_mm) / MM
        if w is not None and w > self._pl_jaws.max_m + 1e-4:
            return None, refusal("too_wide", f"the thing is {w * MM:g} mm wide and the jaws open only "
                                 f"{self._pl_jaws.max_m * MM:.0f} mm: closing on it is refused",
                                 "point at a narrower part of it (a handle, a rim, a thin side), or push it instead")
        if jaw_yaw_deg is not None and not finite(jaw_yaw_deg):
            return None, refusal("jaw_yaw_deg", f"jaw_yaw_deg must be a bearing in degrees, got {jaw_yaw_deg!r}",
                                 "leave jaw_yaw_deg out: the robot takes the bearing from the outline it sees")
        if grasp is not None and grasp not in GRASPS:
            return None, refusal("grasp", f"grasp must be one of {', '.join(GRASPS)}, got {grasp!r}",
                                 "leave grasp out: the robot chooses from what it sees")
        value = given[0]
        what = "label" if (label is not None or isinstance(value, str)) else ("pick" if pick is not None
                                                                              else "point_mm")
        seen, p, lab, bad, doubt = self._pl_spot(value, "pick")       # no motion: a spot costs nothing
        if bad:
            if "refused" in bad and bad["refused"].get("check") in ("pick", "spot"):
                bad = {"refused": dict(bad["refused"], check=what)}
            return None, bad
        st["label"] = lab
        return {"seen": seen if seen is not None else p, "spot": value, "t": t, "width_hint_m": w, "point": p,
                "yaw_rad": None if jaw_yaw_deg is None else float(np.radians(float(jaw_yaw_deg))),
                "grasp": grasp or "auto", "doubt": doubt, "answer": place_near.answer_of(seen)}, None

    def _pl_hold_facts(self):
        """What a carry and a set down read of the hold (place_motion), with where the held thing stands in the
        hand as the core's record states it after the test lift (place_hold.stands: the arm's own readings and
        one look) in place of the pose before the close, where that reading says it moved between the pads by
        more than its own band and does not swing from them: its middle ("offset_T", "middle"), and
        "bearing_rad": the bearing across its
        narrow side on the level for the hand as it stands now (from the pose before the close where the record
        states no such reading; absent when the take did not see the thing whole)."""
        out = super()._pl_hold_facts()
        st = self.state["place"]
        took = st.get("taken")
        if out is None or not took:
            return out
        stands = dict.get(control_hold.held(self) or {}, "stands")
        long_T = None
        if isinstance(stands, dict) and stands.get("read") and stands.get("moved") and not stands.get("swings"):
            off = np.asarray(stands["middle_T"], dtype=float).copy()
            off[0] = out["offset_T"][0]
            out.update(offset_T=off, middle="the middle of the thing as the camera shows it in the hand after "
                                            "the lift")
            long_T = np.asarray(stands["long_T"], dtype=float)
        elif took.get("bearing_rad") is not None and st.get("closed_R") is not None:
            b = float(took["bearing_rad"])
            long_T = np.asarray(st["closed_R"], dtype=float).T @ np.array([-np.sin(b), np.cos(b), 0.0])
        if long_T is not None:
            w = np.asarray(self._sense().R, dtype=float) @ long_T
            out["bearing_rad"] = float(np.arctan2(-w[0], w[1]))
        return out

    def _pl_pick_run(self, a, intent):
        """The take (rcpx/r2/place_pick.py, C14: once per item; this file never takes again)."""
        st = self.state["place"]
        st["picked_at"] = [float(x) for x in a["point"]]
        st["taken"] = place_spot.taken_of(self, a["seen"])      # the thing as the look showed it before the take
        out = self._pl_pick_impl(a["seen"], a["t"], width_hint_m=a["width_hint_m"], yaw_rad=a["yaw_rad"],
                                 grasp=a["grasp"], intent=intent)
        if isinstance(out, dict) and "refused" not in out and a.get("doubt"):
            out["doubt"] = list(out.get("doubt") or []) + [a["doubt"]]
        if isinstance(out, dict) and out.get("held"):
            place_spot.forget(self, a["point"])     # what this session set down there is in the hand now
        self._pl_mark_picked(out)
        return out

    @primitive
    def pick(self, pick=None, point_mm=None, label=None, approach=None, width_mm=None, jaw_yaw_deg=None,
             grasp=None, touch=None, standoff_mm=None, clear=None, gentle=None, firm=None):
        a, bad = self._pl_pick_args(pick, point_mm, label, approach, width_mm, jaw_yaw_deg, grasp)
        if bad:
            return bad
        intent, bad = self._pl_intent(clear=clear, gentle=gentle, firm=firm, touch=touch, standoff_mm=standoff_mm)
        if bad:
            return bad
        self._pl_begin()
        if a["answer"]:           # the look shows the pointed place empty: nothing moves, no control period
            return self._pl_finish(place_near.answered(a["answer"], "pick"))
        return self._pl_finish(self._pl_pick_run(a, intent))

    # ---------------------------------------------------------------- carry
    @primitive
    def carry(self, to=None, to_point_mm=None, to_label=None, clearance_mm=None, expect=None, clear=None,
              gentle=None, firm=None):
        if self._pl_jaws is None:
            return _no_gripper()
        if self._pl_held() is None:
            return _nothing_held()
        given = [v for v in (to, to_point_mm, to_label) if v is not None]
        if len(given) != 1:
            return refusal("to", "give the destination as exactly one spot: to {look, px}, a located label or "
                           "[x, y, z] mm", "to {\"look\": ID, \"px\": [u, v]}: a pixel on the spot it goes over")
        target = resolve_target(self, to=given[0], resolve=self._pl_spot)
        if "refused" in target:
            return target
        intent, bad = self._pl_intent(clear=clear, gentle=gentle, firm=firm, expect=expect,
                                      clearance_mm=clearance_mm)
        if bad:
            return bad
        self._pl_begin()
        t0 = self.sim_time()
        facts = self._pl_hold_facts()
        way = self._pl_way(target, facts, intent, bounds.of(self))
        if "refused" in way:
            return way
        r = self._pl_carry_impl(target, intent, way)     # (one stream: the core counts it before it starts, C10)
        if "refused" in r:
            return r
        out = self._pl_carried(r, target, facts) if r["stopped"] == "goal" else self._pl_stopped(r, "carrying it")
        rec = phase(self, "carry", "carried it over the spot" if out.get("carried") else "the carry stopped", t0, [r])
        rec["strike"] = r.get("strike")
        out["doubt"] = list(r.get("doubt") or []) + list(target.get("doubt") or [])
        return self._pl_finish(out)

    # ---------------------------------------------------------------- place
    @primitive
    def place(self, on=None, into=None, to=None, on_point_mm=None, on_label=None, in_corners=None, in_rim=None,
              release=None, release_mm=None, clearance_mm=None, expect=None, clear=None, gentle=None, firm=None,
              **more):
        if self._pl_jaws is None:
            return _no_gripper()
        if self._pl_held() is None:
            return _nothing_held()
        into = more.pop("in", into)
        if more:
            return refusal("args", f"place takes no argument {sorted(more)[0]!r}", "on, in or to, with clear")
        target, bad = self._pl_place_target(on, into, to, on_point_mm, on_label, in_corners, in_rim, release_mm)
        if bad:
            return bad
        intent, bad = self._pl_intent(clear=clear, gentle=gentle, firm=firm, expect=expect, release=release,
                                      clearance_mm=clearance_mm)
        if bad:
            return bad
        self._pl_begin()
        out = self._pl_place_impl(target, intent, release,
                                  None if release_mm is None else float(release_mm) / MM)
        return self._pl_finish(out)

    def _pl_place_target(self, on, into, to, on_point_mm, on_label, in_corners, in_rim, release_mm):
        if on_point_mm is not None and on_label is not None:
            return None, refusal("target", "give on_point_mm or on_label, not both", "one place target")
        if on is not None and (on_point_mm is not None or on_label is not None):
            return None, refusal("target", "give on, or the older on_point_mm or on_label, not both",
                                 "one place target")
        old = on_point_mm if on_label is None else on_label
        target = resolve_target(self, on=on if on is not None else old, corners=in_corners, rim=in_rim,
                                into=into, to=to, resolve=self._pl_spot)
        if "refused" in target:
            return None, target
        if release_mm is not None and (not finite(release_mm) or float(release_mm) < 0):
            return None, refusal("release_mm", "release_mm is the height the thing's bottom is let go above "
                                 f"the surface, in mm: it cannot be negative, got {release_mm!r}",
                                 "leave it out (the thing is lowered until it touches down), or release: drop")
        return target, None

    def _pl_place_impl(self, target, intent, release=None, release_m=None, counted=False):
        """The held thing to its place: the carry (ONE stream: up, across, down to the carry height), the way down
        (an expected contact, with its own price), the release, the rise; through the open side when the place
        stands under a cover the space book holds (place_side). Its control periods are counted before it moves
        (C12); counted: the caller's budget line held this place already (a put counts the whole item before
        its take: the thing is then in the hand, and each stream piece still counts itself). Returns the
        result or a refusal."""
        st = self.state["place"]
        B = bounds.of(self)
        t0 = self.sim_time()
        facts = self._pl_hold_facts()
        before = (target.get("seen") or {}).get("looks") or []
        picked = st.get("picked_at")
        place_spot.settle(self, target, facts)       # the design rulings: where on the pointed thing the held thing stands
        plan = self._pl_set_plan(target, intent, facts, release, release_m)
        cover = self._pl_cover(target, plan, B)
        if cover is not None:
            out = self._pl_side_place(target, plan, intent, cover, t0)
        else:
            way = self._pl_way(target, facts, intent, B, z_end=plan["z_end"], expect=plan["expect"],
                               z_hand=plan["hand_stop_z"])
            if "refused" in way:
                return way
            count, left = self._pl_place_count(way, facts), self._pl_periods_left()
            if left is not None and count is not None and count["periods"] > left and not counted:
                return _budget("place", count["periods"], left)
            plan["over_xy"] = [float(x) for x in way["over"][:2]]
            r = self._pl_carry_impl(target, intent, way)
            if "refused" in r:
                return r
            rec = phase(self, "carry", f"carried it over the place ({r.get('route')})" if r["stopped"] == "goal"
                        else "the carry stopped", t0, [r])
            rec["strike"] = r.get("strike")
            if r["stopped"] != "goal":
                out = self._pl_stopped(r, "carrying it", {"bounds": r.get("bounds"), "route": r.get("route")})
                out["doubt"] = list(r.get("doubt") or [])
                return out
            t1 = self.sim_time()
            down = {"stopped": "goal", "bounds": None}
            if way["set_leg"] is not None:      # the jaws open where the hand stands: its end is waited for
                down = self._pl_stream([way["set_leg"]], ["set_down"], held=self._pl_held(),
                                       need_m=self._pl_still(B, self._told().q, facts))
                if "refused" in down:
                    return self._pl_refuse_motion(down, "lowering the thing (it is held over the place)")
            down.update(route=r.get("route"), doubt=list(r.get("doubt") or []))
            out = self._pl_after_way(down, target, plan, facts, intent, t1)
            if isinstance(out, dict) and "refused" not in out:
                out["planned_periods"] = count
        if "refused" in out or "release" not in out:
            return out
        arrived, ev = place_spot.arrival(self, target, out, facts, before[0] if before else st.get("scene_look"), picked)
        place_spot.note_release(self, target, out, facts)       # the next free spot reads it (place_spot._since)
        out["arrived"] = arrived
        out["evidence"] = ev
        if arrived is None:
            # "placed" is said only where the views show it at the place: not seen, not said
            out["placed"] = None if out.get("placed") is not False else False
            out["said"] += " Whether it rests there is not seen; a look shows it (0 control periods)."
        elif arrived is False:        # "placed" is true by truth: let go, and not seen at the place
            why = (ev.get("landing") or {}).get("rests") or (ev.get("depth") or {}).get("rests")
            out.update(placed=False, said=out["said"] + " The evidence says it did not arrive" + (
                f" (it rests {why})" if why else "") + ": look before the next step.")
        return place_spot.say(out, target)

    # ---------------------------------------------------------------- put
    @primitive
    def put(self, pick=None, to=None, on=None, into=None, items=None, pick_point_mm=None, pick_label=None,
            on_point_mm=None, on_label=None, in_corners=None, in_rim=None, width_mm=None, jaw_yaw_deg=None,
            grasp=None, release=None, release_mm=None, clearance_mm=None, expect=None, clear=None, gentle=None,
            firm=None, max_n=None, **more):
        into = more.pop("in", into)
        if more:
            return refusal("args", f"put takes no argument {sorted(more)[0]!r}", "pick and to, or items")
        single = any(v is not None for v in (pick, to, on, into, pick_point_mm, pick_label, on_point_mm, on_label,
                                             in_corners, in_rim))
        if (items is not None) == single:
            return refusal("target", "give one thing (pick and to) or several (items), not both and not neither",
                           "put {\"pick\": {\"look\": ID, \"px\": [u, v]}, \"to\": {\"look\": ID, \"px\": [u, v]}}")
        if items is None:
            items = [{"pick": pick if pick is not None else (pick_point_mm if pick_label is None else pick_label),
                      "to": to, "on": on if on is not None else (on_point_mm if on_label is None else on_label),
                      "in": into, "in_corners": in_corners, "in_rim": in_rim, "width_mm": width_mm,
                      "jaw_yaw_deg": jaw_yaw_deg, "grasp": grasp, "release": release, "release_mm": release_mm}]
            if pick is not None and (pick_point_mm is not None or pick_label is not None) or \
                    (pick_point_mm is not None and pick_label is not None):
                return refusal("point", "give the thing as exactly one spot", "pick {look, px}")
        if not isinstance(items, (list, tuple)) or not items or not all(isinstance(i, dict) for i in items):
            return refusal("items", "items must be a list of {pick, to}", "items [{\"pick\": ..., \"to\": ...}]")
        intent, bad = self._pl_intent(clear=clear, gentle=gentle, firm=firm, expect=expect, release=release,
                                      clearance_mm=clearance_mm, max_n=max_n)
        if bad:
            return bad
        self._pl_begin()
        return self._pl_finish(self._pl_put_impl(list(items), intent))

    def _pl_put_item(self, item):
        """The checks of one item before anything moves, the spot of both ends: (pick args, place target,
        refusal)."""
        if not any(item.get(k) is not None for k in ("to", "on", "in", "in_corners", "in_rim")):
            return None, None, refusal("target", "say where it goes: to, on or in", "to {\"look\": ID, \"px\": "
                                       "[u, v]}: a pixel on the spot, the thing or the opening it goes to")
        target, bad = self._pl_place_target(item.get("on"), item.get("in"), item.get("to"), None, None,
                                            item.get("in_corners"), item.get("in_rim"), item.get("release_mm"))
        if bad:
            return None, None, bad
        a, bad = self._pl_pick_args(item.get("pick"), None, None, None, item.get("width_mm"),
                                    item.get("jaw_yaw_deg"), item.get("grasp"))
        if bad:
            return None, None, bad
        return a, target, None

    def _pl_put_impl(self, items, intent):
        """Each item: the spot of both ends with no motion, the budget line (the count of C12), the take (once: the
        pick's own, C14), the place."""
        done, last = [], None
        for i, item in enumerate(items):
            nth = f"item {i + 1} of {len(items)}: " if len(items) > 1 else ""
            a, target, bad = self._pl_put_item(item)      # the place first: a place not found costs no pick
            if bad:
                last = bad if not done else {"said": f"Stopped before {nth}{bad['refused']['why']}",
                                             "held": False, "placed": False, "refusal": bad["refused"]}
                break
            if a["answer"]:       # the look shows the pointed place empty: this item moves nothing
                last = place_near.answered(a["answer"], "put", nth)
                break
            left = self._pl_periods_left()
            price = self._pl_price(a, target, intent) if left is not None else None
            word = self._pl_word(target, intent)
            count = lambda i: self._pl_price(a, target, i)["periods"]     # noqa: E731
            over = price is not None and price["periods"] > left
            priced = None if word is None else dict(word, **self._pl_word_price(word, intent, count, left if over else None))
            if over:
                bad = _budget("put", price["periods"], left, price, priced, _call("put", item, intent))
                last = bad if not done else {"said": f"Did not start {nth}{bad['refused']['why']}", "held": False,
                                             "placed": False, "will_not_fit": {"needs_periods": price["periods"],
                                                                               "periods_left": left}}
                break
            n0 = self._pl_periods_used()
            rel = item.get("release") or intent.get("release")
            got = self._pl_pick_run(a, intent)
            if "refused" in got:
                last = got if not done else {"said": f"Stopped at {nth}the pick was refused: "
                                             f"{got['refused']['why']}", "held": False, "placed": False,
                                             "pick": got}
                break
            if not got.get("held") or self._pl_held() is None:
                last = {"said": f"Did not place {nth}".rstrip(": ") + ": " + got.get("said", ""), "held": False,
                        "placed": False,
                        "pick": got, "doubt": list(got.get("doubt") or [])}
                break
            put = self._pl_place_impl(target, intent, rel, None if item.get("release_mm") is None
                                      else float(item["release_mm"]) / MM, counted=price is not None)
            if "refused" in put:
                last = {"said": f"Picked it but did not place it ({nth.rstrip(': ') or 'the place'}): "
                                f"{put['refused']['why']}", "held": self._pl_held() is not None, "placed": False,
                        "pick": got, "place": put, "next": "place it elsewhere (place), or set it back where it stood"}
                break
            n_item = self._pl_periods_used() - n0
            last = {"said": put.get("said", ""), "held": bool(put.get("held")),
                    "placed": None if put.get("placed") is None else bool(put.get("placed")),
                    "pick": got, "place": put, "arrived": put.get("arrived"),
                    "doubt": list(got.get("doubt") or []) + list(put.get("doubt") or []),
                    "strike": _worst([got.get("strike"), put.get("strike")]), "priced_periods": price}
            if priced is not None:    # what the stated limit cost: the periods it added, and where
                last["limit"] = dict(put.get("limit") or {}, word=word["word"], cost=self._pl_limit_cost(
                    word, priced["without_periods"], n_item, self.state["place"].get("phases")))
            done.append({"placed": last["placed"], "arrived": last["arrived"], "periods": n_item,
                         "said": last["said"]})
            if last["placed"] is False:
                break
        if isinstance(last, dict) and "refused" in last:
            return last
        out = dict(last)
        if len(items) > 1:
            n_ok = sum(1 for d in done if d["placed"] is True)
            out["items"] = done
            out["items_placed"] = n_ok
            out["said"] = (f"Placed {n_ok} of {len(items)} things. " if n_ok < len(items) or n_ok == 0 else
                           f"Placed all {n_ok} things. ") + out["said"]
            out["placed"] = True if n_ok == len(items) else (False if any(d["placed"] is False for d in done) else None)
        return out

    # ---------------------------------------------------------------- the frame of every tool
    def _pl_begin(self):
        """A tool starts: the phases of the last call go, the period log too."""
        st = self.state["place"]
        st["phases"] = []
        reset_log(self)
        st["t_call"] = self.sim_time()

    def _pl_periods_used(self):
        return len(self.state["place"].get("log", {}).get("rows") or [])

    def _pl_finish(self, out):
        """The receipt of a whole-step tool (C12): said first; held and placed; the phases with their control
        periods, each with its own strike; the periods used and left; the largest strike of its motions and
        predicted_peak_n; the bounds with their sources; the word when one was given; the doubt lines."""
        if not isinstance(out, dict) or "refused" in out:
            return out
        st = self.state["place"]
        phases = list(st.get("phases") or [])
        own = out.get("phases") or []
        phases = phases if phases else list(own)
        used = self._pl_periods_used()
        told = sum(p["periods"] for p in phases)
        if used > told and phases:       # periods after the last phase (each phase claims those before it): said
            end = (st.get("log") or {}).get("phase_end_t")
            mv, wt, jw = _split(self, float("-inf") if end is None else float(end))
            phases.append({"phase": "between", "did": "control periods after the last phase", "periods": used - told,
                           "moving": mv, "waiting": wt, "jaw": jw,
                           "seconds": round((used - told) * float(self.dt), 3), "free_share": None, "bounds": None})
        left = self._pl_periods_left()
        head = {"said": out.get("said", "")}
        for k in ("held", "placed"):
            if k in out:
                head[k] = out[k]
        body = {k: v for k, v in out.items() if k not in ("said", "held", "placed", "phases", "bounds", "strike",
                                                          "doubt")}
        res = {**head, "phases": phases, "periods": [[p["phase"], p["periods"]] for p in phases],
               "periods_used": used, "periods_left": left, **body}
        strike = _worst([out.get("strike")] + [p.get("strike") for p in phases])
        res["strike"] = strike
        res["predicted_peak_n"] = None if strike is None else strike.get("peak_n")
        res["doubt"] = _doubts(out.get("doubt"))
        items = [p["bounds"] for p in phases if p.get("bounds")]
        word = st.get("word")
        if word and word.get("given"):
            res["word"] = {"given": True, "look": word.get("look"), "note": word.get("text")}
        if left is not None:
            res["said"] = res["said"].rstrip() + f" {used} control periods used, {left} left."
        try:
            res = bounds.of(self).report(res, items)
        except Exception:
            res["bounds"] = {"binding": items[0] if items else None, "more": max(0, len(items) - 1)}
        return res

    def _pl_mark_picked(self, out):
        """A picked label's point is stale once the thing rose with the jaws."""
        label = self.state["place"].get("label")
        if label and isinstance(out, dict) and out.get("held"):
            ids = [e["id"] for e in self.world.query(label=label) if not e.get("stale")]
            self.world.mark_stale(ids, "picked up: it moves with the jaws", source="place")


def body_patch(ex, session, content):
    if not isinstance(content, dict):
        return content
    J = ex._pl_jaws
    return {**content, "place": None if J is None else J.describe()}


RESOURCE_PATCH = {"robot://body": body_patch}


def setup(ex):
    g = ex.model.spec.get("gripper")
    ex._pl_jaws = Jaws(ex.model) if g else None
    ex.state["place"] = {"phases": []}
    if ex._pl_jaws is not None:
        ex.add_tick(period_tick(ex))
        ex.add_tick(hold_tick(ex))


__all__ = ["ID", "Mixin", "PRACTICES", "RESOURCE_PATCH", "TOOLS", "setup"]
